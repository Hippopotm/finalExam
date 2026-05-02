#!/usr/bin/env python3
"""Frontier-based Exploration Planner with LIFO Stack.

Algorithm overview
------------------
1. The robot starts with a fully-unknown map (provided by lidar_mapper).
2. Every FRONTIER_UPDATE_PERIOD seconds the planner scans the occupancy grid
   for *frontier cells* — free cells that have at least one unknown 4-neighbour.
3. Adjacent frontier cells are clustered via BFS.  Each cluster's centroid is a
   candidate exploration target.
4. New centroids are pushed onto a LIFO stack (most-recently-found = next to
   visit), implementing depth-first frontier exploration.
5. The navigation tick fires at NAV_TICK_HZ:
     a. If a goal is directly reachable (A* path exists), go there.
     b. Otherwise pop the frontier stack and navigate to the top entry.
     c. If the stack is empty, perform a short in-place spin to expose new scan
        data and rebuild frontiers.
6. Path execution follows the same waypoint-stride pattern as astar_planner:
   the A* cell-path is sub-sampled and each sub-goal is fed one at a time to
   the downstream PID controller via /goal_pose.

Topics
------
Subscribes : /map  (nav_msgs/OccupancyGrid)
             /odom (nav_msgs/Odometry)
             /goal_points (visualization_msgs/MarkerArray)
Publishes  : /goal_pose    (geometry_msgs/PoseStamped)
             /planned_path (nav_msgs/Path)
             /frontiers    (visualization_msgs/MarkerArray)  — for RViz
"""
import heapq
import math
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

# ── Tuning constants ──────────────────────────────────────────────────────────
NAV_TICK_HZ          = 10.0    # Hz – navigation loop rate
FRONTIER_UPDATE_HZ   = 1.5     # Hz – frontier detection rate

GOAL_REACH_TOL       = 0.40    # m  – distance to consider a goal reached
WAYPOINT_REACH_TOL   = 0.35    # m  – distance to advance to next waypoint
WAYPOINT_STRIDE      = 6       # cells between consecutive waypoints
OBSTACLE_INFLATION   = 3       # cells to inflate obstacles in cost map

FRONTIER_MERGE_DIST  = 0.80    # m  – centroids closer than this → same cluster
FRONTIER_MIN_CELLS   = 4       # ignore clusters smaller than this (cell count)
FRONTIER_SNAP_RADIUS = 15      # cells – max snap radius for find_nearest_free

STUCK_TIMEOUT        = 10.0    # s  – abandon waypoint if no progress after this
STUCK_PROGRESS_TOL   = 0.05    # m  – movement smaller than this → "stuck"


class FrontierPlanner(Node):

    def __init__(self) -> None:
        super().__init__('frontier_planner')

        # ── Map state ─────────────────────────────────────────────────────────
        self._map_msg:  Optional[OccupancyGrid] = None
        self._occ_grid: Optional[np.ndarray]    = None   # raw, shape (H,W)
        self._inf_grid: Optional[np.ndarray]    = None   # inflated copy

        # ── Robot state ───────────────────────────────────────────────────────
        self._robot_xy: Optional[Tuple[float, float]] = None

        # ── Goals ─────────────────────────────────────────────────────────────
        self._all_goals:       List[Tuple[float, float]] = []   # from /goal_points
        self._remaining_goals: List[Tuple[float, float]] = []
        self._failed_goals:    set                        = set()

        # ── Frontier stack (LIFO) ─────────────────────────────────────────────
        self._frontier_stack: List[Tuple[float, float]] = []

        # ── Active waypoint tracking ──────────────────────────────────────────
        self._active_target:   Optional[Tuple[float, float]] = None
        self._pending_wps:     List[Tuple[float, float]]     = []
        self._wp_idx:          int                            = -1
        self._is_goal_target:  bool                          = False   # True when target is a mission goal

        # ── Stuck detection ───────────────────────────────────────────────────
        self._last_robot_xy:  Optional[Tuple[float, float]] = None
        self._stuck_timer:    float                          = 0.0

        # ── ROS I/O ───────────────────────────────────────────────────────────
        self._map_sub = self.create_subscription(
            OccupancyGrid, '/map', self._map_cb, 10
        )
        self._odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_cb, 20
        )
        self._goals_sub = self.create_subscription(
            MarkerArray, '/goal_points', self._goals_cb, 10
        )

        self._goal_pub     = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self._path_pub     = self.create_publisher(Path, '/planned_path', 10)
        self._frontier_pub = self.create_publisher(MarkerArray, '/frontiers', 10)

        self.create_timer(1.0 / NAV_TICK_HZ,        self._nav_tick)
        self.create_timer(1.0 / FRONTIER_UPDATE_HZ, self._frontier_tick)

        self.get_logger().info(
            'FrontierPlanner ready — waiting for /map, /odom, /goal_points'
        )

    # ── Subscription callbacks ────────────────────────────────────────────────

    def _map_cb(self, msg: OccupancyGrid) -> None:
        self._map_msg = msg
        h = int(msg.info.height)
        w = int(msg.info.width)
        if h == 0 or w == 0:
            return
        raw = np.array(msg.data, dtype=np.int16).reshape((h, w))
        self._occ_grid = raw
        self._inf_grid = self._inflate(raw, OBSTACLE_INFLATION)

    def _odom_cb(self, msg: Odometry) -> None:
        p = self._robot_xy
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self._robot_xy = (x, y)
        if p is not None:
            if self._dist(self._robot_xy, p) > STUCK_PROGRESS_TOL:
                self._last_robot_xy = self._robot_xy
                self._stuck_timer   = 0.0

    def _goals_cb(self, msg: MarkerArray) -> None:
        goals = [
            (float(m.pose.position.x), float(m.pose.position.y))
            for m in msg.markers
        ]
        if not goals:
            return
        if sorted(goals) != sorted(self._all_goals):
            self._all_goals       = goals
            self._remaining_goals = goals.copy()
            self._failed_goals.clear()
            self._active_target = None
            self._pending_wps   = []
            self._wp_idx        = -1
            self.get_logger().info(
                f'Received {len(goals)} goal points: {goals}'
            )

    # ── Navigation tick (NAV_TICK_HZ) ─────────────────────────────────────────

    def _nav_tick(self) -> None:
        if self._inf_grid is None or self._robot_xy is None:
            return

        dt = 1.0 / NAV_TICK_HZ
        self._stuck_timer += dt

        # ── Step 1: Follow active waypoints ───────────────────────────────────
        if self._pending_wps and self._wp_idx >= 0:
            wp = self._pending_wps[self._wp_idx]

            # Check if stuck on this waypoint
            if self._stuck_timer > STUCK_TIMEOUT:
                self.get_logger().warn(
                    f'Stuck at wp {self._wp_idx} toward {self._active_target}; abandoning'
                )
                if self._is_goal_target and self._active_target in self._remaining_goals:
                    self._failed_goals.add(self._active_target)
                self._clear_active()
                return

            dist = self._dist(self._robot_xy, wp)
            if dist <= WAYPOINT_REACH_TOL:
                # Advance waypoint
                if self._wp_idx < len(self._pending_wps) - 1:
                    self._wp_idx += 1
                    self._stuck_timer = 0.0
                    self._pub_goal(self._pending_wps[self._wp_idx])
                    return
                # Final waypoint reached → target reached
                self._on_target_reached()
                return
            self._pub_goal(wp)
            return

        # ── Step 2: Plan toward a goal or the top frontier ────────────────────
        self._select_and_plan()

    # ── Frontier detection tick (FRONTIER_UPDATE_HZ) ──────────────────────────

    def _frontier_tick(self) -> None:
        if self._occ_grid is None or self._robot_xy is None:
            return
        centroids = self._detect_frontier_centroids()
        self._update_stack(centroids)
        self._publish_frontiers(centroids)
        self.get_logger().debug(
            f'Frontiers detected: {len(centroids)}, stack size: {len(self._frontier_stack)}'
        )

    # ── Goal / frontier selection & path planning ──────────────────────────────

    def _select_and_plan(self) -> None:
        # Priority 1: remaining goals reachable via A*
        reachable = []
        for g in self._remaining_goals:
            if g in self._failed_goals:
                continue
            path = self._plan_path(self._robot_xy, g)
            if path:
                reachable.append((self._dist(self._robot_xy, g), g, path))

        if reachable:
            reachable.sort(key=lambda t: t[0])
            _, goal, path = reachable[0]
            self._activate(goal, path, is_goal=True)
            self.get_logger().info(f'Navigating to goal {goal}')
            return

        # Priority 2: pop frontier stack (LIFO)
        while self._frontier_stack:
            candidate = self._frontier_stack.pop()
            if not self._is_still_frontier(candidate):
                continue
            path = self._plan_path(self._robot_xy, candidate)
            if path:
                self._activate(candidate, path, is_goal=False)
                self.get_logger().info(
                    f'Navigating to frontier {candidate:.2f}' if False else
                    f'Navigating to frontier ({candidate[0]:.2f}, {candidate[1]:.2f})'
                )
                return

        if not self._remaining_goals:
            return  # Mission complete

        # Priority 3: all goals failed + stack empty → try failed goals once more
        for g in list(self._failed_goals):
            path = self._plan_path(self._robot_xy, g)
            if path:
                self._failed_goals.discard(g)
                self._remaining_goals.append(g)
                self._activate(g, path, is_goal=True)
                self.get_logger().info(f'Retrying previously-failed goal {g}')
                return

    def _activate(
        self,
        target: Tuple[float, float],
        path_rc: List[Tuple[int, int]],
        is_goal: bool,
    ) -> None:
        wps = self._path_to_waypoints(path_rc)
        if not wps:
            return
        self._active_target  = target
        self._pending_wps    = wps
        self._wp_idx         = 0
        self._is_goal_target = is_goal
        self._stuck_timer    = 0.0
        self._pub_path(path_rc)
        self._pub_goal(wps[0])

    def _clear_active(self) -> None:
        self._active_target = None
        self._pending_wps   = []
        self._wp_idx        = -1
        self._stuck_timer   = 0.0

    def _on_target_reached(self) -> None:
        target = self._active_target
        if self._is_goal_target and target in self._remaining_goals:
            self._remaining_goals.remove(target)
            self.get_logger().info(
                f'Reached goal {target}; '
                f'{len(self._remaining_goals)} goals remaining'
            )
            if not self._remaining_goals:
                self.get_logger().info('All goals reached!  Mission complete.')
        self._clear_active()

    # ── Frontier detection ────────────────────────────────────────────────────

    def _detect_frontier_centroids(self) -> List[Tuple[float, float]]:
        """Return world-frame (x, y) centroids of frontier cell clusters."""
        occ = self._occ_grid
        if occ is None:
            return []

        H, W = occ.shape
        free    = (occ == 0)
        unknown = (occ < 0)

        # Frontier cells: free AND adjacent to unknown (4-connectivity)
        unk_shifted = (
            np.roll(unknown,  1, axis=0)
            | np.roll(unknown, -1, axis=0)
            | np.roll(unknown,  1, axis=1)
            | np.roll(unknown, -1, axis=1)
        )
        # Eliminate roll wrap-around at borders
        unk_shifted[0, :]  = False
        unk_shifted[-1, :] = False
        unk_shifted[:, 0]  = False
        unk_shifted[:, -1] = False

        frontier_mask = free & unk_shifted

        rows_arr, cols_arr = np.where(frontier_mask)
        if len(rows_arr) == 0:
            return []

        visited  = np.zeros((H, W), dtype=bool)
        centroids: List[Tuple[float, float]] = []

        for r0, c0 in zip(rows_arr.tolist(), cols_arr.tolist()):
            if visited[r0, c0]:
                continue
            # BFS over connected frontier cells
            cluster_r: List[int] = [r0]
            cluster_c: List[int] = [c0]
            visited[r0, c0] = True
            idx = 0
            while idx < len(cluster_r):
                r, c = cluster_r[idx], cluster_c[idx]
                idx += 1
                for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                    nr, nc = r + dr, c + dc
                    if (
                        0 <= nr < H and 0 <= nc < W
                        and not visited[nr, nc]
                        and frontier_mask[nr, nc]
                    ):
                        visited[nr, nc] = True
                        cluster_r.append(nr)
                        cluster_c.append(nc)

            if len(cluster_r) < FRONTIER_MIN_CELLS:
                continue

            mean_r = float(sum(cluster_r)) / len(cluster_r)
            mean_c = float(sum(cluster_c)) / len(cluster_c)
            wx, wy = self._grid_to_world(mean_r, mean_c)
            centroids.append((wx, wy))

        return centroids

    def _update_stack(self, centroids: List[Tuple[float, float]]) -> None:
        """Prune stale entries then push novel centroids (LIFO)."""
        # Remove stack entries that are no longer frontier cells
        valid = [
            c for c in self._frontier_stack
            if self._is_still_frontier(c)
        ]
        self._frontier_stack = valid

        # Push new centroids not already well-represented in the stack
        for c in centroids:
            if not any(
                self._dist(c, s) < FRONTIER_MERGE_DIST
                for s in self._frontier_stack
            ):
                self._frontier_stack.append(c)   # push (LIFO: pop from end)

    def _is_still_frontier(self, xy: Tuple[float, float]) -> bool:
        """A frontier is still valid if its map cell is free or unknown,
        and has at least one unknown 4-neighbour."""
        if self._occ_grid is None:
            return True
        rc = self._world_to_grid(xy[0], xy[1])
        if rc is None:
            return False
        r, c = rc
        H, W = self._occ_grid.shape
        # If the cell is now fully known (either free or occupied), check neighbours
        # Accept as still-frontier if the region is not fully explored
        for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            nr, nc = r + dr, c + dc
            if 0 <= nr < H and 0 <= nc < W and self._occ_grid[nr, nc] < 0:
                return True
        return False

    # ── A* path planning ──────────────────────────────────────────────────────

    def _plan_path(
        self,
        start_xy: Tuple[float, float],
        goal_xy:  Tuple[float, float],
    ) -> List[Tuple[int, int]]:
        """Return an A* path (list of (row, col)) or [] if not found."""
        grid = self._inf_grid
        if grid is None or self._map_msg is None:
            return []

        s = self._world_to_grid(start_xy[0], start_xy[1])
        g = self._world_to_grid(goal_xy[0],  goal_xy[1])
        if s is None or g is None:
            return []

        s = self._find_nearest_free(s, grid)
        g = self._find_nearest_free(g, grid)
        if s is None or g is None:
            return []

        return self._a_star(s, g, grid)

    def _a_star(
        self,
        start: Tuple[int, int],
        goal:  Tuple[int, int],
        grid:  np.ndarray,
    ) -> List[Tuple[int, int]]:
        H, W = grid.shape
        if grid[start] >= 50 or grid[goal] >= 50:
            return []

        MOVES = [
            (-1,  0, 1.0), (1,  0, 1.0), ( 0, -1, 1.0), ( 0, 1, 1.0),
            (-1, -1, math.sqrt(2.0)), (-1, 1, math.sqrt(2.0)),
            ( 1, -1, math.sqrt(2.0)), ( 1, 1, math.sqrt(2.0)),
        ]

        heap    = []
        g_score = {start: 0.0}
        came_from: dict = {}
        heapq.heappush(heap, (self._heuristic(start, goal), start))

        while heap:
            _, cur = heapq.heappop(heap)
            if cur == goal:
                return self._reconstruct(came_from, cur)

            cr, cc = cur
            for dr, dc, cost in MOVES:
                nr, nc = cr + dr, cc + dc
                if not (0 <= nr < H and 0 <= nc < W):
                    continue
                if grid[nr, nc] >= 50:
                    continue
                # Prevent diagonal cuts through occupied corners
                if dr != 0 and dc != 0:
                    if grid[cr + dr, cc] >= 50 or grid[cr, cc + dc] >= 50:
                        continue
                nxt = (nr, nc)
                tentative = g_score[cur] + cost
                if tentative < g_score.get(nxt, float('inf')):
                    came_from[nxt] = cur
                    g_score[nxt]   = tentative
                    heapq.heappush(
                        heap, (tentative + self._heuristic(nxt, goal), nxt)
                    )
        return []

    # ── Grid helpers ──────────────────────────────────────────────────────────

    def _inflate(self, grid: np.ndarray, radius: int) -> np.ndarray:
        """Return a copy of *grid* with obstacles expanded by *radius* cells.
        Unknown cells (value < 0) are treated as free for inflation purposes so
        the robot can still plan toward frontier edges."""
        occ     = grid >= 50
        inflated = occ.copy()
        if radius <= 0:
            return np.where(inflated, 100, 0).astype(np.int16)
        H, W = grid.shape
        for r, c in zip(*np.where(occ)):
            r0, r1 = max(0, r - radius), min(H, r + radius + 1)
            c0, c1 = max(0, c - radius), min(W, c + radius + 1)
            inflated[r0:r1, c0:c1] = True
        # Preserve unknown (-1) in non-inflated cells
        result = np.where(inflated, 100, grid).astype(np.int16)
        return result

    def _find_nearest_free(
        self,
        rc: Tuple[int, int],
        grid: np.ndarray,
        max_radius: int = FRONTIER_SNAP_RADIUS,
    ) -> Optional[Tuple[int, int]]:
        r0, c0 = rc
        H, W = grid.shape
        if 0 <= r0 < H and 0 <= c0 < W and grid[r0, c0] < 50:
            return rc
        for radius in range(1, max_radius + 1):
            best, best_d = None, float('inf')
            for r in range(max(0, r0 - radius), min(H, r0 + radius + 1)):
                for c in range(max(0, c0 - radius), min(W, c0 + radius + 1)):
                    if grid[r, c] < 50:
                        d = (r - r0) ** 2 + (c - c0) ** 2
                        if d < best_d:
                            best_d, best = d, (r, c)
            if best is not None:
                return best
        return None

    def _world_to_grid(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        if self._map_msg is None:
            return None
        info = self._map_msg.info
        col = int((x - info.origin.position.x) / info.resolution)
        row = int((y - info.origin.position.y) / info.resolution)
        if 0 <= row < int(info.height) and 0 <= col < int(info.width):
            return (row, col)
        return None

    def _grid_to_world(self, row: float, col: float) -> Tuple[float, float]:
        assert self._map_msg is not None
        info = self._map_msg.info
        x = info.origin.position.x + (col + 0.5) * info.resolution
        y = info.origin.position.y + (row + 0.5) * info.resolution
        return (x, y)

    def _path_to_waypoints(
        self, path_rc: List[Tuple[int, int]]
    ) -> List[Tuple[float, float]]:
        if not path_rc:
            return []
        stride  = max(1, WAYPOINT_STRIDE)
        sampled = path_rc[::stride]
        if sampled[-1] != path_rc[-1]:
            sampled.append(path_rc[-1])
        return [self._grid_to_world(r, c) for r, c in sampled]

    @staticmethod
    def _reconstruct(
        came_from: dict, current: Tuple[int, int]
    ) -> List[Tuple[int, int]]:
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    @staticmethod
    def _heuristic(a: Tuple[int, int], b: Tuple[int, int]) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    @staticmethod
    def _dist(p: Tuple[float, float], q: Tuple[float, float]) -> float:
        return math.hypot(p[0] - q[0], p[1] - q[1])

    # ── Publishers ────────────────────────────────────────────────────────────

    def _pub_goal(self, xy: Tuple[float, float]) -> None:
        msg = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(xy[0])
        msg.pose.position.y = float(xy[1])
        msg.pose.orientation.w = 1.0
        self._goal_pub.publish(msg)

    def _pub_path(self, path_rc: List[Tuple[int, int]]) -> None:
        p = Path()
        p.header.stamp    = self.get_clock().now().to_msg()
        p.header.frame_id = 'map'
        for r, c in path_rc:
            x, y = self._grid_to_world(r, c)
            ps = PoseStamped()
            ps.header = p.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.w = 1.0
            p.poses.append(ps)
        self._path_pub.publish(p)

    def _publish_frontiers(
        self, centroids: List[Tuple[float, float]]
    ) -> None:
        ma = MarkerArray()
        # Delete all previous markers first
        del_marker = Marker()
        del_marker.action = Marker.DELETEALL
        ma.markers.append(del_marker)

        for i, (fx, fy) in enumerate(centroids):
            m = Marker()
            m.header.stamp    = self.get_clock().now().to_msg()
            m.header.frame_id = 'map'
            m.ns     = 'frontiers'
            m.id     = i
            m.type   = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(fx)
            m.pose.position.y = float(fy)
            m.pose.position.z = 0.1
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.25
            m.color.r = 1.0
            m.color.g = 0.5
            m.color.b = 0.0
            m.color.a = 0.8
            m.lifetime.sec = 2
            ma.markers.append(m)
        self._frontier_pub.publish(ma)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FrontierPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
