#!/usr/bin/env python3
"""2D Lidar-based Occupancy Grid Mapper.

Subscribes to /lidar (LaserScan) and /odom (Odometry), builds an occupancy
grid incrementally using log-odds ray-casting, and publishes /map at MAP_PUB_HZ.

Map frame convention
--------------------
  col  →  world X
  row  →  world Y
  origin = bottom-left corner of the grid in world coordinates

The map covers a fixed 24 × 24 m region centred at (0, 0), large enough for
all supplied maze environments.
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header

# ── Map parameters ────────────────────────────────────────────────────────────
MAP_RESOLUTION: float = 0.05          # metres / cell
MAP_HALF_SIDE:  float = 12.0          # half-width of square map (metres)
MAP_ORIGIN:     float = -MAP_HALF_SIDE
GRID_SIZE:      int   = int(2 * MAP_HALF_SIDE / MAP_RESOLUTION)  # 480 cells
MAP_PUB_HZ:     float = 2.0

# ── Log-odds parameters ───────────────────────────────────────────────────────
L_OCC:          float =  0.85   # log-odds added on occupied hit
L_FREE:         float =  0.40   # log-odds removed on free passage
L_MAX:          float =  5.0
L_MIN:          float = -5.0
L_THRESH_FREE:  float = -0.10   # below → free  (OccupancyGrid value 0)
L_THRESH_OCC:   float =  0.10   # above → occupied (value 100); else unknown (-1)

# ── Sensor geometry ───────────────────────────────────────────────────────────
LIDAR_X_OFFSET: float = 0.16   # lidar forward offset from base_link centre (m)


class LidarMapper(Node):
    """Incremental 2-D lidar mapper using log-odds ray casting."""

    def __init__(self) -> None:
        super().__init__('lidar_mapper')

        self._log_odds: np.ndarray = np.zeros(
            (GRID_SIZE, GRID_SIZE), dtype=np.float32
        )
        self._robot_x:   float = 0.0
        self._robot_y:   float = 0.0
        self._robot_yaw: float = 0.0
        self._has_odom:  bool  = False

        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10,
        )

        self._scan_sub = self.create_subscription(
            LaserScan, '/lidar', self._scan_cb, best_effort_qos
        )
        self._odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_cb, 20
        )
        self._map_pub = self.create_publisher(OccupancyGrid, '/map', 10)

        self.create_timer(1.0 / MAP_PUB_HZ, self._publish_map)

        self.get_logger().info(
            f'LidarMapper ready — {GRID_SIZE}×{GRID_SIZE} cells, '
            f'{2 * MAP_HALF_SIDE:.0f}×{2 * MAP_HALF_SIDE:.0f} m, '
            f'res={MAP_RESOLUTION} m/cell'
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._robot_x   = p.x
        self._robot_y   = p.y
        self._robot_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self._has_odom = True

    def _scan_cb(self, msg: LaserScan) -> None:
        if not self._has_odom:
            return

        yaw = self._robot_yaw
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)

        # Lidar origin in world frame (robot centre + forward offset)
        lx = self._robot_x + LIDAR_X_OFFSET * cos_y
        ly = self._robot_y + LIDAR_X_OFFSET * sin_y

        src = self._world_to_grid(lx, ly)
        if src is None:
            return

        angle = msg.angle_min
        for r_range in msg.ranges:
            beam_world = yaw + angle
            angle += msg.angle_increment

            if r_range < msg.range_min or math.isinf(r_range) or math.isnan(r_range):
                continue

            end_x = lx + r_range * math.cos(beam_world)
            end_y = ly + r_range * math.sin(beam_world)

            # Destination clamped to grid boundary
            dst = self._world_to_grid_clamped(end_x, end_y)
            cells = self._bresenham(src[0], src[1], dst[0], dst[1])

            if not cells:
                continue

            hit = r_range < (msg.range_max - 0.1)

            # Free cells along the ray (all except endpoint when there is a hit)
            free_end = len(cells) - 1 if hit else len(cells)
            for row, col in cells[:free_end]:
                self._log_odds[row, col] = max(
                    L_MIN, self._log_odds[row, col] - L_FREE
                )

            # Occupied cell at the ray endpoint
            if hit:
                row, col = cells[-1]
                self._log_odds[row, col] = min(
                    L_MAX, self._log_odds[row, col] + L_OCC
                )

    # ── Grid helpers ──────────────────────────────────────────────────────────

    def _world_to_grid(self, x: float, y: float):
        col = int((x - MAP_ORIGIN) / MAP_RESOLUTION)
        row = int((y - MAP_ORIGIN) / MAP_RESOLUTION)
        if 0 <= row < GRID_SIZE and 0 <= col < GRID_SIZE:
            return (row, col)
        return None

    def _world_to_grid_clamped(self, x: float, y: float):
        col = int((x - MAP_ORIGIN) / MAP_RESOLUTION)
        row = int((y - MAP_ORIGIN) / MAP_RESOLUTION)
        return (
            max(0, min(GRID_SIZE - 1, row)),
            max(0, min(GRID_SIZE - 1, col)),
        )

    @staticmethod
    def _bresenham(r0: int, c0: int, r1: int, c1: int):
        """Bresenham integer line from (r0,c0) to (r1,c1) in (row, col) space.

        Treats col as the X-axis and row as the Y-axis, matching the standard
        Bresenham formulation with x↔col, y↔row.
        """
        dx = abs(c1 - c0)
        dy = abs(r1 - r0)
        sx = 1 if c1 > c0 else -1
        sy = 1 if r1 > r0 else -1
        err = dx - dy

        cells = []
        r, c = r0, c0
        n = GRID_SIZE

        for _ in range(dx + dy + 2):
            if 0 <= r < n and 0 <= c < n:
                cells.append((r, c))
            if r == r1 and c == c1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                c += sx
            if e2 < dx:
                err += dx
                r += sy

        return cells

    # ── Publisher ─────────────────────────────────────────────────────────────

    def _publish_map(self) -> None:
        occ = np.full((GRID_SIZE, GRID_SIZE), -1, dtype=np.int8)
        occ[self._log_odds < L_THRESH_FREE] = 0
        occ[self._log_odds > L_THRESH_OCC]  = 100

        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info.resolution = float(MAP_RESOLUTION)
        msg.info.width  = GRID_SIZE
        msg.info.height = GRID_SIZE
        msg.info.origin.position.x = float(MAP_ORIGIN)
        msg.info.origin.position.y = float(MAP_ORIGIN)
        msg.info.origin.orientation.w = 1.0
        msg.data = occ.flatten().tolist()
        self._map_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidarMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
