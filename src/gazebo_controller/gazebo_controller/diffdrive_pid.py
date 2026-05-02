#!/usr/bin/env python3
import numpy as np
from time import sleep

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
import tf2_ros

def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    # Clamp sinp to avoid domain errors in arcsin
    sinp = np.clip(sinp, -1.0, 1.0)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def normalize_angle(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))


class DiffDrivePID(Node):
    """
    PID controller for differential drive robot using trailer hitch approach.
    Based on EN613 midterm solution.
    """
    def __init__(self):
        super().__init__('diffdrive_pid')

        # PID gains
        self.declare_parameter('kp', 0.8)
        self.declare_parameter('kd', 0.2)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('lookahead', 0.5)  # trailer hitch distance
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('max_linear_vel', 1.5)
        self.declare_parameter('max_angular_vel', 1.5)
        self.declare_parameter('angular_scale', 0.7)  # Scale down angular commands
        self.declare_parameter('goal_tolerance', 0.25)
        self.declare_parameter('heading_rotate_threshold', 1.2)
        self.declare_parameter('heading_slowdown_threshold', 0.45)
        self.declare_parameter('min_turn_speed_scale', 0.35)
        self.declare_parameter('base_frame', 'vehicle_blue/base_link')

        self.k_p = float(self.get_parameter('kp').get_parameter_value().double_value)
        self.k_d = float(self.get_parameter('kd').get_parameter_value().double_value)
        self.k_i = float(self.get_parameter('ki').get_parameter_value().double_value)
        self.length = float(self.get_parameter('lookahead').get_parameter_value().double_value)
        self.publish_rate = float(self.get_parameter('publish_rate').get_parameter_value().double_value)
        self.max_linear_vel = float(self.get_parameter('max_linear_vel').get_parameter_value().double_value)
        self.max_angular_vel = float(self.get_parameter('max_angular_vel').get_parameter_value().double_value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').get_parameter_value().double_value)
        self.heading_rotate_threshold = float(self.get_parameter('heading_rotate_threshold').get_parameter_value().double_value)
        self.heading_slowdown_threshold = float(self.get_parameter('heading_slowdown_threshold').get_parameter_value().double_value)
        self.min_turn_speed_scale = float(self.get_parameter('min_turn_speed_scale').get_parameter_value().double_value)
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.dt = 1.0 / self.publish_rate

        # TF
        self._tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # I/O
        qos_profile = QoSProfile(depth=10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_received, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_received, 20)
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', qos_profile)
        self.trail_pub = self.create_publisher(Path, '/robot_trail', 10)

        # State
        self.desired_goal = np.array([0.0, 0.0])
        self.robot_state = np.array([0.0, 0.0, 0.0])
        self.Xp_last = None
        self.has_goal = False
        self.has_odom = False

        self.trail_path = Path()
        self.trail_path.header.frame_id = 'odom'
        self.last_trail_xy = None

        # TF frames
        self._to_frame = 'odom'
        self._from_frame_candidates = [self.base_frame, 'base_link', 'vehicle_blue/base_link']

        # Timer loop
        self.timer = self.create_timer(self.dt, self.publish_robot_cmd)

        self.get_logger().info(f'DiffDrivePID started: kp={self.k_p}, kd={self.k_d}, ki={self.k_i}, lookahead={self.length}, rate={self.publish_rate} Hz')

    def odom_received(self, msg):
        pose = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion(orientation)
        self.robot_state = np.array([pose.x, pose.y, yaw])
        self.has_odom = True

        # Keep a lightweight trail for RViz path visualization.
        cur_xy = (pose.x, pose.y)
        if self.last_trail_xy is None or np.linalg.norm(np.array(cur_xy) - np.array(self.last_trail_xy)) > 0.05:
            p = PoseStamped()
            p.header.stamp = self.get_clock().now().to_msg()
            p.header.frame_id = 'odom'
            p.pose.position.x = float(pose.x)
            p.pose.position.y = float(pose.y)
            p.pose.position.z = 0.0
            p.pose.orientation.w = 1.0
            self.trail_path.header.stamp = p.header.stamp
            self.trail_path.poses.append(p)
            if len(self.trail_path.poses) > 2000:
                self.trail_path.poses = self.trail_path.poses[-2000:]
            self.trail_pub.publish(self.trail_path)
            self.last_trail_xy = cur_xy

    def goal_received(self, msg):
        """Callback for goal pose messages"""
        # Check if goal is in the correct frame (odom)
        goal_frame = msg.header.frame_id
        
        if goal_frame != 'odom' and goal_frame != '':
            self.get_logger().warn(f'Goal received in frame "{goal_frame}" but expecting "odom". Attempting to transform...')
            try:
                # Try to transform the goal to odom frame
                when = rclpy.time.Time()
                trans = self._tf_buffer.lookup_transform(
                    'odom', goal_frame,
                    when, timeout=Duration(seconds=1.0))
                
                # Get the goal position in the source frame
                goal_x = msg.pose.position.x
                goal_y = msg.pose.position.y
                
                # Get translation
                tx = trans.transform.translation.x
                ty = trans.transform.translation.y
                
                # Get rotation (convert quaternion to yaw)
                quat = trans.transform.rotation
                _, _, yaw = euler_from_quaternion(quat)
                
                # Apply full transformation: rotate then translate
                cos_yaw = np.cos(yaw)
                sin_yaw = np.sin(yaw)
                
                goal_x_odom = cos_yaw * goal_x - sin_yaw * goal_y + tx
                goal_y_odom = sin_yaw * goal_x + cos_yaw * goal_y + ty
                
                self.desired_goal = np.array([goal_x_odom, goal_y_odom])
                self.get_logger().info(f'Goal transformed to odom frame: [{self.desired_goal[0]:.3f}, {self.desired_goal[1]:.3f}]')
            except Exception as e:
                self.get_logger().error(f'Failed to transform goal: {str(e)}')
                return
        else:
            # Goal is already in odom frame (or no frame specified, assume odom)
            self.desired_goal = np.array([msg.pose.position.x, msg.pose.position.y])
            self.get_logger().info(f'Goal set to: [{self.desired_goal[0]:.3f}, {self.desired_goal[1]:.3f}] in odom frame')
        
        # Reset the integral term when a new goal is set
        self.Xp_last = None
        self.has_goal = True

    def compute_vel(self):
        """Compute velocity commands using PID control on trailer hitch point"""
        if not self.has_goal:
            return np.array([0.0, 0.0])

        x_r = self.robot_state[0]
        y_r = self.robot_state[1]
        th_r = self.robot_state[2]

        if np.linalg.norm(self.desired_goal - np.array([x_r, y_r])) < self.goal_tolerance:
            self.has_goal = False
            self.Xp_last = None
            return np.array([0.0, 0.0])

        # Compute a trailer hitch point in front of the agent
        X_p = np.array([x_r + self.length * np.cos(th_r),
                        y_r + self.length * np.sin(th_r)])

        # Compute the proportional error between the desired position and the final position
        # The [:,None] prefix reshapes the vector from (2,) to (2,1)
        p_err = (self.desired_goal - X_p)[:, None]

        if self.Xp_last is None:
            # If this is the first timestep we do not compute the derivative or integral terms
            d_err = 0
            i_err = 0
            p_err_last = None
        else:
            # Compute previous error
            p_err_last = (self.desired_goal - self.Xp_last)[:, None]
            
            # Derivative is the rate of change of error
            d_err = (p_err - p_err_last) / self.dt

            # We use the Trapezoidal rule to integrate the error
            i_err = self.dt * (p_err + p_err_last) / 2

        inv_rot = np.array([[np.cos(th_r), np.sin(th_r)],
                           [-np.sin(th_r), np.cos(th_r)]])
        
        V = inv_rot @ (self.k_p * p_err - self.k_d * d_err + self.k_i * i_err)
        linear_x = V[0, 0]
        angular_z = V[1, 0] / self.length

        # Clamp velocities
        linear_x = np.clip(linear_x, -self.max_linear_vel, self.max_linear_vel)
        angular_z = np.clip(angular_z, -self.max_angular_vel, self.max_angular_vel)

        # In narrow mazes, turning first avoids clipping walls with aggressive forward motion.
        goal_heading = np.arctan2(self.desired_goal[1] - y_r, self.desired_goal[0] - x_r)
        heading_error = normalize_angle(goal_heading - th_r)
        abs_heading_error = np.abs(heading_error)
        if abs_heading_error > self.heading_rotate_threshold:
            linear_x = 0.0
        elif abs_heading_error > self.heading_slowdown_threshold:
            linear_x *= max(self.min_turn_speed_scale, 1.0 - abs_heading_error)

        self.Xp_last = X_p

        return np.array([linear_x, angular_z])

    def publish_robot_cmd(self):
        """Main control loop callback"""
        if not self.has_odom:
            # Fallback to TF only until odom is available.
            trans = None
            for from_frame in self._from_frame_candidates:
                try:
                    when = rclpy.time.Time()
                    trans = self._tf_buffer.lookup_transform(
                        self._to_frame, from_frame,
                        when, timeout=Duration(seconds=0.2))
                    break
                except Exception:
                    continue

            if trans is None:
                self.get_logger().warn('No /odom or TF base transform yet; waiting before publishing motion commands')
                return

            pose = trans.transform.translation
            orientation = trans.transform.rotation
            _, _, yaw = euler_from_quaternion(orientation)
            self.robot_state = np.array([pose.x, pose.y, yaw])

        desired_vel = self.compute_vel()

        msg = Twist()
        msg.linear.x = float(desired_vel[0])
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(desired_vel[1])

        self.vel_pub.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = DiffDrivePID()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
