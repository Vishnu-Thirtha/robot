#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import threading
import sys
import termios
import tty

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from std_msgs.msg import Bool
from tf_transformations import euler_from_quaternion


class PathController(Node):

    def __init__(self):
        super().__init__('path_controller')

        # ----------------------------------
        # Mode selection
        # ----------------------------------
        self.declare_parameter('standalone', True)
        self.standalone = self.get_parameter('standalone').value
        self.control_source = 'KEYBOARD' if self.standalone else 'GUI'

        self.started = False  # motion enable flag

        mode = 'STANDALONE (keyboard)' if self.standalone else 'GUI-controlled (PyQt)'
        self.get_logger().info('=' * 50)
        self.get_logger().info(f'Path Controller started in {mode} mode')
        self.get_logger().info('=' * 50)

        # ----------------------------------
        # Subscribers
        # ----------------------------------
        self.path_sub = self.create_subscription(
            Path, '/planned_path', self.path_callback, 10)

        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        if not self.standalone:
            self.param_sub = self.create_subscription(
                Twist, '/controller_params', self.param_callback, 10)

            self.enable_sub = self.create_subscription(
                Bool, '/controller_enable', self.enable_callback, 10)

        # ----------------------------------
        # Publishers
        # ----------------------------------
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.lookahead_marker_pub = self.create_publisher(
            Marker, '/lookahead_point_marker', 10)

        # ----------------------------------
        # Timer
        # ----------------------------------
        self.timer = self.create_timer(0.02, self.control_loop)

        # ----------------------------------
        # State
        # ----------------------------------
        self.path_xy = None
        self.path_s = None
        self.robot_pose = None
        self.robot_yaw = None

        # ----------------------------------
        # Controller parameters
        # ----------------------------------
        self.lookahead_dist = 0.4
        self.v = 0.3

        # ----------------------------------
        # Keyboard control
        # ----------------------------------
        if self.standalone and sys.stdin.isatty():
            self.keyboard_thread = threading.Thread(
                target=self.keyboard_listener, daemon=True)
            self.keyboard_thread.start()
        else:
            if self.standalone:
                self.get_logger().warn(
                    'Standalone mode requested but no TTY available. Keyboard disabled.')

        self.get_logger().info('Path Controller ready')

    # =====================================================
    # Keyboard input
    # =====================================================
    def keyboard_listener(self):
        settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        try:
            while True:
                key = sys.stdin.read(1)
                if key == 'w':
                    self.started = True
                    self.get_logger().info('[KEYBOARD] START command received')
                elif key == 's':
                    self.started = False
                    self.stop_robot()
                    self.get_logger().info('[KEYBOARD] STOP command received')
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    # =====================================================
    # Callbacks
    # =====================================================
    def path_callback(self, msg):
        self.path_xy = np.array([
            [p.pose.position.x, p.pose.position.y]
            for p in msg.poses
        ])
        self.path_s = self.compute_arc_length(self.path_xy)
        self.get_logger().info('New path received')

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        q = msg.pose.pose.orientation

        self.robot_pose = np.array([pos.x, pos.y])
        self.robot_yaw = euler_from_quaternion(
            [q.x, q.y, q.z, q.w])[2]

    def param_callback(self, msg):
        self.v = msg.linear.x
        self.lookahead_dist = msg.angular.z

        self.get_logger().info(
            f'[GUI] Params updated → v={self.v:.2f} m/s, '
            f'lookahead={self.lookahead_dist:.2f} m'
        )

    def enable_callback(self, msg):
        self.started = msg.data

        if self.started:
            self.get_logger().info('[GUI] START command received')
        else:
            self.get_logger().info('[GUI] STOP command received')
            self.stop_robot()

    # =====================================================
    # Control Loop
    # =====================================================
    def control_loop(self):
        if not self.started:
            return

        if self.path_xy is None or self.robot_pose is None:
            return

        target = self.get_lookahead_point()
        if target is None:
            self.stop_robot()
            return

        v, omega = self.pure_pursuit_control(target)
        self.publish_lookahead_marker(target)
        self.publish_cmd(v, omega)

    # =====================================================
    # Pure Pursuit
    # =====================================================
    def get_lookahead_point(self):
        distances = np.linalg.norm(self.path_xy - self.robot_pose, axis=1)
        closest_idx = np.argmin(distances)

        s_current = self.path_s[closest_idx]
        s_target = s_current + self.lookahead_dist

        if s_target >= self.path_s[-1]:
            return None

        target_idx = np.searchsorted(self.path_s, s_target)
        return self.path_xy[target_idx]

    def pure_pursuit_control(self, target):
        dx = target[0] - self.robot_pose[0]
        dy = target[1] - self.robot_pose[1]

        x_r = np.cos(-self.robot_yaw) * dx - np.sin(-self.robot_yaw) * dy
        y_r = np.sin(-self.robot_yaw) * dx + np.cos(-self.robot_yaw) * dy

        curvature = 2.0 * y_r / (self.lookahead_dist ** 2)
        omega = self.v * curvature

        return self.v, omega

    # =====================================================
    # Utilities
    # =====================================================
    def compute_arc_length(self, path):
        s = [0.0]
        for i in range(1, len(path)):
            s.append(s[-1] + np.linalg.norm(path[i] - path[i - 1]))
        return np.array(s)

    def publish_cmd(self, v, omega):
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(omega)
        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def publish_lookahead_marker(self, point):
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'lookahead'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(point[0])
        marker.pose.position.y = float(point[1])
        marker.pose.position.z = 0.15
        marker.scale.x = marker.scale.y = marker.scale.z = 0.1
        marker.color.b = 1.0
        marker.color.a = 1.0

        self.lookahead_marker_pub.publish(marker)


def main():
    rclpy.init()
    node = PathController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
