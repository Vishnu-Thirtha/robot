#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
from tf_transformations import euler_from_quaternion


class PathController(Node):

    def __init__(self):
        super().__init__('path_controller')

        # Subscribers
        self.path_sub = self.create_subscription(
            Path,
            '/planned_path',
            self.path_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Publisher
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Control loop (50 Hz)
        self.timer = self.create_timer(0.02, self.control_loop)

        # Internal state
        self.path_xy = None
        self.path_s = None
        self.path_t = None
        self.start_time = None

        self.robot_pose = None
        self.robot_yaw = None

        # Controller parameters
        self.lookahead_dist = 0.2
        self.v_max = 0.1
        self.a_max = 0.1

        self.get_logger().info('Path Controller started')

    # -----------------------------
    # PATH CALLBACK
    # -----------------------------
    def path_callback(self, msg):
        self.path_xy = np.array([
            (p.pose.position.x, p.pose.position.y)
            for p in msg.poses
        ])

        self.path_s = self.compute_arc_length(self.path_xy)
        self.path_t = self.trapezoidal_time_parameterization(
            self.path_s,
            self.v_max,
            self.a_max
        )

        self.start_time = self.get_clock().now()
        self.get_logger().info('New path received and time-parameterized')

    # -----------------------------
    # ODOM CALLBACK
    # -----------------------------
    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

        self.robot_pose = np.array([x, y])
        self.robot_yaw = yaw

    # -----------------------------
    # CONTROL LOOP
    # -----------------------------
    def control_loop(self):
        if self.path_xy is None or self.robot_pose is None:
            return

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9

        if elapsed >= self.path_t[-1]:
            self.stop_robot()
            return

        target = self.get_lookahead_point()
        if target is None:
            self.stop_robot()
            return

        v, omega = self.pure_pursuit_control(target)
        self.publish_cmd(v, omega)

    # -----------------------------
    # PURE PURSUIT
    # -----------------------------
    def get_lookahead_point(self):
        distances = np.linalg.norm(self.path_xy - self.robot_pose, axis=1)
        candidates = np.where(distances >= self.lookahead_dist)[0]

        if len(candidates) == 0:
            return None

        return self.path_xy[candidates[0]]

    def pure_pursuit_control(self, target):
        dx = target[0] - self.robot_pose[0]
        dy = target[1] - self.robot_pose[1]

        # Transform to robot frame
        x_r = np.cos(-self.robot_yaw) * dx - np.sin(-self.robot_yaw) * dy
        y_r = np.sin(-self.robot_yaw) * dx + np.cos(-self.robot_yaw) * dy

        curvature = 2.0 * y_r / (self.lookahead_dist ** 2)

        v = self.v_max
        omega = v * curvature

        return v, omega

    # -----------------------------
    # TIME PARAMETERIZATION
    # -----------------------------
    def compute_arc_length(self, path):
        s = [0.0]
        for i in range(1, len(path)):
            s.append(s[-1] + np.linalg.norm(path[i] - path[i - 1]))
        return np.array(s)

    def trapezoidal_time_parameterization(self, s, v_max, a_max):
        total_length = s[-1]

        t_acc = v_max / a_max
        d_acc = 0.5 * a_max * t_acc ** 2

        if 2 * d_acc > total_length:
            t_acc = np.sqrt(total_length / a_max)
            t_total = 2 * t_acc
        else:
            d_cruise = total_length - 2 * d_acc
            t_cruise = d_cruise / v_max
            t_total = 2 * t_acc + t_cruise

        t = np.linspace(0.0, t_total, len(s))
        return t

    # -----------------------------
    # COMMAND PUBLISHING
    # -----------------------------
    def publish_cmd(self, v, omega):
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(omega)
        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = PathController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
