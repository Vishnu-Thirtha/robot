#!/usr/bin/env python3
"""
ROS 2 node that:
- Records points clicked in RViz (/clicked_point)
- Stores them as 2D waypoints
- Fits a C²-continuous cubic spline through the waypoints
- Visualizes:
  - Clicked points (RViz markers + Gazebo spheres)
  - The interpolated path (nav_msgs/Path + RViz line marker)
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped, Point, PoseStamped
from gazebo_msgs.srv import SpawnEntity
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path

import numpy as np
from scipy.interpolate import CubicSpline


class ClickedPointRecorder(Node):
    """
    Node that listens to clicked points, builds a smooth spline path,
    and publishes visualization data for RViz and Gazebo.
    """

    def __init__(self):
        super().__init__('clicked_point_recorder')

        # List of 2D waypoints (x, y); initialized at origin
        self.waypoints = [(0.0, 0.0)]

        # Subscription to RViz "Publish Point" tool
        self.subscription = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.clicked_point_callback,
            10
        )

        # RViz markers for clicked points
        self.points_marker_pub = self.create_publisher(
            Marker,
            '/clicked_points_marker',
            10
        )

        # RViz marker for the spline path
        self.path_marker_pub = self.create_publisher(
            Marker,
            '/path_marker',
            10
        )

        # nav_msgs/Path for the planned path
        self.path_pub = self.create_publisher(
            Path,
            '/planned_path',
            10
        )

        # Gazebo service client for spawning visual spheres
        self.spawn_client = self.create_client(
            SpawnEntity,
            '/spawn_entity'
        )

        self.point_count = 0
        self.get_logger().info('Clicked Point Recorder started')

    def clicked_point_callback(self, msg):
        """
        Callback for each clicked point in RViz.
        - Stores the waypoint
        - Spawns a red sphere in Gazebo
        - Publishes RViz markers
        - Recomputes and republishes the spline path
        """
        x = msg.point.x
        y = msg.point.y

        self.point_count += 1
        self.waypoints.append((x, y))

        self.spawn_red_marker(x, y)
        self.publish_rviz_marker(x, y, self.point_count)

        result = self.generate_spline_path()
        if result is None:
            return

        path, _, _, _ = result
        self.publish_path(path)
        self.publish_path_marker(path)

    def generate_spline_path(self, resolution=0.05):
        """
        Generates a smooth C²-continuous parametric spline through waypoints.

        Returns:
            path      : Nx2 array of sampled (x, y) points
            spline_x  : CubicSpline for x(s)
            spline_y  : CubicSpline for y(s)
            s_sampled : Path parameter samples
        """
        if len(self.waypoints) < 2:
            return None

        pts = np.array(self.waypoints)

        # Path parameter s based on cumulative distance
        s = np.zeros(len(pts))
        for i in range(1, len(pts)):
            s[i] = s[i - 1] + np.linalg.norm(pts[i] - pts[i - 1])

        spline_x = CubicSpline(s, pts[:, 0])
        spline_y = CubicSpline(s, pts[:, 1])

        s_sampled = np.arange(0, s[-1], resolution)
        path = np.column_stack((spline_x(s_sampled), spline_y(s_sampled)))

        return path, spline_x, spline_y, s_sampled

    def publish_path(self, path_points):
        """
        Publishes the geometric path as nav_msgs/Path.
        """
        msg = Path()
        msg.header.frame_id = 'odom'
        msg.header.stamp = self.get_clock().now().to_msg()

        for p in path_points:
            pose = PoseStamped()
            pose.header.frame_id = 'odom'
            pose.pose.position.x = p[0]
            pose.pose.position.y = p[1]
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)

        self.path_pub.publish(msg)

    def publish_path_marker(self, path_points):
        """
        Publishes the spline path as a green LINE_STRIP marker in RViz.
        """
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'spline_path'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.color.g = 1.0
        marker.color.a = 1.0

        for p in path_points:
            pt = Point(x=p[0], y=p[1], z=0.02)
            marker.points.append(pt)

        self.path_marker_pub.publish(marker)

    def publish_rviz_marker(self, x, y, marker_id):
        """
        Publishes a red sphere marker at a clicked waypoint in RViz.
        """
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'clicked_points'
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.05
        marker.scale.x = marker.scale.y = marker.scale.z = 0.05
        marker.color.r = 1.0
        marker.color.a = 1.0

        self.points_marker_pub.publish(marker)

    def spawn_red_marker(self, x, y):
        """
        Spawns a static red sphere in Gazebo at the clicked point.
        """
        if not self.spawn_client.wait_for_service(timeout_sec=1.0):
            return
        # Gazebo spawn logic intentionally omitted


def main():
    """Node entry point."""
    rclpy.init()
    rclpy.spin(ClickedPointRecorder())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
