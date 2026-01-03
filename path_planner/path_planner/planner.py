#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from gazebo_msgs.srv import SpawnEntity
from visualization_msgs.msg import Marker


class ClickedPointRecorder(Node):

    def __init__(self):
        super().__init__('clicked_point_recorder')

        self.subscription = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.clicked_point_callback,
            10
        )

        self.marker_pub = self.create_publisher(
            Marker,
            '/clicked_points_marker',
            10
        )

        self.spawn_client = self.create_client(
            SpawnEntity,
            '/spawn_entity'
        )

        self.point_count = 0

        self.get_logger().info('Clicked Point Recorder started')

    def clicked_point_callback(self, msg):
        self.point_count += 1

        x = msg.point.x
        y = msg.point.y
        z = 0

        self.get_logger().info(
            f'Clicked point {self.point_count}: ({x:.2f}, {y:.2f}, {z:.2f})'
        )

        self.spawn_red_marker(x, y, z)
        self.publish_rviz_marker(x, y, z, self.point_count)

    def spawn_red_marker(self, x, y, z):
        if not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Spawn service not available')
            return

        sdf = f"""
        <sdf version="1.6">
          <model name="clicked_point_{self.point_count}">
            <static>true</static>
            <link name="link">
              <visual name="visual">
                <geometry>
                  <sphere>
                    <radius>0.05</radius>
                  </sphere>
                </geometry>
                <material>
                  <ambient>1 0 0 1</ambient>
                  <diffuse>1 0 0 1</diffuse>
                </material>
              </visual>
            </link>
          </model>
        </sdf>
        """

        request = SpawnEntity.Request()
        request.name = f'clicked_point_{self.point_count}'
        request.xml = sdf
        request.initial_pose.position.x = x
        request.initial_pose.position.y = y
        request.initial_pose.position.z = z + 0.05

        self.spawn_client.call_async(request)

    def publish_rviz_marker(self, x, y, z, marker_id):
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = 'clicked_points'
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z + 0.05

        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.marker_pub.publish(marker)


def main():
    rclpy.init()
    node = ClickedPointRecorder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()