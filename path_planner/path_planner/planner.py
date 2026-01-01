import rclpy
from rclpy.node import Node


class point_logger(Node):

    def __init__(self):
        super().__init__("waypoints_logger")


def main(args):
    rclpy.init(args=args)
    point_loader = point_logger()
    rclpy.spin(point_loader)
    rclpy.shutdown()


if __name__=="__main__":
    main()