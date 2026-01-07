#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from threading import Thread

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from python_qt_binding.QtWidgets import (
    QApplication, QWidget, QVBoxLayout,
    QLabel, QSlider, QPushButton
)
from python_qt_binding.QtCore import Qt


# =====================================================
# ROS Node (Silent)
# =====================================================
class ControllerGui(Node):

    def __init__(self):
        super().__init__('controller_gui')

        self.param_pub = self.create_publisher(
            Twist, '/controller_params', 10)

        self.enable_pub = self.create_publisher(
            Bool, '/controller_enable', 10)


# =====================================================
# Qt Widget
# =====================================================
class ControllerGuiWidget(QWidget):

    def __init__(self, node: ControllerGui):
        super().__init__()
        self.node = node

        self.setWindowTitle('Path Controller GUI')

        layout = QVBoxLayout()

        # --------------------
        # Velocity
        # --------------------
        self.vel_label = QLabel('Velocity: 0.10 m/s')
        self.vel_slider = QSlider(Qt.Horizontal)
        self.vel_slider.setRange(1, 50)
        self.vel_slider.setValue(10)
        self.vel_slider.valueChanged.connect(self.update_params)

        # --------------------
        # Acceleration (reserved)
        # --------------------
        self.acc_label = QLabel('Acceleration: 0.10 m/s²')
        self.acc_slider = QSlider(Qt.Horizontal)
        self.acc_slider.setRange(1, 50)
        self.acc_slider.setValue(10)
        self.acc_slider.valueChanged.connect(self.update_params)

        # --------------------
        # Lookahead
        # --------------------
        self.ld_label = QLabel('Lookahead: 0.40 m')
        self.ld_slider = QSlider(Qt.Horizontal)
        self.ld_slider.setRange(10, 100)
        self.ld_slider.setValue(40)
        self.ld_slider.valueChanged.connect(self.update_params)

        # --------------------
        # Buttons
        # --------------------
        self.start_btn = QPushButton('START')
        self.stop_btn = QPushButton('STOP')

        self.start_btn.clicked.connect(self.start_controller)
        self.stop_btn.clicked.connect(self.stop_controller)

        self.stop_btn.setEnabled(False)

        # --------------------
        # Layout
        # --------------------
        layout.addWidget(self.vel_label)
        layout.addWidget(self.vel_slider)

        layout.addWidget(self.acc_label)
        layout.addWidget(self.acc_slider)

        layout.addWidget(self.ld_label)
        layout.addWidget(self.ld_slider)

        layout.addWidget(self.start_btn)
        layout.addWidget(self.stop_btn)

        self.setLayout(layout)

        # Publish initial parameters once
        self.update_params()

    # =====================================================
    # Callbacks (Silent)
    # =====================================================
    def update_params(self):
        v = self.vel_slider.value() / 100.0
        a = self.acc_slider.value() / 100.0
        ld = self.ld_slider.value() / 100.0

        self.vel_label.setText(f'Velocity: {v:.2f} m/s')
        self.acc_label.setText(f'Acceleration: {a:.2f} m/s²')
        self.ld_label.setText(f'Lookahead: {ld:.2f} m')

        msg = Twist()
        msg.linear.x = v
        msg.linear.y = a
        msg.angular.z = ld

        self.node.param_pub.publish(msg)

    def start_controller(self):
        msg = Bool()
        msg.data = True
        self.node.enable_pub.publish(msg)

        self.start_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)

    def stop_controller(self):
        msg = Bool()
        msg.data = False
        self.node.enable_pub.publish(msg)

        self.start_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)


# =====================================================
# Main
# =====================================================
def main():
    rclpy.init()

    node = ControllerGui()

    ros_thread = Thread(
        target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    app = QApplication(sys.argv)
    widget = ControllerGuiWidget(node)
    widget.show()

    app.exec_()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
