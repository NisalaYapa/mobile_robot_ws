import rclpy
import sys
from rclpy.node import Node
from std_msgs.msg import String
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5 import QtCore, QtGui, QtWidgets
from .GUIs.Ui_robot_test2 import Ui_MainWindow


class ROS2IntegrationNode(Node):
    def __init__(self, main_win):
        super().__init__('robot_gui')

        # Initialize publisher
        self.publisher = self.create_publisher(String, 'robot_control', 10)

        self.main_win = main_win
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self.main_win)

        # Connect buttons to functions
        self.ui.btn_crowdnav.clicked.connect(self.handle_crowdnav)
        self.ui.btn_multinav.clicked.connect(self.handle_muiltinav)
        self.ui.btn_arm.clicked.connect(self.handle_arm)

    def handle_crowdnav(self):
        self.get_logger().info('Crowd Navigation activated')
        msg = String()
        msg.data = 'crowd_navigation'
        self.publisher.publish(msg)

    def handle_muiltinav(self):
        self.get_logger().info('Multifloor Navigation activated')
        msg = String()
        msg.data = 'multifloor_navigation'
        self.publisher.publish(msg)

    def handle_arm(self):
        self.get_logger().info('Arm Manipulator activated')
        msg = String()
        msg.data = 'arm_manipulator'
        self.publisher.publish(msg)


def main():
    rclpy.init()

    app = QApplication(sys.argv)  # QApplication must be initialized before any widgets
    main_win = QMainWindow()
    ros_node = ROS2IntegrationNode(main_win)

    main_win.show()  # Show the main window

    try:
        sys.exit(app.exec_())  # Start the Qt event loop
    except SystemExit:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
