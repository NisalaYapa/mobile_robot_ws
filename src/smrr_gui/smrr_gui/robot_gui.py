import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Example message type
from PyQt5 import QtCore, QtGui, QtWidgets
from .GUIs.Ui_robot import Ui_MainWindow



class ROS2IntegrationNode(Node):
    def __init__(self):
        super().__init__('robot_gui')

        # Initialize publisher
        self.publisher = self.create_publisher(String, 'robot_control', 10)

        # Create a Qt application
        self.app = QtWidgets.QApplication([])

        # Set up the GUI
        self.main_window = QtWidgets.QMainWindow()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self.main_window)

        # Connect GUI buttons to ROS 2 callback functions
        self.ui.btn_crowdnav.clicked.connect(self.handle_crowdnav)
        self.ui.btn_muiltinav.clicked.connect(self.handle_muiltinav)
        self.ui.btn_arm.clicked.connect(self.handle_arm)

        self.main_window.show()

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

    def spin(self):
        # Spin the ROS 2 node and the GUI
        while rclpy.ok():
            rclpy.spin_once(self)
            self.app.processEvents()

def main():
    rclpy.init()

    # Create the ROS 2 node and run the GUI
    node = ROS2IntegrationNode()
    node.spin()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
