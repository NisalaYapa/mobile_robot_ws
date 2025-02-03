import sys
import rclpy
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton
from rclpy.node import Node
from std_msgs.msg import String

class Ros2QtNode(Node):
    def __init__(self):
        super().__init__('qt_ros_node')
        self.publisher = self.create_publisher(String, 'gui_topic', 10)

    def publish_message(self):
        msg = String()
        msg.data = "Hello from Qt GUI"
        self.publisher.publish(msg)
        self.get_logger().info(f"Published: {msg.data}")

class MainWindow(QMainWindow):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.setWindowTitle("ROS 2 Qt GUI")
        self.setGeometry(100, 100, 400, 200)

        self.button = QPushButton("Publish Message", self)
        self.button.setGeometry(100, 80, 200, 40)
        self.button.clicked.connect(self.ros_node.publish_message)

def main():
    rclpy.init()
    ros_node = Ros2QtNode()
    
    app = QApplication(sys.argv)
    main_window = MainWindow(ros_node)
    main_window.show()

    try:
        sys.exit(app.exec_())
    except SystemExit:
        rclpy.shutdown()

if __name__ == '__main__':
    main()

