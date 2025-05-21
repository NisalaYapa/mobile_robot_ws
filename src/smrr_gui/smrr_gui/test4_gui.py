import rclpy
from geometry_msgs.msg import Twist, PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy
import sys
from rclpy.node import Node
from std_msgs.msg import String
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import QProcess
from .GUIs.Ui_robot_test4 import Ui_MainWindow
from functools import partial
import subprocess
import time
from tf_transformations import quaternion_from_euler


class RobotGUI(Node):
    def __init__(self, main_win):
        super().__init__('robot_gui')

        self.reliable_qos = QoSProfile(
                        reliability=ReliabilityPolicy.RELIABLE,
                        depth=10)

        # Initialize publisher
        self.publisher              = self.create_publisher(String, 'robot_control', 10)
        self.multinav_publisher     = self.create_publisher(Twist, 'multinav_goal',self.reliable_qos)
        self.arm_publisher          = self.create_publisher(String, '/gesture_command',self.reliable_qos)
        self.crowdnav_publisher     = self.create_publisher(PoseStamped, '/goal_pose',self.reliable_qos)

        self.main_win = main_win
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self.main_win)

        # Coordinates for multi-floor navigation

        # x, y, z, theta

        self.coordinates = {"entc1"             : [-3.0, -8.0, 1.0, 0.0],
                            "reception"         : [0.0, -4.0, 1.0, 0.0],
                            "uav"               : [0.0, 0.0, 1.0, 0.0],
                            "com_lab"           : [0.0, -16.0, 2.0, 0.0],
                            "office"            : [0.0, 0.0, 2.0, 0.0],
                            "conference_room"   : [0.0, 0.0, 2.0, 0.0],
                            "digi_lab"          : [0.0, 0.0, 3.0, 0.0],
                            "analog_lab"        : [0.0, 0.0, 3.0, 0.0],
                            "tele_lab"          : [0.0, 0.0, 4.0, 0.0],
                            "pg_room"           : [0.0, -2.0, 4.0, 0.0]}
        
        self.crowdnav_coordinates = {"start"       : [0.0, 0.0, 0.0, 0.0],
                                     "end"         : [2.0, 0.0, 0.0, 0.0]}
        

        # Connect buttons to functions
        self.ui.btn_crowdnav.clicked.connect(self.handle_crowdnav)
        self.ui.btn_multifloor.clicked.connect(self.handle_muiltinav)
        self.ui.btn_arm.clicked.connect(self.handle_arm)
        self.ui.btn_dockpage.clicked.connect(self.handle_dock)
        self.ui.Homebtn.clicked.connect(self.handle_homepage)

        self.ui.mainstack.setCurrentWidget(self.ui.HomePage)
        self.ui.floorStack.setCurrentWidget(self.ui.pg_flr_G)

    def handle_crowdnav(self):
        self.get_logger().info('Crowd Navigation activated')

        # Switch to Crowd Navigation page
        self.ui.mainstack.setCurrentWidget(self.ui.CrowdNav)

        # Update label text
        self.ui.label.setText("Crowd Navigation")

        # Publish message
        msg = String()
        msg.data = 'crowd_navigation'
        self.publisher.publish(msg)

        # Connect button click to crowdnav_goal function
        self.ui.btn_crwnav_run.clicked.connect(self.crowdnav_goal)
        self.ui.btn_crowdnav_start.clicked.connect(partial(self.go_location_crowdnav, "start"))
        self.ui.btn_crowdnav_end.clicked.connect(partial(self.go_location_crowdnav, "end"))


    # def crowdnav_goal(self):
    #     goal = self.ui.crwnav_goal_input.toPlainText()  # Use toPlainText() for QTextEdit

    #     if not goal.strip():  # Ensure input is not empty or just spaces
    #         self.ui.Notifications.setText("No goal is provided")
    #     else:
    #         self.ui.Notifications.setText(f"Going to the {goal}")
    #         # Run the ROS 2 command
    #         subprocess.Popen(["gnome-terminal", "--","ros2", "topic", "list"])
            # subprocess.Popen(["gnome-terminal", "--","tmuxinator", "buffer"])  

    def handle_homepage(self):
        self.get_logger().info('Home Page')

        # Switch to Crowd Navigation page
        self.ui.mainstack.setCurrentWidget(self.ui.HomePage)

        # Update label text
        self.ui.label.setText("SANSAR : Mobile Robot Receptionist")


    def handle_dock(self):
        self.get_logger().info('Docking activated')

        # Switch to Crowd Navigation page
        self.ui.mainstack.setCurrentWidget(self.ui.Docking)

        # Update label text
        self.ui.label.setText("Docking")

        # Publish message
        msg = String()
        msg.data = 'Docking'
        self.publisher.publish(msg)

        # Connect button click to crowdnav_goal function
        self.ui.btn_dock.clicked.connect(self.crowdnav_goal)
        

    def go_location_crowdnav(self, location):
        """Publish a navigation goal to the /goal_pose topic
        
        Args:
            location (str): The key for the target location in self.coordinates
        """
        self.ui.Notifications.setText(f"Going to {location} position")
        
        # Create a PoseStamped message
        goal_pose = PoseStamped()
        
        # Set the header
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.header.frame_id = "map"  # Or your preferred frame
        
        # Set the position coordinates
        goal_pose.pose.position.x = self.crowdnav_coordinates[location][0]
        goal_pose.pose.position.y = self.crowdnav_coordinates[location][1]
        goal_pose.pose.position.z = self.crowdnav_coordinates[location][2]
        
        # Convert yaw angle to quaternion for orientation
        yaw = self.crowdnav_coordinates[location][3]
        q = quaternion_from_euler(0, 0, yaw)  # Roll, pitch, yaw
        goal_pose.pose.orientation.x = q[0]
        goal_pose.pose.orientation.y = q[1]
        goal_pose.pose.orientation.z = q[2]
        goal_pose.pose.orientation.w = q[3]
        
        # Log the goal information
        self.get_logger().info(
            f'Publishing goal to {location}: '
            f'x:{goal_pose.pose.position.x:.2f}, '
            f'y:{goal_pose.pose.position.y:.2f}, '
            f'z:{goal_pose.pose.position.z:.2f}, '
            f'yaw:{yaw:.2f} rad'
        )
        
        # Publish the goal
        self.crowdnav_publisher.publish(goal_pose)
        self.get_logger().info("Navigation goal published to /goal_pose!")

    def crowdnav_goal(self):
        goal = self.ui.crwnav_goal_input.toPlainText()

        if not goal.strip():
            self.ui.Notifications.setText("No goal is provided")
            return

        self.ui.Notifications.setText(f"Going to the {goal}")

        # 1. Ensure tmux session exists (create if missing)
        try:
            subprocess.run(["tmux", "has-session", "-t", "ros2_commands"], check=True)
        except subprocess.CalledProcessError:
            subprocess.run(["tmux", "new-session", "-d", "-s", "ros2_commands"])

        # 2. Check if any terminal is already attached to this session
        try:
            # Get list of clients attached to this tmux session
            clients = subprocess.check_output(
                ["tmux", "list-clients", "-t", "ros2_commands"]
            ).decode().strip()
            
            # If no clients are attached, open a new terminal
            if not clients:
                subprocess.Popen(["gnome-terminal", "--", "tmux", "attach", "-t", "ros2_commands"])
        except subprocess.CalledProcessError:
            # If list-clients fails, assume no terminal is attached
            subprocess.Popen(["gnome-terminal", "--", "tmux", "attach", "-t", "ros2_commands"])

        # 3. Send commands to tmux (with small delay if needed)
        time.sleep(0.5)  # Allow terminal to initialize if just opened
        subprocess.run([
            "tmux", "send-keys", "-t", "ros2_commands",
            f"ros2 topic list && ros2 run some_package some_node --goal {goal}",
            "Enter"
        ])
    
    def handle_muiltinav(self):
        self.get_logger().info('Multifloor Navigation activated')
        self.ui.mainstack.setCurrentWidget(self.ui.Multifloor)
        self.ui.label.setText("Multifloor Navigation")
        msg = String()
        msg.data = 'multifloor_navigation'        


        self.ui.btn_flr_G.clicked.connect(partial(self.go_floor, "FloorG"))
        self.ui.btn_flr_1.clicked.connect(partial(self.go_floor, "Floor1"))
        self.ui.btn_flr_2.clicked.connect(partial(self.go_floor, "Floor2"))
        self.ui.btn_flr_3.clicked.connect(partial(self.go_floor, "Floor3"))  

        self.ui.btn_entc1.clicked.connect(partial(self.go_location, "entc1"))
        self.ui.btn_reception.clicked.connect(partial(self.go_location, "reception"))
        self.ui.btn_uav.clicked.connect(partial(self.go_location, "uav"))
        self.ui.btn_com_lab.clicked.connect(partial(self.go_location, "com_lab"))
        self.ui.btn_office.clicked.connect(partial(self.go_location, "office"))
        self.ui.btn_conference.clicked.connect(partial(self.go_location, "conference_room"))
        self.ui.btn_analog_lab.clicked.connect(partial(self.go_location, "analog_lab"))
        self.ui.btn_digi_lab.clicked.connect(partial(self.go_location, "digi_lab"))
        self.ui.btn_tele_lab.clicked.connect(partial(self.go_location, "tele_lab"))
        self.ui.btn_pg_room.clicked.connect(partial(self.go_location, "pg_room"))



        self.publisher.publish(msg)


    def go_floor(self,floor):

        if floor =="FloorG":
            self.ui.floorStack.setCurrentWidget(self.ui.pg_flr_G)
            self.ui.Notifications_2.setText("Ground Floor")
        elif floor =="Floor1":
            self.ui.floorStack.setCurrentWidget(self.ui.pg_flr_1)
            self.ui.Notifications_2.setText("First Floor")
        elif floor =="Floor2":
            self.ui.floorStack.setCurrentWidget(self.ui.pg_flr_2)
            self.ui.Notifications_2.setText("Second Floor")
        elif floor =="Floor3":
            self.ui.floorStack.setCurrentWidget(self.ui.pg_flr_3)
            self.ui.Notifications_2.setText("Third Floor")


    def go_location(self,location):
        self.ui.Notifications.setText(f"Going to {location} ")
        multinav_goal = Twist()

        self.get_logger().info(f'Coordinates acquired: x:{self.coordinates[location][0]}, y:{self.coordinates[location][1]}, z:{self.coordinates[location][2]}, theta:{self.coordinates[location][3]}')

        multinav_goal.linear.x  = self.coordinates[location][0]
        multinav_goal.linear.y  = self.coordinates[location][1]
        multinav_goal.linear.z  = self.coordinates[location][2]
        multinav_goal.angular.z = self.coordinates[location][3]

        self.multinav_publisher.publish(multinav_goal)
        self.get_logger().info("Multi-floor goal published!")



    def handle_arm(self):
        self.get_logger().info('Arm Manipulator activated')
        
        # Switch to Arm Manipulator page
        self.ui.mainstack.setCurrentWidget(self.ui.Arm)

        # Update label text
        self.ui.label.setText("Arm Manipulator")

        # Publish a message
        msg = String()
        msg.data = 'arm_manipulator'
        # self.publisher.publish(msg)

        # Connect buttons with gesture function correctly
        self.ui.btn_ayubowan.clicked.connect(partial(self.gesture, "aubowan"))
        self.ui.btn_show_left.clicked.connect(partial(self.gesture, "show_left"))
        self.ui.btn_show_right.clicked.connect(partial(self.gesture, "show_right"))

    def gesture(self, gesture_name):
        self.ui.Notifications.setText(f"Doing {gesture_name} Gesture")

        msg = String()
        msg.data = gesture_name
        self.arm_publisher.publish(msg)



def main():
    rclpy.init()

    app = QApplication(sys.argv)  # QApplication must be initialized before any widgets
    main_win = QMainWindow()
    ros_node = RobotGUI(main_win)

    main_win.show()  # Show the main window

    try:
        sys.exit(app.exec_())  # Start the Qt event loop
    except SystemExit:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
