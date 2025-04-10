import rclpy
import sys
from rclpy.node import Node
from std_msgs.msg import String
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import QProcess
from .GUIs.Ui_robot_test3 import Ui_MainWindow
from functools import partial
import subprocess
import time




class RobotGUI(Node):
    def __init__(self, main_win):
        super().__init__('robot_gui')

        # Initialize publisher
        self.publisher = self.create_publisher(String, 'robot_control', 10)

        self.main_win = main_win
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self.main_win)


        # Connect buttons to functions
        self.ui.btn_crowdnav.clicked.connect(self.handle_crowdnav)
        self.ui.btn_multifloor.clicked.connect(self.handle_muiltinav)
        self.ui.btn_arm.clicked.connect(self.handle_arm)

        self.ui.mainstack.setCurrentWidget(self.ui.CrowdNav)
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

    # def crowdnav_goal(self):
    #     goal = self.ui.crwnav_goal_input.toPlainText()  # Use toPlainText() for QTextEdit

    #     if not goal.strip():  # Ensure input is not empty or just spaces
    #         self.ui.Notifications.setText("No goal is provided")
    #     else:
    #         self.ui.Notifications.setText(f"Going to the {goal}")
    #         # Run the ROS 2 command
    #         subprocess.Popen(["gnome-terminal", "--","ros2", "topic", "list"])
            # subprocess.Popen(["gnome-terminal", "--","tmuxinator", "buffer"])  



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
        elif floor =="Floor1":
            self.ui.floorStack.setCurrentWidget(self.ui.pg_flr_1)
        elif floor =="Floor2":
            self.ui.floorStack.setCurrentWidget(self.ui.pg_flr_2)
        elif floor =="Floor3":
            self.ui.floorStack.setCurrentWidget(self.ui.pg_flr_3)


    def go_location(self,location):
        self.ui.Notifications.setText(f"Goint to {location} ")


    
    



    def handle_arm(self):
        self.get_logger().info('Arm Manipulator activated')
        
        # Switch to Arm Manipulator page
        self.ui.mainstack.setCurrentWidget(self.ui.Arm)

        # Update label text
        self.ui.label.setText("Arm Manipulator")

        # Publish a message
        msg = String()
        msg.data = 'arm_manipulator'
        self.publisher.publish(msg)

        # Connect buttons with gesture function correctly
        self.ui.btn_ayubowan.clicked.connect(partial(self.gesture, "Ayubowan"))
        self.ui.btn_hi.clicked.connect(partial(self.gesture, "Hi"))
        self.ui.btn_highfive.clicked.connect(partial(self.gesture, "Highfive"))

    def gesture(self, gesture_name):
        self.ui.Notifications.setText(f"Doing {gesture_name} Gesture")



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
