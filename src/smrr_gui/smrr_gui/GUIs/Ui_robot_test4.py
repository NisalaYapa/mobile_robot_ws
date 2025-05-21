# -*- coding: utf-8 -*-

from PyQt5 import QtCore, QtGui, QtWidgets
import os


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(800, 480)
        
        # Load external stylesheet
        self.load_stylesheet(MainWindow)
        
        # Set up central widget with a main layout
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.main_layout = QtWidgets.QVBoxLayout(self.centralwidget)
        self.main_layout.setContentsMargins(5, 5, 5, 5)
        self.main_layout.setSpacing(5)
        
        # Top section with home button and notifications
        self.top_section = QtWidgets.QHBoxLayout()
        self.Homebtn = QtWidgets.QPushButton(self.centralwidget)
        self.Homebtn.setMinimumSize(10, 10)
        self.Homebtn.setMaximumSize(30, 30)
        self.Homebtn.setText("Home")
        icon7 = QtGui.QIcon()
        icon7.addPixmap(QtGui.QPixmap("Icons/home.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.Homebtn.setIcon(icon7)
        self.Homebtn.setIconSize(QtCore.QSize(30, 30))
        self.Homebtn.setObjectName("Homebtn")
        
        self.top_section.addWidget(self.Homebtn)
        
        # Notifications label
        self.Notifications = QtWidgets.QLabel(self.centralwidget)
        self.Notifications.setWordWrap(True)
        self.Notifications.setObjectName("Notifications")
        self.top_section.addWidget(self.Notifications)
        self.main_layout.addLayout(self.top_section)
        
        # Title section
        self.title_frame = QtWidgets.QFrame(self.centralwidget)
        self.title_frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.title_frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.title_frame.setObjectName("frame_3")
        self.title_layout = QtWidgets.QVBoxLayout(self.title_frame)
        self.title_layout.setContentsMargins(5, 5, 5, 5)
        
        self.label = QtWidgets.QLabel(self.title_frame)
        self.label.setAlignment(QtCore.Qt.AlignCenter)
        self.label.setObjectName("label")
        self.title_layout.addWidget(self.label)
        self.main_layout.addWidget(self.title_frame)
        
        # Main content stack
        self.mainstack = QtWidgets.QStackedWidget(self.centralwidget)
        self.mainstack.setObjectName("mainstack")
        self.main_layout.addWidget(self.mainstack, 1)  # Give stretch factor to grow
        
        # Create pages for stacked widget
        # Home Page
        self.HomePage = QtWidgets.QWidget()
        self.HomePage.setObjectName("HomePage")
        self.home_layout = QtWidgets.QVBoxLayout(self.HomePage)
        
        self.label_2 = QtWidgets.QLabel(self.HomePage)
        self.label_2.setAlignment(QtCore.Qt.AlignCenter)
        self.label_2.setObjectName("label_2")
        self.home_layout.addWidget(self.label_2)
        self.mainstack.addWidget(self.HomePage)
        
        # CrowdNav Page
        self.CrowdNav = QtWidgets.QWidget()
        self.CrowdNav.setObjectName("CrowdNav")
        self.crwnav_layout = QtWidgets.QVBoxLayout(self.CrowdNav)
        
        self.crwnav_top = QtWidgets.QHBoxLayout()
        self.crwnav_goal_input = QtWidgets.QTextEdit(self.CrowdNav)
        self.crwnav_goal_input.setMaximumHeight(100)
        self.crwnav_goal_input.setObjectName("crwnav_goal_input")
        self.crwnav_top.addWidget(self.crwnav_goal_input)
        
        self.btn_crwnav_run = QtWidgets.QPushButton(self.CrowdNav)
        self.btn_crwnav_run.setMinimumSize(130, 41)
        self.btn_crwnav_run.setObjectName("btn_crwnav_run")
        self.crwnav_top.addWidget(self.btn_crwnav_run)
        self.crwnav_layout.addLayout(self.crwnav_top)
        
        self.crwnav_buttons = QtWidgets.QHBoxLayout()
        self.btn_crowdnav_start = QtWidgets.QPushButton(self.CrowdNav)
        self.btn_crowdnav_start.setMinimumSize(100, 60)
        self.btn_crowdnav_start.setObjectName("btn_crowdnav_start")
        self.crwnav_buttons.addWidget(self.btn_crowdnav_start)
        
        self.btn_crowdnav_end = QtWidgets.QPushButton(self.CrowdNav)
        self.btn_crowdnav_end.setMinimumSize(100, 60)
        self.btn_crowdnav_end.setObjectName("btn_crowdnav_end")
        self.crwnav_buttons.addWidget(self.btn_crowdnav_end)
        self.crwnav_layout.addLayout(self.crwnav_buttons)
        self.mainstack.addWidget(self.CrowdNav)
        
        # Multifloor Page
        self.Multifloor = QtWidgets.QWidget()
        self.Multifloor.setObjectName("Multifloor")
        self.multifloor_layout = QtWidgets.QVBoxLayout(self.Multifloor)
        
        self.Notifications_2 = QtWidgets.QLabel(self.Multifloor)
        self.Notifications_2.setWordWrap(True)
        self.Notifications_2.setAlignment(QtCore.Qt.AlignCenter)
        self.Notifications_2.setObjectName("Notifications_2")
        self.multifloor_layout.addWidget(self.Notifications_2)
        
        # Floor Stack for showing different floor options
        self.floorStack = QtWidgets.QStackedWidget(self.Multifloor)
        self.floorStack.setObjectName("floorStack")
        
        # Ground Floor
        self.pg_flr_G = QtWidgets.QWidget()
        self.pg_flr_G.setObjectName("pg_flr_G")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.pg_flr_G)
        self.horizontalLayout.setObjectName("horizontalLayout")
        
        self.btn_reception = QtWidgets.QPushButton(self.pg_flr_G)
        self.btn_reception.setObjectName("btn_reception")
        self.horizontalLayout.addWidget(self.btn_reception)
        
        self.btn_entc1 = QtWidgets.QPushButton(self.pg_flr_G)
        self.btn_entc1.setObjectName("btn_entc1")
        self.horizontalLayout.addWidget(self.btn_entc1)
        
        self.btn_uav = QtWidgets.QPushButton(self.pg_flr_G)
        self.btn_uav.setObjectName("btn_uav")
        self.horizontalLayout.addWidget(self.btn_uav)
        self.floorStack.addWidget(self.pg_flr_G)
        
        # 1st Floor
        self.pg_flr_1 = QtWidgets.QWidget()
        self.pg_flr_1.setObjectName("pg_flr_1")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout(self.pg_flr_1)
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        
        self.btn_com_lab = QtWidgets.QPushButton(self.pg_flr_1)
        self.btn_com_lab.setObjectName("btn_com_lab")
        self.horizontalLayout_2.addWidget(self.btn_com_lab)
        
        self.btn_office = QtWidgets.QPushButton(self.pg_flr_1)
        self.btn_office.setObjectName("btn_office")
        self.horizontalLayout_2.addWidget(self.btn_office)
        
        self.btn_conference = QtWidgets.QPushButton(self.pg_flr_1)
        self.btn_conference.setObjectName("btn_conference")
        self.horizontalLayout_2.addWidget(self.btn_conference)
        self.floorStack.addWidget(self.pg_flr_1)
        
        # 2nd Floor
        self.pg_flr_2 = QtWidgets.QWidget()
        self.pg_flr_2.setObjectName("pg_flr_2")
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout(self.pg_flr_2)
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        
        self.btn_digi_lab = QtWidgets.QPushButton(self.pg_flr_2)
        self.btn_digi_lab.setObjectName("btn_digi_lab")
        self.horizontalLayout_3.addWidget(self.btn_digi_lab)
        
        self.btn_analog_lab = QtWidgets.QPushButton(self.pg_flr_2)
        self.btn_analog_lab.setObjectName("btn_analog_lab")
        self.horizontalLayout_3.addWidget(self.btn_analog_lab)
        self.floorStack.addWidget(self.pg_flr_2)
        
        # 3rd Floor
        self.pg_flr_3 = QtWidgets.QWidget()
        self.pg_flr_3.setObjectName("pg_flr_3")
        self.horizontalLayout_5 = QtWidgets.QHBoxLayout(self.pg_flr_3)
        self.horizontalLayout_5.setObjectName("horizontalLayout_5")
        
        self.btn_pg_room = QtWidgets.QPushButton(self.pg_flr_3)
        self.btn_pg_room.setObjectName("btn_pg_room")
        self.horizontalLayout_5.addWidget(self.btn_pg_room)
        
        self.btn_tele_lab = QtWidgets.QPushButton(self.pg_flr_3)
        self.btn_tele_lab.setObjectName("btn_tele_lab")
        self.horizontalLayout_5.addWidget(self.btn_tele_lab)
        self.floorStack.addWidget(self.pg_flr_3)
        
        self.multifloor_layout.addWidget(self.floorStack)
        
        # Floor selection buttons
        self.frame = QtWidgets.QFrame(self.Multifloor)
        self.frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame.setObjectName("frame")
        self.flr_buttons_layout = QtWidgets.QHBoxLayout(self.frame)
        self.flr_buttons_layout.setObjectName("horizontalLayout_4")
        
        self.btn_flr_G = QtWidgets.QPushButton(self.frame)
        self.btn_flr_G.setObjectName("btn_flr_G")
        self.flr_buttons_layout.addWidget(self.btn_flr_G)
        
        self.btn_flr_1 = QtWidgets.QPushButton(self.frame)
        self.btn_flr_1.setObjectName("btn_flr_1")
        self.flr_buttons_layout.addWidget(self.btn_flr_1)
        
        self.btn_flr_2 = QtWidgets.QPushButton(self.frame)
        self.btn_flr_2.setObjectName("btn_flr_2")
        self.flr_buttons_layout.addWidget(self.btn_flr_2)
        
        self.btn_flr_3 = QtWidgets.QPushButton(self.frame)
        self.btn_flr_3.setObjectName("btn_flr_3")
        self.flr_buttons_layout.addWidget(self.btn_flr_3)
        
        self.multifloor_layout.addWidget(self.frame)
        self.mainstack.addWidget(self.Multifloor)
        
        # Docking Page
        self.Docking = QtWidgets.QWidget()
        self.Docking.setObjectName("Docking")
        self.docking_layout = QtWidgets.QVBoxLayout(self.Docking)
        
        self.btn_dock = QtWidgets.QPushButton(self.Docking)
        self.btn_dock.setMinimumSize(170, 80)
        self.btn_dock.setObjectName("btn_dock")
        
        # Use alignment to center the button
        self.docking_layout.addStretch(1)
        self.docking_layout.addWidget(self.btn_dock, 0, QtCore.Qt.AlignCenter)
        self.docking_layout.addStretch(1)
        
        self.mainstack.addWidget(self.Docking)
        
        # Arm Page
        self.Arm = QtWidgets.QWidget()
        self.Arm.setObjectName("Arm")
        self.arm_layout = QtWidgets.QVBoxLayout(self.Arm)
        
        self.arm_buttons = QtWidgets.QHBoxLayout()
        self.arm_buttons.setObjectName("horizontalLayout_7")
        
        self.btn_ayubowan = QtWidgets.QPushButton(self.Arm)
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap("../../../../../.designer/backup/Icons/aybowan.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.btn_ayubowan.setIcon(icon)
        self.btn_ayubowan.setIconSize(QtCore.QSize(35, 35))
        self.btn_ayubowan.setObjectName("btn_ayubowan")
        self.arm_buttons.addWidget(self.btn_ayubowan)

        self.btn_handshake = QtWidgets.QPushButton(self.Arm)
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap("../../../../../.designer/backup/Icons/handshake.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.btn_handshake.setIcon(icon)
        self.btn_handshake.setIconSize(QtCore.QSize(35, 35))
        self.btn_handshake.setObjectName("btn_handshake")
        self.arm_buttons.addWidget(self.btn_handshake)
        
        
        self.btn_hi = QtWidgets.QPushButton(self.Arm)
        icon1 = QtGui.QIcon()
        icon1.addPixmap(QtGui.QPixmap("../../../../../.designer/backup/Icons/Hi.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.btn_hi.setIcon(icon1)
        self.btn_hi.setIconSize(QtCore.QSize(35, 35))
        self.btn_hi.setObjectName("btn_hi")
        self.arm_buttons.addWidget(self.btn_hi)
        
        self.btn_highfive = QtWidgets.QPushButton(self.Arm)
        icon2 = QtGui.QIcon()
        icon2.addPixmap(QtGui.QPixmap("../../../../../.designer/backup/Icons/highfive.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.btn_highfive.setIcon(icon2)
        self.btn_highfive.setIconSize(QtCore.QSize(30, 30))
        self.btn_highfive.setObjectName("btn_highfive")
        self.arm_buttons.addWidget(self.btn_highfive)
        
        self.arm_layout.addStretch(1)
        self.arm_layout.addLayout(self.arm_buttons)
        self.arm_layout.addStretch(1)
        
        self.mainstack.addWidget(self.Arm)
        
        # Navigation buttons layout at bottom
        self.frame_2 = QtWidgets.QFrame(self.centralwidget)
        self.frame_2.setMinimumHeight(60)
        self.frame_2.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_2.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_2.setObjectName("frame_2")
        self.nav_buttons_layout = QtWidgets.QHBoxLayout(self.frame_2)
        self.nav_buttons_layout.setObjectName("horizontalLayout_6")
        
        self.btn_crowdnav = QtWidgets.QPushButton(self.frame_2)
        self.btn_crowdnav.setMinimumSize(100, 40)
        icon3 = QtGui.QIcon()
        icon3.addPixmap(QtGui.QPixmap("Icons/crowdnav.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.btn_crowdnav.setIcon(icon3)
        self.btn_crowdnav.setIconSize(QtCore.QSize(30, 30))
        self.btn_crowdnav.setObjectName("btn_crowdnav")
        self.nav_buttons_layout.addWidget(self.btn_crowdnav)
        
        self.btn_multifloor = QtWidgets.QPushButton(self.frame_2)
        self.btn_multifloor.setMinimumSize(100, 40)
        icon4 = QtGui.QIcon()
        icon4.addPixmap(QtGui.QPixmap("Icons/multifloor.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.btn_multifloor.setIcon(icon4)
        self.btn_multifloor.setIconSize(QtCore.QSize(30, 30))
        self.btn_multifloor.setObjectName("btn_multifloor")
        self.nav_buttons_layout.addWidget(self.btn_multifloor)
        
        self.btn_arm = QtWidgets.QPushButton(self.frame_2)
        self.btn_arm.setMinimumSize(100, 40)
        icon5 = QtGui.QIcon()
        icon5.addPixmap(QtGui.QPixmap("Icons/arm.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.btn_arm.setIcon(icon5)
        self.btn_arm.setIconSize(QtCore.QSize(30, 30))
        self.btn_arm.setObjectName("btn_arm")
        self.nav_buttons_layout.addWidget(self.btn_arm)
        
        self.btn_dockpage = QtWidgets.QPushButton(self.frame_2)
        self.btn_dockpage.setMinimumSize(100, 40)
        icon6 = QtGui.QIcon()
        icon6.addPixmap(QtGui.QPixmap("Icons/docking.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.btn_dockpage.setIcon(icon6)
        self.btn_dockpage.setIconSize(QtCore.QSize(30, 30))
        self.btn_dockpage.setObjectName("btn_dockpage")
        self.nav_buttons_layout.addWidget(self.btn_dockpage)
        
        self.main_layout.addWidget(self.frame_2)
        
        MainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(MainWindow)
        self.mainstack.setCurrentIndex(0)  # Default to homepage
        self.floorStack.setCurrentIndex(0)  # Default to ground floor
        QtCore.QMetaObject.connectSlotsByName(MainWindow)
        
        # Connect buttons to change stacked widgets
        self.btn_crowdnav.clicked.connect(lambda: self.mainstack.setCurrentIndex(1))
        self.btn_multifloor.clicked.connect(lambda: self.mainstack.setCurrentIndex(2))
        self.btn_arm.clicked.connect(lambda: self.mainstack.setCurrentIndex(4))
        self.btn_dockpage.clicked.connect(lambda: self.mainstack.setCurrentIndex(3))
        self.Homebtn.clicked.connect(lambda: self.mainstack.setCurrentIndex(0))
        
        # Connect floor buttons
        self.btn_flr_G.clicked.connect(lambda: self.change_floor(0, "Ground Floor"))
        self.btn_flr_1.clicked.connect(lambda: self.change_floor(1, "1st Floor"))
        self.btn_flr_2.clicked.connect(lambda: self.change_floor(2, "2nd Floor"))
        self.btn_flr_3.clicked.connect(lambda: self.change_floor(3, "3rd Floor"))

    def load_stylesheet(self, MainWindow):
        """Load the external stylesheet"""
        try:
            # Get the directory of the current script
            script_dir = os.path.dirname(os.path.abspath(__file__))
            
            # Path to the CSS file
            css_file = os.path.join(script_dir, 'styles.css')
            
            # Check if the file exists
            if os.path.exists(css_file):
                with open(css_file, 'r') as f:
                    stylesheet = f.read()
                    MainWindow.setStyleSheet(stylesheet)
            else:
                print(f"Warning: Stylesheet file not found at {css_file}")
                
        except Exception as e:
            print(f"Error loading stylesheet: {e}")

    def change_floor(self, index, floor_name):
        """Change floor in the multifloor navigation page"""
        self.floorStack.setCurrentIndex(index)
        self.Notifications_2.setText(floor_name)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "SANSAR Robot"))
        self.label_2.setText(_translate("MainWindow", "WELCOME"))
        self.btn_crwnav_run.setText(_translate("MainWindow", "Run"))
        self.btn_crowdnav_start.setText(_translate("MainWindow", "Start"))
        self.btn_crowdnav_end.setText(_translate("MainWindow", "End"))
        self.btn_flr_G.setText(_translate("MainWindow", "Ground Floor"))
        self.btn_flr_1.setText(_translate("MainWindow", "1st Floor"))
        self.btn_flr_2.setText(_translate("MainWindow", "2nd Floor"))
        self.btn_flr_3.setText(_translate("MainWindow", "3rd Floor"))
        self.btn_reception.setText(_translate("MainWindow", "Reception"))
        self.btn_entc1.setText(_translate("MainWindow", "ENTC1"))
        self.btn_uav.setText(_translate("MainWindow", "UAV"))
        self.btn_com_lab.setText(_translate("MainWindow", "Computer lab"))
        self.btn_office.setText(_translate("MainWindow", "Office"))
        self.btn_conference.setText(_translate("MainWindow", "Conference Room"))
        self.btn_digi_lab.setText(_translate("MainWindow", "Digital lab"))
        self.btn_analog_lab.setText(_translate("MainWindow", "Analog Lab"))
        self.btn_pg_room.setText(_translate("MainWindow", "PG Seminar Room"))
        self.btn_tele_lab.setText(_translate("MainWindow", "Telecom Lab"))
        self.Notifications_2.setText(_translate("MainWindow", "Ground Floor"))
        self.btn_dock.setText(_translate("MainWindow", "Start Docking"))
        self.btn_ayubowan.setText(_translate("MainWindow", "Ayubowan"))
        self.btn_handshake.setText(_translate("MainWindow", "Handshake"))
        self.btn_hi.setText(_translate("MainWindow", "Hi"))
        self.btn_highfive.setText(_translate("MainWindow", "HighFive"))
        self.Notifications.setText(_translate("MainWindow", "Notifications"))
        self.btn_crowdnav.setText(_translate("MainWindow", "Crowd Nav"))
        self.btn_multifloor.setText(_translate("MainWindow", "Multifloor Nav"))
        self.btn_arm.setText(_translate("MainWindow", "Arm Manipulator"))
        self.btn_dockpage.setText(_translate("MainWindow", "Docking"))
        self.label.setText(_translate("MainWindow", "SANSAR : Mobile Robot Receptionist"))


# Add a main function to run the application
if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())