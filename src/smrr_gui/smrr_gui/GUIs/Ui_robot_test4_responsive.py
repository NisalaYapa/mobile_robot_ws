import sys
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import Qt

# This is a modified version of your original UI class with larger buttons and font
class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1200, 800)  # More reasonable size

        self.setWindowFlags(QtCore.Qt.Window | 
                       QtCore.Qt.WindowMinimizeButtonHint |
                       QtCore.Qt.WindowMaximizeButtonHint |
                       QtCore.Qt.WindowCloseButtonHint |
                       QtCore.Qt.WindowSystemMenuHint)
        
        # Apply base stylesheet with larger font sizes
        MainWindow.setStyleSheet(self.get_base_stylesheet())
        
        # Create central widget with layout
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.main_layout = QtWidgets.QVBoxLayout(self.centralwidget)
        self.main_layout.setObjectName("main_layout")
        
        # Add home button at top - INCREASED SIZE
        self.home_button_layout = QtWidgets.QHBoxLayout()
        self.Homebtn = QtWidgets.QPushButton(self.centralwidget)
        self.Homebtn.setObjectName("Homebtn")
        self.Homebtn.setText("")
        self.Homebtn.setIcon(QtGui.QIcon("Icons/home.png"))
        self.Homebtn.setIconSize(QtCore.QSize(60, 60))  # INCREASED from 40,40
        self.Homebtn.setFixedSize(60, 60)  # INCREASED from 40,40
        self.Homebtn.setStyleSheet("""
            QPushButton {
                background-color: #e6f2ff;
                border: 2px solid #b3d1ff;
                border-radius: 8px;
                padding: 8px;
                color: #0066cc;
            }
            QPushButton:hover {
                background-color: #cce5ff;
            }
            QPushButton:pressed {
                background-color: #99ccff;
            }
        """)
        self.home_button_layout.addWidget(self.Homebtn)
        self.home_button_layout.addStretch(1)
        self.main_layout.addLayout(self.home_button_layout)
        
        # Add title section - INCREASED FONT SIZE
        self.title_frame = QtWidgets.QFrame(self.centralwidget)
        self.title_frame.setObjectName("title_frame")
        self.title_frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.title_frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.title_frame.setMaximumHeight(80)  # INCREASED from 60
        self.title_layout = QtWidgets.QVBoxLayout(self.title_frame)
        self.title_layout.setObjectName("title_layout")
        
        self.label = QtWidgets.QLabel(self.title_frame)
        self.label.setObjectName("label")
        self.label.setText("SANSAR : Mobile Robot Receptionist")
        self.label.setStyleSheet("font-size: 28px; font-weight: bold;")  # INCREASED from 20px
        self.label.setAlignment(Qt.AlignCenter)
        self.title_layout.addWidget(self.label)
        
        self.main_layout.addWidget(self.title_frame)
        
        # Add notification section - INCREASED FONT SIZE
        self.Notifications = QtWidgets.QLabel(self.centralwidget)
        self.Notifications.setObjectName("Notifications")
        self.Notifications.setText("Notifications")
        self.Notifications.setAlignment(Qt.AlignCenter)
        self.Notifications.setStyleSheet("font-size: 18px; font-weight: bold;")  # INCREASED font size
        self.Notifications.setWordWrap(True)
        self.main_layout.addWidget(self.Notifications)
        
        # Add main stack widget (contains all pages)
        self.mainstack = QtWidgets.QStackedWidget(self.centralwidget)
        self.mainstack.setObjectName("mainstack")
        self.main_layout.addWidget(self.mainstack)
        
        # Create all pages
        self.setup_home_page()
        self.setup_crowd_nav_page()
        self.setup_multifloor_page()
        self.setup_docking_page()
        self.setup_arm_page()
        
        # Add navigation buttons at bottom - INCREASED HEIGHT
        self.nav_frame = QtWidgets.QFrame(self.centralwidget)
        self.nav_frame.setObjectName("nav_frame")
        self.nav_frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.nav_frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.nav_frame.setMaximumHeight(120)  # INCREASED from 80
        self.nav_frame.setStyleSheet("""
            QFrame {
                background-color: #ffffff;
                border: 3px solid #00C9A7;  /* INCREASED border thickness */
                border-radius: 10px;
                padding: 16px;
            }
        """)
        
        self.nav_layout = QtWidgets.QHBoxLayout(self.nav_frame)
        self.nav_layout.setObjectName("nav_layout")
        
        # Create navigation buttons - LARGER BUTTONS
        self.btn_crowdnav = self.create_nav_button("Crowd Nav", "Icons/crowdnav.png")
        self.btn_multifloor = self.create_nav_button("Multifloor Nav", "Icons/multifloor.png")
        self.btn_arm = self.create_nav_button("Arm Manipulator", "Icons/arm.png")
        self.btn_dockpage= self.create_nav_button("Docking", "Icons/docking.png")
        
        self.nav_layout.addWidget(self.btn_crowdnav)
        self.nav_layout.addWidget(self.btn_multifloor)
        self.nav_layout.addWidget(self.btn_arm)
        self.nav_layout.addWidget(self.btn_dockpage)
        
        self.main_layout.addWidget(self.nav_frame)
        
        # Set central widget
        MainWindow.setCentralWidget(self.centralwidget)
        
        # Set initial page
        self.mainstack.setCurrentIndex(0)
        
        # Connect signals
        self.connect_signals()
    
    def create_nav_button(self, text, icon_path):
        """Helper method to create navigation buttons with consistent styling - LARGER SIZE"""
        button = QtWidgets.QPushButton(text)
        button.setStyleSheet("""
            font-size: 18px;  /* INCREASED font size */
            font-weight: bold;
            padding: 12px;    /* INCREASED padding */
            min-height: 60px; /* INCREASED height */
        """)
        button.setIcon(QtGui.QIcon(icon_path))
        button.setIconSize(QtCore.QSize(40, 40))  # INCREASED from 30,30
        return button
    
    def setup_home_page(self):
        """Set up the home page with proper layout"""
        self.HomePage = QtWidgets.QWidget()
        self.HomePage.setObjectName("HomePage")
        home_layout = QtWidgets.QVBoxLayout(self.HomePage)
        
        # Add spacer at top
        home_layout.addStretch(1)
        
        # Add welcome label - INCREASED SIZE
        self.label_2 = QtWidgets.QLabel("WELCOME")
        self.label_2.setObjectName("label_2")
        self.label_2.setAlignment(Qt.AlignCenter)
        self.label_2.setStyleSheet("""
            QLabel {
                color: #FF9A8B;
                font-size: 100px;  /* INCREASED from 80px */
                font-weight: bold;
            }
            QLabel:hover {
                color: #FF6B95;
            }
        """)
        home_layout.addWidget(self.label_2)
        
        # Add spacer at bottom
        home_layout.addStretch(1)
        
        # Add page to stack
        self.mainstack.addWidget(self.HomePage)
    
    def setup_crowd_nav_page(self):
        """Set up the crowd navigation page with proper layout"""
        self.CrowdNav = QtWidgets.QWidget()
        self.CrowdNav.setObjectName("CrowdNav")
        
        # Use grid layout for more complex arrangement
        crowd_layout = QtWidgets.QGridLayout(self.CrowdNav)
        
        # Add text input - INCREASED FONT SIZE
        self.crwnav_goal_input = QtWidgets.QTextEdit()
        self.crwnav_goal_input.setObjectName("crwnav_goal_input")
        self.crwnav_goal_input.setStyleSheet("""
            background-color: #F8F9FA;
            color: #333333;
            font-size: 20px;  /* INCREASED from 16px */
            padding: 12px;    /* INCREASED padding */
            border: 3px solid #CCCCCC;  /* INCREASED border */
            border-radius: 8px;
        """)
        self.crwnav_goal_input.setMaximumHeight(120)  # INCREASED from 100
        crowd_layout.addWidget(self.crwnav_goal_input, 0, 0, 1, 2)
        
        # Add run button - LARGER BUTTON
        self.btn_crwnav_run = QtWidgets.QPushButton("Run")
        self.btn_crwnav_run.setObjectName("btn_crwnav_run")
        self.btn_crwnav_run.setStyleSheet("""
            font-size: 20px;
            font-weight: bold;
            min-height: 60px;
        """)
        crowd_layout.addWidget(self.btn_crwnav_run, 0, 2, 1, 1)
        
        # Add Start/End buttons - LARGER BUTTONS
        self.btn_crowdnav_ = QtWidgets.QPushButton("Start")
        self.btn_crowdnav_.setObjectName("btn_crowdnav_")
        self.btn_crowdnav_.setStyleSheet("font-size: 24px; min-height: 70px;")  # INCREASED from 20px
        crowd_layout.addWidget(self.btn_crowdnav_, 1, 0, 1, 2)
        
        self.pushButton_3 = QtWidgets.QPushButton("End")
        self.pushButton_3.setObjectName("pushButton_3")
        self.pushButton_3.setStyleSheet("font-size: 24px; min-height: 70px;")  # INCREASED from 20px
        crowd_layout.addWidget(self.pushButton_3, 1, 2, 1, 1)
        
        # Add spacer
        crowd_layout.setRowStretch(2, 1)
        
        # Add page to stack
        self.mainstack.addWidget(self.CrowdNav)
    
    def setup_multifloor_page(self):
        """Set up the multifloor navigation page with proper layout"""
        self.Multifloor = QtWidgets.QWidget()
        self.Multifloor.setObjectName("Multifloor")
        multifloor_layout = QtWidgets.QVBoxLayout(self.Multifloor)
        
        # Add title - INCREASED FONT SIZE
        self.Notifications_2 = QtWidgets.QLabel("Ground Floor")
        self.Notifications_2.setObjectName("Notifications_2")
        self.Notifications_2.setAlignment(Qt.AlignCenter)
        self.Notifications_2.setStyleSheet("font-size: 22px; font-weight: bold;")  # INCREASED font size
        multifloor_layout.addWidget(self.Notifications_2)
        
        # Add floor stack widget
        self.floorStack = QtWidgets.QStackedWidget()
        self.floorStack.setObjectName("floorStack")
        multifloor_layout.addWidget(self.floorStack)
        
        # Add floor buttons - LARGER BUTTONS
        floor_buttons_frame = QtWidgets.QFrame()
        floor_buttons_frame.setObjectName("floor_buttons_frame")
        floor_buttons_layout = QtWidgets.QHBoxLayout(floor_buttons_frame)
        
        # Make all floor buttons larger
        self.btn_flr_G = self.create_floor_button("Ground Floor")
        self.btn_flr_1 = self.create_floor_button("1st Floor")
        self.btn_flr_2 = self.create_floor_button("2nd Floor")
        self.btn_flr_3 = self.create_floor_button("3rd Floor")
        
        floor_buttons_layout.addWidget(self.btn_flr_G)
        floor_buttons_layout.addWidget(self.btn_flr_1)
        floor_buttons_layout.addWidget(self.btn_flr_2)
        floor_buttons_layout.addWidget(self.btn_flr_3)
        
        multifloor_layout.addWidget(floor_buttons_frame)
        
        # Set up floor pages
        self.setup_floor_pages()
        
        # Add page to stack
        self.mainstack.addWidget(self.Multifloor)
    
    def create_floor_button(self, text):
        """Helper method for creating floor buttons with consistent styling"""
        button = QtWidgets.QPushButton(text)
        button.setStyleSheet("""
            font-size: 18px;  /* Larger font */
            font-weight: bold;
            min-height: 60px; /* Taller button */
            padding: 10px;
        """)
        return button
    
    def setup_floor_pages(self):
        """Set up pages for different floors - WITH LARGER BUTTONS"""
        # Ground floor
        self.pg_flr_G = QtWidgets.QWidget()
        self.pg_flr_G.setObjectName("pg_flr_G")
        g_layout = QtWidgets.QHBoxLayout(self.pg_flr_G)
        
        # Create buttons with larger size
        self.btn_reception = self.create_location_button("Reception")
        self.btn_entc1 = self.create_location_button("ENTC1")
        self.btn_uav = self.create_location_button("UAV")
        
        g_layout.addWidget(self.btn_reception)
        g_layout.addWidget(self.btn_entc1)
        g_layout.addWidget(self.btn_uav)
        
        self.floorStack.addWidget(self.pg_flr_G)
        
        # 1st floor
        self.pg_flr_1 = QtWidgets.QWidget()
        self.pg_flr_1.setObjectName("pg_flr_1")
        flr1_layout = QtWidgets.QHBoxLayout(self.pg_flr_1)
        
        self.btn_com_lab = self.create_location_button("Computer lab")
        self.btn_office = self.create_location_button("Office") 
        self.btn_conference = self.create_location_button("Conference Room")
        
        flr1_layout.addWidget(self.btn_com_lab)
        flr1_layout.addWidget(self.btn_office)
        flr1_layout.addWidget(self.btn_conference)
        
        self.floorStack.addWidget(self.pg_flr_1)
        
        # 2nd floor
        self.pg_flr_2 = QtWidgets.QWidget()
        self.pg_flr_2.setObjectName("pg_flr_2")
        flr2_layout = QtWidgets.QHBoxLayout(self.pg_flr_2)
        
        self.btn_digi_lab = self.create_location_button("Digitab lab")
        self.btn_analog_lab = self.create_location_button("Analog Lab")
        
        flr2_layout.addWidget(self.btn_digi_lab)
        flr2_layout.addWidget(self.btn_analog_lab)
        
        self.floorStack.addWidget(self.pg_flr_2)
        
        # 3rd floor
        self.pg_flr_3 = QtWidgets.QWidget()
        self.pg_flr_3.setObjectName("pg_flr_3")
        flr3_layout = QtWidgets.QHBoxLayout(self.pg_flr_3)
        
        self.btn_pg_room = self.create_location_button("PG Seminar Room")
        self.btn_tele_lab = self.create_location_button("Telecom Lab")
        
        flr3_layout.addWidget(self.btn_pg_room)
        flr3_layout.addWidget(self.btn_tele_lab)
        
        self.floorStack.addWidget(self.pg_flr_3)
    
    def create_location_button(self, text):
        """Helper method for creating location buttons with consistent styling"""
        button = QtWidgets.QPushButton(text)
        button.setStyleSheet("""
            font-size: 18px;  /* Larger font */
            font-weight: bold;
            min-height: 80px; /* Taller button */
            padding: 12px;
        """)
        return button
    
    def setup_docking_page(self):
        """Set up the docking page with proper layout"""
        self.Docking = QtWidgets.QWidget()
        self.Docking.setObjectName("Docking")
        docking_layout = QtWidgets.QVBoxLayout(self.Docking)
        
        # Add spacer at top
        docking_layout.addStretch(1)
        
        # Add dock button - LARGER BUTTON
        self.btn_dock = QtWidgets.QPushButton("Start Docking")
        self.btn_dock.setObjectName("btn_dock")
        self.btn_dock.setStyleSheet("font-size: 28px; font-weight: bold;")  # INCREASED from 20px
        self.btn_dock.setMinimumHeight(120)  # INCREASED from 80
        
        # Center the button
        dock_button_layout = QtWidgets.QHBoxLayout()
        dock_button_layout.addStretch(1)
        dock_button_layout.addWidget(self.btn_dock)
        dock_button_layout.addStretch(1)
        
        docking_layout.addLayout(dock_button_layout)
        
        # Add spacer at bottom
        docking_layout.addStretch(1)
        
        # Add page to stack
        self.mainstack.addWidget(self.Docking)
    
    def setup_arm_page(self):
        """Set up the arm manipulation page with proper layout"""
        self.Arm = QtWidgets.QWidget()
        self.Arm.setObjectName("Arm")
        arm_layout = QtWidgets.QVBoxLayout(self.Arm)
        
        # Add spacer at top
        arm_layout.addStretch(1)
        
        # Add buttons layout
        buttons_layout = QtWidgets.QHBoxLayout()
        
        # Define arm buttons - LARGER BUTTONS
        arm_button_style = """
            QPushButton {
                background-color: #B8E0D2;
                color: #2E3440;
                font-size: 22px;  /* INCREASED from 16px */
                padding: 16px 24px;  /* INCREASED padding */
                border-radius: 12px;
                border: 3px solid #90C9B7;  /* INCREASED border */
                min-height: 80px;  /* INCREASED height */
                min-width: 160px;  /* INCREASED width */
            }
            QPushButton:hover {
                background-color: #90C9B7;
                border: 3px solid #6CA893;
            }
        """
        
        self.btn_ayubowan = QtWidgets.QPushButton("Ayubowan")
        self.btn_ayubowan.setObjectName("btn_ayubowan")
        self.btn_ayubowan.setStyleSheet(arm_button_style)
        buttons_layout.addWidget(self.btn_ayubowan)
        
        self.btn_hi = QtWidgets.QPushButton("Hi")
        self.btn_hi.setObjectName("btn_hi")
        self.btn_hi.setStyleSheet(arm_button_style)
        buttons_layout.addWidget(self.btn_hi)
        
        self.btn_highfive = QtWidgets.QPushButton("HighFive")
        self.btn_highfive.setObjectName("btn_highfive")
        self.btn_highfive.setStyleSheet(arm_button_style)
        buttons_layout.addWidget(self.btn_highfive)
        
        arm_layout.addLayout(buttons_layout)
        
        # Add spacer at bottom
        arm_layout.addStretch(1)
        
        # Add page to stack
        self.mainstack.addWidget(self.Arm)
    
    def connect_signals(self):
        """Connect button signals to their slots"""
        # Home button
        self.Homebtn.clicked.connect(lambda: self.mainstack.setCurrentIndex(0))
        
        # Navigation buttons
        self.btn_crowdnav.clicked.connect(lambda: self.mainstack.setCurrentIndex(1))
        self.btn_multifloor.clicked.connect(lambda: self.mainstack.setCurrentIndex(2))
        self.btn_dockpage.clicked.connect(lambda: self.mainstack.setCurrentIndex(3))
        self.btn_arm.clicked.connect(lambda: self.mainstack.setCurrentIndex(4))
        
        # Floor buttons
        self.btn_flr_G.clicked.connect(lambda: self.floorStack.setCurrentIndex(0))
        self.btn_flr_1.clicked.connect(lambda: self.floorStack.setCurrentIndex(1))
        self.btn_flr_2.clicked.connect(lambda: self.floorStack.setCurrentIndex(2))
        self.btn_flr_3.clicked.connect(lambda: self.floorStack.setCurrentIndex(3))
    
    def get_base_stylesheet(self):
        """Get the base stylesheet for the application - WITH LARGER FONTS"""
        return """
        /* Main application background */
        QWidget {
            background-color: #E0F7FA;
            font-family: Arial, sans-serif;
            font-size: 16px;  /* Base font size increased */
        }
        
        /* Buttons */
        QPushButton {
            background-color: #00C9A7;
            color: #FFFFFF;
            border: 3px solid #008B74;  /* Increased border thickness */
            border-radius: 10px;        /* Increased radius */
            padding: 12px 20px;         /* Increased padding */
            font-weight: bold;
            font-size: 18px;            /* Increased font size */
            min-height: 50px;           /* Minimum height */
        }
        
        QPushButton:hover {
            background-color: #00E6D3;
            border-color: #00C9A7;
        }
        
        QPushButton:pressed {
            background-color: #008B74;
        }
        
        /* Labels */
        QLabel {
            color: #00796B;
            font-size: 18px;  /* Increased font size */
        }
        
        QLabel:hover {
            color: #00C9A7;
        }
        """

# Main application class with resize handling
class SansarApp(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        
        # Store reference size
        self.reference_size = QtCore.QSize(1200, 800)
        
        # Setup initial scale factors
        self.width_scale = 1.0
        self.height_scale = 1.0
        
        # Install event filter for resize events
        self.installEventFilter(self)
        
        # Set size policies for widgets to handle resizing
        self.setup_size_policies()
        
        # Set window title
        self.setWindowTitle("SANSAR - Robot Receptionist")
    
    def eventFilter(self, obj, event):
        """Event filter to handle window resize events"""
        if obj is self and event.type() == QtCore.QEvent.Resize:
            self.calculate_scale_factors()
            self.update_font_sizes()
            self.update_icon_sizes()
        return super().eventFilter(obj, event)
    
    def calculate_scale_factors(self):
        """Calculate scale factors based on current window size"""
        self.width_scale = self.width() / self.reference_size.width()
        self.height_scale = self.height() / self.reference_size.height()
    
    def setup_size_policies(self):
        """Set size policies for widgets to handle resizing properly"""
        # You can add custom size policies for specific widgets here if needed
        pass
    
    def update_font_sizes(self):
        """Update font sizes based on scale factors"""
        # This could be implemented to dynamically adjust fonts on resize
        pass
    
    def update_icon_sizes(self):
        """Update icon sizes based on scale factors"""
        # This could be implemented to dynamically adjust icons on resize
        pass


# Entry point for the application
if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = SansarApp()
    window.show()
    sys.exit(app.exec_())