# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'robot_gui.ui'
#
# Created by: PyQt5 UI code generator 5.15.6
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(800, 600)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.btn_crowdnav = QtWidgets.QPushButton(self.centralwidget)
        self.btn_crowdnav.setGeometry(QtCore.QRect(70, 190, 89, 25))
        self.btn_crowdnav.setObjectName("btn_crowdnav")
        self.btn_multinav = QtWidgets.QPushButton(self.centralwidget)
        self.btn_multinav.setGeometry(QtCore.QRect(240, 190, 89, 25))
        self.btn_multinav.setObjectName("btn_multinav")
        self.btn_arm = QtWidgets.QPushButton(self.centralwidget)
        self.btn_arm.setGeometry(QtCore.QRect(400, 190, 89, 25))
        self.btn_arm.setObjectName("btn_arm")
        MainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.btn_crowdnav.setText(_translate("MainWindow", "Crowd"))
        self.btn_multinav.setText(_translate("MainWindow", "Multi"))
        self.btn_arm.setText(_translate("MainWindow", "Arm"))
