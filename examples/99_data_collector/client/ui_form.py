# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'ui_form.ui'
##
## Created by: Qt User Interface Compiler version 6.8.0
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QBrush, QColor, QConicalGradient, QCursor,
    QFont, QFontDatabase, QGradient, QIcon,
    QImage, QKeySequence, QLinearGradient, QPainter,
    QPalette, QPixmap, QRadialGradient, QTransform)
from PySide6.QtWidgets import (QApplication, QFrame, QGridLayout, QGroupBox,
    QHBoxLayout, QLabel, QLineEdit, QMainWindow,
    QMenuBar, QPushButton, QSizePolicy, QStatusBar,
    QVBoxLayout, QWidget)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(1112, 600)
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.horizontalLayout = QHBoxLayout(self.centralwidget)
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.frame = QFrame(self.centralwidget)
        self.frame.setObjectName(u"frame")
        self.frame.setFrameShape(QFrame.Shape.StyledPanel)
        self.frame.setFrameShadow(QFrame.Shadow.Raised)
        self.verticalLayout = QVBoxLayout(self.frame)
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.gridLayout = QGridLayout()
        self.gridLayout.setObjectName(u"gridLayout")
        self.label_2 = QLabel(self.frame)
        self.label_2.setObjectName(u"label_2")

        self.gridLayout.addWidget(self.label_2, 2, 0, 1, 1)

        self.label_4 = QLabel(self.frame)
        self.label_4.setObjectName(u"label_4")
        font = QFont()
        font.setBold(True)
        self.label_4.setFont(font)

        self.gridLayout.addWidget(self.label_4, 3, 0, 1, 2)

        self.label_6 = QLabel(self.frame)
        self.label_6.setObjectName(u"label_6")

        self.gridLayout.addWidget(self.label_6, 5, 0, 1, 1)

        self.LE_ServoOn = QLineEdit(self.frame)
        self.LE_ServoOn.setObjectName(u"LE_ServoOn")
        self.LE_ServoOn.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.LE_ServoOn.setReadOnly(True)

        self.gridLayout.addWidget(self.LE_ServoOn, 4, 1, 1, 1)

        self.LE_12v = QLineEdit(self.frame)
        self.LE_12v.setObjectName(u"LE_12v")
        self.LE_12v.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.LE_12v.setReadOnly(True)

        self.gridLayout.addWidget(self.LE_12v, 1, 1, 1, 1)

        self.label_3 = QLabel(self.frame)
        self.label_3.setObjectName(u"label_3")
        self.label_3.setFont(font)

        self.gridLayout.addWidget(self.label_3, 0, 0, 1, 2)

        self.label_5 = QLabel(self.frame)
        self.label_5.setObjectName(u"label_5")

        self.gridLayout.addWidget(self.label_5, 4, 0, 1, 1)

        self.label = QLabel(self.frame)
        self.label.setObjectName(u"label")

        self.gridLayout.addWidget(self.label, 1, 0, 1, 1)

        self.LE_48v = QLineEdit(self.frame)
        self.LE_48v.setObjectName(u"LE_48v")
        self.LE_48v.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.LE_48v.setReadOnly(True)

        self.gridLayout.addWidget(self.LE_48v, 2, 1, 1, 1)

        self.LE_ControlManager = QLineEdit(self.frame)
        self.LE_ControlManager.setObjectName(u"LE_ControlManager")
        self.LE_ControlManager.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.LE_ControlManager.setReadOnly(True)

        self.gridLayout.addWidget(self.LE_ControlManager, 5, 1, 1, 1)

        self.label_13 = QLabel(self.frame)
        self.label_13.setObjectName(u"label_13")

        self.gridLayout.addWidget(self.label_13, 6, 0, 1, 1)

        self.LE_Running = QLineEdit(self.frame)
        self.LE_Running.setObjectName(u"LE_Running")
        self.LE_Running.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.LE_Running.setReadOnly(True)

        self.gridLayout.addWidget(self.LE_Running, 6, 1, 1, 1)


        self.verticalLayout.addLayout(self.gridLayout)

        self.line_2 = QFrame(self.frame)
        self.line_2.setObjectName(u"line_2")
        self.line_2.setLineWidth(1)
        self.line_2.setFrameShape(QFrame.Shape.HLine)
        self.line_2.setFrameShadow(QFrame.Shadow.Sunken)

        self.verticalLayout.addWidget(self.line_2)

        self.PB_ResetMaster = QPushButton(self.frame)
        self.PB_ResetMaster.setObjectName(u"PB_ResetMaster")
        sizePolicy = QSizePolicy(QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.PB_ResetMaster.sizePolicy().hasHeightForWidth())
        self.PB_ResetMaster.setSizePolicy(sizePolicy)

        self.verticalLayout.addWidget(self.PB_ResetMaster)

        self.PB_ResetSlave = QPushButton(self.frame)
        self.PB_ResetSlave.setObjectName(u"PB_ResetSlave")
        sizePolicy.setHeightForWidth(self.PB_ResetSlave.sizePolicy().hasHeightForWidth())
        self.PB_ResetSlave.setSizePolicy(sizePolicy)

        self.verticalLayout.addWidget(self.PB_ResetSlave)

        self.line = QFrame(self.frame)
        self.line.setObjectName(u"line")
        self.line.setFrameShape(QFrame.Shape.HLine)
        self.line.setFrameShadow(QFrame.Shadow.Sunken)

        self.verticalLayout.addWidget(self.line)

        self.PB_Zero = QPushButton(self.frame)
        self.PB_Zero.setObjectName(u"PB_Zero")
        sizePolicy.setHeightForWidth(self.PB_Zero.sizePolicy().hasHeightForWidth())
        self.PB_Zero.setSizePolicy(sizePolicy)

        self.verticalLayout.addWidget(self.PB_Zero)

        self.PB_Ready = QPushButton(self.frame)
        self.PB_Ready.setObjectName(u"PB_Ready")
        sizePolicy.setHeightForWidth(self.PB_Ready.sizePolicy().hasHeightForWidth())
        self.PB_Ready.setSizePolicy(sizePolicy)

        self.verticalLayout.addWidget(self.PB_Ready)

        self.PB_StartTeleoperation = QPushButton(self.frame)
        self.PB_StartTeleoperation.setObjectName(u"PB_StartTeleoperation")
        sizePolicy.setHeightForWidth(self.PB_StartTeleoperation.sizePolicy().hasHeightForWidth())
        self.PB_StartTeleoperation.setSizePolicy(sizePolicy)

        self.verticalLayout.addWidget(self.PB_StartTeleoperation)

        self.PB_StopTeleoperation = QPushButton(self.frame)
        self.PB_StopTeleoperation.setObjectName(u"PB_StopTeleoperation")
        sizePolicy.setHeightForWidth(self.PB_StopTeleoperation.sizePolicy().hasHeightForWidth())
        self.PB_StopTeleoperation.setSizePolicy(sizePolicy)
        self.PB_StopTeleoperation.setStyleSheet(u"QPushButton {background-color: #f16a6f}")

        self.verticalLayout.addWidget(self.PB_StopTeleoperation)


        self.horizontalLayout.addWidget(self.frame)

        self.verticalLayout_2 = QVBoxLayout()
        self.verticalLayout_2.setObjectName(u"verticalLayout_2")
        self.horizontalLayout_2 = QHBoxLayout()
        self.horizontalLayout_2.setObjectName(u"horizontalLayout_2")
        self.verticalLayout_4 = QVBoxLayout()
        self.verticalLayout_4.setObjectName(u"verticalLayout_4")
        self.groupBox = QGroupBox(self.centralwidget)
        self.groupBox.setObjectName(u"groupBox")
        self.groupBox.setFont(font)
        self.horizontalLayout_3 = QHBoxLayout(self.groupBox)
        self.horizontalLayout_3.setObjectName(u"horizontalLayout_3")
        self.label_7 = QLabel(self.groupBox)
        self.label_7.setObjectName(u"label_7")
        font1 = QFont()
        font1.setBold(False)
        self.label_7.setFont(font1)

        self.horizontalLayout_3.addWidget(self.label_7)

        self.LE_UPCStorageFree = QLineEdit(self.groupBox)
        self.LE_UPCStorageFree.setObjectName(u"LE_UPCStorageFree")
        self.LE_UPCStorageFree.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.LE_UPCStorageFree.setReadOnly(True)

        self.horizontalLayout_3.addWidget(self.LE_UPCStorageFree)

        self.label_8 = QLabel(self.groupBox)
        self.label_8.setObjectName(u"label_8")
        self.label_8.setFont(font1)

        self.horizontalLayout_3.addWidget(self.label_8)

        self.LE_UPCStorageAvailable = QLineEdit(self.groupBox)
        self.LE_UPCStorageAvailable.setObjectName(u"LE_UPCStorageAvailable")
        self.LE_UPCStorageAvailable.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.LE_UPCStorageAvailable.setReadOnly(True)

        self.horizontalLayout_3.addWidget(self.LE_UPCStorageAvailable)

        self.label_9 = QLabel(self.groupBox)
        self.label_9.setObjectName(u"label_9")
        self.label_9.setFont(font1)

        self.horizontalLayout_3.addWidget(self.label_9)

        self.LE_UPCStorageCapacity = QLineEdit(self.groupBox)
        self.LE_UPCStorageCapacity.setObjectName(u"LE_UPCStorageCapacity")
        self.LE_UPCStorageCapacity.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.LE_UPCStorageCapacity.setReadOnly(True)

        self.horizontalLayout_3.addWidget(self.LE_UPCStorageCapacity)


        self.verticalLayout_4.addWidget(self.groupBox)

        self.gridLayout_3 = QGridLayout()
        self.gridLayout_3.setObjectName(u"gridLayout_3")
        self.label_11 = QLabel(self.centralwidget)
        self.label_11.setObjectName(u"label_11")

        self.gridLayout_3.addWidget(self.label_11, 2, 0, 1, 1)

        self.Pb_ResetEpisodeNumber = QPushButton(self.centralwidget)
        self.Pb_ResetEpisodeNumber.setObjectName(u"Pb_ResetEpisodeNumber")

        self.gridLayout_3.addWidget(self.Pb_ResetEpisodeNumber, 2, 2, 1, 1)

        self.LE_EpisodeName = QLineEdit(self.centralwidget)
        self.LE_EpisodeName.setObjectName(u"LE_EpisodeName")

        self.gridLayout_3.addWidget(self.LE_EpisodeName, 1, 1, 1, 2)

        self.label_10 = QLabel(self.centralwidget)
        self.label_10.setObjectName(u"label_10")

        self.gridLayout_3.addWidget(self.label_10, 1, 0, 1, 1)

        self.LE_EpisodeNumber = QLineEdit(self.centralwidget)
        self.LE_EpisodeNumber.setObjectName(u"LE_EpisodeNumber")

        self.gridLayout_3.addWidget(self.LE_EpisodeNumber, 2, 1, 1, 1)

        self.label_12 = QLabel(self.centralwidget)
        self.label_12.setObjectName(u"label_12")
        self.label_12.setFont(font)

        self.gridLayout_3.addWidget(self.label_12, 0, 0, 1, 3)

        self.gridLayout_3.setColumnStretch(0, 1)
        self.gridLayout_3.setColumnStretch(1, 4)

        self.verticalLayout_4.addLayout(self.gridLayout_3)


        self.horizontalLayout_2.addLayout(self.verticalLayout_4)

        self.PB_Close = QPushButton(self.centralwidget)
        self.PB_Close.setObjectName(u"PB_Close")
        sizePolicy.setHeightForWidth(self.PB_Close.sizePolicy().hasHeightForWidth())
        self.PB_Close.setSizePolicy(sizePolicy)

        self.horizontalLayout_2.addWidget(self.PB_Close)

        self.horizontalLayout_2.setStretch(0, 4)
        self.horizontalLayout_2.setStretch(1, 1)

        self.verticalLayout_2.addLayout(self.horizontalLayout_2)

        self.horizontalLayout_4 = QHBoxLayout()
        self.horizontalLayout_4.setObjectName(u"horizontalLayout_4")
        self.PB_StartRecording = QPushButton(self.centralwidget)
        self.PB_StartRecording.setObjectName(u"PB_StartRecording")
        sizePolicy.setHeightForWidth(self.PB_StartRecording.sizePolicy().hasHeightForWidth())
        self.PB_StartRecording.setSizePolicy(sizePolicy)

        self.horizontalLayout_4.addWidget(self.PB_StartRecording)

        self.PB_StopRecording = QPushButton(self.centralwidget)
        self.PB_StopRecording.setObjectName(u"PB_StopRecording")
        sizePolicy.setHeightForWidth(self.PB_StopRecording.sizePolicy().hasHeightForWidth())
        self.PB_StopRecording.setSizePolicy(sizePolicy)
        self.PB_StopRecording.setStyleSheet(u"QPushButton {background-color: #f16a6f}")

        self.horizontalLayout_4.addWidget(self.PB_StopRecording)

        self.LE_Recording = QLineEdit(self.centralwidget)
        self.LE_Recording.setObjectName(u"LE_Recording")
        sizePolicy1 = QSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        sizePolicy1.setHorizontalStretch(0)
        sizePolicy1.setVerticalStretch(0)
        sizePolicy1.setHeightForWidth(self.LE_Recording.sizePolicy().hasHeightForWidth())
        self.LE_Recording.setSizePolicy(sizePolicy1)
        self.LE_Recording.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.LE_Recording.setReadOnly(True)

        self.horizontalLayout_4.addWidget(self.LE_Recording)

        self.frame_2 = QFrame(self.centralwidget)
        self.frame_2.setObjectName(u"frame_2")
        self.frame_2.setFrameShape(QFrame.Shape.StyledPanel)
        self.frame_2.setFrameShadow(QFrame.Shadow.Raised)
        self.verticalLayout_5 = QVBoxLayout(self.frame_2)
        self.verticalLayout_5.setObjectName(u"verticalLayout_5")
        self.L_RecordingCount = QLabel(self.frame_2)
        self.L_RecordingCount.setObjectName(u"L_RecordingCount")
        self.L_RecordingCount.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.verticalLayout_5.addWidget(self.L_RecordingCount)


        self.horizontalLayout_4.addWidget(self.frame_2)

        self.horizontalLayout_4.setStretch(0, 1)
        self.horizontalLayout_4.setStretch(1, 1)
        self.horizontalLayout_4.setStretch(2, 1)
        self.horizontalLayout_4.setStretch(3, 1)

        self.verticalLayout_2.addLayout(self.horizontalLayout_4)

        self.horizontalLayout_5 = QHBoxLayout()
        self.horizontalLayout_5.setObjectName(u"horizontalLayout_5")
        self.frame_3 = QFrame(self.centralwidget)
        self.frame_3.setObjectName(u"frame_3")
        self.frame_3.setFrameShape(QFrame.Shape.StyledPanel)
        self.frame_3.setFrameShadow(QFrame.Shadow.Raised)

        self.horizontalLayout_5.addWidget(self.frame_3)

        self.frame_4 = QFrame(self.centralwidget)
        self.frame_4.setObjectName(u"frame_4")
        self.frame_4.setFrameShape(QFrame.Shape.StyledPanel)
        self.frame_4.setFrameShadow(QFrame.Shadow.Raised)
        self.gridLayout_2 = QGridLayout(self.frame_4)
        self.gridLayout_2.setObjectName(u"gridLayout_2")
        self.label_15 = QLabel(self.frame_4)
        self.label_15.setObjectName(u"label_15")
        self.label_15.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.gridLayout_2.addWidget(self.label_15, 1, 1, 1, 1)

        self.label_14 = QLabel(self.frame_4)
        self.label_14.setObjectName(u"label_14")
        self.label_14.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.gridLayout_2.addWidget(self.label_14, 0, 0, 1, 1)

        self.label_17 = QLabel(self.frame_4)
        self.label_17.setObjectName(u"label_17")
        self.label_17.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.gridLayout_2.addWidget(self.label_17, 1, 0, 1, 1)

        self.label_16 = QLabel(self.frame_4)
        self.label_16.setObjectName(u"label_16")
        self.label_16.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.gridLayout_2.addWidget(self.label_16, 0, 1, 1, 1)

        self.label_18 = QLabel(self.frame_4)
        self.label_18.setObjectName(u"label_18")
        self.label_18.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.gridLayout_2.addWidget(self.label_18, 0, 2, 1, 1)

        self.label_19 = QLabel(self.frame_4)
        self.label_19.setObjectName(u"label_19")
        self.label_19.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.gridLayout_2.addWidget(self.label_19, 1, 2, 1, 1)


        self.horizontalLayout_5.addWidget(self.frame_4)

        self.frame_5 = QFrame(self.centralwidget)
        self.frame_5.setObjectName(u"frame_5")
        self.frame_5.setFrameShape(QFrame.Shape.StyledPanel)
        self.frame_5.setFrameShadow(QFrame.Shadow.Raised)

        self.horizontalLayout_5.addWidget(self.frame_5)

        self.horizontalLayout_5.setStretch(0, 1)
        self.horizontalLayout_5.setStretch(1, 3)
        self.horizontalLayout_5.setStretch(2, 1)

        self.verticalLayout_2.addLayout(self.horizontalLayout_5)

        self.verticalLayout_2.setStretch(0, 1)
        self.verticalLayout_2.setStretch(1, 1)
        self.verticalLayout_2.setStretch(2, 3)

        self.horizontalLayout.addLayout(self.verticalLayout_2)

        self.horizontalLayout.setStretch(0, 1)
        self.horizontalLayout.setStretch(1, 5)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QMenuBar(MainWindow)
        self.menubar.setObjectName(u"menubar")
        self.menubar.setGeometry(QRect(0, 0, 1112, 22))
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QStatusBar(MainWindow)
        self.statusbar.setObjectName(u"statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)

        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"MainWindow", None))
        self.label_2.setText(QCoreApplication.translate("MainWindow", u"48v (Robot)", None))
        self.label_4.setText(QCoreApplication.translate("MainWindow", u"Robot", None))
        self.label_6.setText(QCoreApplication.translate("MainWindow", u"Control Manager", None))
        self.label_3.setText(QCoreApplication.translate("MainWindow", u"Power", None))
        self.label_5.setText(QCoreApplication.translate("MainWindow", u"Servo On", None))
        self.label.setText(QCoreApplication.translate("MainWindow", u"12v (Master Arm)", None))
        self.label_13.setText(QCoreApplication.translate("MainWindow", u"Running", None))
        self.PB_ResetMaster.setText(QCoreApplication.translate("MainWindow", u"Reset Master (MasterArm)", None))
        self.PB_ResetSlave.setText(QCoreApplication.translate("MainWindow", u"Reset Slave (Robot)", None))
        self.PB_Zero.setText(QCoreApplication.translate("MainWindow", u"Zero", None))
        self.PB_Ready.setText(QCoreApplication.translate("MainWindow", u"Ready", None))
        self.PB_StartTeleoperation.setText(QCoreApplication.translate("MainWindow", u"Start Teleoperation", None))
        self.PB_StopTeleoperation.setText(QCoreApplication.translate("MainWindow", u"Stop Teleoperation", None))
        self.groupBox.setTitle(QCoreApplication.translate("MainWindow", u"UPC Storage", None))
        self.label_7.setText(QCoreApplication.translate("MainWindow", u"Free", None))
        self.label_8.setText(QCoreApplication.translate("MainWindow", u"Available", None))
        self.label_9.setText(QCoreApplication.translate("MainWindow", u"Capacity", None))
        self.label_11.setText(QCoreApplication.translate("MainWindow", u"#", None))
        self.Pb_ResetEpisodeNumber.setText(QCoreApplication.translate("MainWindow", u"Reset", None))
        self.label_10.setText(QCoreApplication.translate("MainWindow", u"Name", None))
        self.label_12.setText(QCoreApplication.translate("MainWindow", u"Episode", None))
        self.PB_Close.setText(QCoreApplication.translate("MainWindow", u"Close", None))
        self.PB_StartRecording.setText(QCoreApplication.translate("MainWindow", u"Start\n"
"Recording", None))
        self.PB_StopRecording.setText(QCoreApplication.translate("MainWindow", u"Stop\n"
"Recording", None))
        self.L_RecordingCount.setText(QCoreApplication.translate("MainWindow", u"0", None))
        self.label_15.setText(QCoreApplication.translate("MainWindow", u"Image Placeholder", None))
        self.label_14.setText(QCoreApplication.translate("MainWindow", u"Image Placeholder", None))
        self.label_17.setText(QCoreApplication.translate("MainWindow", u"Image Placeholder", None))
        self.label_16.setText(QCoreApplication.translate("MainWindow", u"Image Placeholder", None))
        self.label_18.setText(QCoreApplication.translate("MainWindow", u"Image Placeholder", None))
        self.label_19.setText(QCoreApplication.translate("MainWindow", u"Image Placeholder", None))
    # retranslateUi

