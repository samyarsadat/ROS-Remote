# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'main_window.ui'
##
## Created by: Qt User Interface Compiler version 6.7.2
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QAction, QBrush, QColor, QConicalGradient,
    QCursor, QFont, QFontDatabase, QGradient,
    QIcon, QImage, QKeySequence, QLinearGradient,
    QPainter, QPalette, QPixmap, QRadialGradient,
    QTransform)
from PySide6.QtWidgets import (QApplication, QCheckBox, QComboBox, QFrame,
    QGridLayout, QGroupBox, QHBoxLayout, QLCDNumber,
    QLabel, QLayout, QLineEdit, QMainWindow,
    QMenu, QMenuBar, QPushButton, QSizePolicy,
    QSlider, QSpacerItem, QTabWidget, QTextBrowser,
    QVBoxLayout, QWidget)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(1024, 600)
        sizePolicy = QSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(MainWindow.sizePolicy().hasHeightForWidth())
        MainWindow.setSizePolicy(sizePolicy)
        MainWindow.setMinimumSize(QSize(1024, 600))
        MainWindow.setMaximumSize(QSize(1024, 600))
        self.actionRestart_Application = QAction(MainWindow)
        self.actionRestart_Application.setObjectName(u"actionRestart_Application")
        self.actionRestart_RPi = QAction(MainWindow)
        self.actionRestart_RPi.setObjectName(u"actionRestart_RPi")
        self.actionShutdown_System = QAction(MainWindow)
        self.actionShutdown_System.setObjectName(u"actionShutdown_System")
        self.actionCheck_Robot_Connection = QAction(MainWindow)
        self.actionCheck_Robot_Connection.setObjectName(u"actionCheck_Robot_Connection")
        self.actionCheck_Wi_Fi_Connection = QAction(MainWindow)
        self.actionCheck_Wi_Fi_Connection.setObjectName(u"actionCheck_Wi_Fi_Connection")
        self.actionCheck_WAN_Connection = QAction(MainWindow)
        self.actionCheck_WAN_Connection.setObjectName(u"actionCheck_WAN_Connection")
        self.actionJoystick = QAction(MainWindow)
        self.actionJoystick.setObjectName(u"actionJoystick")
        self.actionTest_Buttons_Switches = QAction(MainWindow)
        self.actionTest_Buttons_Switches.setObjectName(u"actionTest_Buttons_Switches")
        self.actionAbout = QAction(MainWindow)
        self.actionAbout.setObjectName(u"actionAbout")
        self.centralWidget = QWidget(MainWindow)
        self.centralWidget.setObjectName(u"centralWidget")
        self.centralWidget.setEnabled(True)
        self.gridLayout_1 = QGridLayout(self.centralWidget)
        self.gridLayout_1.setObjectName(u"gridLayout_1")
        self.gridLayout_1.setContentsMargins(0, 0, 0, 0)
        self.pages = QTabWidget(self.centralWidget)
        self.pages.setObjectName(u"pages")
        sizePolicy1 = QSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        sizePolicy1.setHorizontalStretch(0)
        sizePolicy1.setVerticalStretch(0)
        sizePolicy1.setHeightForWidth(self.pages.sizePolicy().hasHeightForWidth())
        self.pages.setSizePolicy(sizePolicy1)
        self.pages.setCursor(QCursor(Qt.CursorShape.ArrowCursor))
        self.pages.setTabsClosable(False)
        self.pages.setMovable(False)
        self.pages.setTabBarAutoHide(False)
        self.mainTab = QWidget()
        self.mainTab.setObjectName(u"mainTab")
        self.viewport = QLabel(self.mainTab)
        self.viewport.setObjectName(u"viewport")
        self.viewport.setGeometry(QRect(20, 20, 980, 430))
        sizePolicy.setHeightForWidth(self.viewport.sizePolicy().hasHeightForWidth())
        self.viewport.setSizePolicy(sizePolicy)
        self.viewport.setMinimumSize(QSize(980, 430))
        self.viewport.setMaximumSize(QSize(980, 430))
        self.viewport.setMouseTracking(False)
        self.viewport.setFrameShape(QFrame.Shape.Box)
        self.viewport.setFrameShadow(QFrame.Shadow.Sunken)
        self.viewport.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.viewportOptsBox = QGroupBox(self.mainTab)
        self.viewportOptsBox.setObjectName(u"viewportOptsBox")
        self.viewportOptsBox.setGeometry(QRect(20, 460, 360, 70))
        self.gridLayout_2 = QGridLayout(self.viewportOptsBox)
        self.gridLayout_2.setObjectName(u"gridLayout_2")
        self.viewportFpsLabel = QLabel(self.viewportOptsBox)
        self.viewportFpsLabel.setObjectName(u"viewportFpsLabel")

        self.gridLayout_2.addWidget(self.viewportFpsLabel, 0, 2, 1, 1)

        self.viewportFpsValue = QLineEdit(self.viewportOptsBox)
        self.viewportFpsValue.setObjectName(u"viewportFpsValue")
        sizePolicy2 = QSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)
        sizePolicy2.setHorizontalStretch(0)
        sizePolicy2.setVerticalStretch(0)
        sizePolicy2.setHeightForWidth(self.viewportFpsValue.sizePolicy().hasHeightForWidth())
        self.viewportFpsValue.setSizePolicy(sizePolicy2)
        font = QFont()
        font.setPointSize(12)
        self.viewportFpsValue.setFont(font)
        self.viewportFpsValue.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.viewportFpsValue.setFrame(True)
        self.viewportFpsValue.setDragEnabled(True)
        self.viewportFpsValue.setReadOnly(True)

        self.gridLayout_2.addWidget(self.viewportFpsValue, 0, 3, 1, 1)

        self.viewportSelector = QComboBox(self.viewportOptsBox)
        self.viewportSelector.addItem("")
        self.viewportSelector.addItem("")
        self.viewportSelector.addItem("")
        self.viewportSelector.setObjectName(u"viewportSelector")
        sizePolicy2.setHeightForWidth(self.viewportSelector.sizePolicy().hasHeightForWidth())
        self.viewportSelector.setSizePolicy(sizePolicy2)
        self.viewportSelector.setCursor(QCursor(Qt.CursorShape.CrossCursor))

        self.gridLayout_2.addWidget(self.viewportSelector, 0, 0, 1, 1)

        self.horizontalSpacer_1 = QSpacerItem(10, 20, QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Minimum)

        self.gridLayout_2.addItem(self.horizontalSpacer_1, 0, 1, 1, 1)

        self.camLedsBox = QGroupBox(self.mainTab)
        self.camLedsBox.setObjectName(u"camLedsBox")
        self.camLedsBox.setGeometry(QRect(400, 460, 375, 70))
        sizePolicy.setHeightForWidth(self.camLedsBox.sizePolicy().hasHeightForWidth())
        self.camLedsBox.setSizePolicy(sizePolicy)
        self.horizontalLayout_1 = QHBoxLayout(self.camLedsBox)
        self.horizontalLayout_1.setObjectName(u"horizontalLayout_1")
        self.horizontalLayout_1.setContentsMargins(12, -1, -1, -1)
        self.camLedsBrightnessSlider = QSlider(self.camLedsBox)
        self.camLedsBrightnessSlider.setObjectName(u"camLedsBrightnessSlider")
        self.camLedsBrightnessSlider.setMaximum(4095)
        self.camLedsBrightnessSlider.setSingleStep(4)
        self.camLedsBrightnessSlider.setOrientation(Qt.Orientation.Horizontal)

        self.horizontalLayout_1.addWidget(self.camLedsBrightnessSlider)

        self.camLedsBrightnessLabel = QLabel(self.camLedsBox)
        self.camLedsBrightnessLabel.setObjectName(u"camLedsBrightnessLabel")

        self.horizontalLayout_1.addWidget(self.camLedsBrightnessLabel)

        self.horizontalSpacer_2 = QSpacerItem(10, 20, QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Minimum)

        self.horizontalLayout_1.addItem(self.horizontalSpacer_2)

        self.camLed1Check = QCheckBox(self.camLedsBox)
        self.camLed1Check.setObjectName(u"camLed1Check")

        self.horizontalLayout_1.addWidget(self.camLed1Check)

        self.camLed2Check = QCheckBox(self.camLedsBox)
        self.camLed2Check.setObjectName(u"camLed2Check")

        self.horizontalLayout_1.addWidget(self.camLed2Check)

        self.camLed3Check = QCheckBox(self.camLedsBox)
        self.camLed3Check.setObjectName(u"camLed3Check")

        self.horizontalLayout_1.addWidget(self.camLed3Check)

        self.camLed4Check = QCheckBox(self.camLedsBox)
        self.camLed4Check.setObjectName(u"camLed4Check")

        self.horizontalLayout_1.addWidget(self.camLed4Check)

        self.telemetryBox = QGroupBox(self.mainTab)
        self.telemetryBox.setObjectName(u"telemetryBox")
        self.telemetryBox.setGeometry(QRect(795, 460, 204, 70))
        sizePolicy.setHeightForWidth(self.telemetryBox.sizePolicy().hasHeightForWidth())
        self.telemetryBox.setSizePolicy(sizePolicy)
        self.horizontalLayout_2 = QHBoxLayout(self.telemetryBox)
        self.horizontalLayout_2.setObjectName(u"horizontalLayout_2")
        self.horizontalLayout_2.setContentsMargins(12, -1, 12, -1)
        self.teleBatteryLabel = QLabel(self.telemetryBox)
        self.teleBatteryLabel.setObjectName(u"teleBatteryLabel")
        sizePolicy3 = QSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        sizePolicy3.setHorizontalStretch(0)
        sizePolicy3.setVerticalStretch(0)
        sizePolicy3.setHeightForWidth(self.teleBatteryLabel.sizePolicy().hasHeightForWidth())
        self.teleBatteryLabel.setSizePolicy(sizePolicy3)

        self.horizontalLayout_2.addWidget(self.teleBatteryLabel)

        self.teleBatteryValue = QLineEdit(self.telemetryBox)
        self.teleBatteryValue.setObjectName(u"teleBatteryValue")
        sizePolicy4 = QSizePolicy(QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Fixed)
        sizePolicy4.setHorizontalStretch(0)
        sizePolicy4.setVerticalStretch(0)
        sizePolicy4.setHeightForWidth(self.teleBatteryValue.sizePolicy().hasHeightForWidth())
        self.teleBatteryValue.setSizePolicy(sizePolicy4)
        self.teleBatteryValue.setFont(font)
        self.teleBatteryValue.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.teleBatteryValue.setFrame(True)
        self.teleBatteryValue.setDragEnabled(True)
        self.teleBatteryValue.setReadOnly(True)

        self.horizontalLayout_2.addWidget(self.teleBatteryValue)

        self.teleBatteryUnits = QLabel(self.telemetryBox)
        self.teleBatteryUnits.setObjectName(u"teleBatteryUnits")

        self.horizontalLayout_2.addWidget(self.teleBatteryUnits)

        self.pages.addTab(self.mainTab, "")
        self.sensorsTab = QWidget()
        self.sensorsTab.setObjectName(u"sensorsTab")
        self.microswSensBox = QGroupBox(self.sensorsTab)
        self.microswSensBox.setObjectName(u"microswSensBox")
        self.microswSensBox.setGeometry(QRect(462, 20, 195, 110))
        self.gridLayout_3 = QGridLayout(self.microswSensBox)
        self.gridLayout_3.setObjectName(u"gridLayout_3")
        self.microswSensIndicatorFL = QLabel(self.microswSensBox)
        self.microswSensIndicatorFL.setObjectName(u"microswSensIndicatorFL")
        sizePolicy1.setHeightForWidth(self.microswSensIndicatorFL.sizePolicy().hasHeightForWidth())
        self.microswSensIndicatorFL.setSizePolicy(sizePolicy1)
        self.microswSensIndicatorFL.setAutoFillBackground(False)
        self.microswSensIndicatorFL.setFrameShape(QFrame.Shape.Box)
        self.microswSensIndicatorFL.setFrameShadow(QFrame.Shadow.Plain)
        self.microswSensIndicatorFL.setLineWidth(1)
        self.microswSensIndicatorFL.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.gridLayout_3.addWidget(self.microswSensIndicatorFL, 0, 0, 1, 1)

        self.microswSensIndicatorBL = QLabel(self.microswSensBox)
        self.microswSensIndicatorBL.setObjectName(u"microswSensIndicatorBL")
        sizePolicy1.setHeightForWidth(self.microswSensIndicatorBL.sizePolicy().hasHeightForWidth())
        self.microswSensIndicatorBL.setSizePolicy(sizePolicy1)
        self.microswSensIndicatorBL.setAutoFillBackground(False)
        self.microswSensIndicatorBL.setFrameShape(QFrame.Shape.Box)
        self.microswSensIndicatorBL.setFrameShadow(QFrame.Shadow.Plain)
        self.microswSensIndicatorBL.setLineWidth(1)
        self.microswSensIndicatorBL.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.gridLayout_3.addWidget(self.microswSensIndicatorBL, 2, 0, 1, 1)

        self.microswSensIndicatorFR = QLabel(self.microswSensBox)
        self.microswSensIndicatorFR.setObjectName(u"microswSensIndicatorFR")
        sizePolicy1.setHeightForWidth(self.microswSensIndicatorFR.sizePolicy().hasHeightForWidth())
        self.microswSensIndicatorFR.setSizePolicy(sizePolicy1)
        self.microswSensIndicatorFR.setAutoFillBackground(False)
        self.microswSensIndicatorFR.setFrameShape(QFrame.Shape.Box)
        self.microswSensIndicatorFR.setFrameShadow(QFrame.Shadow.Plain)
        self.microswSensIndicatorFR.setLineWidth(1)
        self.microswSensIndicatorFR.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.gridLayout_3.addWidget(self.microswSensIndicatorFR, 0, 1, 1, 1)

        self.microswSensIndicatorBR = QLabel(self.microswSensBox)
        self.microswSensIndicatorBR.setObjectName(u"microswSensIndicatorBR")
        sizePolicy1.setHeightForWidth(self.microswSensIndicatorBR.sizePolicy().hasHeightForWidth())
        self.microswSensIndicatorBR.setSizePolicy(sizePolicy1)
        self.microswSensIndicatorBR.setAutoFillBackground(False)
        self.microswSensIndicatorBR.setFrameShape(QFrame.Shape.Box)
        self.microswSensIndicatorBR.setFrameShadow(QFrame.Shadow.Plain)
        self.microswSensIndicatorBR.setLineWidth(1)
        self.microswSensIndicatorBR.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.gridLayout_3.addWidget(self.microswSensIndicatorBR, 2, 1, 1, 1)

        self.imuSensBox = QGroupBox(self.sensorsTab)
        self.imuSensBox.setObjectName(u"imuSensBox")
        self.imuSensBox.setGeometry(QRect(245, 215, 195, 305))
        self.verticalLayout_1 = QVBoxLayout(self.imuSensBox)
        self.verticalLayout_1.setObjectName(u"verticalLayout_1")
        self.imuSensAccelXBox = QHBoxLayout()
        self.imuSensAccelXBox.setObjectName(u"imuSensAccelXBox")
        self.imuSensAccelXLabel = QLabel(self.imuSensBox)
        self.imuSensAccelXLabel.setObjectName(u"imuSensAccelXLabel")
        font1 = QFont()
        font1.setPointSize(11)
        self.imuSensAccelXLabel.setFont(font1)
        self.imuSensAccelXLabel.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.imuSensAccelXBox.addWidget(self.imuSensAccelXLabel)

        self.horizontalSpacer_3 = QSpacerItem(5, 20, QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Minimum)

        self.imuSensAccelXBox.addItem(self.horizontalSpacer_3)

        self.imuSensAccelXValue = QLineEdit(self.imuSensBox)
        self.imuSensAccelXValue.setObjectName(u"imuSensAccelXValue")
        sizePolicy4.setHeightForWidth(self.imuSensAccelXValue.sizePolicy().hasHeightForWidth())
        self.imuSensAccelXValue.setSizePolicy(sizePolicy4)
        self.imuSensAccelXValue.setFont(font)
        self.imuSensAccelXValue.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.imuSensAccelXValue.setFrame(True)
        self.imuSensAccelXValue.setDragEnabled(True)
        self.imuSensAccelXValue.setReadOnly(True)

        self.imuSensAccelXBox.addWidget(self.imuSensAccelXValue)

        self.imuSensAccelXUnit = QLabel(self.imuSensBox)
        self.imuSensAccelXUnit.setObjectName(u"imuSensAccelXUnit")

        self.imuSensAccelXBox.addWidget(self.imuSensAccelXUnit)


        self.verticalLayout_1.addLayout(self.imuSensAccelXBox)

        self.imuSensAccelYBox = QHBoxLayout()
        self.imuSensAccelYBox.setObjectName(u"imuSensAccelYBox")
        self.imuSensAccelYLabel = QLabel(self.imuSensBox)
        self.imuSensAccelYLabel.setObjectName(u"imuSensAccelYLabel")
        self.imuSensAccelYLabel.setFont(font1)
        self.imuSensAccelYLabel.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.imuSensAccelYBox.addWidget(self.imuSensAccelYLabel)

        self.horizontalSpacer_4 = QSpacerItem(5, 20, QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Minimum)

        self.imuSensAccelYBox.addItem(self.horizontalSpacer_4)

        self.imuSensAccelYValue = QLineEdit(self.imuSensBox)
        self.imuSensAccelYValue.setObjectName(u"imuSensAccelYValue")
        sizePolicy4.setHeightForWidth(self.imuSensAccelYValue.sizePolicy().hasHeightForWidth())
        self.imuSensAccelYValue.setSizePolicy(sizePolicy4)
        self.imuSensAccelYValue.setFont(font)
        self.imuSensAccelYValue.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.imuSensAccelYValue.setFrame(True)
        self.imuSensAccelYValue.setDragEnabled(True)
        self.imuSensAccelYValue.setReadOnly(True)

        self.imuSensAccelYBox.addWidget(self.imuSensAccelYValue)

        self.imuSensAccelYUnit = QLabel(self.imuSensBox)
        self.imuSensAccelYUnit.setObjectName(u"imuSensAccelYUnit")

        self.imuSensAccelYBox.addWidget(self.imuSensAccelYUnit)


        self.verticalLayout_1.addLayout(self.imuSensAccelYBox)

        self.imuSensAccelZBox = QHBoxLayout()
        self.imuSensAccelZBox.setObjectName(u"imuSensAccelZBox")
        self.imuSensAccelZLabel = QLabel(self.imuSensBox)
        self.imuSensAccelZLabel.setObjectName(u"imuSensAccelZLabel")
        self.imuSensAccelZLabel.setFont(font1)
        self.imuSensAccelZLabel.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.imuSensAccelZBox.addWidget(self.imuSensAccelZLabel)

        self.horizontalSpacer_5 = QSpacerItem(5, 20, QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Minimum)

        self.imuSensAccelZBox.addItem(self.horizontalSpacer_5)

        self.imuSensAccelZValue = QLineEdit(self.imuSensBox)
        self.imuSensAccelZValue.setObjectName(u"imuSensAccelZValue")
        sizePolicy4.setHeightForWidth(self.imuSensAccelZValue.sizePolicy().hasHeightForWidth())
        self.imuSensAccelZValue.setSizePolicy(sizePolicy4)
        self.imuSensAccelZValue.setFont(font)
        self.imuSensAccelZValue.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.imuSensAccelZValue.setFrame(True)
        self.imuSensAccelZValue.setDragEnabled(True)
        self.imuSensAccelZValue.setReadOnly(True)

        self.imuSensAccelZBox.addWidget(self.imuSensAccelZValue)

        self.imuSensAccelZUnit = QLabel(self.imuSensBox)
        self.imuSensAccelZUnit.setObjectName(u"imuSensAccelZUnit")

        self.imuSensAccelZBox.addWidget(self.imuSensAccelZUnit)


        self.verticalLayout_1.addLayout(self.imuSensAccelZBox)

        self.verticalSpacer_1 = QSpacerItem(20, 8, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Fixed)

        self.verticalLayout_1.addItem(self.verticalSpacer_1)

        self.imuSensGyroXBox = QHBoxLayout()
        self.imuSensGyroXBox.setObjectName(u"imuSensGyroXBox")
        self.imuSensGyroXLabel = QLabel(self.imuSensBox)
        self.imuSensGyroXLabel.setObjectName(u"imuSensGyroXLabel")
        self.imuSensGyroXLabel.setFont(font1)
        self.imuSensGyroXLabel.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.imuSensGyroXBox.addWidget(self.imuSensGyroXLabel)

        self.horizontalSpacer_6 = QSpacerItem(5, 20, QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Minimum)

        self.imuSensGyroXBox.addItem(self.horizontalSpacer_6)

        self.imuSensGyroXValue = QLineEdit(self.imuSensBox)
        self.imuSensGyroXValue.setObjectName(u"imuSensGyroXValue")
        sizePolicy4.setHeightForWidth(self.imuSensGyroXValue.sizePolicy().hasHeightForWidth())
        self.imuSensGyroXValue.setSizePolicy(sizePolicy4)
        self.imuSensGyroXValue.setFont(font)
        self.imuSensGyroXValue.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.imuSensGyroXValue.setFrame(True)
        self.imuSensGyroXValue.setDragEnabled(True)
        self.imuSensGyroXValue.setReadOnly(True)

        self.imuSensGyroXBox.addWidget(self.imuSensGyroXValue)

        self.imuSensGyroXUnit = QLabel(self.imuSensBox)
        self.imuSensGyroXUnit.setObjectName(u"imuSensGyroXUnit")

        self.imuSensGyroXBox.addWidget(self.imuSensGyroXUnit)


        self.verticalLayout_1.addLayout(self.imuSensGyroXBox)

        self.imuSensGyroYBox = QHBoxLayout()
        self.imuSensGyroYBox.setObjectName(u"imuSensGyroYBox")
        self.imuSensGyroYLabel = QLabel(self.imuSensBox)
        self.imuSensGyroYLabel.setObjectName(u"imuSensGyroYLabel")
        self.imuSensGyroYLabel.setFont(font1)
        self.imuSensGyroYLabel.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.imuSensGyroYBox.addWidget(self.imuSensGyroYLabel)

        self.horizontalSpacer_7 = QSpacerItem(5, 20, QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Minimum)

        self.imuSensGyroYBox.addItem(self.horizontalSpacer_7)

        self.imuSensGyroYValue = QLineEdit(self.imuSensBox)
        self.imuSensGyroYValue.setObjectName(u"imuSensGyroYValue")
        sizePolicy4.setHeightForWidth(self.imuSensGyroYValue.sizePolicy().hasHeightForWidth())
        self.imuSensGyroYValue.setSizePolicy(sizePolicy4)
        self.imuSensGyroYValue.setFont(font)
        self.imuSensGyroYValue.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.imuSensGyroYValue.setFrame(True)
        self.imuSensGyroYValue.setDragEnabled(True)
        self.imuSensGyroYValue.setReadOnly(True)

        self.imuSensGyroYBox.addWidget(self.imuSensGyroYValue)

        self.imuSensGyroYUnit = QLabel(self.imuSensBox)
        self.imuSensGyroYUnit.setObjectName(u"imuSensGyroYUnit")

        self.imuSensGyroYBox.addWidget(self.imuSensGyroYUnit)


        self.verticalLayout_1.addLayout(self.imuSensGyroYBox)

        self.imuSensGyroZBox = QHBoxLayout()
        self.imuSensGyroZBox.setObjectName(u"imuSensGyroZBox")
        self.imuSensGyroZLabel = QLabel(self.imuSensBox)
        self.imuSensGyroZLabel.setObjectName(u"imuSensGyroZLabel")
        self.imuSensGyroZLabel.setFont(font1)
        self.imuSensGyroZLabel.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.imuSensGyroZBox.addWidget(self.imuSensGyroZLabel)

        self.horizontalSpacer_8 = QSpacerItem(5, 20, QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Minimum)

        self.imuSensGyroZBox.addItem(self.horizontalSpacer_8)

        self.imuSensGyroZValue = QLineEdit(self.imuSensBox)
        self.imuSensGyroZValue.setObjectName(u"imuSensGyroZValue")
        sizePolicy4.setHeightForWidth(self.imuSensGyroZValue.sizePolicy().hasHeightForWidth())
        self.imuSensGyroZValue.setSizePolicy(sizePolicy4)
        self.imuSensGyroZValue.setFont(font)
        self.imuSensGyroZValue.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.imuSensGyroZValue.setFrame(True)
        self.imuSensGyroZValue.setDragEnabled(True)
        self.imuSensGyroZValue.setReadOnly(True)

        self.imuSensGyroZBox.addWidget(self.imuSensGyroZValue)

        self.imuSensGyroZUnit = QLabel(self.imuSensBox)
        self.imuSensGyroZUnit.setObjectName(u"imuSensGyroZUnit")

        self.imuSensGyroZBox.addWidget(self.imuSensGyroZUnit)


        self.verticalLayout_1.addLayout(self.imuSensGyroZBox)

        self.verticalSpacer_2 = QSpacerItem(20, 8, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Fixed)

        self.verticalLayout_1.addItem(self.verticalSpacer_2)

        self.imuSensCompXBox = QHBoxLayout()
        self.imuSensCompXBox.setObjectName(u"imuSensCompXBox")
        self.imuSensCompXLabel = QLabel(self.imuSensBox)
        self.imuSensCompXLabel.setObjectName(u"imuSensCompXLabel")
        self.imuSensCompXLabel.setFont(font1)
        self.imuSensCompXLabel.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.imuSensCompXBox.addWidget(self.imuSensCompXLabel)

        self.horizontalSpacer_9 = QSpacerItem(5, 20, QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Minimum)

        self.imuSensCompXBox.addItem(self.horizontalSpacer_9)

        self.imuSensCompXValue = QLineEdit(self.imuSensBox)
        self.imuSensCompXValue.setObjectName(u"imuSensCompXValue")
        sizePolicy4.setHeightForWidth(self.imuSensCompXValue.sizePolicy().hasHeightForWidth())
        self.imuSensCompXValue.setSizePolicy(sizePolicy4)
        self.imuSensCompXValue.setFont(font)
        self.imuSensCompXValue.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.imuSensCompXValue.setFrame(True)
        self.imuSensCompXValue.setDragEnabled(True)
        self.imuSensCompXValue.setReadOnly(True)

        self.imuSensCompXBox.addWidget(self.imuSensCompXValue)

        self.imuSensCompXUnit = QLabel(self.imuSensBox)
        self.imuSensCompXUnit.setObjectName(u"imuSensCompXUnit")

        self.imuSensCompXBox.addWidget(self.imuSensCompXUnit)


        self.verticalLayout_1.addLayout(self.imuSensCompXBox)

        self.imuSensCompYBox = QHBoxLayout()
        self.imuSensCompYBox.setObjectName(u"imuSensCompYBox")
        self.imuSensCompYLabel = QLabel(self.imuSensBox)
        self.imuSensCompYLabel.setObjectName(u"imuSensCompYLabel")
        self.imuSensCompYLabel.setFont(font1)
        self.imuSensCompYLabel.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.imuSensCompYBox.addWidget(self.imuSensCompYLabel)

        self.horizontalSpacer_10 = QSpacerItem(5, 20, QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Minimum)

        self.imuSensCompYBox.addItem(self.horizontalSpacer_10)

        self.imuSensCompYValue = QLineEdit(self.imuSensBox)
        self.imuSensCompYValue.setObjectName(u"imuSensCompYValue")
        sizePolicy4.setHeightForWidth(self.imuSensCompYValue.sizePolicy().hasHeightForWidth())
        self.imuSensCompYValue.setSizePolicy(sizePolicy4)
        self.imuSensCompYValue.setFont(font)
        self.imuSensCompYValue.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.imuSensCompYValue.setFrame(True)
        self.imuSensCompYValue.setDragEnabled(True)
        self.imuSensCompYValue.setReadOnly(True)

        self.imuSensCompYBox.addWidget(self.imuSensCompYValue)

        self.imuSensCompYUnit = QLabel(self.imuSensBox)
        self.imuSensCompYUnit.setObjectName(u"imuSensCompYUnit")

        self.imuSensCompYBox.addWidget(self.imuSensCompYUnit)


        self.verticalLayout_1.addLayout(self.imuSensCompYBox)

        self.imuSensCompZBox = QHBoxLayout()
        self.imuSensCompZBox.setObjectName(u"imuSensCompZBox")
        self.imuSensCompZLabel = QLabel(self.imuSensBox)
        self.imuSensCompZLabel.setObjectName(u"imuSensCompZLabel")
        self.imuSensCompZLabel.setFont(font1)
        self.imuSensCompZLabel.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.imuSensCompZBox.addWidget(self.imuSensCompZLabel)

        self.horizontalSpacer_11 = QSpacerItem(5, 20, QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Minimum)

        self.imuSensCompZBox.addItem(self.horizontalSpacer_11)

        self.imuSensCompZValue = QLineEdit(self.imuSensBox)
        self.imuSensCompZValue.setObjectName(u"imuSensCompZValue")
        sizePolicy4.setHeightForWidth(self.imuSensCompZValue.sizePolicy().hasHeightForWidth())
        self.imuSensCompZValue.setSizePolicy(sizePolicy4)
        self.imuSensCompZValue.setFont(font)
        self.imuSensCompZValue.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.imuSensCompZValue.setFrame(True)
        self.imuSensCompZValue.setDragEnabled(True)
        self.imuSensCompZValue.setReadOnly(True)

        self.imuSensCompZBox.addWidget(self.imuSensCompZValue)

        self.imuSensCompZUnit = QLabel(self.imuSensBox)
        self.imuSensCompZUnit.setObjectName(u"imuSensCompZUnit")

        self.imuSensCompZBox.addWidget(self.imuSensCompZUnit)


        self.verticalLayout_1.addLayout(self.imuSensCompZBox)

        self.envSensBox = QGroupBox(self.sensorsTab)
        self.envSensBox.setObjectName(u"envSensBox")
        self.envSensBox.setGeometry(QRect(28, 185, 195, 210))
        sizePolicy.setHeightForWidth(self.envSensBox.sizePolicy().hasHeightForWidth())
        self.envSensBox.setSizePolicy(sizePolicy)
        self.horizontalLayout_3 = QHBoxLayout(self.envSensBox)
        self.horizontalLayout_3.setObjectName(u"horizontalLayout_3")
        self.envSensLabelsBox = QVBoxLayout()
        self.envSensLabelsBox.setObjectName(u"envSensLabelsBox")
        self.envSensTempLabel = QLabel(self.envSensBox)
        self.envSensTempLabel.setObjectName(u"envSensTempLabel")
        sizePolicy5 = QSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Preferred)
        sizePolicy5.setHorizontalStretch(0)
        sizePolicy5.setVerticalStretch(0)
        sizePolicy5.setHeightForWidth(self.envSensTempLabel.sizePolicy().hasHeightForWidth())
        self.envSensTempLabel.setSizePolicy(sizePolicy5)

        self.envSensLabelsBox.addWidget(self.envSensTempLabel)

        self.envSensHumLabel = QLabel(self.envSensBox)
        self.envSensHumLabel.setObjectName(u"envSensHumLabel")

        self.envSensLabelsBox.addWidget(self.envSensHumLabel)

        self.envSensImuTempLabel = QLabel(self.envSensBox)
        self.envSensImuTempLabel.setObjectName(u"envSensImuTempLabel")

        self.envSensLabelsBox.addWidget(self.envSensImuTempLabel)

        self.envSensPicoATempLabel = QLabel(self.envSensBox)
        self.envSensPicoATempLabel.setObjectName(u"envSensPicoATempLabel")

        self.envSensLabelsBox.addWidget(self.envSensPicoATempLabel)

        self.envSensPicoBTempLabel = QLabel(self.envSensBox)
        self.envSensPicoBTempLabel.setObjectName(u"envSensPicoBTempLabel")

        self.envSensLabelsBox.addWidget(self.envSensPicoBTempLabel)

        self.envSensPiTempLabel = QLabel(self.envSensBox)
        self.envSensPiTempLabel.setObjectName(u"envSensPiTempLabel")

        self.envSensLabelsBox.addWidget(self.envSensPiTempLabel)


        self.horizontalLayout_3.addLayout(self.envSensLabelsBox)

        self.horizontalSpacer_12 = QSpacerItem(2, 20, QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Minimum)

        self.horizontalLayout_3.addItem(self.horizontalSpacer_12)

        self.envSensValuesBox = QVBoxLayout()
        self.envSensValuesBox.setObjectName(u"envSensValuesBox")
        self.envSensValuesBox.setSizeConstraint(QLayout.SizeConstraint.SetDefaultConstraint)
        self.envSensTempValue = QLineEdit(self.envSensBox)
        self.envSensTempValue.setObjectName(u"envSensTempValue")
        sizePolicy2.setHeightForWidth(self.envSensTempValue.sizePolicy().hasHeightForWidth())
        self.envSensTempValue.setSizePolicy(sizePolicy2)
        self.envSensTempValue.setFont(font)
        self.envSensTempValue.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.envSensTempValue.setFrame(True)
        self.envSensTempValue.setDragEnabled(True)
        self.envSensTempValue.setReadOnly(True)

        self.envSensValuesBox.addWidget(self.envSensTempValue)

        self.envSensHumValue = QLineEdit(self.envSensBox)
        self.envSensHumValue.setObjectName(u"envSensHumValue")
        sizePolicy2.setHeightForWidth(self.envSensHumValue.sizePolicy().hasHeightForWidth())
        self.envSensHumValue.setSizePolicy(sizePolicy2)
        self.envSensHumValue.setFont(font)
        self.envSensHumValue.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.envSensHumValue.setFrame(True)
        self.envSensHumValue.setDragEnabled(True)
        self.envSensHumValue.setReadOnly(True)

        self.envSensValuesBox.addWidget(self.envSensHumValue)

        self.envSensImuTempValue = QLineEdit(self.envSensBox)
        self.envSensImuTempValue.setObjectName(u"envSensImuTempValue")
        sizePolicy2.setHeightForWidth(self.envSensImuTempValue.sizePolicy().hasHeightForWidth())
        self.envSensImuTempValue.setSizePolicy(sizePolicy2)
        self.envSensImuTempValue.setFont(font)
        self.envSensImuTempValue.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.envSensImuTempValue.setFrame(True)
        self.envSensImuTempValue.setDragEnabled(True)
        self.envSensImuTempValue.setReadOnly(True)

        self.envSensValuesBox.addWidget(self.envSensImuTempValue)

        self.envSensPicoATempValue = QLineEdit(self.envSensBox)
        self.envSensPicoATempValue.setObjectName(u"envSensPicoATempValue")
        sizePolicy2.setHeightForWidth(self.envSensPicoATempValue.sizePolicy().hasHeightForWidth())
        self.envSensPicoATempValue.setSizePolicy(sizePolicy2)
        self.envSensPicoATempValue.setFont(font)
        self.envSensPicoATempValue.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.envSensPicoATempValue.setFrame(True)
        self.envSensPicoATempValue.setDragEnabled(True)
        self.envSensPicoATempValue.setReadOnly(True)

        self.envSensValuesBox.addWidget(self.envSensPicoATempValue)

        self.envSensPicoBTempValue = QLineEdit(self.envSensBox)
        self.envSensPicoBTempValue.setObjectName(u"envSensPicoBTempValue")
        sizePolicy2.setHeightForWidth(self.envSensPicoBTempValue.sizePolicy().hasHeightForWidth())
        self.envSensPicoBTempValue.setSizePolicy(sizePolicy2)
        self.envSensPicoBTempValue.setFont(font)
        self.envSensPicoBTempValue.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.envSensPicoBTempValue.setFrame(True)
        self.envSensPicoBTempValue.setDragEnabled(True)
        self.envSensPicoBTempValue.setReadOnly(True)

        self.envSensValuesBox.addWidget(self.envSensPicoBTempValue)

        self.envSensPiTempValue = QLineEdit(self.envSensBox)
        self.envSensPiTempValue.setObjectName(u"envSensPiTempValue")
        sizePolicy2.setHeightForWidth(self.envSensPiTempValue.sizePolicy().hasHeightForWidth())
        self.envSensPiTempValue.setSizePolicy(sizePolicy2)
        self.envSensPiTempValue.setFont(font)
        self.envSensPiTempValue.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.envSensPiTempValue.setFrame(True)
        self.envSensPiTempValue.setDragEnabled(True)
        self.envSensPiTempValue.setReadOnly(True)

        self.envSensValuesBox.addWidget(self.envSensPiTempValue)


        self.horizontalLayout_3.addLayout(self.envSensValuesBox)

        self.envSensUnitsBox = QVBoxLayout()
        self.envSensUnitsBox.setObjectName(u"envSensUnitsBox")
        self.envSensTempUnit = QLabel(self.envSensBox)
        self.envSensTempUnit.setObjectName(u"envSensTempUnit")

        self.envSensUnitsBox.addWidget(self.envSensTempUnit)

        self.envSensHumUnit = QLabel(self.envSensBox)
        self.envSensHumUnit.setObjectName(u"envSensHumUnit")

        self.envSensUnitsBox.addWidget(self.envSensHumUnit)

        self.envSensImuTempUnit = QLabel(self.envSensBox)
        self.envSensImuTempUnit.setObjectName(u"envSensImuTempUnit")

        self.envSensUnitsBox.addWidget(self.envSensImuTempUnit)

        self.envSensPicoATempUnit = QLabel(self.envSensBox)
        self.envSensPicoATempUnit.setObjectName(u"envSensPicoATempUnit")

        self.envSensUnitsBox.addWidget(self.envSensPicoATempUnit)

        self.envSensPicoBTempUnit = QLabel(self.envSensBox)
        self.envSensPicoBTempUnit.setObjectName(u"envSensPicoBTempUnit")

        self.envSensUnitsBox.addWidget(self.envSensPicoBTempUnit)

        self.envSensPiTempUnit = QLabel(self.envSensBox)
        self.envSensPiTempUnit.setObjectName(u"envSensPiTempUnit")

        self.envSensUnitsBox.addWidget(self.envSensPiTempUnit)


        self.horizontalLayout_3.addLayout(self.envSensUnitsBox)

        self.cliffSensBox = QGroupBox(self.sensorsTab)
        self.cliffSensBox.setObjectName(u"cliffSensBox")
        self.cliffSensBox.setGeometry(QRect(245, 20, 195, 180))
        sizePolicy5.setHeightForWidth(self.cliffSensBox.sizePolicy().hasHeightForWidth())
        self.cliffSensBox.setSizePolicy(sizePolicy5)
        self.gridLayout_4 = QGridLayout(self.cliffSensBox)
        self.gridLayout_4.setObjectName(u"gridLayout_4")
        self.cliffSensIndicatorF2 = QLabel(self.cliffSensBox)
        self.cliffSensIndicatorF2.setObjectName(u"cliffSensIndicatorF2")
        sizePolicy1.setHeightForWidth(self.cliffSensIndicatorF2.sizePolicy().hasHeightForWidth())
        self.cliffSensIndicatorF2.setSizePolicy(sizePolicy1)
        self.cliffSensIndicatorF2.setAutoFillBackground(False)
        self.cliffSensIndicatorF2.setFrameShape(QFrame.Shape.Box)
        self.cliffSensIndicatorF2.setFrameShadow(QFrame.Shadow.Plain)
        self.cliffSensIndicatorF2.setLineWidth(1)
        self.cliffSensIndicatorF2.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.gridLayout_4.addWidget(self.cliffSensIndicatorF2, 1, 1, 1, 1)

        self.cliffSensIndicatorB2 = QLabel(self.cliffSensBox)
        self.cliffSensIndicatorB2.setObjectName(u"cliffSensIndicatorB2")
        sizePolicy1.setHeightForWidth(self.cliffSensIndicatorB2.sizePolicy().hasHeightForWidth())
        self.cliffSensIndicatorB2.setSizePolicy(sizePolicy1)
        self.cliffSensIndicatorB2.setAutoFillBackground(False)
        self.cliffSensIndicatorB2.setFrameShape(QFrame.Shape.Box)
        self.cliffSensIndicatorB2.setFrameShadow(QFrame.Shadow.Plain)
        self.cliffSensIndicatorB2.setLineWidth(1)
        self.cliffSensIndicatorB2.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.gridLayout_4.addWidget(self.cliffSensIndicatorB2, 1, 0, 1, 1)

        self.cliffSensIndicatorF3 = QLabel(self.cliffSensBox)
        self.cliffSensIndicatorF3.setObjectName(u"cliffSensIndicatorF3")
        sizePolicy1.setHeightForWidth(self.cliffSensIndicatorF3.sizePolicy().hasHeightForWidth())
        self.cliffSensIndicatorF3.setSizePolicy(sizePolicy1)
        self.cliffSensIndicatorF3.setAutoFillBackground(False)
        self.cliffSensIndicatorF3.setFrameShape(QFrame.Shape.Box)
        self.cliffSensIndicatorF3.setFrameShadow(QFrame.Shadow.Plain)
        self.cliffSensIndicatorF3.setLineWidth(1)
        self.cliffSensIndicatorF3.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.gridLayout_4.addWidget(self.cliffSensIndicatorF3, 2, 1, 1, 1)

        self.cliffSensIndicatorB1 = QLabel(self.cliffSensBox)
        self.cliffSensIndicatorB1.setObjectName(u"cliffSensIndicatorB1")
        sizePolicy1.setHeightForWidth(self.cliffSensIndicatorB1.sizePolicy().hasHeightForWidth())
        self.cliffSensIndicatorB1.setSizePolicy(sizePolicy1)
        self.cliffSensIndicatorB1.setAutoFillBackground(False)
        self.cliffSensIndicatorB1.setFrameShape(QFrame.Shape.Box)
        self.cliffSensIndicatorB1.setFrameShadow(QFrame.Shadow.Plain)
        self.cliffSensIndicatorB1.setLineWidth(1)
        self.cliffSensIndicatorB1.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.gridLayout_4.addWidget(self.cliffSensIndicatorB1, 0, 0, 1, 1)

        self.cliffSensIndicatorB3 = QLabel(self.cliffSensBox)
        self.cliffSensIndicatorB3.setObjectName(u"cliffSensIndicatorB3")
        sizePolicy1.setHeightForWidth(self.cliffSensIndicatorB3.sizePolicy().hasHeightForWidth())
        self.cliffSensIndicatorB3.setSizePolicy(sizePolicy1)
        self.cliffSensIndicatorB3.setAutoFillBackground(False)
        self.cliffSensIndicatorB3.setFrameShape(QFrame.Shape.Box)
        self.cliffSensIndicatorB3.setFrameShadow(QFrame.Shadow.Plain)
        self.cliffSensIndicatorB3.setLineWidth(1)
        self.cliffSensIndicatorB3.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.gridLayout_4.addWidget(self.cliffSensIndicatorB3, 2, 0, 1, 1)

        self.cliffSensIndicatorF1 = QLabel(self.cliffSensBox)
        self.cliffSensIndicatorF1.setObjectName(u"cliffSensIndicatorF1")
        sizePolicy1.setHeightForWidth(self.cliffSensIndicatorF1.sizePolicy().hasHeightForWidth())
        self.cliffSensIndicatorF1.setSizePolicy(sizePolicy1)
        self.cliffSensIndicatorF1.setAutoFillBackground(False)
        self.cliffSensIndicatorF1.setFrameShape(QFrame.Shape.Box)
        self.cliffSensIndicatorF1.setFrameShadow(QFrame.Shadow.Plain)
        self.cliffSensIndicatorF1.setLineWidth(1)
        self.cliffSensIndicatorF1.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.gridLayout_4.addWidget(self.cliffSensIndicatorF1, 0, 1, 1, 1)

        self.cliffSensIndicatorB4 = QLabel(self.cliffSensBox)
        self.cliffSensIndicatorB4.setObjectName(u"cliffSensIndicatorB4")
        sizePolicy1.setHeightForWidth(self.cliffSensIndicatorB4.sizePolicy().hasHeightForWidth())
        self.cliffSensIndicatorB4.setSizePolicy(sizePolicy1)
        self.cliffSensIndicatorB4.setAutoFillBackground(False)
        self.cliffSensIndicatorB4.setFrameShape(QFrame.Shape.Box)
        self.cliffSensIndicatorB4.setFrameShadow(QFrame.Shadow.Plain)
        self.cliffSensIndicatorB4.setLineWidth(1)
        self.cliffSensIndicatorB4.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.gridLayout_4.addWidget(self.cliffSensIndicatorB4, 3, 0, 1, 1)

        self.cliffSensIndicatorF4 = QLabel(self.cliffSensBox)
        self.cliffSensIndicatorF4.setObjectName(u"cliffSensIndicatorF4")
        sizePolicy1.setHeightForWidth(self.cliffSensIndicatorF4.sizePolicy().hasHeightForWidth())
        self.cliffSensIndicatorF4.setSizePolicy(sizePolicy1)
        self.cliffSensIndicatorF4.setAutoFillBackground(False)
        self.cliffSensIndicatorF4.setFrameShape(QFrame.Shape.Box)
        self.cliffSensIndicatorF4.setFrameShadow(QFrame.Shadow.Plain)
        self.cliffSensIndicatorF4.setLineWidth(1)
        self.cliffSensIndicatorF4.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.gridLayout_4.addWidget(self.cliffSensIndicatorF4, 3, 1, 1, 1)

        self.ultrasonicSensBox = QGroupBox(self.sensorsTab)
        self.ultrasonicSensBox.setObjectName(u"ultrasonicSensBox")
        self.ultrasonicSensBox.setGeometry(QRect(28, 20, 195, 150))
        sizePolicy.setHeightForWidth(self.ultrasonicSensBox.sizePolicy().hasHeightForWidth())
        self.ultrasonicSensBox.setSizePolicy(sizePolicy)
        self.horizontalLayout_4 = QHBoxLayout(self.ultrasonicSensBox)
        self.horizontalLayout_4.setObjectName(u"horizontalLayout_4")
        self.ultrasonicSensLabelsBox = QVBoxLayout()
        self.ultrasonicSensLabelsBox.setObjectName(u"ultrasonicSensLabelsBox")
        self.ultrasonicSensLabelF = QLabel(self.ultrasonicSensBox)
        self.ultrasonicSensLabelF.setObjectName(u"ultrasonicSensLabelF")

        self.ultrasonicSensLabelsBox.addWidget(self.ultrasonicSensLabelF)

        self.ultrasonicSensLabelB = QLabel(self.ultrasonicSensBox)
        self.ultrasonicSensLabelB.setObjectName(u"ultrasonicSensLabelB")

        self.ultrasonicSensLabelsBox.addWidget(self.ultrasonicSensLabelB)

        self.ultrasonicSensLabelR = QLabel(self.ultrasonicSensBox)
        self.ultrasonicSensLabelR.setObjectName(u"ultrasonicSensLabelR")

        self.ultrasonicSensLabelsBox.addWidget(self.ultrasonicSensLabelR)

        self.ultrasonicSensLabelL = QLabel(self.ultrasonicSensBox)
        self.ultrasonicSensLabelL.setObjectName(u"ultrasonicSensLabelL")

        self.ultrasonicSensLabelsBox.addWidget(self.ultrasonicSensLabelL)


        self.horizontalLayout_4.addLayout(self.ultrasonicSensLabelsBox)

        self.horizontalSpacer_13 = QSpacerItem(8, 20, QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Minimum)

        self.horizontalLayout_4.addItem(self.horizontalSpacer_13)

        self.ultrasonicSensValuesBox = QVBoxLayout()
        self.ultrasonicSensValuesBox.setObjectName(u"ultrasonicSensValuesBox")
        self.ultrasonicSensValueF = QLineEdit(self.ultrasonicSensBox)
        self.ultrasonicSensValueF.setObjectName(u"ultrasonicSensValueF")
        sizePolicy4.setHeightForWidth(self.ultrasonicSensValueF.sizePolicy().hasHeightForWidth())
        self.ultrasonicSensValueF.setSizePolicy(sizePolicy4)
        self.ultrasonicSensValueF.setFont(font)
        self.ultrasonicSensValueF.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.ultrasonicSensValueF.setFrame(True)
        self.ultrasonicSensValueF.setDragEnabled(True)
        self.ultrasonicSensValueF.setReadOnly(True)

        self.ultrasonicSensValuesBox.addWidget(self.ultrasonicSensValueF)

        self.ultrasonicSensValueB = QLineEdit(self.ultrasonicSensBox)
        self.ultrasonicSensValueB.setObjectName(u"ultrasonicSensValueB")
        sizePolicy4.setHeightForWidth(self.ultrasonicSensValueB.sizePolicy().hasHeightForWidth())
        self.ultrasonicSensValueB.setSizePolicy(sizePolicy4)
        self.ultrasonicSensValueB.setFont(font)
        self.ultrasonicSensValueB.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.ultrasonicSensValueB.setFrame(True)
        self.ultrasonicSensValueB.setDragEnabled(True)
        self.ultrasonicSensValueB.setReadOnly(True)

        self.ultrasonicSensValuesBox.addWidget(self.ultrasonicSensValueB)

        self.ultrasonicSensValueR = QLineEdit(self.ultrasonicSensBox)
        self.ultrasonicSensValueR.setObjectName(u"ultrasonicSensValueR")
        sizePolicy4.setHeightForWidth(self.ultrasonicSensValueR.sizePolicy().hasHeightForWidth())
        self.ultrasonicSensValueR.setSizePolicy(sizePolicy4)
        self.ultrasonicSensValueR.setFont(font)
        self.ultrasonicSensValueR.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.ultrasonicSensValueR.setFrame(True)
        self.ultrasonicSensValueR.setDragEnabled(True)
        self.ultrasonicSensValueR.setReadOnly(True)

        self.ultrasonicSensValuesBox.addWidget(self.ultrasonicSensValueR)

        self.ultrasonicSensValueL = QLineEdit(self.ultrasonicSensBox)
        self.ultrasonicSensValueL.setObjectName(u"ultrasonicSensValueL")
        sizePolicy4.setHeightForWidth(self.ultrasonicSensValueL.sizePolicy().hasHeightForWidth())
        self.ultrasonicSensValueL.setSizePolicy(sizePolicy4)
        self.ultrasonicSensValueL.setFont(font)
        self.ultrasonicSensValueL.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.ultrasonicSensValueL.setFrame(True)
        self.ultrasonicSensValueL.setDragEnabled(True)
        self.ultrasonicSensValueL.setReadOnly(True)

        self.ultrasonicSensValuesBox.addWidget(self.ultrasonicSensValueL)


        self.horizontalLayout_4.addLayout(self.ultrasonicSensValuesBox)

        self.ultrasonicSensUnitsBox = QVBoxLayout()
        self.ultrasonicSensUnitsBox.setObjectName(u"ultrasonicSensUnitsBox")
        self.ultrasonicSensUnitF = QLabel(self.ultrasonicSensBox)
        self.ultrasonicSensUnitF.setObjectName(u"ultrasonicSensUnitF")

        self.ultrasonicSensUnitsBox.addWidget(self.ultrasonicSensUnitF)

        self.ultrasonicSensUnitB = QLabel(self.ultrasonicSensBox)
        self.ultrasonicSensUnitB.setObjectName(u"ultrasonicSensUnitB")

        self.ultrasonicSensUnitsBox.addWidget(self.ultrasonicSensUnitB)

        self.ultrasonicSensUnitR = QLabel(self.ultrasonicSensBox)
        self.ultrasonicSensUnitR.setObjectName(u"ultrasonicSensUnitR")

        self.ultrasonicSensUnitsBox.addWidget(self.ultrasonicSensUnitR)

        self.ultrasonicSensUnitL = QLabel(self.ultrasonicSensBox)
        self.ultrasonicSensUnitL.setObjectName(u"ultrasonicSensUnitL")

        self.ultrasonicSensUnitsBox.addWidget(self.ultrasonicSensUnitL)


        self.horizontalLayout_4.addLayout(self.ultrasonicSensUnitsBox)

        self.batterySensBox = QGroupBox(self.sensorsTab)
        self.batterySensBox.setObjectName(u"batterySensBox")
        self.batterySensBox.setGeometry(QRect(28, 410, 195, 110))
        self.horizontalLayout_5 = QHBoxLayout(self.batterySensBox)
        self.horizontalLayout_5.setObjectName(u"horizontalLayout_5")
        self.batterySensLabelsBox = QVBoxLayout()
        self.batterySensLabelsBox.setObjectName(u"batterySensLabelsBox")
        self.batterySensVoltageLabel = QLabel(self.batterySensBox)
        self.batterySensVoltageLabel.setObjectName(u"batterySensVoltageLabel")
        sizePolicy6 = QSizePolicy(QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Minimum)
        sizePolicy6.setHorizontalStretch(0)
        sizePolicy6.setVerticalStretch(0)
        sizePolicy6.setHeightForWidth(self.batterySensVoltageLabel.sizePolicy().hasHeightForWidth())
        self.batterySensVoltageLabel.setSizePolicy(sizePolicy6)

        self.batterySensLabelsBox.addWidget(self.batterySensVoltageLabel)

        self.batterySensCurrentLabel = QLabel(self.batterySensBox)
        self.batterySensCurrentLabel.setObjectName(u"batterySensCurrentLabel")
        sizePolicy6.setHeightForWidth(self.batterySensCurrentLabel.sizePolicy().hasHeightForWidth())
        self.batterySensCurrentLabel.setSizePolicy(sizePolicy6)

        self.batterySensLabelsBox.addWidget(self.batterySensCurrentLabel)


        self.horizontalLayout_5.addLayout(self.batterySensLabelsBox)

        self.horizontalSpacer_14 = QSpacerItem(8, 20, QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Minimum)

        self.horizontalLayout_5.addItem(self.horizontalSpacer_14)

        self.batterySensValuesBox = QVBoxLayout()
        self.batterySensValuesBox.setObjectName(u"batterySensValuesBox")
        self.batterySensVoltageValue = QLCDNumber(self.batterySensBox)
        self.batterySensVoltageValue.setObjectName(u"batterySensVoltageValue")
        sizePolicy1.setHeightForWidth(self.batterySensVoltageValue.sizePolicy().hasHeightForWidth())
        self.batterySensVoltageValue.setSizePolicy(sizePolicy1)
        self.batterySensVoltageValue.setFont(font1)
        self.batterySensVoltageValue.setDigitCount(6)
        self.batterySensVoltageValue.setSegmentStyle(QLCDNumber.SegmentStyle.Flat)
        self.batterySensVoltageValue.setProperty("value", 12.458000000000000)

        self.batterySensValuesBox.addWidget(self.batterySensVoltageValue)

        self.batterySensCurrentValue = QLCDNumber(self.batterySensBox)
        self.batterySensCurrentValue.setObjectName(u"batterySensCurrentValue")
        sizePolicy1.setHeightForWidth(self.batterySensCurrentValue.sizePolicy().hasHeightForWidth())
        self.batterySensCurrentValue.setSizePolicy(sizePolicy1)
        self.batterySensCurrentValue.setFont(font1)
        self.batterySensCurrentValue.setDigitCount(6)
        self.batterySensCurrentValue.setSegmentStyle(QLCDNumber.SegmentStyle.Flat)
        self.batterySensCurrentValue.setProperty("value", 3.284000000000000)

        self.batterySensValuesBox.addWidget(self.batterySensCurrentValue)


        self.horizontalLayout_5.addLayout(self.batterySensValuesBox)

        self.enableDisableBox = QGroupBox(self.sensorsTab)
        self.enableDisableBox.setObjectName(u"enableDisableBox")
        self.enableDisableBox.setGeometry(QRect(462, 145, 195, 110))
        self.gridLayout_5 = QGridLayout(self.enableDisableBox)
        self.gridLayout_5.setObjectName(u"gridLayout_5")
        self.toggleEmittersEnabled = QPushButton(self.enableDisableBox)
        self.toggleEmittersEnabled.setObjectName(u"toggleEmittersEnabled")
        sizePolicy7 = QSizePolicy(QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Preferred)
        sizePolicy7.setHorizontalStretch(0)
        sizePolicy7.setVerticalStretch(0)
        sizePolicy7.setHeightForWidth(self.toggleEmittersEnabled.sizePolicy().hasHeightForWidth())
        self.toggleEmittersEnabled.setSizePolicy(sizePolicy7)
        self.toggleEmittersEnabled.setCheckable(True)
        self.toggleEmittersEnabled.setChecked(True)

        self.gridLayout_5.addWidget(self.toggleEmittersEnabled, 0, 1, 1, 1)

        self.togglePicoARelaEnable = QPushButton(self.enableDisableBox)
        self.togglePicoARelaEnable.setObjectName(u"togglePicoARelaEnable")
        sizePolicy7.setHeightForWidth(self.togglePicoARelaEnable.sizePolicy().hasHeightForWidth())
        self.togglePicoARelaEnable.setSizePolicy(sizePolicy7)
        self.togglePicoARelaEnable.setCheckable(True)
        self.togglePicoARelaEnable.setChecked(False)

        self.gridLayout_5.addWidget(self.togglePicoARelaEnable, 1, 1, 1, 1)

        self.pages.addTab(self.sensorsTab, "")
        self.motorCtrlTab = QWidget()
        self.motorCtrlTab.setObjectName(u"motorCtrlTab")
        self.encOdomBox = QGroupBox(self.motorCtrlTab)
        self.encOdomBox.setObjectName(u"encOdomBox")
        self.encOdomBox.setGeometry(QRect(28, 20, 200, 185))
        sizePolicy.setHeightForWidth(self.encOdomBox.sizePolicy().hasHeightForWidth())
        self.encOdomBox.setSizePolicy(sizePolicy)
        self.horizontalLayout_6 = QHBoxLayout(self.encOdomBox)
        self.horizontalLayout_6.setObjectName(u"horizontalLayout_6")
        self.encOdomLabels = QVBoxLayout()
        self.encOdomLabels.setObjectName(u"encOdomLabels")
        self.encOdomPosXLabel = QLabel(self.encOdomBox)
        self.encOdomPosXLabel.setObjectName(u"encOdomPosXLabel")
        sizePolicy5.setHeightForWidth(self.encOdomPosXLabel.sizePolicy().hasHeightForWidth())
        self.encOdomPosXLabel.setSizePolicy(sizePolicy5)

        self.encOdomLabels.addWidget(self.encOdomPosXLabel)

        self.encOdomPosYLabel = QLabel(self.encOdomBox)
        self.encOdomPosYLabel.setObjectName(u"encOdomPosYLabel")

        self.encOdomLabels.addWidget(self.encOdomPosYLabel)

        self.encOdomYawLabel = QLabel(self.encOdomBox)
        self.encOdomYawLabel.setObjectName(u"encOdomYawLabel")

        self.encOdomLabels.addWidget(self.encOdomYawLabel)

        self.verticalSpacer_3 = QSpacerItem(20, 10, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Fixed)

        self.encOdomLabels.addItem(self.verticalSpacer_3)

        self.encOdomLinVelLabel = QLabel(self.encOdomBox)
        self.encOdomLinVelLabel.setObjectName(u"encOdomLinVelLabel")

        self.encOdomLabels.addWidget(self.encOdomLinVelLabel)

        self.encOdomAngVelLabel = QLabel(self.encOdomBox)
        self.encOdomAngVelLabel.setObjectName(u"encOdomAngVelLabel")

        self.encOdomLabels.addWidget(self.encOdomAngVelLabel)


        self.horizontalLayout_6.addLayout(self.encOdomLabels)

        self.horizontalSpacer_15 = QSpacerItem(4, 20, QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Minimum)

        self.horizontalLayout_6.addItem(self.horizontalSpacer_15)

        self.encOdomValues = QVBoxLayout()
        self.encOdomValues.setObjectName(u"encOdomValues")
        self.encOdomValues.setSizeConstraint(QLayout.SizeConstraint.SetDefaultConstraint)
        self.encOdomPosXValue = QLineEdit(self.encOdomBox)
        self.encOdomPosXValue.setObjectName(u"encOdomPosXValue")
        sizePolicy2.setHeightForWidth(self.encOdomPosXValue.sizePolicy().hasHeightForWidth())
        self.encOdomPosXValue.setSizePolicy(sizePolicy2)
        self.encOdomPosXValue.setFont(font)
        self.encOdomPosXValue.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.encOdomPosXValue.setFrame(True)
        self.encOdomPosXValue.setDragEnabled(True)
        self.encOdomPosXValue.setReadOnly(True)

        self.encOdomValues.addWidget(self.encOdomPosXValue)

        self.encOdomPosYValue = QLineEdit(self.encOdomBox)
        self.encOdomPosYValue.setObjectName(u"encOdomPosYValue")
        sizePolicy2.setHeightForWidth(self.encOdomPosYValue.sizePolicy().hasHeightForWidth())
        self.encOdomPosYValue.setSizePolicy(sizePolicy2)
        self.encOdomPosYValue.setFont(font)
        self.encOdomPosYValue.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.encOdomPosYValue.setFrame(True)
        self.encOdomPosYValue.setDragEnabled(True)
        self.encOdomPosYValue.setReadOnly(True)

        self.encOdomValues.addWidget(self.encOdomPosYValue)

        self.encOdomYawValue = QLineEdit(self.encOdomBox)
        self.encOdomYawValue.setObjectName(u"encOdomYawValue")
        sizePolicy2.setHeightForWidth(self.encOdomYawValue.sizePolicy().hasHeightForWidth())
        self.encOdomYawValue.setSizePolicy(sizePolicy2)
        self.encOdomYawValue.setFont(font)
        self.encOdomYawValue.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.encOdomYawValue.setFrame(True)
        self.encOdomYawValue.setDragEnabled(True)
        self.encOdomYawValue.setReadOnly(True)

        self.encOdomValues.addWidget(self.encOdomYawValue)

        self.verticalSpacer_4 = QSpacerItem(20, 10, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Fixed)

        self.encOdomValues.addItem(self.verticalSpacer_4)

        self.encOdomLinVelValue = QLineEdit(self.encOdomBox)
        self.encOdomLinVelValue.setObjectName(u"encOdomLinVelValue")
        sizePolicy2.setHeightForWidth(self.encOdomLinVelValue.sizePolicy().hasHeightForWidth())
        self.encOdomLinVelValue.setSizePolicy(sizePolicy2)
        self.encOdomLinVelValue.setFont(font)
        self.encOdomLinVelValue.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.encOdomLinVelValue.setFrame(True)
        self.encOdomLinVelValue.setDragEnabled(True)
        self.encOdomLinVelValue.setReadOnly(True)

        self.encOdomValues.addWidget(self.encOdomLinVelValue)

        self.encOdomAngVelValue = QLineEdit(self.encOdomBox)
        self.encOdomAngVelValue.setObjectName(u"encOdomAngVelValue")
        sizePolicy2.setHeightForWidth(self.encOdomAngVelValue.sizePolicy().hasHeightForWidth())
        self.encOdomAngVelValue.setSizePolicy(sizePolicy2)
        self.encOdomAngVelValue.setFont(font)
        self.encOdomAngVelValue.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.encOdomAngVelValue.setFrame(True)
        self.encOdomAngVelValue.setDragEnabled(True)
        self.encOdomAngVelValue.setReadOnly(True)

        self.encOdomValues.addWidget(self.encOdomAngVelValue)


        self.horizontalLayout_6.addLayout(self.encOdomValues)

        self.encOdomUnits = QVBoxLayout()
        self.encOdomUnits.setObjectName(u"encOdomUnits")
        self.encOdomPosXUnit = QLabel(self.encOdomBox)
        self.encOdomPosXUnit.setObjectName(u"encOdomPosXUnit")

        self.encOdomUnits.addWidget(self.encOdomPosXUnit)

        self.encOdomPosYUnit = QLabel(self.encOdomBox)
        self.encOdomPosYUnit.setObjectName(u"encOdomPosYUnit")

        self.encOdomUnits.addWidget(self.encOdomPosYUnit)

        self.encOdomYawUnit = QLabel(self.encOdomBox)
        self.encOdomYawUnit.setObjectName(u"encOdomYawUnit")

        self.encOdomUnits.addWidget(self.encOdomYawUnit)

        self.verticalSpacer_5 = QSpacerItem(20, 10, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Fixed)

        self.encOdomUnits.addItem(self.verticalSpacer_5)

        self.encOdomLinVelUnit = QLabel(self.encOdomBox)
        self.encOdomLinVelUnit.setObjectName(u"encOdomLinVelUnit")

        self.encOdomUnits.addWidget(self.encOdomLinVelUnit)

        self.encOdomAngVelUnit = QLabel(self.encOdomBox)
        self.encOdomAngVelUnit.setObjectName(u"encOdomAngVelUnit")

        self.encOdomUnits.addWidget(self.encOdomAngVelUnit)


        self.horizontalLayout_6.addLayout(self.encOdomUnits)

        self.mtrCtrlEnableBox = QGroupBox(self.motorCtrlTab)
        self.mtrCtrlEnableBox.setObjectName(u"mtrCtrlEnableBox")
        self.mtrCtrlEnableBox.setGeometry(QRect(687, 20, 200, 120))
        self.gridLayout_6 = QGridLayout(self.mtrCtrlEnableBox)
        self.gridLayout_6.setObjectName(u"gridLayout_6")
        self.mtrCtrlEnableButtonsBox = QHBoxLayout()
        self.mtrCtrlEnableButtonsBox.setObjectName(u"mtrCtrlEnableButtonsBox")
        self.mtrCtrlEnableButton = QPushButton(self.mtrCtrlEnableBox)
        self.mtrCtrlEnableButton.setObjectName(u"mtrCtrlEnableButton")
        sizePolicy5.setHeightForWidth(self.mtrCtrlEnableButton.sizePolicy().hasHeightForWidth())
        self.mtrCtrlEnableButton.setSizePolicy(sizePolicy5)
        self.mtrCtrlEnableButton.setCheckable(False)
        self.mtrCtrlEnableButton.setChecked(False)

        self.mtrCtrlEnableButtonsBox.addWidget(self.mtrCtrlEnableButton)

        self.mtrCtrlDisableButton = QPushButton(self.mtrCtrlEnableBox)
        self.mtrCtrlDisableButton.setObjectName(u"mtrCtrlDisableButton")
        sizePolicy5.setHeightForWidth(self.mtrCtrlDisableButton.sizePolicy().hasHeightForWidth())
        self.mtrCtrlDisableButton.setSizePolicy(sizePolicy5)
        self.mtrCtrlDisableButton.setCheckable(False)
        self.mtrCtrlDisableButton.setChecked(False)

        self.mtrCtrlEnableButtonsBox.addWidget(self.mtrCtrlDisableButton)


        self.gridLayout_6.addLayout(self.mtrCtrlEnableButtonsBox, 0, 0, 1, 1)

        self.mtrCtrlEnableIndicatorsBox = QHBoxLayout()
        self.mtrCtrlEnableIndicatorsBox.setObjectName(u"mtrCtrlEnableIndicatorsBox")
        self.mtrCtrlEnableLIndicator = QLabel(self.mtrCtrlEnableBox)
        self.mtrCtrlEnableLIndicator.setObjectName(u"mtrCtrlEnableLIndicator")
        sizePolicy3.setHeightForWidth(self.mtrCtrlEnableLIndicator.sizePolicy().hasHeightForWidth())
        self.mtrCtrlEnableLIndicator.setSizePolicy(sizePolicy3)
        self.mtrCtrlEnableLIndicator.setAutoFillBackground(False)
        self.mtrCtrlEnableLIndicator.setFrameShape(QFrame.Shape.Box)
        self.mtrCtrlEnableLIndicator.setFrameShadow(QFrame.Shadow.Plain)
        self.mtrCtrlEnableLIndicator.setLineWidth(1)
        self.mtrCtrlEnableLIndicator.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.mtrCtrlEnableIndicatorsBox.addWidget(self.mtrCtrlEnableLIndicator)

        self.mtrCtrlEnableRIndicator = QLabel(self.mtrCtrlEnableBox)
        self.mtrCtrlEnableRIndicator.setObjectName(u"mtrCtrlEnableRIndicator")
        sizePolicy3.setHeightForWidth(self.mtrCtrlEnableRIndicator.sizePolicy().hasHeightForWidth())
        self.mtrCtrlEnableRIndicator.setSizePolicy(sizePolicy3)
        self.mtrCtrlEnableRIndicator.setAutoFillBackground(False)
        self.mtrCtrlEnableRIndicator.setFrameShape(QFrame.Shape.Box)
        self.mtrCtrlEnableRIndicator.setFrameShadow(QFrame.Shadow.Plain)
        self.mtrCtrlEnableRIndicator.setLineWidth(1)
        self.mtrCtrlEnableRIndicator.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.mtrCtrlEnableIndicatorsBox.addWidget(self.mtrCtrlEnableRIndicator)


        self.gridLayout_6.addLayout(self.mtrCtrlEnableIndicatorsBox, 1, 0, 1, 1)

        self.rpmMeasuredBox = QGroupBox(self.motorCtrlTab)
        self.rpmMeasuredBox.setObjectName(u"rpmMeasuredBox")
        self.rpmMeasuredBox.setGeometry(QRect(247, 20, 200, 165))
        sizePolicy.setHeightForWidth(self.rpmMeasuredBox.sizePolicy().hasHeightForWidth())
        self.rpmMeasuredBox.setSizePolicy(sizePolicy)
        self.horizontalLayout_7 = QHBoxLayout(self.rpmMeasuredBox)
        self.horizontalLayout_7.setObjectName(u"horizontalLayout_7")
        self.rpmMeasuredLabelsBox = QVBoxLayout()
        self.rpmMeasuredLabelsBox.setObjectName(u"rpmMeasuredLabelsBox")
        self.rpmMeasuredFRLabel = QLabel(self.rpmMeasuredBox)
        self.rpmMeasuredFRLabel.setObjectName(u"rpmMeasuredFRLabel")
        sizePolicy5.setHeightForWidth(self.rpmMeasuredFRLabel.sizePolicy().hasHeightForWidth())
        self.rpmMeasuredFRLabel.setSizePolicy(sizePolicy5)

        self.rpmMeasuredLabelsBox.addWidget(self.rpmMeasuredFRLabel)

        self.rpmMeasuredFLLabel = QLabel(self.rpmMeasuredBox)
        self.rpmMeasuredFLLabel.setObjectName(u"rpmMeasuredFLLabel")

        self.rpmMeasuredLabelsBox.addWidget(self.rpmMeasuredFLLabel)

        self.verticalSpacer_6 = QSpacerItem(20, 10, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Fixed)

        self.rpmMeasuredLabelsBox.addItem(self.verticalSpacer_6)

        self.rpmMeasuredBRLabel = QLabel(self.rpmMeasuredBox)
        self.rpmMeasuredBRLabel.setObjectName(u"rpmMeasuredBRLabel")

        self.rpmMeasuredLabelsBox.addWidget(self.rpmMeasuredBRLabel)

        self.rpmMeasuredBLLabel = QLabel(self.rpmMeasuredBox)
        self.rpmMeasuredBLLabel.setObjectName(u"rpmMeasuredBLLabel")

        self.rpmMeasuredLabelsBox.addWidget(self.rpmMeasuredBLLabel)


        self.horizontalLayout_7.addLayout(self.rpmMeasuredLabelsBox)

        self.horizontalSpacer_16 = QSpacerItem(4, 20, QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Minimum)

        self.horizontalLayout_7.addItem(self.horizontalSpacer_16)

        self.rpmMeasuredValuesBox = QVBoxLayout()
        self.rpmMeasuredValuesBox.setObjectName(u"rpmMeasuredValuesBox")
        self.rpmMeasuredValuesBox.setSizeConstraint(QLayout.SizeConstraint.SetDefaultConstraint)
        self.rpmMeasuredFRValue = QLineEdit(self.rpmMeasuredBox)
        self.rpmMeasuredFRValue.setObjectName(u"rpmMeasuredFRValue")
        sizePolicy2.setHeightForWidth(self.rpmMeasuredFRValue.sizePolicy().hasHeightForWidth())
        self.rpmMeasuredFRValue.setSizePolicy(sizePolicy2)
        self.rpmMeasuredFRValue.setFont(font)
        self.rpmMeasuredFRValue.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.rpmMeasuredFRValue.setFrame(True)
        self.rpmMeasuredFRValue.setDragEnabled(True)
        self.rpmMeasuredFRValue.setReadOnly(True)

        self.rpmMeasuredValuesBox.addWidget(self.rpmMeasuredFRValue)

        self.rpmMeasuredFLValue = QLineEdit(self.rpmMeasuredBox)
        self.rpmMeasuredFLValue.setObjectName(u"rpmMeasuredFLValue")
        sizePolicy2.setHeightForWidth(self.rpmMeasuredFLValue.sizePolicy().hasHeightForWidth())
        self.rpmMeasuredFLValue.setSizePolicy(sizePolicy2)
        self.rpmMeasuredFLValue.setFont(font)
        self.rpmMeasuredFLValue.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.rpmMeasuredFLValue.setFrame(True)
        self.rpmMeasuredFLValue.setDragEnabled(True)
        self.rpmMeasuredFLValue.setReadOnly(True)

        self.rpmMeasuredValuesBox.addWidget(self.rpmMeasuredFLValue)

        self.verticalSpacer_7 = QSpacerItem(20, 10, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Fixed)

        self.rpmMeasuredValuesBox.addItem(self.verticalSpacer_7)

        self.rpmMeasuredBRValue = QLineEdit(self.rpmMeasuredBox)
        self.rpmMeasuredBRValue.setObjectName(u"rpmMeasuredBRValue")
        sizePolicy2.setHeightForWidth(self.rpmMeasuredBRValue.sizePolicy().hasHeightForWidth())
        self.rpmMeasuredBRValue.setSizePolicy(sizePolicy2)
        self.rpmMeasuredBRValue.setFont(font)
        self.rpmMeasuredBRValue.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.rpmMeasuredBRValue.setFrame(True)
        self.rpmMeasuredBRValue.setDragEnabled(True)
        self.rpmMeasuredBRValue.setReadOnly(True)

        self.rpmMeasuredValuesBox.addWidget(self.rpmMeasuredBRValue)

        self.rpmMeasuredBLValue = QLineEdit(self.rpmMeasuredBox)
        self.rpmMeasuredBLValue.setObjectName(u"rpmMeasuredBLValue")
        sizePolicy2.setHeightForWidth(self.rpmMeasuredBLValue.sizePolicy().hasHeightForWidth())
        self.rpmMeasuredBLValue.setSizePolicy(sizePolicy2)
        self.rpmMeasuredBLValue.setFont(font)
        self.rpmMeasuredBLValue.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.rpmMeasuredBLValue.setFrame(True)
        self.rpmMeasuredBLValue.setDragEnabled(True)
        self.rpmMeasuredBLValue.setReadOnly(True)

        self.rpmMeasuredValuesBox.addWidget(self.rpmMeasuredBLValue)


        self.horizontalLayout_7.addLayout(self.rpmMeasuredValuesBox)

        self.rpmMeasuredUnitsBox = QVBoxLayout()
        self.rpmMeasuredUnitsBox.setObjectName(u"rpmMeasuredUnitsBox")
        self.rpmMeasuredFRUnit = QLabel(self.rpmMeasuredBox)
        self.rpmMeasuredFRUnit.setObjectName(u"rpmMeasuredFRUnit")

        self.rpmMeasuredUnitsBox.addWidget(self.rpmMeasuredFRUnit)

        self.rpmMeasuredFLUnit = QLabel(self.rpmMeasuredBox)
        self.rpmMeasuredFLUnit.setObjectName(u"rpmMeasuredFLUnit")

        self.rpmMeasuredUnitsBox.addWidget(self.rpmMeasuredFLUnit)

        self.verticalSpacer_8 = QSpacerItem(20, 10, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Fixed)

        self.rpmMeasuredUnitsBox.addItem(self.verticalSpacer_8)

        self.rpmMeasuredBRUnit = QLabel(self.rpmMeasuredBox)
        self.rpmMeasuredBRUnit.setObjectName(u"rpmMeasuredBRUnit")

        self.rpmMeasuredUnitsBox.addWidget(self.rpmMeasuredBRUnit)

        self.rpmMeasuredBLUnit = QLabel(self.rpmMeasuredBox)
        self.rpmMeasuredBLUnit.setObjectName(u"rpmMeasuredBLUnit")

        self.rpmMeasuredUnitsBox.addWidget(self.rpmMeasuredBLUnit)


        self.horizontalLayout_7.addLayout(self.rpmMeasuredUnitsBox)

        self.rpmTargetBox = QGroupBox(self.motorCtrlTab)
        self.rpmTargetBox.setObjectName(u"rpmTargetBox")
        self.rpmTargetBox.setGeometry(QRect(247, 200, 200, 165))
        sizePolicy.setHeightForWidth(self.rpmTargetBox.sizePolicy().hasHeightForWidth())
        self.rpmTargetBox.setSizePolicy(sizePolicy)
        self.horizontalLayout_8 = QHBoxLayout(self.rpmTargetBox)
        self.horizontalLayout_8.setObjectName(u"horizontalLayout_8")
        self.rpmTargetLabelsBox = QVBoxLayout()
        self.rpmTargetLabelsBox.setObjectName(u"rpmTargetLabelsBox")
        self.rpmTargetFRLabel = QLabel(self.rpmTargetBox)
        self.rpmTargetFRLabel.setObjectName(u"rpmTargetFRLabel")
        sizePolicy5.setHeightForWidth(self.rpmTargetFRLabel.sizePolicy().hasHeightForWidth())
        self.rpmTargetFRLabel.setSizePolicy(sizePolicy5)

        self.rpmTargetLabelsBox.addWidget(self.rpmTargetFRLabel)

        self.rpmTargetFLLabel = QLabel(self.rpmTargetBox)
        self.rpmTargetFLLabel.setObjectName(u"rpmTargetFLLabel")

        self.rpmTargetLabelsBox.addWidget(self.rpmTargetFLLabel)

        self.verticalSpacer_9 = QSpacerItem(20, 10, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Fixed)

        self.rpmTargetLabelsBox.addItem(self.verticalSpacer_9)

        self.rpmTargetBRLabel = QLabel(self.rpmTargetBox)
        self.rpmTargetBRLabel.setObjectName(u"rpmTargetBRLabel")

        self.rpmTargetLabelsBox.addWidget(self.rpmTargetBRLabel)

        self.rpmTargetBLLabel = QLabel(self.rpmTargetBox)
        self.rpmTargetBLLabel.setObjectName(u"rpmTargetBLLabel")

        self.rpmTargetLabelsBox.addWidget(self.rpmTargetBLLabel)


        self.horizontalLayout_8.addLayout(self.rpmTargetLabelsBox)

        self.horizontalSpacer_17 = QSpacerItem(4, 20, QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Minimum)

        self.horizontalLayout_8.addItem(self.horizontalSpacer_17)

        self.rpmTargetValuesBox = QVBoxLayout()
        self.rpmTargetValuesBox.setObjectName(u"rpmTargetValuesBox")
        self.rpmTargetValuesBox.setSizeConstraint(QLayout.SizeConstraint.SetDefaultConstraint)
        self.rpmTargetFRValue = QLineEdit(self.rpmTargetBox)
        self.rpmTargetFRValue.setObjectName(u"rpmTargetFRValue")
        sizePolicy2.setHeightForWidth(self.rpmTargetFRValue.sizePolicy().hasHeightForWidth())
        self.rpmTargetFRValue.setSizePolicy(sizePolicy2)
        self.rpmTargetFRValue.setFont(font)
        self.rpmTargetFRValue.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.rpmTargetFRValue.setFrame(True)
        self.rpmTargetFRValue.setDragEnabled(True)
        self.rpmTargetFRValue.setReadOnly(True)

        self.rpmTargetValuesBox.addWidget(self.rpmTargetFRValue)

        self.rpmTargetFLValue = QLineEdit(self.rpmTargetBox)
        self.rpmTargetFLValue.setObjectName(u"rpmTargetFLValue")
        sizePolicy2.setHeightForWidth(self.rpmTargetFLValue.sizePolicy().hasHeightForWidth())
        self.rpmTargetFLValue.setSizePolicy(sizePolicy2)
        self.rpmTargetFLValue.setFont(font)
        self.rpmTargetFLValue.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.rpmTargetFLValue.setFrame(True)
        self.rpmTargetFLValue.setDragEnabled(True)
        self.rpmTargetFLValue.setReadOnly(True)

        self.rpmTargetValuesBox.addWidget(self.rpmTargetFLValue)

        self.verticalSpacer_10 = QSpacerItem(20, 10, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Fixed)

        self.rpmTargetValuesBox.addItem(self.verticalSpacer_10)

        self.rpmTargetBRValue = QLineEdit(self.rpmTargetBox)
        self.rpmTargetBRValue.setObjectName(u"rpmTargetBRValue")
        sizePolicy2.setHeightForWidth(self.rpmTargetBRValue.sizePolicy().hasHeightForWidth())
        self.rpmTargetBRValue.setSizePolicy(sizePolicy2)
        self.rpmTargetBRValue.setFont(font)
        self.rpmTargetBRValue.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.rpmTargetBRValue.setFrame(True)
        self.rpmTargetBRValue.setDragEnabled(True)
        self.rpmTargetBRValue.setReadOnly(True)

        self.rpmTargetValuesBox.addWidget(self.rpmTargetBRValue)

        self.rpmTargetBLValue = QLineEdit(self.rpmTargetBox)
        self.rpmTargetBLValue.setObjectName(u"rpmTargetBLValue")
        sizePolicy2.setHeightForWidth(self.rpmTargetBLValue.sizePolicy().hasHeightForWidth())
        self.rpmTargetBLValue.setSizePolicy(sizePolicy2)
        self.rpmTargetBLValue.setFont(font)
        self.rpmTargetBLValue.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.rpmTargetBLValue.setFrame(True)
        self.rpmTargetBLValue.setDragEnabled(True)
        self.rpmTargetBLValue.setReadOnly(True)

        self.rpmTargetValuesBox.addWidget(self.rpmTargetBLValue)


        self.horizontalLayout_8.addLayout(self.rpmTargetValuesBox)

        self.rpmTargetUnitsBox = QVBoxLayout()
        self.rpmTargetUnitsBox.setObjectName(u"rpmTargetUnitsBox")
        self.rpmTargetFRUnit = QLabel(self.rpmTargetBox)
        self.rpmTargetFRUnit.setObjectName(u"rpmTargetFRUnit")

        self.rpmTargetUnitsBox.addWidget(self.rpmTargetFRUnit)

        self.rpmTargetFLUnit = QLabel(self.rpmTargetBox)
        self.rpmTargetFLUnit.setObjectName(u"rpmTargetFLUnit")

        self.rpmTargetUnitsBox.addWidget(self.rpmTargetFLUnit)

        self.verticalSpacer_11 = QSpacerItem(20, 10, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Fixed)

        self.rpmTargetUnitsBox.addItem(self.verticalSpacer_11)

        self.rpmTargetBRUnit = QLabel(self.rpmTargetBox)
        self.rpmTargetBRUnit.setObjectName(u"rpmTargetBRUnit")

        self.rpmTargetUnitsBox.addWidget(self.rpmTargetBRUnit)

        self.rpmTargetBLUnit = QLabel(self.rpmTargetBox)
        self.rpmTargetBLUnit.setObjectName(u"rpmTargetBLUnit")

        self.rpmTargetUnitsBox.addWidget(self.rpmTargetBLUnit)


        self.horizontalLayout_8.addLayout(self.rpmTargetUnitsBox)

        self.encPulseCtrsBox = QGroupBox(self.motorCtrlTab)
        self.encPulseCtrsBox.setObjectName(u"encPulseCtrsBox")
        self.encPulseCtrsBox.setGeometry(QRect(28, 220, 200, 165))
        sizePolicy.setHeightForWidth(self.encPulseCtrsBox.sizePolicy().hasHeightForWidth())
        self.encPulseCtrsBox.setSizePolicy(sizePolicy)
        self.horizontalLayout_9 = QHBoxLayout(self.encPulseCtrsBox)
        self.horizontalLayout_9.setObjectName(u"horizontalLayout_9")
        self.encPulseCtrsLabelsBox = QVBoxLayout()
        self.encPulseCtrsLabelsBox.setObjectName(u"encPulseCtrsLabelsBox")
        self.encPulseCtrsFRLabel = QLabel(self.encPulseCtrsBox)
        self.encPulseCtrsFRLabel.setObjectName(u"encPulseCtrsFRLabel")
        sizePolicy5.setHeightForWidth(self.encPulseCtrsFRLabel.sizePolicy().hasHeightForWidth())
        self.encPulseCtrsFRLabel.setSizePolicy(sizePolicy5)

        self.encPulseCtrsLabelsBox.addWidget(self.encPulseCtrsFRLabel)

        self.encPulseCtrsFLLabel = QLabel(self.encPulseCtrsBox)
        self.encPulseCtrsFLLabel.setObjectName(u"encPulseCtrsFLLabel")

        self.encPulseCtrsLabelsBox.addWidget(self.encPulseCtrsFLLabel)

        self.verticalSpacer_12 = QSpacerItem(20, 10, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Fixed)

        self.encPulseCtrsLabelsBox.addItem(self.verticalSpacer_12)

        self.encPulseCtrsBRLabel = QLabel(self.encPulseCtrsBox)
        self.encPulseCtrsBRLabel.setObjectName(u"encPulseCtrsBRLabel")

        self.encPulseCtrsLabelsBox.addWidget(self.encPulseCtrsBRLabel)

        self.encPulseCtrsBLLabel = QLabel(self.encPulseCtrsBox)
        self.encPulseCtrsBLLabel.setObjectName(u"encPulseCtrsBLLabel")

        self.encPulseCtrsLabelsBox.addWidget(self.encPulseCtrsBLLabel)


        self.horizontalLayout_9.addLayout(self.encPulseCtrsLabelsBox)

        self.horizontalSpacer_18 = QSpacerItem(4, 20, QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Minimum)

        self.horizontalLayout_9.addItem(self.horizontalSpacer_18)

        self.encPulseCtrsValuesBox = QVBoxLayout()
        self.encPulseCtrsValuesBox.setObjectName(u"encPulseCtrsValuesBox")
        self.encPulseCtrsValuesBox.setSizeConstraint(QLayout.SizeConstraint.SetDefaultConstraint)
        self.encPulseCtrsFRValue = QLineEdit(self.encPulseCtrsBox)
        self.encPulseCtrsFRValue.setObjectName(u"encPulseCtrsFRValue")
        sizePolicy2.setHeightForWidth(self.encPulseCtrsFRValue.sizePolicy().hasHeightForWidth())
        self.encPulseCtrsFRValue.setSizePolicy(sizePolicy2)
        self.encPulseCtrsFRValue.setFont(font)
        self.encPulseCtrsFRValue.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.encPulseCtrsFRValue.setFrame(True)
        self.encPulseCtrsFRValue.setDragEnabled(True)
        self.encPulseCtrsFRValue.setReadOnly(True)

        self.encPulseCtrsValuesBox.addWidget(self.encPulseCtrsFRValue)

        self.encPulseCtrsFLValue = QLineEdit(self.encPulseCtrsBox)
        self.encPulseCtrsFLValue.setObjectName(u"encPulseCtrsFLValue")
        sizePolicy2.setHeightForWidth(self.encPulseCtrsFLValue.sizePolicy().hasHeightForWidth())
        self.encPulseCtrsFLValue.setSizePolicy(sizePolicy2)
        self.encPulseCtrsFLValue.setFont(font)
        self.encPulseCtrsFLValue.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.encPulseCtrsFLValue.setFrame(True)
        self.encPulseCtrsFLValue.setDragEnabled(True)
        self.encPulseCtrsFLValue.setReadOnly(True)

        self.encPulseCtrsValuesBox.addWidget(self.encPulseCtrsFLValue)

        self.verticalSpacer_13 = QSpacerItem(20, 10, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Fixed)

        self.encPulseCtrsValuesBox.addItem(self.verticalSpacer_13)

        self.encPulseCtrsBRValue = QLineEdit(self.encPulseCtrsBox)
        self.encPulseCtrsBRValue.setObjectName(u"encPulseCtrsBRValue")
        sizePolicy2.setHeightForWidth(self.encPulseCtrsBRValue.sizePolicy().hasHeightForWidth())
        self.encPulseCtrsBRValue.setSizePolicy(sizePolicy2)
        self.encPulseCtrsBRValue.setFont(font)
        self.encPulseCtrsBRValue.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.encPulseCtrsBRValue.setFrame(True)
        self.encPulseCtrsBRValue.setDragEnabled(True)
        self.encPulseCtrsBRValue.setReadOnly(True)

        self.encPulseCtrsValuesBox.addWidget(self.encPulseCtrsBRValue)

        self.encPulseCtrsBLValue = QLineEdit(self.encPulseCtrsBox)
        self.encPulseCtrsBLValue.setObjectName(u"encPulseCtrsBLValue")
        sizePolicy2.setHeightForWidth(self.encPulseCtrsBLValue.sizePolicy().hasHeightForWidth())
        self.encPulseCtrsBLValue.setSizePolicy(sizePolicy2)
        self.encPulseCtrsBLValue.setFont(font)
        self.encPulseCtrsBLValue.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.encPulseCtrsBLValue.setFrame(True)
        self.encPulseCtrsBLValue.setDragEnabled(True)
        self.encPulseCtrsBLValue.setReadOnly(True)

        self.encPulseCtrsValuesBox.addWidget(self.encPulseCtrsBLValue)


        self.horizontalLayout_9.addLayout(self.encPulseCtrsValuesBox)

        self.leftPidBox = QGroupBox(self.motorCtrlTab)
        self.leftPidBox.setObjectName(u"leftPidBox")
        self.leftPidBox.setGeometry(QRect(467, 20, 200, 165))
        sizePolicy.setHeightForWidth(self.leftPidBox.sizePolicy().hasHeightForWidth())
        self.leftPidBox.setSizePolicy(sizePolicy)
        self.horizontalLayout_10 = QHBoxLayout(self.leftPidBox)
        self.horizontalLayout_10.setObjectName(u"horizontalLayout_10")
        self.leftPidLabelsBox = QVBoxLayout()
        self.leftPidLabelsBox.setObjectName(u"leftPidLabelsBox")
        self.leftPidPLabel = QLabel(self.leftPidBox)
        self.leftPidPLabel.setObjectName(u"leftPidPLabel")
        sizePolicy5.setHeightForWidth(self.leftPidPLabel.sizePolicy().hasHeightForWidth())
        self.leftPidPLabel.setSizePolicy(sizePolicy5)

        self.leftPidLabelsBox.addWidget(self.leftPidPLabel)

        self.leftPidILabel = QLabel(self.leftPidBox)
        self.leftPidILabel.setObjectName(u"leftPidILabel")

        self.leftPidLabelsBox.addWidget(self.leftPidILabel)

        self.leftPidDLabel = QLabel(self.leftPidBox)
        self.leftPidDLabel.setObjectName(u"leftPidDLabel")

        self.leftPidLabelsBox.addWidget(self.leftPidDLabel)

        self.verticalSpacer_14 = QSpacerItem(20, 10, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Fixed)

        self.leftPidLabelsBox.addItem(self.verticalSpacer_14)

        self.leftPidOutLabel = QLabel(self.leftPidBox)
        self.leftPidOutLabel.setObjectName(u"leftPidOutLabel")

        self.leftPidLabelsBox.addWidget(self.leftPidOutLabel)


        self.horizontalLayout_10.addLayout(self.leftPidLabelsBox)

        self.horizontalSpacer_19 = QSpacerItem(4, 20, QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Minimum)

        self.horizontalLayout_10.addItem(self.horizontalSpacer_19)

        self.leftPidValuesBox = QVBoxLayout()
        self.leftPidValuesBox.setObjectName(u"leftPidValuesBox")
        self.leftPidValuesBox.setSizeConstraint(QLayout.SizeConstraint.SetDefaultConstraint)
        self.leftPidPValue = QLineEdit(self.leftPidBox)
        self.leftPidPValue.setObjectName(u"leftPidPValue")
        sizePolicy2.setHeightForWidth(self.leftPidPValue.sizePolicy().hasHeightForWidth())
        self.leftPidPValue.setSizePolicy(sizePolicy2)
        self.leftPidPValue.setFont(font)
        self.leftPidPValue.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.leftPidPValue.setFrame(True)
        self.leftPidPValue.setDragEnabled(True)
        self.leftPidPValue.setReadOnly(True)

        self.leftPidValuesBox.addWidget(self.leftPidPValue)

        self.leftPidIValue = QLineEdit(self.leftPidBox)
        self.leftPidIValue.setObjectName(u"leftPidIValue")
        sizePolicy2.setHeightForWidth(self.leftPidIValue.sizePolicy().hasHeightForWidth())
        self.leftPidIValue.setSizePolicy(sizePolicy2)
        self.leftPidIValue.setFont(font)
        self.leftPidIValue.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.leftPidIValue.setFrame(True)
        self.leftPidIValue.setDragEnabled(True)
        self.leftPidIValue.setReadOnly(True)

        self.leftPidValuesBox.addWidget(self.leftPidIValue)

        self.leftPidDValue = QLineEdit(self.leftPidBox)
        self.leftPidDValue.setObjectName(u"leftPidDValue")
        sizePolicy2.setHeightForWidth(self.leftPidDValue.sizePolicy().hasHeightForWidth())
        self.leftPidDValue.setSizePolicy(sizePolicy2)
        self.leftPidDValue.setFont(font)
        self.leftPidDValue.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.leftPidDValue.setFrame(True)
        self.leftPidDValue.setDragEnabled(True)
        self.leftPidDValue.setReadOnly(True)

        self.leftPidValuesBox.addWidget(self.leftPidDValue)

        self.verticalSpacer_15 = QSpacerItem(20, 10, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Fixed)

        self.leftPidValuesBox.addItem(self.verticalSpacer_15)

        self.leftPidOutValue = QLineEdit(self.leftPidBox)
        self.leftPidOutValue.setObjectName(u"leftPidOutValue")
        sizePolicy2.setHeightForWidth(self.leftPidOutValue.sizePolicy().hasHeightForWidth())
        self.leftPidOutValue.setSizePolicy(sizePolicy2)
        self.leftPidOutValue.setFont(font)
        self.leftPidOutValue.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.leftPidOutValue.setFrame(True)
        self.leftPidOutValue.setDragEnabled(True)
        self.leftPidOutValue.setReadOnly(True)

        self.leftPidValuesBox.addWidget(self.leftPidOutValue)


        self.horizontalLayout_10.addLayout(self.leftPidValuesBox)

        self.rightPidBox = QGroupBox(self.motorCtrlTab)
        self.rightPidBox.setObjectName(u"rightPidBox")
        self.rightPidBox.setGeometry(QRect(467, 200, 200, 165))
        sizePolicy.setHeightForWidth(self.rightPidBox.sizePolicy().hasHeightForWidth())
        self.rightPidBox.setSizePolicy(sizePolicy)
        self.horizontalLayout_11 = QHBoxLayout(self.rightPidBox)
        self.horizontalLayout_11.setObjectName(u"horizontalLayout_11")
        self.rightPidLabelsBox = QVBoxLayout()
        self.rightPidLabelsBox.setObjectName(u"rightPidLabelsBox")
        self.rightPidPLabel = QLabel(self.rightPidBox)
        self.rightPidPLabel.setObjectName(u"rightPidPLabel")
        sizePolicy5.setHeightForWidth(self.rightPidPLabel.sizePolicy().hasHeightForWidth())
        self.rightPidPLabel.setSizePolicy(sizePolicy5)

        self.rightPidLabelsBox.addWidget(self.rightPidPLabel)

        self.rightPidILabel = QLabel(self.rightPidBox)
        self.rightPidILabel.setObjectName(u"rightPidILabel")

        self.rightPidLabelsBox.addWidget(self.rightPidILabel)

        self.rightPidDLabel = QLabel(self.rightPidBox)
        self.rightPidDLabel.setObjectName(u"rightPidDLabel")

        self.rightPidLabelsBox.addWidget(self.rightPidDLabel)

        self.verticalSpacer_16 = QSpacerItem(20, 10, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Fixed)

        self.rightPidLabelsBox.addItem(self.verticalSpacer_16)

        self.rightPidOutLabel = QLabel(self.rightPidBox)
        self.rightPidOutLabel.setObjectName(u"rightPidOutLabel")

        self.rightPidLabelsBox.addWidget(self.rightPidOutLabel)


        self.horizontalLayout_11.addLayout(self.rightPidLabelsBox)

        self.horizontalSpacer_20 = QSpacerItem(4, 20, QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Minimum)

        self.horizontalLayout_11.addItem(self.horizontalSpacer_20)

        self.rightPidValuesBox = QVBoxLayout()
        self.rightPidValuesBox.setObjectName(u"rightPidValuesBox")
        self.rightPidValuesBox.setSizeConstraint(QLayout.SizeConstraint.SetDefaultConstraint)
        self.rightPidPValue = QLineEdit(self.rightPidBox)
        self.rightPidPValue.setObjectName(u"rightPidPValue")
        sizePolicy2.setHeightForWidth(self.rightPidPValue.sizePolicy().hasHeightForWidth())
        self.rightPidPValue.setSizePolicy(sizePolicy2)
        self.rightPidPValue.setFont(font)
        self.rightPidPValue.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.rightPidPValue.setFrame(True)
        self.rightPidPValue.setDragEnabled(True)
        self.rightPidPValue.setReadOnly(True)

        self.rightPidValuesBox.addWidget(self.rightPidPValue)

        self.rightPidIValue = QLineEdit(self.rightPidBox)
        self.rightPidIValue.setObjectName(u"rightPidIValue")
        sizePolicy2.setHeightForWidth(self.rightPidIValue.sizePolicy().hasHeightForWidth())
        self.rightPidIValue.setSizePolicy(sizePolicy2)
        self.rightPidIValue.setFont(font)
        self.rightPidIValue.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.rightPidIValue.setFrame(True)
        self.rightPidIValue.setDragEnabled(True)
        self.rightPidIValue.setReadOnly(True)

        self.rightPidValuesBox.addWidget(self.rightPidIValue)

        self.rightPidDValue = QLineEdit(self.rightPidBox)
        self.rightPidDValue.setObjectName(u"rightPidDValue")
        sizePolicy2.setHeightForWidth(self.rightPidDValue.sizePolicy().hasHeightForWidth())
        self.rightPidDValue.setSizePolicy(sizePolicy2)
        self.rightPidDValue.setFont(font)
        self.rightPidDValue.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.rightPidDValue.setFrame(True)
        self.rightPidDValue.setDragEnabled(True)
        self.rightPidDValue.setReadOnly(True)

        self.rightPidValuesBox.addWidget(self.rightPidDValue)

        self.verticalSpacer_17 = QSpacerItem(20, 10, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Fixed)

        self.rightPidValuesBox.addItem(self.verticalSpacer_17)

        self.rightPidOutValue = QLineEdit(self.rightPidBox)
        self.rightPidOutValue.setObjectName(u"rightPidOutValue")
        sizePolicy2.setHeightForWidth(self.rightPidOutValue.sizePolicy().hasHeightForWidth())
        self.rightPidOutValue.setSizePolicy(sizePolicy2)
        self.rightPidOutValue.setFont(font)
        self.rightPidOutValue.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.rightPidOutValue.setFrame(True)
        self.rightPidOutValue.setDragEnabled(True)
        self.rightPidOutValue.setReadOnly(True)

        self.rightPidValuesBox.addWidget(self.rightPidOutValue)


        self.horizontalLayout_11.addLayout(self.rightPidValuesBox)

        self.pages.addTab(self.motorCtrlTab, "")
        self.powerTab = QWidget()
        self.powerTab.setObjectName(u"powerTab")
        self.batteryInfoBox = QGroupBox(self.powerTab)
        self.batteryInfoBox.setObjectName(u"batteryInfoBox")
        self.batteryInfoBox.setGeometry(QRect(28, 20, 195, 110))
        self.horizontalLayout_12 = QHBoxLayout(self.batteryInfoBox)
        self.horizontalLayout_12.setObjectName(u"horizontalLayout_12")
        self.batteryInfoLabels = QVBoxLayout()
        self.batteryInfoLabels.setObjectName(u"batteryInfoLabels")
        self.batteryVoltageLabel = QLabel(self.batteryInfoBox)
        self.batteryVoltageLabel.setObjectName(u"batteryVoltageLabel")
        sizePolicy6.setHeightForWidth(self.batteryVoltageLabel.sizePolicy().hasHeightForWidth())
        self.batteryVoltageLabel.setSizePolicy(sizePolicy6)

        self.batteryInfoLabels.addWidget(self.batteryVoltageLabel)

        self.batteryCurrentLabel = QLabel(self.batteryInfoBox)
        self.batteryCurrentLabel.setObjectName(u"batteryCurrentLabel")
        sizePolicy6.setHeightForWidth(self.batteryCurrentLabel.sizePolicy().hasHeightForWidth())
        self.batteryCurrentLabel.setSizePolicy(sizePolicy6)

        self.batteryInfoLabels.addWidget(self.batteryCurrentLabel)


        self.horizontalLayout_12.addLayout(self.batteryInfoLabels)

        self.horizontalSpacer_21 = QSpacerItem(8, 20, QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Minimum)

        self.horizontalLayout_12.addItem(self.horizontalSpacer_21)

        self.batteryInfoValues = QVBoxLayout()
        self.batteryInfoValues.setObjectName(u"batteryInfoValues")
        self.batteryVoltageValue = QLCDNumber(self.batteryInfoBox)
        self.batteryVoltageValue.setObjectName(u"batteryVoltageValue")
        sizePolicy1.setHeightForWidth(self.batteryVoltageValue.sizePolicy().hasHeightForWidth())
        self.batteryVoltageValue.setSizePolicy(sizePolicy1)
        self.batteryVoltageValue.setFont(font1)
        self.batteryVoltageValue.setDigitCount(6)
        self.batteryVoltageValue.setSegmentStyle(QLCDNumber.SegmentStyle.Flat)
        self.batteryVoltageValue.setProperty("value", 12.458000000000000)

        self.batteryInfoValues.addWidget(self.batteryVoltageValue)

        self.batteryCurrentValue = QLCDNumber(self.batteryInfoBox)
        self.batteryCurrentValue.setObjectName(u"batteryCurrentValue")
        sizePolicy1.setHeightForWidth(self.batteryCurrentValue.sizePolicy().hasHeightForWidth())
        self.batteryCurrentValue.setSizePolicy(sizePolicy1)
        self.batteryCurrentValue.setFont(font1)
        self.batteryCurrentValue.setDigitCount(6)
        self.batteryCurrentValue.setSegmentStyle(QLCDNumber.SegmentStyle.Flat)
        self.batteryCurrentValue.setProperty("value", 3.284000000000000)

        self.batteryInfoValues.addWidget(self.batteryCurrentValue)


        self.horizontalLayout_12.addLayout(self.batteryInfoValues)

        self.powerTabNote = QLabel(self.powerTab)
        self.powerTabNote.setObjectName(u"powerTabNote")
        self.powerTabNote.setGeometry(QRect(20, 510, 251, 18))
        self.pages.addTab(self.powerTab, "")
        self.diagTab = QWidget()
        self.diagTab.setObjectName(u"diagTab")
        self.diagMsgsBox = QGroupBox(self.diagTab)
        self.diagMsgsBox.setObjectName(u"diagMsgsBox")
        self.diagMsgsBox.setGeometry(QRect(305, 20, 695, 505))
        sizePolicy1.setHeightForWidth(self.diagMsgsBox.sizePolicy().hasHeightForWidth())
        self.diagMsgsBox.setSizePolicy(sizePolicy1)
        self.verticalLayout_2 = QVBoxLayout(self.diagMsgsBox)
        self.verticalLayout_2.setObjectName(u"verticalLayout_2")
        self.diagMsgsText = QTextBrowser(self.diagMsgsBox)
        self.diagMsgsText.setObjectName(u"diagMsgsText")
        sizePolicy8 = QSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Expanding)
        sizePolicy8.setHorizontalStretch(0)
        sizePolicy8.setVerticalStretch(0)
        sizePolicy8.setHeightForWidth(self.diagMsgsText.sizePolicy().hasHeightForWidth())
        self.diagMsgsText.setSizePolicy(sizePolicy8)

        self.verticalLayout_2.addWidget(self.diagMsgsText)

        self.diagMsgsOptionsBox = QHBoxLayout()
        self.diagMsgsOptionsBox.setObjectName(u"diagMsgsOptionsBox")
        self.diagMsgsOptionsBox.setContentsMargins(5, -1, 5, -1)
        self.diagMsgOptsAutoScrollCheck = QCheckBox(self.diagMsgsBox)
        self.diagMsgOptsAutoScrollCheck.setObjectName(u"diagMsgOptsAutoScrollCheck")
        sizePolicy2.setHeightForWidth(self.diagMsgOptsAutoScrollCheck.sizePolicy().hasHeightForWidth())
        self.diagMsgOptsAutoScrollCheck.setSizePolicy(sizePolicy2)
        self.diagMsgOptsAutoScrollCheck.setChecked(True)

        self.diagMsgsOptionsBox.addWidget(self.diagMsgOptsAutoScrollCheck)

        self.horizontalSpacer_22 = QSpacerItem(10, 20, QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Minimum)

        self.diagMsgsOptionsBox.addItem(self.horizontalSpacer_22)

        self.diagMsgOptsShowTimeCheck = QCheckBox(self.diagMsgsBox)
        self.diagMsgOptsShowTimeCheck.setObjectName(u"diagMsgOptsShowTimeCheck")
        sizePolicy2.setHeightForWidth(self.diagMsgOptsShowTimeCheck.sizePolicy().hasHeightForWidth())
        self.diagMsgOptsShowTimeCheck.setSizePolicy(sizePolicy2)

        self.diagMsgsOptionsBox.addWidget(self.diagMsgOptsShowTimeCheck)

        self.horizontalSpacer_23 = QSpacerItem(10, 20, QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Minimum)

        self.diagMsgsOptionsBox.addItem(self.horizontalSpacer_23)

        self.diagMsgOptsShowKeyValCheck = QCheckBox(self.diagMsgsBox)
        self.diagMsgOptsShowKeyValCheck.setObjectName(u"diagMsgOptsShowKeyValCheck")
        sizePolicy2.setHeightForWidth(self.diagMsgOptsShowKeyValCheck.sizePolicy().hasHeightForWidth())
        self.diagMsgOptsShowKeyValCheck.setSizePolicy(sizePolicy2)

        self.diagMsgsOptionsBox.addWidget(self.diagMsgOptsShowKeyValCheck)

        self.horizontalSpacer_24 = QSpacerItem(40, 20, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.diagMsgsOptionsBox.addItem(self.horizontalSpacer_24)


        self.verticalLayout_2.addLayout(self.diagMsgsOptionsBox)

        self.calibBox = QGroupBox(self.diagTab)
        self.calibBox.setObjectName(u"calibBox")
        self.calibBox.setGeometry(QRect(28, 20, 255, 170))
        self.verticalLayout_3 = QVBoxLayout(self.calibBox)
        self.verticalLayout_3.setObjectName(u"verticalLayout_3")
        self.verticalLayout_3.setContentsMargins(-1, -1, -1, 7)
        self.calibPidTuneButtonL = QPushButton(self.calibBox)
        self.calibPidTuneButtonL.setObjectName(u"calibPidTuneButtonL")

        self.verticalLayout_3.addWidget(self.calibPidTuneButtonL)

        self.calibPidTuneButtonR = QPushButton(self.calibBox)
        self.calibPidTuneButtonR.setObjectName(u"calibPidTuneButtonR")

        self.verticalLayout_3.addWidget(self.calibPidTuneButtonR)

        self.calibCliffOffsetButton = QPushButton(self.calibBox)
        self.calibCliffOffsetButton.setObjectName(u"calibCliffOffsetButton")

        self.verticalLayout_3.addWidget(self.calibCliffOffsetButton)

        self.calibImuCompassButton = QPushButton(self.calibBox)
        self.calibImuCompassButton.setObjectName(u"calibImuCompassButton")

        self.verticalLayout_3.addWidget(self.calibImuCompassButton)

        self.selftestBox = QGroupBox(self.diagTab)
        self.selftestBox.setObjectName(u"selftestBox")
        self.selftestBox.setGeometry(QRect(28, 210, 255, 110))
        self.verticalLayout_4 = QVBoxLayout(self.selftestBox)
        self.verticalLayout_4.setObjectName(u"verticalLayout_4")
        self.verticalLayout_4.setContentsMargins(-1, -1, -1, 7)
        self.selftestPicoAButton = QPushButton(self.selftestBox)
        self.selftestPicoAButton.setObjectName(u"selftestPicoAButton")

        self.verticalLayout_4.addWidget(self.selftestPicoAButton)

        self.selftestPicoBButton = QPushButton(self.selftestBox)
        self.selftestPicoBButton.setObjectName(u"selftestPicoBButton")

        self.verticalLayout_4.addWidget(self.selftestPicoBButton)

        self.pages.addTab(self.diagTab, "")

        self.gridLayout_1.addWidget(self.pages, 0, 0, 1, 1)

        MainWindow.setCentralWidget(self.centralWidget)
        self.menubar = QMenuBar(MainWindow)
        self.menubar.setObjectName(u"menubar")
        self.menubar.setEnabled(True)
        self.menubar.setGeometry(QRect(0, 0, 1024, 23))
        self.menubar.setNativeMenuBar(True)
        self.menuFile = QMenu(self.menubar)
        self.menuFile.setObjectName(u"menuFile")
        self.menuFile.setCursor(QCursor(Qt.CursorShape.CrossCursor))
        self.menuConnection = QMenu(self.menubar)
        self.menuConnection.setObjectName(u"menuConnection")
        self.menuConnection.setCursor(QCursor(Qt.CursorShape.CrossCursor))
        self.menuConfiguration = QMenu(self.menubar)
        self.menuConfiguration.setObjectName(u"menuConfiguration")
        self.menuConfiguration.setCursor(QCursor(Qt.CursorShape.CrossCursor))
        MainWindow.setMenuBar(self.menubar)

        self.menubar.addAction(self.menuFile.menuAction())
        self.menubar.addAction(self.menuConnection.menuAction())
        self.menubar.addAction(self.menuConfiguration.menuAction())
        self.menuFile.addAction(self.actionRestart_Application)
        self.menuFile.addSeparator()
        self.menuFile.addAction(self.actionRestart_RPi)
        self.menuFile.addAction(self.actionShutdown_System)
        self.menuConnection.addAction(self.actionCheck_Robot_Connection)
        self.menuConnection.addSeparator()
        self.menuConnection.addAction(self.actionCheck_Wi_Fi_Connection)
        self.menuConnection.addAction(self.actionCheck_WAN_Connection)
        self.menuConfiguration.addAction(self.actionJoystick)
        self.menuConfiguration.addAction(self.actionTest_Buttons_Switches)
        self.menuConfiguration.addSeparator()
        self.menuConfiguration.addAction(self.actionAbout)

        self.retranslateUi(MainWindow)

        self.pages.setCurrentIndex(0)


        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"ROS Remote UI", None))
        self.actionRestart_Application.setText(QCoreApplication.translate("MainWindow", u"Restart Application", None))
        self.actionRestart_RPi.setText(QCoreApplication.translate("MainWindow", u"Restart System", None))
        self.actionShutdown_System.setText(QCoreApplication.translate("MainWindow", u"Shutdown System", None))
        self.actionCheck_Robot_Connection.setText(QCoreApplication.translate("MainWindow", u"Check Robot Connection", None))
        self.actionCheck_Wi_Fi_Connection.setText(QCoreApplication.translate("MainWindow", u"Check Wi-Fi Connection", None))
        self.actionCheck_WAN_Connection.setText(QCoreApplication.translate("MainWindow", u"Check WAN Connection", None))
        self.actionJoystick.setText(QCoreApplication.translate("MainWindow", u"Configure Joystick", None))
        self.actionTest_Buttons_Switches.setText(QCoreApplication.translate("MainWindow", u"Test Buttons", None))
        self.actionAbout.setText(QCoreApplication.translate("MainWindow", u"About", None))
        self.viewport.setText("")
        self.viewportOptsBox.setTitle(QCoreApplication.translate("MainWindow", u"Viewport", None))
        self.viewportFpsLabel.setText(QCoreApplication.translate("MainWindow", u"FPS:", None))
        self.viewportFpsValue.setText(QCoreApplication.translate("MainWindow", u"0", None))
        self.viewportSelector.setItemText(0, QCoreApplication.translate("MainWindow", u"Camera View", None))
        self.viewportSelector.setItemText(1, QCoreApplication.translate("MainWindow", u"Camera View (Overlay)", None))
        self.viewportSelector.setItemText(2, QCoreApplication.translate("MainWindow", u"LiDAR View", None))

        self.viewportSelector.setCurrentText(QCoreApplication.translate("MainWindow", u"Camera View", None))
        self.camLedsBox.setTitle(QCoreApplication.translate("MainWindow", u"Camera LEDs", None))
        self.camLedsBrightnessLabel.setText(QCoreApplication.translate("MainWindow", u"0", None))
        self.camLed1Check.setText(QCoreApplication.translate("MainWindow", u"1", None))
        self.camLed2Check.setText(QCoreApplication.translate("MainWindow", u"2", None))
        self.camLed3Check.setText(QCoreApplication.translate("MainWindow", u"3", None))
        self.camLed4Check.setText(QCoreApplication.translate("MainWindow", u"4", None))
        self.telemetryBox.setTitle(QCoreApplication.translate("MainWindow", u"Telemetry", None))
        self.teleBatteryLabel.setText(QCoreApplication.translate("MainWindow", u"Battery:", None))
        self.teleBatteryValue.setText(QCoreApplication.translate("MainWindow", u"0", None))
        self.teleBatteryUnits.setText(QCoreApplication.translate("MainWindow", u"volts", None))
        self.pages.setTabText(self.pages.indexOf(self.mainTab), QCoreApplication.translate("MainWindow", u"Main", None))
        self.microswSensBox.setTitle(QCoreApplication.translate("MainWindow", u"Microswitches", None))
        self.microswSensIndicatorFL.setText(QCoreApplication.translate("MainWindow", u"Front-L", None))
        self.microswSensIndicatorBL.setText(QCoreApplication.translate("MainWindow", u"Back-L", None))
        self.microswSensIndicatorFR.setText(QCoreApplication.translate("MainWindow", u"Front-R", None))
        self.microswSensIndicatorBR.setText(QCoreApplication.translate("MainWindow", u"Back-R", None))
        self.imuSensBox.setTitle(QCoreApplication.translate("MainWindow", u"IMU", None))
        self.imuSensAccelXLabel.setText(QCoreApplication.translate("MainWindow", u"Accel. X", None))
        self.imuSensAccelXValue.setText(QCoreApplication.translate("MainWindow", u"0", None))
        self.imuSensAccelXUnit.setText(QCoreApplication.translate("MainWindow", u"m/s", None))
        self.imuSensAccelYLabel.setText(QCoreApplication.translate("MainWindow", u"Accel. Y", None))
        self.imuSensAccelYValue.setText(QCoreApplication.translate("MainWindow", u"0", None))
        self.imuSensAccelYUnit.setText(QCoreApplication.translate("MainWindow", u"m/s", None))
        self.imuSensAccelZLabel.setText(QCoreApplication.translate("MainWindow", u"Accel. Z", None))
        self.imuSensAccelZValue.setText(QCoreApplication.translate("MainWindow", u"0", None))
        self.imuSensAccelZUnit.setText(QCoreApplication.translate("MainWindow", u"m/s", None))
        self.imuSensGyroXLabel.setText(QCoreApplication.translate("MainWindow", u"Gyro. X", None))
        self.imuSensGyroXValue.setText(QCoreApplication.translate("MainWindow", u"0", None))
        self.imuSensGyroXUnit.setText(QCoreApplication.translate("MainWindow", u"\u00b0/s", None))
        self.imuSensGyroYLabel.setText(QCoreApplication.translate("MainWindow", u"Gyro. Y", None))
        self.imuSensGyroYValue.setText(QCoreApplication.translate("MainWindow", u"0", None))
        self.imuSensGyroYUnit.setText(QCoreApplication.translate("MainWindow", u"\u00b0/s", None))
        self.imuSensGyroZLabel.setText(QCoreApplication.translate("MainWindow", u"Gyro. Z", None))
        self.imuSensGyroZValue.setText(QCoreApplication.translate("MainWindow", u"0", None))
        self.imuSensGyroZUnit.setText(QCoreApplication.translate("MainWindow", u"\u00b0/s", None))
        self.imuSensCompXLabel.setText(QCoreApplication.translate("MainWindow", u"Comp. X", None))
        self.imuSensCompXValue.setText(QCoreApplication.translate("MainWindow", u"0", None))
        self.imuSensCompXUnit.setText(QCoreApplication.translate("MainWindow", u"\u00b0", None))
        self.imuSensCompYLabel.setText(QCoreApplication.translate("MainWindow", u"Comp. Y", None))
        self.imuSensCompYValue.setText(QCoreApplication.translate("MainWindow", u"0", None))
        self.imuSensCompYUnit.setText(QCoreApplication.translate("MainWindow", u"\u00b0", None))
        self.imuSensCompZLabel.setText(QCoreApplication.translate("MainWindow", u"Comp. Z", None))
        self.imuSensCompZValue.setText(QCoreApplication.translate("MainWindow", u"0", None))
        self.imuSensCompZUnit.setText(QCoreApplication.translate("MainWindow", u"\u00b0", None))
        self.envSensBox.setTitle(QCoreApplication.translate("MainWindow", u"Environment", None))
        self.envSensTempLabel.setText(QCoreApplication.translate("MainWindow", u"DHT Temp", None))
        self.envSensHumLabel.setText(QCoreApplication.translate("MainWindow", u"DHT Hum", None))
        self.envSensImuTempLabel.setText(QCoreApplication.translate("MainWindow", u"IMU Temp", None))
        self.envSensPicoATempLabel.setText(QCoreApplication.translate("MainWindow", u"Pico A Temp", None))
        self.envSensPicoBTempLabel.setText(QCoreApplication.translate("MainWindow", u"Pico B Temp", None))
        self.envSensPiTempLabel.setText(QCoreApplication.translate("MainWindow", u"Pi 5 Temp", None))
        self.envSensTempValue.setText(QCoreApplication.translate("MainWindow", u"0", None))
        self.envSensHumValue.setText(QCoreApplication.translate("MainWindow", u"0", None))
        self.envSensImuTempValue.setText(QCoreApplication.translate("MainWindow", u"0", None))
        self.envSensPicoATempValue.setText(QCoreApplication.translate("MainWindow", u"0", None))
        self.envSensPicoBTempValue.setText(QCoreApplication.translate("MainWindow", u"0", None))
        self.envSensPiTempValue.setText(QCoreApplication.translate("MainWindow", u"0", None))
        self.envSensTempUnit.setText(QCoreApplication.translate("MainWindow", u"\u00b0", None))
        self.envSensHumUnit.setText(QCoreApplication.translate("MainWindow", u"%", None))
        self.envSensImuTempUnit.setText(QCoreApplication.translate("MainWindow", u"\u00b0", None))
        self.envSensPicoATempUnit.setText(QCoreApplication.translate("MainWindow", u"\u00b0", None))
        self.envSensPicoBTempUnit.setText(QCoreApplication.translate("MainWindow", u"\u00b0", None))
        self.envSensPiTempUnit.setText(QCoreApplication.translate("MainWindow", u"\u00b0", None))
        self.cliffSensBox.setTitle(QCoreApplication.translate("MainWindow", u"Cliff Sensors", None))
        self.cliffSensIndicatorF2.setText(QCoreApplication.translate("MainWindow", u"Front 2", None))
        self.cliffSensIndicatorB2.setText(QCoreApplication.translate("MainWindow", u"Back 2", None))
        self.cliffSensIndicatorF3.setText(QCoreApplication.translate("MainWindow", u"Front 3", None))
        self.cliffSensIndicatorB1.setText(QCoreApplication.translate("MainWindow", u"Back 1", None))
        self.cliffSensIndicatorB3.setText(QCoreApplication.translate("MainWindow", u"Back 3", None))
        self.cliffSensIndicatorF1.setText(QCoreApplication.translate("MainWindow", u"Front 1", None))
        self.cliffSensIndicatorB4.setText(QCoreApplication.translate("MainWindow", u"Back 4", None))
        self.cliffSensIndicatorF4.setText(QCoreApplication.translate("MainWindow", u"Front 4", None))
        self.ultrasonicSensBox.setTitle(QCoreApplication.translate("MainWindow", u"Ultrasonic", None))
        self.ultrasonicSensLabelF.setText(QCoreApplication.translate("MainWindow", u"Front", None))
        self.ultrasonicSensLabelB.setText(QCoreApplication.translate("MainWindow", u"Back", None))
        self.ultrasonicSensLabelR.setText(QCoreApplication.translate("MainWindow", u"Right", None))
        self.ultrasonicSensLabelL.setText(QCoreApplication.translate("MainWindow", u"Left", None))
        self.ultrasonicSensValueF.setText(QCoreApplication.translate("MainWindow", u"0", None))
        self.ultrasonicSensValueB.setText(QCoreApplication.translate("MainWindow", u"0", None))
        self.ultrasonicSensValueR.setText(QCoreApplication.translate("MainWindow", u"0", None))
        self.ultrasonicSensValueL.setText(QCoreApplication.translate("MainWindow", u"0", None))
        self.ultrasonicSensUnitF.setText(QCoreApplication.translate("MainWindow", u"m", None))
        self.ultrasonicSensUnitB.setText(QCoreApplication.translate("MainWindow", u"m", None))
        self.ultrasonicSensUnitR.setText(QCoreApplication.translate("MainWindow", u"m", None))
        self.ultrasonicSensUnitL.setText(QCoreApplication.translate("MainWindow", u"m", None))
        self.batterySensBox.setTitle(QCoreApplication.translate("MainWindow", u"Battery", None))
        self.batterySensVoltageLabel.setText(QCoreApplication.translate("MainWindow", u"Voltage", None))
        self.batterySensCurrentLabel.setText(QCoreApplication.translate("MainWindow", u"Current", None))
        self.enableDisableBox.setTitle(QCoreApplication.translate("MainWindow", u"Enable/Disable", None))
        self.toggleEmittersEnabled.setText(QCoreApplication.translate("MainWindow", u"Emitters", None))
        self.togglePicoARelaEnable.setText(QCoreApplication.translate("MainWindow", u"Pico A Relay", None))
        self.pages.setTabText(self.pages.indexOf(self.sensorsTab), QCoreApplication.translate("MainWindow", u"Sensors", None))
        self.encOdomBox.setTitle(QCoreApplication.translate("MainWindow", u"Encoder Odometry", None))
        self.encOdomPosXLabel.setText(QCoreApplication.translate("MainWindow", u"Pos. X", None))
        self.encOdomPosYLabel.setText(QCoreApplication.translate("MainWindow", u"Pos. Y", None))
        self.encOdomYawLabel.setText(QCoreApplication.translate("MainWindow", u"Yaw", None))
        self.encOdomLinVelLabel.setText(QCoreApplication.translate("MainWindow", u"Lin. Vel.", None))
        self.encOdomAngVelLabel.setText(QCoreApplication.translate("MainWindow", u"Ang. Vel.", None))
        self.encOdomPosXValue.setText(QCoreApplication.translate("MainWindow", u"0", None))
        self.encOdomPosYValue.setText(QCoreApplication.translate("MainWindow", u"0", None))
        self.encOdomYawValue.setText(QCoreApplication.translate("MainWindow", u"0", None))
        self.encOdomLinVelValue.setText(QCoreApplication.translate("MainWindow", u"0", None))
        self.encOdomAngVelValue.setText(QCoreApplication.translate("MainWindow", u"0", None))
        self.encOdomPosXUnit.setText(QCoreApplication.translate("MainWindow", u"m", None))
        self.encOdomPosYUnit.setText(QCoreApplication.translate("MainWindow", u"m", None))
        self.encOdomYawUnit.setText(QCoreApplication.translate("MainWindow", u"\u00b0", None))
        self.encOdomLinVelUnit.setText(QCoreApplication.translate("MainWindow", u"m/s", None))
        self.encOdomAngVelUnit.setText(QCoreApplication.translate("MainWindow", u"r/s", None))
        self.mtrCtrlEnableBox.setTitle(QCoreApplication.translate("MainWindow", u"Motor Controllers", None))
        self.mtrCtrlEnableButton.setText(QCoreApplication.translate("MainWindow", u"Enable", None))
        self.mtrCtrlDisableButton.setText(QCoreApplication.translate("MainWindow", u"Disable", None))
        self.mtrCtrlEnableLIndicator.setText(QCoreApplication.translate("MainWindow", u"Left", None))
        self.mtrCtrlEnableRIndicator.setText(QCoreApplication.translate("MainWindow", u"Right", None))
        self.rpmMeasuredBox.setTitle(QCoreApplication.translate("MainWindow", u"RPMs - Measured", None))
        self.rpmMeasuredFRLabel.setText(QCoreApplication.translate("MainWindow", u"Front-R", None))
        self.rpmMeasuredFLLabel.setText(QCoreApplication.translate("MainWindow", u"Front-L", None))
        self.rpmMeasuredBRLabel.setText(QCoreApplication.translate("MainWindow", u"Back-R", None))
        self.rpmMeasuredBLLabel.setText(QCoreApplication.translate("MainWindow", u"Back-L", None))
        self.rpmMeasuredFRValue.setText(QCoreApplication.translate("MainWindow", u"0", None))
        self.rpmMeasuredFLValue.setText(QCoreApplication.translate("MainWindow", u"0", None))
        self.rpmMeasuredBRValue.setText(QCoreApplication.translate("MainWindow", u"0", None))
        self.rpmMeasuredBLValue.setText(QCoreApplication.translate("MainWindow", u"0", None))
        self.rpmMeasuredFRUnit.setText(QCoreApplication.translate("MainWindow", u"rpm", None))
        self.rpmMeasuredFLUnit.setText(QCoreApplication.translate("MainWindow", u"rpm", None))
        self.rpmMeasuredBRUnit.setText(QCoreApplication.translate("MainWindow", u"rpm", None))
        self.rpmMeasuredBLUnit.setText(QCoreApplication.translate("MainWindow", u"rpm", None))
        self.rpmTargetBox.setTitle(QCoreApplication.translate("MainWindow", u"RPMs - Target", None))
        self.rpmTargetFRLabel.setText(QCoreApplication.translate("MainWindow", u"Front-R", None))
        self.rpmTargetFLLabel.setText(QCoreApplication.translate("MainWindow", u"Front-L", None))
        self.rpmTargetBRLabel.setText(QCoreApplication.translate("MainWindow", u"Back-R", None))
        self.rpmTargetBLLabel.setText(QCoreApplication.translate("MainWindow", u"Back-L", None))
        self.rpmTargetFRValue.setText(QCoreApplication.translate("MainWindow", u"0", None))
        self.rpmTargetFLValue.setText(QCoreApplication.translate("MainWindow", u"0", None))
        self.rpmTargetBRValue.setText(QCoreApplication.translate("MainWindow", u"0", None))
        self.rpmTargetBLValue.setText(QCoreApplication.translate("MainWindow", u"0", None))
        self.rpmTargetFRUnit.setText(QCoreApplication.translate("MainWindow", u"rpm", None))
        self.rpmTargetFLUnit.setText(QCoreApplication.translate("MainWindow", u"rpm", None))
        self.rpmTargetBRUnit.setText(QCoreApplication.translate("MainWindow", u"rpm", None))
        self.rpmTargetBLUnit.setText(QCoreApplication.translate("MainWindow", u"rpm", None))
        self.encPulseCtrsBox.setTitle(QCoreApplication.translate("MainWindow", u"Encoder Pulse Counters", None))
        self.encPulseCtrsFRLabel.setText(QCoreApplication.translate("MainWindow", u"Front-R", None))
        self.encPulseCtrsFLLabel.setText(QCoreApplication.translate("MainWindow", u"Front-L", None))
        self.encPulseCtrsBRLabel.setText(QCoreApplication.translate("MainWindow", u"Back-R", None))
        self.encPulseCtrsBLLabel.setText(QCoreApplication.translate("MainWindow", u"Back-L", None))
        self.encPulseCtrsFRValue.setText(QCoreApplication.translate("MainWindow", u"0", None))
        self.encPulseCtrsFLValue.setText(QCoreApplication.translate("MainWindow", u"0", None))
        self.encPulseCtrsBRValue.setText(QCoreApplication.translate("MainWindow", u"0", None))
        self.encPulseCtrsBLValue.setText(QCoreApplication.translate("MainWindow", u"0", None))
        self.leftPidBox.setTitle(QCoreApplication.translate("MainWindow", u"Left Controller PID", None))
        self.leftPidPLabel.setText(QCoreApplication.translate("MainWindow", u"PID (P)", None))
        self.leftPidILabel.setText(QCoreApplication.translate("MainWindow", u"PID (I)", None))
        self.leftPidDLabel.setText(QCoreApplication.translate("MainWindow", u"PID (D)", None))
        self.leftPidOutLabel.setText(QCoreApplication.translate("MainWindow", u"PID Out", None))
        self.leftPidPValue.setText(QCoreApplication.translate("MainWindow", u"0", None))
        self.leftPidIValue.setText(QCoreApplication.translate("MainWindow", u"0", None))
        self.leftPidDValue.setText(QCoreApplication.translate("MainWindow", u"0", None))
        self.leftPidOutValue.setText(QCoreApplication.translate("MainWindow", u"0", None))
        self.rightPidBox.setTitle(QCoreApplication.translate("MainWindow", u"Right Controller PID", None))
        self.rightPidPLabel.setText(QCoreApplication.translate("MainWindow", u"PID (P)", None))
        self.rightPidILabel.setText(QCoreApplication.translate("MainWindow", u"PID (I)", None))
        self.rightPidDLabel.setText(QCoreApplication.translate("MainWindow", u"PID (D)", None))
        self.rightPidOutLabel.setText(QCoreApplication.translate("MainWindow", u"PID Out", None))
        self.rightPidPValue.setText(QCoreApplication.translate("MainWindow", u"0", None))
        self.rightPidIValue.setText(QCoreApplication.translate("MainWindow", u"0", None))
        self.rightPidDValue.setText(QCoreApplication.translate("MainWindow", u"0", None))
        self.rightPidOutValue.setText(QCoreApplication.translate("MainWindow", u"0", None))
        self.pages.setTabText(self.pages.indexOf(self.motorCtrlTab), QCoreApplication.translate("MainWindow", u"Motor Controllers", None))
        self.batteryInfoBox.setTitle(QCoreApplication.translate("MainWindow", u"Battery", None))
        self.batteryVoltageLabel.setText(QCoreApplication.translate("MainWindow", u"Voltage", None))
        self.batteryCurrentLabel.setText(QCoreApplication.translate("MainWindow", u"Current", None))
        self.powerTabNote.setText(QCoreApplication.translate("MainWindow", u"This page is for ROS Robot version 2.", None))
        self.pages.setTabText(self.pages.indexOf(self.powerTab), QCoreApplication.translate("MainWindow", u"Power", None))
        self.diagMsgsBox.setTitle(QCoreApplication.translate("MainWindow", u"Diagnostic Messages", None))
        self.diagMsgsText.setHtml(QCoreApplication.translate("MainWindow", u"<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><meta charset=\"utf-8\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"hr { height: 1px; border-width: 0; }\n"
"li.unchecked::marker { content: \"\\2610\"; }\n"
"li.checked::marker { content: \"\\2612\"; }\n"
"</style></head><body style=\" font-family:'Ubuntu Sans'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><br /></p></body></html>", None))
        self.diagMsgOptsAutoScrollCheck.setText(QCoreApplication.translate("MainWindow", u"Auto-scroll", None))
        self.diagMsgOptsShowTimeCheck.setText(QCoreApplication.translate("MainWindow", u"Show timestamps", None))
        self.diagMsgOptsShowKeyValCheck.setText(QCoreApplication.translate("MainWindow", u"Show key-values", None))
        self.calibBox.setTitle(QCoreApplication.translate("MainWindow", u"Calibration", None))
        self.calibPidTuneButtonL.setText(QCoreApplication.translate("MainWindow", u"PID Auto-tune (Left)", None))
        self.calibPidTuneButtonR.setText(QCoreApplication.translate("MainWindow", u"PID Auto-tune (Right)", None))
        self.calibCliffOffsetButton.setText(QCoreApplication.translate("MainWindow", u"Cliff Sensors Offset", None))
        self.calibImuCompassButton.setText(QCoreApplication.translate("MainWindow", u"IMU - Compass", None))
        self.selftestBox.setTitle(QCoreApplication.translate("MainWindow", u"Self-Test", None))
        self.selftestPicoAButton.setText(QCoreApplication.translate("MainWindow", u"Pico A Routine", None))
        self.selftestPicoBButton.setText(QCoreApplication.translate("MainWindow", u"Pico B Routine", None))
        self.pages.setTabText(self.pages.indexOf(self.diagTab), QCoreApplication.translate("MainWindow", u"Diagnostics", None))
        self.menuFile.setTitle(QCoreApplication.translate("MainWindow", u"Power", None))
        self.menuConnection.setTitle(QCoreApplication.translate("MainWindow", u"Connection", None))
        self.menuConfiguration.setTitle(QCoreApplication.translate("MainWindow", u"Configuration", None))
    # retranslateUi

