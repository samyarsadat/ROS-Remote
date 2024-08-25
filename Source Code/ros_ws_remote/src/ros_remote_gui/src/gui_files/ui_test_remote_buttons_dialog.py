# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'test_remote_buttons_dialog.ui'
##
## Created by: Qt User Interface Compiler version 6.7.2
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
from PySide6.QtWidgets import (QAbstractButton, QApplication, QDialog, QDialogButtonBox,
    QFrame, QHBoxLayout, QLabel, QLineEdit,
    QPushButton, QSizePolicy, QSpacerItem, QVBoxLayout,
    QWidget)

class Ui_Dialog(object):
    def setupUi(self, Dialog):
        if not Dialog.objectName():
            Dialog.setObjectName(u"Dialog")
        Dialog.resize(800, 458)
        self.buttonBox = QDialogButtonBox(Dialog)
        self.buttonBox.setObjectName(u"buttonBox")
        self.buttonBox.setGeometry(QRect(434, 412, 341, 32))
        self.buttonBox.setOrientation(Qt.Orientation.Horizontal)
        self.buttonBox.setStandardButtons(QDialogButtonBox.StandardButton.Ok)
        self.mainFrame = QFrame(Dialog)
        self.mainFrame.setObjectName(u"mainFrame")
        self.mainFrame.setGeometry(QRect(25, 25, 750, 375))
        self.mainFrame.setFrameShape(QFrame.Shape.StyledPanel)
        self.mainFrame.setFrameShadow(QFrame.Shadow.Raised)
        self.middleDivisionLine = QFrame(self.mainFrame)
        self.middleDivisionLine.setObjectName(u"middleDivisionLine")
        self.middleDivisionLine.setGeometry(QRect(375, 0, 2, 415))
        self.middleDivisionLine.setFrameShape(QFrame.Shape.VLine)
        self.middleDivisionLine.setFrameShadow(QFrame.Shadow.Sunken)
        self.verticalLayout_1 = QWidget(self.mainFrame)
        self.verticalLayout_1.setObjectName(u"verticalLayout_1")
        self.verticalLayout_1.setGeometry(QRect(12, 12, 351, 351))
        self.rightSectionBox = QVBoxLayout(self.verticalLayout_1)
        self.rightSectionBox.setObjectName(u"rightSectionBox")
        self.rightSectionBox.setContentsMargins(0, 0, 0, 0)
        self.rightInnerBox_4 = QHBoxLayout()
        self.rightInnerBox_4.setSpacing(10)
        self.rightInnerBox_4.setObjectName(u"rightInnerBox_4")
        self.rightKeySwitchIndicator = QLabel(self.verticalLayout_1)
        self.rightKeySwitchIndicator.setObjectName(u"rightKeySwitchIndicator")
        sizePolicy = QSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.rightKeySwitchIndicator.sizePolicy().hasHeightForWidth())
        self.rightKeySwitchIndicator.setSizePolicy(sizePolicy)
        self.rightKeySwitchIndicator.setMinimumSize(QSize(0, 55))
        self.rightKeySwitchIndicator.setAutoFillBackground(False)
        self.rightKeySwitchIndicator.setFrameShape(QFrame.Shape.Box)
        self.rightKeySwitchIndicator.setFrameShadow(QFrame.Shadow.Plain)
        self.rightKeySwitchIndicator.setLineWidth(1)
        self.rightKeySwitchIndicator.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.rightInnerBox_4.addWidget(self.rightKeySwitchIndicator)

        self.rightSwitchIndicator_4 = QLabel(self.verticalLayout_1)
        self.rightSwitchIndicator_4.setObjectName(u"rightSwitchIndicator_4")
        sizePolicy.setHeightForWidth(self.rightSwitchIndicator_4.sizePolicy().hasHeightForWidth())
        self.rightSwitchIndicator_4.setSizePolicy(sizePolicy)
        self.rightSwitchIndicator_4.setMinimumSize(QSize(0, 55))
        self.rightSwitchIndicator_4.setAutoFillBackground(False)
        self.rightSwitchIndicator_4.setFrameShape(QFrame.Shape.Box)
        self.rightSwitchIndicator_4.setFrameShadow(QFrame.Shadow.Plain)
        self.rightSwitchIndicator_4.setLineWidth(1)
        self.rightSwitchIndicator_4.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.rightInnerBox_4.addWidget(self.rightSwitchIndicator_4)


        self.rightSectionBox.addLayout(self.rightInnerBox_4)

        self.rightInnerBox_3 = QHBoxLayout()
        self.rightInnerBox_3.setSpacing(32)
        self.rightInnerBox_3.setObjectName(u"rightInnerBox_3")
        self.rightButtonIndicator_4 = QLabel(self.verticalLayout_1)
        self.rightButtonIndicator_4.setObjectName(u"rightButtonIndicator_4")
        sizePolicy.setHeightForWidth(self.rightButtonIndicator_4.sizePolicy().hasHeightForWidth())
        self.rightButtonIndicator_4.setSizePolicy(sizePolicy)
        self.rightButtonIndicator_4.setMinimumSize(QSize(0, 55))
        self.rightButtonIndicator_4.setAutoFillBackground(False)
        self.rightButtonIndicator_4.setFrameShape(QFrame.Shape.Box)
        self.rightButtonIndicator_4.setFrameShadow(QFrame.Shadow.Plain)
        self.rightButtonIndicator_4.setLineWidth(1)
        self.rightButtonIndicator_4.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.rightInnerBox_3.addWidget(self.rightButtonIndicator_4)

        self.rightButtonIndicator_5 = QLabel(self.verticalLayout_1)
        self.rightButtonIndicator_5.setObjectName(u"rightButtonIndicator_5")
        sizePolicy.setHeightForWidth(self.rightButtonIndicator_5.sizePolicy().hasHeightForWidth())
        self.rightButtonIndicator_5.setSizePolicy(sizePolicy)
        self.rightButtonIndicator_5.setMinimumSize(QSize(0, 55))
        self.rightButtonIndicator_5.setAutoFillBackground(False)
        self.rightButtonIndicator_5.setFrameShape(QFrame.Shape.Box)
        self.rightButtonIndicator_5.setFrameShadow(QFrame.Shadow.Plain)
        self.rightButtonIndicator_5.setLineWidth(1)
        self.rightButtonIndicator_5.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.rightInnerBox_3.addWidget(self.rightButtonIndicator_5)


        self.rightSectionBox.addLayout(self.rightInnerBox_3)

        self.rightInnerBox_2 = QHBoxLayout()
        self.rightInnerBox_2.setObjectName(u"rightInnerBox_2")
        self.rightSwitchIndicator_1 = QLabel(self.verticalLayout_1)
        self.rightSwitchIndicator_1.setObjectName(u"rightSwitchIndicator_1")
        sizePolicy.setHeightForWidth(self.rightSwitchIndicator_1.sizePolicy().hasHeightForWidth())
        self.rightSwitchIndicator_1.setSizePolicy(sizePolicy)
        self.rightSwitchIndicator_1.setMinimumSize(QSize(0, 55))
        self.rightSwitchIndicator_1.setAutoFillBackground(False)
        self.rightSwitchIndicator_1.setFrameShape(QFrame.Shape.Box)
        self.rightSwitchIndicator_1.setFrameShadow(QFrame.Shadow.Plain)
        self.rightSwitchIndicator_1.setLineWidth(1)
        self.rightSwitchIndicator_1.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.rightInnerBox_2.addWidget(self.rightSwitchIndicator_1)

        self.rightSwitchIndicator_2 = QLabel(self.verticalLayout_1)
        self.rightSwitchIndicator_2.setObjectName(u"rightSwitchIndicator_2")
        sizePolicy.setHeightForWidth(self.rightSwitchIndicator_2.sizePolicy().hasHeightForWidth())
        self.rightSwitchIndicator_2.setSizePolicy(sizePolicy)
        self.rightSwitchIndicator_2.setMinimumSize(QSize(0, 55))
        self.rightSwitchIndicator_2.setAutoFillBackground(False)
        self.rightSwitchIndicator_2.setFrameShape(QFrame.Shape.Box)
        self.rightSwitchIndicator_2.setFrameShadow(QFrame.Shadow.Plain)
        self.rightSwitchIndicator_2.setLineWidth(1)
        self.rightSwitchIndicator_2.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.rightInnerBox_2.addWidget(self.rightSwitchIndicator_2)

        self.rightSwitchIndicator_3 = QLabel(self.verticalLayout_1)
        self.rightSwitchIndicator_3.setObjectName(u"rightSwitchIndicator_3")
        sizePolicy.setHeightForWidth(self.rightSwitchIndicator_3.sizePolicy().hasHeightForWidth())
        self.rightSwitchIndicator_3.setSizePolicy(sizePolicy)
        self.rightSwitchIndicator_3.setMinimumSize(QSize(0, 55))
        self.rightSwitchIndicator_3.setAutoFillBackground(False)
        self.rightSwitchIndicator_3.setFrameShape(QFrame.Shape.Box)
        self.rightSwitchIndicator_3.setFrameShadow(QFrame.Shadow.Plain)
        self.rightSwitchIndicator_3.setLineWidth(1)
        self.rightSwitchIndicator_3.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.rightInnerBox_2.addWidget(self.rightSwitchIndicator_3)


        self.rightSectionBox.addLayout(self.rightInnerBox_2)

        self.verticalSpacer_1 = QSpacerItem(20, 70, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.rightSectionBox.addItem(self.verticalSpacer_1)

        self.rightButtonIndicator_3 = QLabel(self.verticalLayout_1)
        self.rightButtonIndicator_3.setObjectName(u"rightButtonIndicator_3")
        sizePolicy.setHeightForWidth(self.rightButtonIndicator_3.sizePolicy().hasHeightForWidth())
        self.rightButtonIndicator_3.setSizePolicy(sizePolicy)
        self.rightButtonIndicator_3.setMinimumSize(QSize(0, 55))
        self.rightButtonIndicator_3.setAutoFillBackground(False)
        self.rightButtonIndicator_3.setFrameShape(QFrame.Shape.Box)
        self.rightButtonIndicator_3.setFrameShadow(QFrame.Shadow.Plain)
        self.rightButtonIndicator_3.setLineWidth(1)
        self.rightButtonIndicator_3.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.rightSectionBox.addWidget(self.rightButtonIndicator_3)

        self.rightInnerBox_1 = QHBoxLayout()
        self.rightInnerBox_1.setSpacing(8)
        self.rightInnerBox_1.setObjectName(u"rightInnerBox_1")
        self.encoderBox = QVBoxLayout()
        self.encoderBox.setObjectName(u"encoderBox")
        self.encoderLabel = QLabel(self.verticalLayout_1)
        self.encoderLabel.setObjectName(u"encoderLabel")
        self.encoderLabel.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.encoderBox.addWidget(self.encoderLabel)

        self.encoderValue = QLineEdit(self.verticalLayout_1)
        self.encoderValue.setObjectName(u"encoderValue")
        sizePolicy1 = QSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)
        sizePolicy1.setHorizontalStretch(0)
        sizePolicy1.setVerticalStretch(0)
        sizePolicy1.setHeightForWidth(self.encoderValue.sizePolicy().hasHeightForWidth())
        self.encoderValue.setSizePolicy(sizePolicy1)
        font = QFont()
        font.setPointSize(12)
        self.encoderValue.setFont(font)
        self.encoderValue.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.encoderValue.setFrame(True)
        self.encoderValue.setDragEnabled(True)
        self.encoderValue.setReadOnly(True)

        self.encoderBox.addWidget(self.encoderValue)


        self.rightInnerBox_1.addLayout(self.encoderBox)

        self.horizontalSpacer_1 = QSpacerItem(40, 20, QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Minimum)

        self.rightInnerBox_1.addItem(self.horizontalSpacer_1)

        self.rightButtonIndicator_1 = QLabel(self.verticalLayout_1)
        self.rightButtonIndicator_1.setObjectName(u"rightButtonIndicator_1")
        sizePolicy.setHeightForWidth(self.rightButtonIndicator_1.sizePolicy().hasHeightForWidth())
        self.rightButtonIndicator_1.setSizePolicy(sizePolicy)
        self.rightButtonIndicator_1.setMinimumSize(QSize(0, 55))
        self.rightButtonIndicator_1.setAutoFillBackground(False)
        self.rightButtonIndicator_1.setFrameShape(QFrame.Shape.Box)
        self.rightButtonIndicator_1.setFrameShadow(QFrame.Shadow.Plain)
        self.rightButtonIndicator_1.setLineWidth(1)
        self.rightButtonIndicator_1.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.rightInnerBox_1.addWidget(self.rightButtonIndicator_1)

        self.rightButtonIndicator_2 = QLabel(self.verticalLayout_1)
        self.rightButtonIndicator_2.setObjectName(u"rightButtonIndicator_2")
        sizePolicy.setHeightForWidth(self.rightButtonIndicator_2.sizePolicy().hasHeightForWidth())
        self.rightButtonIndicator_2.setSizePolicy(sizePolicy)
        self.rightButtonIndicator_2.setMinimumSize(QSize(0, 55))
        self.rightButtonIndicator_2.setAutoFillBackground(False)
        self.rightButtonIndicator_2.setFrameShape(QFrame.Shape.Box)
        self.rightButtonIndicator_2.setFrameShadow(QFrame.Shadow.Plain)
        self.rightButtonIndicator_2.setLineWidth(1)
        self.rightButtonIndicator_2.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.rightInnerBox_1.addWidget(self.rightButtonIndicator_2)


        self.rightSectionBox.addLayout(self.rightInnerBox_1)

        self.verticalLayout_2 = QWidget(self.mainFrame)
        self.verticalLayout_2.setObjectName(u"verticalLayout_2")
        self.verticalLayout_2.setGeometry(QRect(388, 12, 351, 351))
        self.leftSectionBox = QVBoxLayout(self.verticalLayout_2)
        self.leftSectionBox.setObjectName(u"leftSectionBox")
        self.leftSectionBox.setContentsMargins(0, 0, 0, 0)
        self.leftInnerBox_2 = QHBoxLayout()
        self.leftInnerBox_2.setSpacing(8)
        self.leftInnerBox_2.setObjectName(u"leftInnerBox_2")
        self.eStopIndicator = QLabel(self.verticalLayout_2)
        self.eStopIndicator.setObjectName(u"eStopIndicator")
        sizePolicy.setHeightForWidth(self.eStopIndicator.sizePolicy().hasHeightForWidth())
        self.eStopIndicator.setSizePolicy(sizePolicy)
        self.eStopIndicator.setMinimumSize(QSize(0, 55))
        self.eStopIndicator.setAutoFillBackground(False)
        self.eStopIndicator.setFrameShape(QFrame.Shape.Box)
        self.eStopIndicator.setFrameShadow(QFrame.Shadow.Plain)
        self.eStopIndicator.setLineWidth(1)
        self.eStopIndicator.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.leftInnerBox_2.addWidget(self.eStopIndicator)

        self.leftSwitchIndicator = QLabel(self.verticalLayout_2)
        self.leftSwitchIndicator.setObjectName(u"leftSwitchIndicator")
        sizePolicy.setHeightForWidth(self.leftSwitchIndicator.sizePolicy().hasHeightForWidth())
        self.leftSwitchIndicator.setSizePolicy(sizePolicy)
        self.leftSwitchIndicator.setMinimumSize(QSize(0, 55))
        self.leftSwitchIndicator.setAutoFillBackground(False)
        self.leftSwitchIndicator.setFrameShape(QFrame.Shape.Box)
        self.leftSwitchIndicator.setFrameShadow(QFrame.Shadow.Plain)
        self.leftSwitchIndicator.setLineWidth(1)
        self.leftSwitchIndicator.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.leftInnerBox_2.addWidget(self.leftSwitchIndicator)


        self.leftSectionBox.addLayout(self.leftInnerBox_2)

        self.verticalSpacer_2 = QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.leftSectionBox.addItem(self.verticalSpacer_2)

        self.leftInnerBox_1 = QHBoxLayout()
        self.leftInnerBox_1.setSpacing(8)
        self.leftInnerBox_1.setObjectName(u"leftInnerBox_1")
        self.potentiometerBox = QVBoxLayout()
        self.potentiometerBox.setObjectName(u"potentiometerBox")
        self.potentiometerLabel = QLabel(self.verticalLayout_2)
        self.potentiometerLabel.setObjectName(u"potentiometerLabel")
        self.potentiometerLabel.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.potentiometerBox.addWidget(self.potentiometerLabel)

        self.potentiometerValue = QLineEdit(self.verticalLayout_2)
        self.potentiometerValue.setObjectName(u"potentiometerValue")
        sizePolicy1.setHeightForWidth(self.potentiometerValue.sizePolicy().hasHeightForWidth())
        self.potentiometerValue.setSizePolicy(sizePolicy1)
        self.potentiometerValue.setFont(font)
        self.potentiometerValue.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.potentiometerValue.setFrame(True)
        self.potentiometerValue.setDragEnabled(True)
        self.potentiometerValue.setReadOnly(True)

        self.potentiometerBox.addWidget(self.potentiometerValue)


        self.leftInnerBox_1.addLayout(self.potentiometerBox)

        self.horizontalSpacer_2 = QSpacerItem(40, 20, QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Minimum)

        self.leftInnerBox_1.addItem(self.horizontalSpacer_2)

        self.leftButtonIndicator = QLabel(self.verticalLayout_2)
        self.leftButtonIndicator.setObjectName(u"leftButtonIndicator")
        sizePolicy.setHeightForWidth(self.leftButtonIndicator.sizePolicy().hasHeightForWidth())
        self.leftButtonIndicator.setSizePolicy(sizePolicy)
        self.leftButtonIndicator.setMinimumSize(QSize(0, 55))
        self.leftButtonIndicator.setAutoFillBackground(False)
        self.leftButtonIndicator.setFrameShape(QFrame.Shape.Box)
        self.leftButtonIndicator.setFrameShadow(QFrame.Shadow.Plain)
        self.leftButtonIndicator.setLineWidth(1)
        self.leftButtonIndicator.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.leftInnerBox_1.addWidget(self.leftButtonIndicator)


        self.leftSectionBox.addLayout(self.leftInnerBox_1)

        self.testLedsButton = QPushButton(Dialog)
        self.testLedsButton.setObjectName(u"testLedsButton")
        self.testLedsButton.setGeometry(QRect(25, 410, 130, 30))

        self.retranslateUi(Dialog)
        self.buttonBox.accepted.connect(Dialog.accept)

        QMetaObject.connectSlotsByName(Dialog)
    # setupUi

    def retranslateUi(self, Dialog):
        Dialog.setWindowTitle(QCoreApplication.translate("Dialog", u"Test Buttons & Switches", None))
        self.rightKeySwitchIndicator.setText(QCoreApplication.translate("Dialog", u"Key Switch", None))
        self.rightSwitchIndicator_4.setText(QCoreApplication.translate("Dialog", u"Switch", None))
        self.rightButtonIndicator_4.setText(QCoreApplication.translate("Dialog", u"Button", None))
        self.rightButtonIndicator_5.setText(QCoreApplication.translate("Dialog", u"Button", None))
        self.rightSwitchIndicator_1.setText(QCoreApplication.translate("Dialog", u"Switch", None))
        self.rightSwitchIndicator_2.setText(QCoreApplication.translate("Dialog", u"Switch", None))
        self.rightSwitchIndicator_3.setText(QCoreApplication.translate("Dialog", u"Switch", None))
        self.rightButtonIndicator_3.setText(QCoreApplication.translate("Dialog", u"Button", None))
        self.encoderLabel.setText(QCoreApplication.translate("Dialog", u"Encoder", None))
        self.encoderValue.setText(QCoreApplication.translate("Dialog", u"0", None))
        self.rightButtonIndicator_1.setText(QCoreApplication.translate("Dialog", u"Button", None))
        self.rightButtonIndicator_2.setText(QCoreApplication.translate("Dialog", u"Button", None))
        self.eStopIndicator.setText(QCoreApplication.translate("Dialog", u"E-Stop", None))
        self.leftSwitchIndicator.setText(QCoreApplication.translate("Dialog", u"Switch", None))
        self.potentiometerLabel.setText(QCoreApplication.translate("Dialog", u"Potentiometer", None))
        self.potentiometerValue.setText(QCoreApplication.translate("Dialog", u"0", None))
        self.leftButtonIndicator.setText(QCoreApplication.translate("Dialog", u"Button", None))
        self.testLedsButton.setText(QCoreApplication.translate("Dialog", u"Test LEDs", None))
    # retranslateUi

