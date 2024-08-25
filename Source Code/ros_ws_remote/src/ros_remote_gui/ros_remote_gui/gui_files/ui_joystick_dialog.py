# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'joystick_dialog.ui'
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
    QFrame, QHBoxLayout, QLabel, QProgressBar,
    QSizePolicy, QSpacerItem, QSpinBox, QVBoxLayout,
    QWidget)

class Ui_Dialog(object):
    def setupUi(self, Dialog):
        if not Dialog.objectName():
            Dialog.setObjectName(u"Dialog")
        Dialog.resize(635, 357)
        self.verticalLayout_1 = QVBoxLayout(Dialog)
        self.verticalLayout_1.setObjectName(u"verticalLayout_1")
        self.mainBox = QVBoxLayout()
        self.mainBox.setObjectName(u"mainBox")
        self.mainBox.setContentsMargins(15, 20, 15, 5)
        self.xAxisBox = QHBoxLayout()
        self.xAxisBox.setSpacing(10)
        self.xAxisBox.setObjectName(u"xAxisBox")
        self.xAxisBox.setContentsMargins(0, -1, 0, -1)
        self.xAxisLabel = QLabel(Dialog)
        self.xAxisLabel.setObjectName(u"xAxisLabel")
        self.xAxisLabel.setMinimumSize(QSize(100, 0))
        self.xAxisLabel.setBaseSize(QSize(0, 0))
        self.xAxisLabel.setFrameShape(QFrame.Shape.Box)
        self.xAxisLabel.setFrameShadow(QFrame.Shadow.Plain)
        self.xAxisLabel.setMidLineWidth(0)
        self.xAxisLabel.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.xAxisBox.addWidget(self.xAxisLabel)

        self.xAxisBar = QProgressBar(Dialog)
        self.xAxisBar.setObjectName(u"xAxisBar")
        self.xAxisBar.setMinimum(-512)
        self.xAxisBar.setMaximum(512)
        self.xAxisBar.setValue(-200)

        self.xAxisBox.addWidget(self.xAxisBar)


        self.mainBox.addLayout(self.xAxisBox)

        self.yAxisBox = QHBoxLayout()
        self.yAxisBox.setSpacing(10)
        self.yAxisBox.setObjectName(u"yAxisBox")
        self.yAxisBox.setContentsMargins(0, -1, 0, -1)
        self.yAxisLabel = QLabel(Dialog)
        self.yAxisLabel.setObjectName(u"yAxisLabel")
        self.yAxisLabel.setMinimumSize(QSize(100, 0))
        self.yAxisLabel.setFrameShape(QFrame.Shape.Box)
        self.yAxisLabel.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.yAxisBox.addWidget(self.yAxisLabel)

        self.yAxisBar = QProgressBar(Dialog)
        self.yAxisBar.setObjectName(u"yAxisBar")
        self.yAxisBar.setMinimum(-512)
        self.yAxisBar.setMaximum(512)
        self.yAxisBar.setValue(-200)

        self.yAxisBox.addWidget(self.yAxisBar)


        self.mainBox.addLayout(self.yAxisBox)

        self.verticalSpacer_1 = QSpacerItem(20, 25, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Fixed)

        self.mainBox.addItem(self.verticalSpacer_1)

        self.xDeadzoneBox = QHBoxLayout()
        self.xDeadzoneBox.setSpacing(10)
        self.xDeadzoneBox.setObjectName(u"xDeadzoneBox")
        self.xDeadzoneBox.setContentsMargins(-1, 0, -1, -1)
        self.xDeadzoneLabel = QLabel(Dialog)
        self.xDeadzoneLabel.setObjectName(u"xDeadzoneLabel")
        self.xDeadzoneLabel.setMaximumSize(QSize(100, 16777215))
        self.xDeadzoneLabel.setFrameShape(QFrame.Shape.Box)
        self.xDeadzoneLabel.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.xDeadzoneBox.addWidget(self.xDeadzoneLabel)

        self.xDeadzoneInput = QSpinBox(Dialog)
        self.xDeadzoneInput.setObjectName(u"xDeadzoneInput")
        self.xDeadzoneInput.setMinimum(-512)
        self.xDeadzoneInput.setMaximum(512)

        self.xDeadzoneBox.addWidget(self.xDeadzoneInput)


        self.mainBox.addLayout(self.xDeadzoneBox)

        self.yDeadzoneBox = QHBoxLayout()
        self.yDeadzoneBox.setSpacing(10)
        self.yDeadzoneBox.setObjectName(u"yDeadzoneBox")
        self.yDeadzoneBox.setContentsMargins(0, -1, 0, -1)
        self.yDeadzoneLabel = QLabel(Dialog)
        self.yDeadzoneLabel.setObjectName(u"yDeadzoneLabel")
        self.yDeadzoneLabel.setMinimumSize(QSize(100, 0))
        self.yDeadzoneLabel.setMaximumSize(QSize(100, 16777215))
        self.yDeadzoneLabel.setFrameShape(QFrame.Shape.Box)
        self.yDeadzoneLabel.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.yDeadzoneBox.addWidget(self.yDeadzoneLabel)

        self.yDeadzoneInput = QSpinBox(Dialog)
        self.yDeadzoneInput.setObjectName(u"yDeadzoneInput")
        self.yDeadzoneInput.setMinimum(-512)
        self.yDeadzoneInput.setMaximum(512)

        self.yDeadzoneBox.addWidget(self.yDeadzoneInput)


        self.mainBox.addLayout(self.yDeadzoneBox)

        self.verticalSpacer_2 = QSpacerItem(20, 8, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Fixed)

        self.mainBox.addItem(self.verticalSpacer_2)

        self.yOffsetBox = QHBoxLayout()
        self.yOffsetBox.setSpacing(10)
        self.yOffsetBox.setObjectName(u"yOffsetBox")
        self.yOffsetBox.setContentsMargins(0, 0, -1, -1)
        self.yOffsetLabel = QLabel(Dialog)
        self.yOffsetLabel.setObjectName(u"yOffsetLabel")
        self.yOffsetLabel.setMaximumSize(QSize(100, 16777215))
        self.yOffsetLabel.setFrameShape(QFrame.Shape.Box)
        self.yOffsetLabel.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.yOffsetBox.addWidget(self.yOffsetLabel)

        self.yOffsetInput = QSpinBox(Dialog)
        self.yOffsetInput.setObjectName(u"yOffsetInput")
        self.yOffsetInput.setMinimum(-512)
        self.yOffsetInput.setMaximum(512)

        self.yOffsetBox.addWidget(self.yOffsetInput)


        self.mainBox.addLayout(self.yOffsetBox)

        self.xOffsetBox = QHBoxLayout()
        self.xOffsetBox.setSpacing(10)
        self.xOffsetBox.setObjectName(u"xOffsetBox")
        self.xOffsetBox.setContentsMargins(-1, 0, -1, -1)
        self.xOffsetLabel = QLabel(Dialog)
        self.xOffsetLabel.setObjectName(u"xOffsetLabel")
        self.xOffsetLabel.setMaximumSize(QSize(100, 16777215))
        self.xOffsetLabel.setFrameShape(QFrame.Shape.Box)
        self.xOffsetLabel.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.xOffsetBox.addWidget(self.xOffsetLabel)

        self.xOffsetInput = QSpinBox(Dialog)
        self.xOffsetInput.setObjectName(u"xOffsetInput")
        self.xOffsetInput.setMinimum(-512)
        self.xOffsetInput.setMaximum(512)

        self.xOffsetBox.addWidget(self.xOffsetInput)


        self.mainBox.addLayout(self.xOffsetBox)

        self.verticalSpacer_3 = QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.mainBox.addItem(self.verticalSpacer_3)

        self.buttonBox = QDialogButtonBox(Dialog)
        self.buttonBox.setObjectName(u"buttonBox")
        self.buttonBox.setOrientation(Qt.Orientation.Horizontal)
        self.buttonBox.setStandardButtons(QDialogButtonBox.StandardButton.Ok)

        self.mainBox.addWidget(self.buttonBox)

        self.verticalSpacer_4 = QSpacerItem(20, 10, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Fixed)

        self.mainBox.addItem(self.verticalSpacer_4)


        self.verticalLayout_1.addLayout(self.mainBox)


        self.retranslateUi(Dialog)
        self.buttonBox.accepted.connect(Dialog.accept)

        QMetaObject.connectSlotsByName(Dialog)
    # setupUi

    def retranslateUi(self, Dialog):
        Dialog.setWindowTitle(QCoreApplication.translate("Dialog", u"Joystick Configuration", None))
        self.xAxisLabel.setText(QCoreApplication.translate("Dialog", u"X Axis", None))
        self.xAxisBar.setFormat(QCoreApplication.translate("Dialog", u"%p", None))
        self.yAxisLabel.setText(QCoreApplication.translate("Dialog", u"Y Axis", None))
        self.yAxisBar.setFormat(QCoreApplication.translate("Dialog", u"%p", None))
        self.xDeadzoneLabel.setText(QCoreApplication.translate("Dialog", u"X Deadzone", None))
        self.yDeadzoneLabel.setText(QCoreApplication.translate("Dialog", u"Y Deadzone", None))
        self.yOffsetLabel.setText(QCoreApplication.translate("Dialog", u"Y Offset", None))
        self.xOffsetLabel.setText(QCoreApplication.translate("Dialog", u"X Offset", None))
    # retranslateUi

