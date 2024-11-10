# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'selftest_result_dialog.ui'
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
from PySide6.QtWidgets import (QAbstractButton, QApplication, QDialog, QDialogButtonBox,
    QFrame, QHBoxLayout, QLabel, QSizePolicy,
    QSpacerItem, QTextBrowser, QVBoxLayout, QWidget)

class Ui_Dialog(object):
    def setupUi(self, Dialog):
        if not Dialog.objectName():
            Dialog.setObjectName(u"Dialog")
        Dialog.resize(800, 450)
        self.verticalLayout_1 = QVBoxLayout(Dialog)
        self.verticalLayout_1.setObjectName(u"verticalLayout_1")
        self.verticalLayout_1.setContentsMargins(20, 20, 20, 15)
        self.resultBox = QHBoxLayout()
        self.resultBox.setObjectName(u"resultBox")
        self.resultBox.setContentsMargins(-1, -1, -1, 0)
        self.resultLabel = QLabel(Dialog)
        self.resultLabel.setObjectName(u"resultLabel")
        self.resultLabel.setTextFormat(Qt.TextFormat.MarkdownText)

        self.resultBox.addWidget(self.resultLabel)

        self.horizontalSpacer_1 = QSpacerItem(5, 20, QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Minimum)

        self.resultBox.addItem(self.horizontalSpacer_1)

        self.resultStatusLabel = QLabel(Dialog)
        self.resultStatusLabel.setObjectName(u"resultStatusLabel")
        sizePolicy = QSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.resultStatusLabel.sizePolicy().hasHeightForWidth())
        self.resultStatusLabel.setSizePolicy(sizePolicy)
        self.resultStatusLabel.setFrameShape(QFrame.Shape.Box)
        self.resultStatusLabel.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.resultStatusLabel.setMargin(3)

        self.resultBox.addWidget(self.resultStatusLabel)

        self.horizontalSpacer_2 = QSpacerItem(40, 20, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.resultBox.addItem(self.horizontalSpacer_2)

        self.sourceLabel = QLabel(Dialog)
        self.sourceLabel.setObjectName(u"sourceLabel")
        self.sourceLabel.setFrameShape(QFrame.Shape.Box)
        self.sourceLabel.setMargin(3)

        self.resultBox.addWidget(self.sourceLabel)


        self.verticalLayout_1.addLayout(self.resultBox)

        self.verticalSpacer_1 = QSpacerItem(20, 10, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Fixed)

        self.verticalLayout_1.addItem(self.verticalSpacer_1)

        self.resultMessage = QTextBrowser(Dialog)
        self.resultMessage.setObjectName(u"resultMessage")
        font = QFont()
        font.setFamilies([u"Liberation Mono"])
        font.setPointSize(10)
        self.resultMessage.setFont(font)
        self.resultMessage.setFocusPolicy(Qt.FocusPolicy.NoFocus)

        self.verticalLayout_1.addWidget(self.resultMessage)

        self.verticalSpacer_2 = QSpacerItem(20, 10, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Fixed)

        self.verticalLayout_1.addItem(self.verticalSpacer_2)

        self.buttonBox = QDialogButtonBox(Dialog)
        self.buttonBox.setObjectName(u"buttonBox")
        self.buttonBox.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        self.buttonBox.setOrientation(Qt.Orientation.Horizontal)
        self.buttonBox.setStandardButtons(QDialogButtonBox.StandardButton.Ok)

        self.verticalLayout_1.addWidget(self.buttonBox)


        self.retranslateUi(Dialog)
        self.buttonBox.accepted.connect(Dialog.accept)

        QMetaObject.connectSlotsByName(Dialog)
    # setupUi

    def retranslateUi(self, Dialog):
        Dialog.setWindowTitle(QCoreApplication.translate("Dialog", u"Self-Test Result", None))
        self.resultLabel.setText(QCoreApplication.translate("Dialog", u"**Result:**", None))
        self.resultStatusLabel.setText(QCoreApplication.translate("Dialog", u"STATUS", None))
        self.sourceLabel.setText(QCoreApplication.translate("Dialog", u"SOURCE", None))
    # retranslateUi

