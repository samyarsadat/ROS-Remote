# This Python file uses the following encoding: utf-8
import sys

from PySide6.QtWidgets import QApplication, QMainWindow, QDialog
from PySide6.QtGui import QPixmap
from PySide6 import QtCore

from gui_files.ui_main_window import Ui_MainWindow as UiMainWindow
from gui_files.ui_about_dialog import Ui_Dialog as UiAboutDialog
from gui_files.ui_calibration_result_dialog import Ui_Dialog as UiCalibResultDialog
from gui_files.ui_joystick_dialog import Ui_Dialog as UiJoystickConfigDialog
from gui_files.ui_selftest_result_dialog import Ui_Dialog as UiSelftestResultDialog
from gui_files.ui_test_remote_buttons_dialog import Ui_Dialog as UiButtonTestDialog


class AboutDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.ui = UiAboutDialog()
        self.ui.setupUi(self)
        self.ui.textBrowser.setHtml(self.ui.textBrowser.document().toHtml()
                                    .replace("{date}", "01/08/2024")
                                    .replace("{version}", "v1.0"))

class CalibResDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.ui = UiCalibResultDialog()
        self.ui.setupUi(self)

class JoystickDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.ui = UiJoystickConfigDialog()
        self.ui.setupUi(self)

class SelftestResDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.ui = UiSelftestResultDialog()
        self.ui.setupUi(self)

class ButtonTestDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.ui = UiButtonTestDialog()
        self.ui.setupUi(self)


class MainWindow(QMainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.ui = UiMainWindow()
        self.ui.setupUi(self)

        self.about_dialog = AboutDialog()
        self.calib_res_dialog = CalibResDialog()
        self.joystick_conf_dialog = JoystickDialog()
        self.selftest_result_dialog = SelftestResDialog()
        self.button_test_dialog = ButtonTestDialog()

        self.ui.actionJoystick.triggered.connect(self.show_joystick_dialog)
        self.ui.actionAbout.triggered.connect(self.show_about_dialog)
        self.ui.selftestPicoAButton.clicked.connect(self.show_selftest)
        self.ui.calibPidTuneButtonR.clicked.connect(self.show_calib)
        self.ui.actionTest_Buttons_Switches.triggered.connect(self.show_button_test)

        self.ui.viewport.setPixmap(QPixmap("sample.jpg"))
        self.ui.viewport.setScaledContents(True)
        self.ui.camLedsBrightnessSlider.valueChanged.connect(self.slider_changed)

    @QtCore.Slot()
    def show_joystick_dialog(self):
        self.joystick_conf_dialog.show()

    @QtCore.Slot()
    def show_about_dialog(self):
        self.about_dialog.show()

    @QtCore.Slot()
    def show_selftest(self):
        self.selftest_result_dialog.show()

    @QtCore.Slot()
    def show_calib(self):
        self.calib_res_dialog.show()

    @QtCore.Slot()
    def show_button_test(self):
        self.button_test_dialog.show()

    @QtCore.Slot()
    def slider_changed(self, value):
        self.ui.camLedsBrightnessLabel.setText(str(value))


if __name__ == "__main__":
    app = QApplication(sys.argv)
    widget = MainWindow()
    widget.show()
    sys.exit(app.exec())
