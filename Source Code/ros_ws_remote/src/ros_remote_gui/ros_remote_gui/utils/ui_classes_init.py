#  The ROS remote project (GUI package)
#  PySide6 UI classes init file (excluding the main window)
#  Copyright 2024 Samyar Sadat Akhavi
#  Written by Samyar Sadat Akhavi, 2024.
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see <https: www.gnu.org/licenses/>.

from ros_remote_gui.config import ProgramInfoConfig
from PySide6.QtWidgets import QDialog
from PySide6 import __version__ as pyside_version
from PySide6.QtCore import qVersion
from ros_remote_gui.gui_files.ui_test_remote_buttons_dialog import Ui_Dialog as UiTestRemoteButtonsDialog
from ros_remote_gui.gui_files.ui_about_dialog import Ui_Dialog as UiAboutDialog
from ros_remote_gui.gui_files.ui_joystick_dialog import Ui_Dialog as UiJoystickDialog
from ros_remote_gui.gui_files.ui_selftest_result_dialog import Ui_Dialog as UiSelftestResultDialog
from ros_remote_gui.gui_files.ui_calibration_result_dialog import Ui_Dialog as UiCalibrationResultDialog


# ---- Classes ----

# About dialog
class AboutDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.ui = UiAboutDialog()
        self.ui.setupUi(self)
        self.ui.textBrowser.setHtml(self.ui.textBrowser.document().toHtml()
                                    .replace("{date}", ProgramInfoConfig.VERSION_DATE)
                                    .replace("{version}", ProgramInfoConfig.VERSION)
                                    .replace("{qt_version}", qVersion())
                                    .replace("{pyside_version}", pyside_version))


# Calibration result dialog
class CalibResDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.ui = UiCalibrationResultDialog()
        self.ui.setupUi(self)


# Joystick configuration dialog
class JoystickDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.ui = UiJoystickDialog()
        self.ui.setupUi(self)


# Self-test result dialog
class SelftestResDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.ui = UiSelftestResultDialog()
        self.ui.setupUi(self)


# Remote button test dialog
class ButtonTestDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.ui = UiTestRemoteButtonsDialog()
        self.ui.setupUi(self)
