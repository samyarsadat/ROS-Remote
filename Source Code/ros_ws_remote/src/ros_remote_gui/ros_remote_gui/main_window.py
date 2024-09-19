#  The ROS remote project (GUI package) - GUI related setup
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

import subprocess
from ros_remote_gui.init import qt_app
from ros_remote_gui.utils.ui_classes_init import AboutDialog
from ros_remote_gui.gui_files.ui_main_window import Ui_MainWindow
from PySide6.QtWidgets import QMainWindow
from PySide6 import QtCore


# ---- QT MainWindow class ----
class MainWindow(QMainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.restart_on_quit = False

        # Thread pool
        self.thread_pool = QtCore.QThreadPool()

        # Menu actions
        self.ui.actionAbout.triggered.connect(self.show_about_dialog)
        self.ui.actionRestart_RPi.triggered.connect(self.reboot_host_rpi)
        self.ui.actionShutdown_System.triggered.connect(self.shutdown_host_rpi)
        self.ui.actionRestart_Application.triggered.connect(self.restart_qt_app)

        # Misc.
        self.ui.camLedsBrightnessSlider.valueChanged.connect(self.slider_changed)
        self.about_dialog = AboutDialog()

    @QtCore.Slot()
    def show_about_dialog(self):
        self.about_dialog.show()

    @QtCore.Slot()
    def shutdown_host_rpi(self):
        from ros_remote_gui.ros_main import get_ros_node
        get_ros_node().get_logger().warn("System shutdown requested!")
        subprocess.run(["sudo", "shutdown", "now"])

    @QtCore.Slot()
    def reboot_host_rpi(self):
        from ros_remote_gui.ros_main import get_ros_node
        get_ros_node().get_logger().warn("System reboot requested!")
        subprocess.run(["sudo", "reboot", "now"])

    @QtCore.Slot()
    def restart_qt_app(self):
        from ros_remote_gui.ros_main import get_ros_node
        get_ros_node().get_logger().warn("Application restart requested!")
        self.restart_on_quit = True
        qt_app.quit()

    @QtCore.Slot()
    def slider_changed(self, value):
        self.ui.camLedsBrightnessLabel.setText(str(value))


# ---- Main window instance ----
qt_main_window = MainWindow()

def get_main_window() -> MainWindow:
    global qt_main_window
    return qt_main_window
