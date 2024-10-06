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

import socket
import subprocess
from asyncio import Future
from ros_remote_gui.init import qt_app
from ros_remote_gui.ros_main import get_ros_node
from ros_remote_gui.utils.ui_classes_init import AboutDialog
from ros_remote_gui.gui_files.ui_main_window import Ui_MainWindow
from PySide6.QtWidgets import QMainWindow, QMessageBox
from PySide6 import QtCore
from ros_robot_msgs.srv import GetBool


# ---- QT MainWindow class ----
class MainWindow(QMainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.restart_on_quit = False

        self.default_thread_pool = QtCore.QThreadPool()

        self.ui.actionAbout.triggered.connect(self._show_about_dialog)
        self.ui.actionRestart_RPi.triggered.connect(self._reboot_host_rpi)
        self.ui.actionShutdown_System.triggered.connect(self._shutdown_host_rpi)
        self.ui.actionRestart_Application.triggered.connect(self._restart_qt_app)

        self.ui.actionCheck_WAN_Connection.triggered.connect(self._check_internet_connection)
        self.ui.actionCheck_Robot_Connection.triggered.connect(self._ping_robot_driver_node)

        self.ui.camLedsBrightnessSlider.valueChanged.connect(self._slider_changed)
        self._about_dialog = AboutDialog(self)

    @QtCore.Slot()
    def _show_about_dialog(self) -> None:
        self._about_dialog.show()

    @QtCore.Slot()
    def _shutdown_host_rpi(self) -> None:
        from ros_remote_gui.ros_main import get_ros_node
        get_ros_node().get_logger().warn("System shutdown requested!")
        subprocess.run(["sudo", "shutdown", "now"])

    @QtCore.Slot()
    def _reboot_host_rpi(self) -> None:
        from ros_remote_gui.ros_main import get_ros_node
        get_ros_node().get_logger().warn("System reboot requested!")
        subprocess.run(["sudo", "reboot", "now"])

    @QtCore.Slot()
    def _restart_qt_app(self) -> None:
        from ros_remote_gui.ros_main import get_ros_node
        get_ros_node().get_logger().warn("Application restart requested!")
        self.restart_on_quit = True
        qt_app.quit()

    @QtCore.Slot(int)
    def _slider_changed(self, value: int) -> None:
        self.ui.camLedsBrightnessLabel.setText(str(value))

    @QtCore.Slot()
    def _check_internet_connection(self) -> None:
        try:
            socket.setdefaulttimeout(4)
            socket.socket(socket.AF_INET, socket.SOCK_STREAM).connect(("8.8.8.8", 53))
            QMessageBox.information(self, "Connection Check", "Internet connection is OK!", buttons=QMessageBox.Ok, defaultButton=QMessageBox.Ok)
        except socket.error:
            QMessageBox.warning(self, "Connection Check", "No internet connection!", buttons=QMessageBox.Ok, defaultButton=QMessageBox.Ok)

    @staticmethod
    def _ping_robot_driver_srv_call(future: Future) -> None:
        if future.exception() or not future.result().data:
            QMessageBox.warning(get_main_window(), "Robot Connection", "Driver connection failed.", buttons=QMessageBox.Ok, defaultButton=QMessageBox.Ok)
        else:
            QMessageBox.information(get_main_window(), "Robot Connection", "Driver connection is OK!", buttons=QMessageBox.Ok, defaultButton=QMessageBox.Ok)

    @QtCore.Slot()
    def _ping_robot_driver_node(self) -> None:
        if get_ros_node().ping_driver_srvcl.service_is_ready():
            future = get_ros_node().ping_driver_srvcl.call_async(GetBool.Request())
            future.add_done_callback(self._ping_robot_driver_srv_call)
        else:
            QMessageBox.warning(get_main_window(), "Robot Connection", "Driver connection not available.", buttons=QMessageBox.Ok, defaultButton=QMessageBox.Ok)


# ---- Main window instance ----
qt_main_window = MainWindow()

def get_main_window() -> MainWindow:
    global qt_main_window
    return qt_main_window
