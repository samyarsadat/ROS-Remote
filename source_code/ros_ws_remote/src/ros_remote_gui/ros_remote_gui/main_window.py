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
from PySide6.QtCore import QTimer
from PySide6.QtGui import Qt
from ros_remote_gui.config import ProgramConfig
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
        self.setWindowState(Qt.WindowFullScreen)
        self.restart_on_quit = False

        self.default_thread_pool = QtCore.QThreadPool()

        self.main_tab_ui_handler = None
        self.sensors_tab_ui_handler = None
        self.power_tab_ui_handler = None
        self.motor_tab_ui_handler = None
        self.diag_tab_ui_handler = None
        self._previous_tab_sel = None

        self._update_ui_tmr_states = QTimer()
        self._update_ui_tmr_states.timeout.connect(self._update_ui_update_timer_states)
        self._update_ui_tmr_states.start(ProgramConfig.UI_TIMER_STATE_UPDATE_INTERVAL_MS)

        self.ui.actionAbout.triggered.connect(self._show_about_dialog)
        self.ui.actionRestart_RPi.triggered.connect(self._reboot_host_rpi)
        self.ui.actionShutdown_System.triggered.connect(self._shutdown_host_rpi)
        self.ui.actionRestart_Application.triggered.connect(self._restart_qt_app)
        self.ui.actionQuit_Application.triggered.connect(self._quit_qt_app)

        self.ui.actionCheck_WAN_Connection.triggered.connect(self._check_internet_connection)
        self.ui.actionCheck_Robot_Connection.triggered.connect(self._ping_robot_driver_node)

        self.ui.camLedsBrightnessSlider.valueChanged.connect(self._slider_changed)
        self._about_dialog = AboutDialog(self)

    def init_ui_handlers(self) -> None:
        from ros_remote_gui.modules.main_tab import MainTab
        from ros_remote_gui.modules.sensors_tab import SensorTab
        from ros_remote_gui.modules.power_tab import PowerTab
        from ros_remote_gui.modules.motor_tab import MotorTab
        from ros_remote_gui.modules.diag_tab import DiagnosticsTab
        self.main_tab_ui_handler = MainTab()
        self.sensors_tab_ui_handler = SensorTab()
        self.power_tab_ui_handler = PowerTab()
        self.motor_tab_ui_handler = MotorTab()
        self.diag_tab_ui_handler = DiagnosticsTab()

    def _update_ui_update_timer_states(self) -> None:
        if not self._previous_tab_sel:
            self._previous_tab_sel = self.ui.pages.currentWidget().objectName()

        if self.ui.pages.currentWidget().objectName() == self.ui.mainTab.objectName():
            if self._previous_tab_sel != self.ui.mainTab.objectName() or not self.main_tab_ui_handler._update_ui_tmr.isActive():
                self.main_tab_ui_handler._update_ui_tmr.start(ProgramConfig.UI_DATA_UPDATE_INTERVAL_MS)
                self.main_tab_ui_handler.viewport_thread.start()
        else:
            if self.main_tab_ui_handler._update_ui_tmr.isActive():
                self.main_tab_ui_handler._update_ui_tmr.stop()
                self.main_tab_ui_handler.viewport_thread.stop()

        if self.ui.pages.currentWidget().objectName() == self.ui.sensorsTab.objectName():
            if self._previous_tab_sel != self.ui.sensorsTab.objectName() or not self.sensors_tab_ui_handler._update_ui_tmr.isActive():
                self.sensors_tab_ui_handler._update_ui_tmr.start(ProgramConfig.UI_DATA_UPDATE_INTERVAL_MS)
        else:
            if self.sensors_tab_ui_handler._update_ui_tmr.isActive():
                self.sensors_tab_ui_handler._update_ui_tmr.stop()

        if self.ui.pages.currentWidget().objectName() == self.ui.powerTab.objectName():
            if self._previous_tab_sel != self.ui.powerTab.objectName() or not self.power_tab_ui_handler._update_ui_tmr.isActive():
                self.power_tab_ui_handler._update_ui_tmr.start(ProgramConfig.UI_DATA_UPDATE_INTERVAL_MS)
        else:
            if self.power_tab_ui_handler._update_ui_tmr.isActive():
                self.power_tab_ui_handler._update_ui_tmr.stop()

        if self.ui.pages.currentWidget().objectName() == self.ui.motorCtrlTab.objectName():
            if self._previous_tab_sel != self.ui.motorCtrlTab.objectName() or not self.motor_tab_ui_handler._update_ui_tmr.isActive():
                self.motor_tab_ui_handler._update_ui_tmr.start(ProgramConfig.UI_DATA_UPDATE_INTERVAL_MS)
        else:
            if self.motor_tab_ui_handler._update_ui_tmr.isActive():
                self.motor_tab_ui_handler._update_ui_tmr.stop()

        self._previous_tab_sel = self.ui.pages.currentWidget().objectName()

    @QtCore.Slot()
    def _show_about_dialog(self) -> None:
        self._about_dialog.show()

    @QtCore.Slot()
    def _shutdown_host_rpi(self) -> None:
        from ros_remote_gui.ros_main import get_ros_node
        get_ros_node().get_logger().warn("System shutdown requested!")
        subprocess.run(["poweroff"])

    @QtCore.Slot()
    def _reboot_host_rpi(self) -> None:
        from ros_remote_gui.ros_main import get_ros_node
        get_ros_node().get_logger().warn("System reboot requested!")
        subprocess.run(["reboot"])

    @QtCore.Slot()
    def _quit_qt_app(self) -> None:
        from ros_remote_gui.ros_main import get_ros_node
        from ros_remote_gui.main import qt_app
        get_ros_node().get_logger().warn("Application shutdown requested!")
        qt_app.quit()

    @QtCore.Slot()
    def _restart_qt_app(self) -> None:
        from ros_remote_gui.ros_main import get_ros_node
        from ros_remote_gui.main import qt_app
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
        # FIXME: We really shouldn't be creating MessageBoxes from threads other than Main Thread.
        if future.exception() or not future.result().data:
            QMessageBox.warning(get_main_window(), "Robot Connection", "Driver connection failed.", buttons=QMessageBox.Ok, defaultButton=QMessageBox.Ok)
        else:
            QMessageBox.information(get_main_window(), "Robot Connection", "Driver connection is OK!", buttons=QMessageBox.Ok, defaultButton=QMessageBox.Ok)

    @QtCore.Slot()
    def _ping_robot_driver_node(self) -> None:
        from ros_remote_gui.ros_main import get_ros_node

        if get_ros_node().ping_driver_srvcl.service_is_ready():
            future = get_ros_node().ping_driver_srvcl.call_async(GetBool.Request())
            future.add_done_callback(self._ping_robot_driver_srv_call)
        else:
            QMessageBox.warning(get_main_window(), "Robot Connection", "Driver connection not available.", buttons=QMessageBox.Ok, defaultButton=QMessageBox.Ok)


# ---- Main window instance ----
_qt_main_window = MainWindow()

def get_main_window() -> MainWindow:
    global _qt_main_window
    return _qt_main_window
