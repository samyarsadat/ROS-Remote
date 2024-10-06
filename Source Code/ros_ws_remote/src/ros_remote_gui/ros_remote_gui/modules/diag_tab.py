#  The ROS remote project (GUI package) - Diagnostics tab
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

import struct
from datetime import datetime
from enum import Enum
from time import sleep
from diagnostic_msgs.msg import DiagnosticStatus
from asyncio import Future
from PySide6.QtCore import Slot, Signal, QObject
from diagnostic_msgs.srv import SelfTest
from ros_remote_gui.ros_main import get_ros_node
from ros_remote_gui.main_window import get_main_window
from ros_remote_gui.config import ProgramConfig
from ros_remote_gui.utils.gui_utils import srvcl_failed_show_err, generate_indicator_stylesheet
from ros_remote_gui.utils.ui_classes_init import CalibResDialog, SelftestResDialog
from ros_robot_msgs.srv import RunCalibrationsA


# ---- Individual message object ----
class DiagMsgObj:
    timestamp: str
    level: "DiagMsgObj.Level"
    name: str
    message: str
    hardware_id: str
    key_values: dict

    def from_ros_message(self, report: DiagnosticStatus) -> None:
        self.timestamp = datetime.now().strftime("%H:%M:%S")
        self.level = DiagMsgObj.Level(int.from_bytes(report.level, "big"))
        self.name = report.name
        self.message = report.message
        self.hardware_id = report.hardware_id
        self.key_values = {}

        for pair in report.values:
            self.key_values.update({pair.key: pair.value})

    def as_dict(self) -> dict:
        return self.__dict__

    def from_dict(self, data: dict):
        self.timestamp = data["timestamp"]
        self.level = DiagMsgObj.Level(data["level"])
        self.name = data["name"]
        self.message = data["message"]
        self.hardware_id = data["hardware_id"]
        self.key_values = data["key_values"]

    class Level(Enum):
        OK = 0
        WARN = 1
        ERROR = 2
        STALE = 3


# ---- Tab data display update and interaction handler ----
class DiagnosticsTab(QObject):
    rcv_diag_msg_sig = Signal(DiagnosticStatus)
    _show_calib_res_sig = Signal(str, bool)
    _show_selftest_res_sig = Signal(str, bool, str)
    _message_buffer: list[DiagMsgObj]

    def __init__(self):
        super().__init__()
        self._message_buffer = []

        self.rcv_diag_msg_sig.connect(self._rcv_diag_msg_call)
        self._show_calib_res_sig.connect(self._show_calib_res_dialog)
        self._show_selftest_res_sig.connect(self._show_selftest_res_dialog)

        get_main_window().ui.diagMsgOptsAutoScrollCheck.clicked.connect(self._auto_scroll_check_clicked)
        get_main_window().ui.diagMsgOptsShowTimeCheck.clicked.connect(self._show_time_check_clicked)
        get_main_window().ui.diagMsgOptsShowKeyValCheck.clicked.connect(self._show_keyvals_check_clicked)
        get_main_window().ui.diagMsgOptsClearBtn.clicked.connect(self._clear_msg_display)

        get_main_window().ui.calibImuCompassButton.clicked.connect(self._calib_imu_clicked)
        get_main_window().ui.calibCliffOffsetButton.clicked.connect(self._calib_cliff_clicked)
        get_main_window().ui.calibPidTuneButtonL.clicked.connect(self._calib_pid_l_clicked)
        get_main_window().ui.calibPidTuneButtonR.clicked.connect(self._calib_pid_r_clicked)
        get_main_window().ui.selftestPicoAButton.clicked.connect(self._selftest_pico_a_clicked)
        get_main_window().ui.selftestPicoBButton.clicked.connect(self._selftest_pico_b_clicked)

    @Slot(DiagnosticStatus)
    def _rcv_diag_msg_call(self, msg: DiagnosticStatus) -> None:
        diag_msg_obj = DiagMsgObj()
        diag_msg_obj.from_ros_message(msg)
        self._message_buffer.append(diag_msg_obj)
        if len(self._message_buffer) > ProgramConfig.MAX_DIAG_MSG_HISTORY: del self._message_buffer[-1]
        get_main_window().ui.diagMsgsText.append(self._generate_diag_msg_txt(diag_msg_obj, get_main_window().ui.diagMsgOptsShowTimeCheck.isChecked(),
                                                                             get_main_window().ui.diagMsgOptsShowKeyValCheck.isChecked()))
        self._handle_auto_scroll()

    @Slot()
    def _auto_scroll_check_clicked(self) -> None:
        self._handle_auto_scroll()

    @Slot()
    def _show_time_check_clicked(self) -> None:
        self._rerender_all_msgs()

    @Slot()
    def _show_keyvals_check_clicked(self) -> None:
        self._rerender_all_msgs()

    @Slot()
    def _clear_msg_display(self) -> None:
        self._message_buffer = []
        get_main_window().ui.diagMsgsText.setText("")

    def _rerender_all_msgs(self) -> None:
        time_is_checked = get_main_window().ui.diagMsgOptsShowTimeCheck.isChecked()
        keyvals_is_checked = get_main_window().ui.diagMsgOptsShowKeyValCheck.isChecked()
        final_full_txt = ""

        for msg in self._message_buffer:
            final_full_txt = final_full_txt + self._generate_diag_msg_txt(msg, time_is_checked, keyvals_is_checked) + "\r\n"

        get_main_window().ui.diagMsgsText.setText(final_full_txt)
        sleep(0.01)
        self._handle_auto_scroll()

    @staticmethod
    def _handle_auto_scroll() -> None:
        if get_main_window().ui.diagMsgOptsAutoScrollCheck.isChecked():
            vert_scroll_bar = get_main_window().ui.diagMsgsText.verticalScrollBar()
            vert_scroll_bar.setValue(vert_scroll_bar.maximum())

    @staticmethod
    def _generate_diag_msg_txt(msg: DiagMsgObj, timestamp_en: bool, key_vals_en: bool) -> str:
        msg_timestamp = f"[{msg.timestamp}] " if timestamp_en else ""
        msg_line = f"{msg_timestamp}[{msg.name}/{msg.hardware_id}] [{msg.level.name}]: {msg.message}\r\n"

        if key_vals_en:
            key_vals_txt = ""

            for index, (key, val) in enumerate(msg.key_values.items()):
                key_vals_txt = key_vals_txt + f"{' ' * len(msg_timestamp)}[PARAM {index}]: [{key}: {val}]\r\n"
            msg_line = msg_line + key_vals_txt
        return msg_line

    @Slot(str, bool)
    def _show_calib_res_dialog(self, msg: str, status: bool):
        dialog = CalibResDialog(get_main_window())
        dialog.ui.resultStatusLabel.setText("PASSED" if status else "FAILED")
        dialog.ui.resultStatusLabel.setStyleSheet(generate_indicator_stylesheet(status, "green", "red"))
        dialog.ui.resultMessage.setText(msg)
        dialog.show()

    @Slot(str, bool, str)
    def _show_selftest_res_dialog(self, msg: str, status: bool, src: str):
        dialog = SelftestResDialog(get_main_window())
        dialog.ui.resultStatusLabel.setText("PASSED" if status else "FAILED")
        dialog.ui.resultStatusLabel.setStyleSheet(generate_indicator_stylesheet(status, "green", "red"))
        dialog.ui.resultMessage.setText(msg)
        dialog.ui.sourceLabel.setText(src)
        dialog.show()

    def _calib_srvcl_done_call(self, future: Future) -> None:
        if future.exception():
            get_ros_node().get_logger().warn(f"ROS service call failed! (Diagnostics tab button)")
            srvcl_failed_show_err(False)
        else:
            self._show_calib_res_sig.emit(future.result().message, future.result().success)

    def _selftest_srvcl_done_call(self, future: Future) -> None:
        if future.exception():
            get_ros_node().get_logger().warn(f"ROS service call failed! (Diagnostics tab button)")
            srvcl_failed_show_err(False)
        else:
            msg_text = ""

            for msg in future.result().status:
                diag_msg_obj = DiagMsgObj()
                diag_msg_obj.from_ros_message(msg)
                msg_text = msg_text + self._generate_diag_msg_txt(diag_msg_obj, False, True) + "\r\n"
            self._show_selftest_res_sig.emit(msg_text, struct.unpack("?", future.result().passed)[0], future.result().id)

    def _run_calib_request(self, imu: bool, cliff: bool, pid_l: bool, pid_r: bool):
        if get_ros_node().pico_a_calibrate_srvcl.service_is_ready():
            req = RunCalibrationsA.Request()
            req.calib_imu = imu
            req.calib_ir_edge = cliff
            req.calib_pid_left = pid_l
            req.calib_pid_right = pid_r
            future = get_ros_node().pico_a_calibrate_srvcl.call_async(req)
            future.add_done_callback(self._calib_srvcl_done_call)
        else:
            srvcl_failed_show_err(True)

    @Slot()
    def _calib_imu_clicked(self) -> None:
        self._run_calib_request(True, False, False, False)

    @Slot()
    def _calib_cliff_clicked(self) -> None:
        self._run_calib_request(False, True, False, False)

    @Slot()
    def _calib_pid_l_clicked(self) -> None:
        self._run_calib_request(False, False, True, False)

    @Slot()
    def _calib_pid_r_clicked(self) -> None:
        self._run_calib_request(False, False, False, True)

    @Slot()
    def _selftest_pico_a_clicked(self) -> None:
        if get_ros_node().pico_a_selftest_srvcl.service_is_ready():
            req = SelfTest.Request()
            future = get_ros_node().pico_a_selftest_srvcl.call_async(req)
            future.add_done_callback(self._selftest_srvcl_done_call)
        else:
            srvcl_failed_show_err(True)

    @Slot()
    def _selftest_pico_b_clicked(self) -> None:
        if get_ros_node().pico_b_selftest_srvcl.service_is_ready():
            req = SelfTest.Request()
            future = get_ros_node().pico_b_selftest_srvcl.call_async(req)
            future.add_done_callback(self._selftest_srvcl_done_call)
        else:
            srvcl_failed_show_err(True)
