#  The ROS remote project (GUI package) - Motor controllers tab
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

from asyncio import Future
from datetime import datetime, timedelta
from PySide6.QtCore import QTimer, Slot
from ros_remote_gui.ros_main import get_ros_node
from ros_remote_gui.main_window import get_main_window
from ros_remote_gui.config import ProgramConfig
from ros_remote_gui.utils.gui_utils import generate_indicator_stylesheet, srvcl_failed_show_err
from std_srvs.srv import SetBool


# ---- Tab data display update and interaction handler ----
class MotorTab:
    position_x: float
    position_y: float
    orientation_yaw: float
    linear_velocity: float
    angular_velocity: float
    encoder_pulse_ctrs_left: list[int]
    encoder_pulse_ctrs_right: list[int]
    rpm_measured_left: list[float]
    rpm_measured_right: list[float]
    rpm_target_left: float
    rpm_target_right: float
    left_ctrl_pid_tunings: list[float]
    right_ctrl_pid_tunings: list[float]
    left_ctrl_pid_out: int
    right_ctrl_pid_out: int
    left_ctrl_enabled: bool
    right_ctrl_enabled: bool
    last_right_data_rcv: datetime
    last_left_data_rcv: datetime

    def __init__(self):
        self.position_x = 0.0
        self.position_y = 0.0
        self.orientation_yaw = 0.0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.encoder_pulse_ctrs_left = [0, 0]
        self.encoder_pulse_ctrs_right = [0, 0]
        self.rpm_measured_left = [0.0, 0.0]
        self.rpm_measured_right = [0.0, 0.0]
        self.rpm_target_left = 0.0
        self.rpm_target_right = 0.0
        self.left_ctrl_pid_tunings = [0.0, 0.0, 0.0]
        self.right_ctrl_pid_tunings = [0.0, 0.0, 0.0]
        self.left_ctrl_pid_out = 0
        self.right_ctrl_pid_out = 0
        self.left_ctrl_enabled = False
        self.right_ctrl_enabled = False
        self.last_right_data_rcv = datetime(1, 1, 1)
        self.last_left_data_rcv = datetime(1, 1, 1)

        get_main_window().ui.mtrCtrlEnableButton.clicked.connect(self._enable_mtr_ctrls_call)
        get_main_window().ui.mtrCtrlDisableButton.clicked.connect(self._disable_mtr_ctrls_call)

        # UI data update timer
        self._update_ui_tmr = QTimer()
        self._update_ui_tmr.timeout.connect(self._update_ui_tmr_call)
        self._update_ui_tmr.start(ProgramConfig.UI_DATA_UPDATE_INTERVAL_MS)

    def _update_ui_tmr_call(self) -> None:
        if get_main_window().ui.pages.currentWidget().objectName() == get_main_window().ui.motorCtrlTab.objectName():
            get_main_window().ui.encOdomPosXValue.setText(str(round(self.position_x, 3)))
            get_main_window().ui.encOdomPosYValue.setText(str(round(self.position_y, 3)))
            get_main_window().ui.encOdomYawValue.setText(str(round(self.orientation_yaw, 1)))
            get_main_window().ui.encOdomLinVelValue.setText(str(round(self.linear_velocity, 2)))
            get_main_window().ui.encOdomAngVelValue.setText(str(round(self.angular_velocity, 2)))
            get_main_window().ui.rpmMeasuredFRValue.setText(str(round(self.rpm_measured_right[0], 1)))
            get_main_window().ui.rpmMeasuredFLValue.setText(str(round(self.rpm_measured_left[0], 1)))
            get_main_window().ui.rpmMeasuredBRValue.setText(str(round(self.rpm_measured_right[1], 1)))
            get_main_window().ui.rpmMeasuredBLValue.setText(str(round(self.rpm_measured_left[1], 1)))
            get_main_window().ui.rpmTargetFRValue.setText(str(round(self.rpm_target_right, 1)))
            get_main_window().ui.rpmTargetFLValue.setText(str(round(self.rpm_target_left, 1)))
            get_main_window().ui.rpmTargetBRValue.setText(str(round(self.rpm_target_right, 1)))
            get_main_window().ui.rpmTargetBLValue.setText(str(round(self.rpm_target_left, 1)))
            get_main_window().ui.rightPidOutValue.setText(str(self.right_ctrl_pid_out))
            get_main_window().ui.leftPidOutValue.setText(str(self.left_ctrl_pid_out))
            get_main_window().ui.rightPidPValue.setText(str(round(self.right_ctrl_pid_tunings[0], 2)))
            get_main_window().ui.leftPidPValue.setText(str(round(self.left_ctrl_pid_tunings[0], 2)))
            get_main_window().ui.rightPidIValue.setText(str(round(self.right_ctrl_pid_tunings[1], 2)))
            get_main_window().ui.leftPidIValue.setText(str(round(self.left_ctrl_pid_tunings[1], 2)))
            get_main_window().ui.rightPidDValue.setText(str(round(self.right_ctrl_pid_tunings[2], 2)))
            get_main_window().ui.leftPidDValue.setText(str(round(self.left_ctrl_pid_tunings[2], 2)))
            get_main_window().ui.encPulseCtrsFRValue.setText(str(self.encoder_pulse_ctrs_right[0]))
            get_main_window().ui.encPulseCtrsFLValue.setText(str(self.encoder_pulse_ctrs_left[0]))
            get_main_window().ui.encPulseCtrsBRValue.setText(str(self.encoder_pulse_ctrs_right[1]))
            get_main_window().ui.encPulseCtrsBLValue.setText(str(self.encoder_pulse_ctrs_left[1]))

            # For stale information
            if not (datetime.now() - self.last_right_data_rcv) > timedelta(seconds=ProgramConfig.STALE_DATA_TIMEOUT):
                get_main_window().ui.mtrCtrlEnableRIndicator.setStyleSheet(generate_indicator_stylesheet(self.right_ctrl_enabled, "green"))
            elif self.right_ctrl_enabled:
                get_main_window().ui.mtrCtrlEnableRIndicator.setStyleSheet(generate_indicator_stylesheet(True, "yellow"))

            if not (datetime.now() - self.last_left_data_rcv) > timedelta(seconds=ProgramConfig.STALE_DATA_TIMEOUT):
                get_main_window().ui.mtrCtrlEnableLIndicator.setStyleSheet(generate_indicator_stylesheet(self.left_ctrl_enabled, "green"))
            elif self.left_ctrl_enabled:
                get_main_window().ui.mtrCtrlEnableLIndicator.setStyleSheet(generate_indicator_stylesheet(True, "yellow"))

    @staticmethod
    def _btn_srvcl_done_call(future: Future) -> None:
        if future.exception():
            get_ros_node().get_logger().warn(f"ROS service call failed! (Motor tab button)")
            srvcl_failed_show_err(False)
        elif not future.result().success:
            get_ros_node().get_logger().warn(f"ROS service call failed! (Motor tab button): {future.result().message}")
            srvcl_failed_show_err(False)

    @Slot()
    def _enable_mtr_ctrls_call(self) -> None:
        if get_ros_node().mtr_ctrl_enable_srvcl.service_is_ready():
            req = SetBool.Request()
            req.data = True
            future = get_ros_node().mtr_ctrl_enable_srvcl.call_async(req)
            future.add_done_callback(self._btn_srvcl_done_call)
        else:
            srvcl_failed_show_err(True)

    @Slot()
    def _disable_mtr_ctrls_call(self) -> None:
        if get_ros_node().mtr_ctrl_enable_srvcl.service_is_ready():
            req = SetBool.Request()
            req.data = False
            future = get_ros_node().mtr_ctrl_enable_srvcl.call_async(req)
            future.add_done_callback(self._btn_srvcl_done_call)
        else:
            srvcl_failed_show_err(True)