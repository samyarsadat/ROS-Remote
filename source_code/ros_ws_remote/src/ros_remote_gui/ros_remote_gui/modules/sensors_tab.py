#  The ROS remote project (GUI package) - Sensors tab
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
from PySide6.QtCore import QTimer, Slot
from ros_remote_gui.ros_main import get_ros_node
from ros_remote_gui.main_window import get_main_window
from ros_remote_gui.config import ProgramConfig
from ros_remote_gui.utils.gui_utils import generate_indicator_stylesheet, srvcl_failed_show_err
from std_srvs.srv import SetBool


# ---- Tab data display update and interaction handler ----
class SensorTab:
    ultra_dists: list[float]
    env_dht_temp: float
    env_dht_humidity: int
    env_imu_temp: float
    env_pico_a_temp: float
    env_pico_b_temp: float
    env_rpi_temp: float
    batt_voltage: float
    batt_current: float
    cliff_front_states: list[float]
    cliff_back_states: list[float]
    imu_accels: list[float]
    imu_gyros: list[float]
    imu_comps: list[float]
    micro_sw_states: list[float]
    emitters_enabled: bool

    def __init__(self):
        self.ultra_dists = [0.0, 0.0, 0.0, 0.0]
        self.env_dht_temp = 0.0
        self.env_dht_humidity = 0
        self.env_imu_temp = 0.0
        self.env_pico_a_temp = 0.0
        self.env_pico_b_temp = 0.0
        self.env_rpi_temp = 0.0
        self.batt_voltage = 0.0
        self.batt_current = 0.0
        self.cliff_front_states = [0.0, 0.0, 0.0, 0.0]
        self.cliff_back_states = [0.0, 0.0, 0.0, 0.0]
        self.imu_accels = [0.0, 0.0, 0.0]
        self.imu_gyros = [0.0, 0.0, 0.0]
        self.imu_comps = [0.0, 0.0, 0.0]
        self.micro_sw_states = [0.0, 0.0, 0.0, 0.0]

        get_main_window().ui.togglePicoARelayEnable.clicked.connect(self._pico_a_relay_btn_call)
        get_main_window().ui.toggleEmittersEnabled.clicked.connect(self._en_emitters_btn_call)

        #emitters_enabled_future = get_ros_node().get_emitters_enabled_srvcl.call_async(GetBool.Request())
        #emitters_enabled_future.add_done_callback(self._get_emitters_enabled_call)

        # UI data update timer
        self._update_ui_tmr = QTimer()
        self._update_ui_tmr.timeout.connect(self._update_ui_tmr_call)

    def _update_ui_tmr_call(self) -> None:
        if get_main_window().ui.pages.currentWidget().objectName() == get_main_window().ui.sensorsTab.objectName():
            get_main_window().ui.ultrasonicSensValueF.setText(str(round(self.ultra_dists[0], 1)))
            get_main_window().ui.ultrasonicSensValueB.setText(str(round(self.ultra_dists[1], 1)))
            get_main_window().ui.ultrasonicSensValueR.setText(str(round(self.ultra_dists[2], 1)))
            get_main_window().ui.ultrasonicSensValueL.setText(str(round(self.ultra_dists[3], 1)))
            get_main_window().ui.envSensTempValue.setText(str(round(self.env_dht_temp, 1)))
            get_main_window().ui.envSensHumValue.setText(str(self.env_dht_humidity))
            get_main_window().ui.envSensImuTempValue.setText(str(round(self.env_imu_temp, 1)))
            get_main_window().ui.envSensPicoATempValue.setText(str(round(self.env_pico_a_temp, 1)))
            get_main_window().ui.envSensPicoBTempValue.setText(str(round(self.env_pico_b_temp, 1)))
            get_main_window().ui.envSensPiTempValue.setText(str(round(self.env_rpi_temp, 1)))
            get_main_window().ui.batterySensVoltageValue.display(round(self.batt_voltage, 2))
            get_main_window().ui.batterySensCurrentValue.display(round((self.batt_current / 1000), 3))
            get_main_window().ui.microswSensIndicatorFR.setStyleSheet(generate_indicator_stylesheet(self.micro_sw_states[0] == float("-inf")))
            get_main_window().ui.microswSensIndicatorFL.setStyleSheet(generate_indicator_stylesheet(self.micro_sw_states[1] == float("-inf")))
            get_main_window().ui.microswSensIndicatorBR.setStyleSheet(generate_indicator_stylesheet(self.micro_sw_states[2] == float("-inf")))
            get_main_window().ui.microswSensIndicatorBL.setStyleSheet(generate_indicator_stylesheet(self.micro_sw_states[3] == float("-inf")))
            get_main_window().ui.ultrasonicSensValueF.setText(str(round(self.ultra_dists[0], 1)))
            get_main_window().ui.ultrasonicSensValueB.setText(str(round(self.ultra_dists[1], 1)))
            get_main_window().ui.ultrasonicSensValueR.setText(str(round(self.ultra_dists[2], 1)))
            get_main_window().ui.ultrasonicSensValueL.setText(str(round(self.ultra_dists[3], 1)))
            get_main_window().ui.cliffSensIndicatorF1.setStyleSheet(generate_indicator_stylesheet(self.cliff_front_states[0] == float("inf")))
            get_main_window().ui.cliffSensIndicatorF2.setStyleSheet(generate_indicator_stylesheet(self.cliff_front_states[1] == float("inf")))
            get_main_window().ui.cliffSensIndicatorF3.setStyleSheet(generate_indicator_stylesheet(self.cliff_front_states[2] == float("inf")))
            get_main_window().ui.cliffSensIndicatorF4.setStyleSheet(generate_indicator_stylesheet(self.cliff_front_states[3] == float("inf")))
            get_main_window().ui.cliffSensIndicatorB1.setStyleSheet(generate_indicator_stylesheet(self.cliff_back_states[0] == float("inf")))
            get_main_window().ui.cliffSensIndicatorB2.setStyleSheet(generate_indicator_stylesheet(self.cliff_back_states[1] == float("inf")))
            get_main_window().ui.cliffSensIndicatorB4.setStyleSheet(generate_indicator_stylesheet(self.cliff_back_states[2] == float("inf")))
            get_main_window().ui.cliffSensIndicatorB3.setStyleSheet(generate_indicator_stylesheet(self.cliff_back_states[3] == float("inf")))
            get_main_window().ui.imuSensAccelXValue.setText(str(round(self.imu_accels[0], 2)))
            get_main_window().ui.imuSensAccelYValue.setText(str(round(self.imu_accels[1], 2)))
            get_main_window().ui.imuSensAccelZValue.setText(str(round(self.imu_accels[2], 2)))
            get_main_window().ui.imuSensGyroXValue.setText(str(round(self.imu_gyros[0], 2)))
            get_main_window().ui.imuSensGyroYValue.setText(str(round(self.imu_gyros[1], 2)))
            get_main_window().ui.imuSensGyroZValue.setText(str(round(self.imu_gyros[2], 2)))
            get_main_window().ui.imuSensCompXValue.setText(str(round(self.imu_comps[0], 1)))
            get_main_window().ui.imuSensCompYValue.setText(str(round(self.imu_comps[1], 1)))
            get_main_window().ui.imuSensCompZValue.setText(str(round(self.imu_comps[2], 1)))

    @staticmethod
    def _get_emitters_enabled_call(future: Future):
        if not future.exception() and future.result():
            get_main_window().ui.toggleEmittersEnabled.setChecked(future.result().data)
        else:
            get_ros_node().get_logger().error("Failed to get emitters enabled status.")

    @staticmethod
    def _btn_srvcl_done_call(future: Future) -> None:
        if future.exception():
            get_ros_node().get_logger().warn(f"ROS service call failed! (Sens. tab button)")
            srvcl_failed_show_err(False)
        elif not future.result().success:
            get_ros_node().get_logger().warn(f"ROS service call failed! (Sens. tab button): {future.result().message}")
            srvcl_failed_show_err(False)

    @Slot()
    def _pico_a_relay_btn_call(self) -> None:
        if get_ros_node().enable_relay_srvcl.service_is_ready():
            req = SetBool.Request()
            req.data = get_main_window().ui.togglePicoARelayEnable.isChecked()
            future = get_ros_node().enable_relay_srvcl.call_async(req)
            future.add_done_callback(self._btn_srvcl_done_call)
        else:
            srvcl_failed_show_err(True)

    @Slot()
    def _en_emitters_btn_call(self) -> None:
        if get_ros_node().enable_emitters_srvcl.service_is_ready():
            req = SetBool.Request()
            req.data = get_main_window().ui.toggleEmittersEnabled.isChecked()
            future = get_ros_node().enable_emitters_srvcl.call_async(req)
            future.add_done_callback(self._btn_srvcl_done_call)
        else:
            srvcl_failed_show_err(True)
