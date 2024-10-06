#  The ROS remote project (GUI package) - Power tab
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

from PySide6.QtCore import QTimer, Slot
from ros_remote_gui.main_window import get_main_window
from ros_remote_gui.config import ProgramConfig


# ---- Tab data display update and interaction handler ----
class PowerTab:
    batt_voltage: float
    batt_current: float

    def __init__(self):
        self.batt_voltage = 0.0
        self.batt_current = 0.0

        # UI data update timer
        self._update_ui_tmr = QTimer()
        self._update_ui_tmr.timeout.connect(self._update_ui_tmr_call)
        self._update_ui_tmr.start(ProgramConfig.UI_DATA_UPDATE_INTERVAL_MS)

    def _update_ui_tmr_call(self) -> None:
        if get_main_window().ui.pages.currentWidget().objectName() == get_main_window().ui.powerTab.objectName():
            get_main_window().ui.batteryVoltageValue.display(round(self.batt_voltage, 2))
            get_main_window().ui.batteryCurrentValue.display(round((self.batt_current / 1000), 3))
