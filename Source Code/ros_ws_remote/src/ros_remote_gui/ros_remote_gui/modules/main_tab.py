#  The ROS remote project (GUI package) - Main tab
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
from ros_remote_gui.ros_main import get_ros_node
from ros_remote_gui.main_window import get_main_window
from ros_remote_gui.config import ProgramConfig
from typing import Any


class MainTab:
    _batt_voltage: float
    _cv_image_cam: Any
    _cv_image_cam_overlay: Any

    def __init__(self):
        self._batt_voltage = 0.0
        self._cv_image_cam = None
        self._cv_image_cam_overlay = None

        get_main_window().ui.camLedsBrightnessSlider.valueChanged.connect(self._cam_led_slider_change)

        # UI data update timer
        self._update_ui_tmr = QTimer()
        self._update_ui_tmr.timeout.connect(self._update_ui_tmr_call)
        self._update_ui_tmr.start(ProgramConfig.UI_DATA_UPDATE_INTERVAL_MS)

        # Viewport update timer
        self._update_viewport_tmr = QTimer()
        self._update_viewport_tmr.timeout.connect(self._update_viewport_tmr_call)
        self._update_viewport_tmr.start(ProgramConfig.UI_VIEWPORT_UPDATE_INTERVAL_MS)

    def _update_ui_tmr_call(self):
        pass

    def _update_viewport_tmr_call(self):
        pass

    @Slot()
    def _cam_led_slider_change(self, value):
        print(str(value))
