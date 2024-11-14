#  The ROS remote project (GUI package) - GUI utilities.
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

import ros_remote_gui
from PySide6.QtWidgets import QMessageBox


# Generate stylesheet for an indicator label
def generate_indicator_stylesheet(enabled: bool, active_color="red", inactive_color="transparent") -> str:
    return f"background-color: {active_color}" if enabled else f"background-color: {inactive_color}"


# Service call callback helpers
# FIXME: This function is often called from threads other than Main Thread, however, creating MessageBoxes from other treads
#        and setting their parents to main_window results in memory-related issues and potential crashes.
def srvcl_failed_show_err(unavail: bool):
    if not unavail:
        QMessageBox.critical(ros_remote_gui.main_window.get_main_window(), "Error", "ROS service call failed!", buttons=QMessageBox.Ok, defaultButton=QMessageBox.Ok)
    else:
        QMessageBox.warning(ros_remote_gui.main_window.get_main_window(), "Warning", "ROS service unavailable!", buttons=QMessageBox.Ok, defaultButton=QMessageBox.Ok)
