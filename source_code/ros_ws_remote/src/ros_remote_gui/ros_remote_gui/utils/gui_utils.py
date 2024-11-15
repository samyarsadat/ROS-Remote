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
from PySide6.QtCore import QObject, Signal, Slot


# Generate stylesheet for an indicator label
def generate_indicator_stylesheet(enabled: bool, active_color="red", inactive_color="transparent") -> str:
    return f"background-color: {active_color}" if enabled else f"background-color: {inactive_color}"


# Message box helper. Used for displaying message boxes from threads other than Main Thread.
class MessageBoxHelper(QObject):
    # (type {"info", "warn", "error"}, title, text)
    show_msg_box_sig = Signal(str, str, str)

    def __init__(self):
        super().__init__()
        self.show_msg_box_sig.connect(self._show_msg_box)
        self._default_buttons = QMessageBox.Ok

    @Slot(str, str, str)
    def _show_msg_box(self, type: str, title: str, text: str) -> None:
        match type:
            case "info":
                QMessageBox.information(ros_remote_gui.main_window.get_main_window(), title, text, buttons=self._default_buttons)
            case "warn":
                QMessageBox.warning(ros_remote_gui.main_window.get_main_window(), title, text, buttons=self._default_buttons)
            case "error":
                QMessageBox.critical(ros_remote_gui.main_window.get_main_window(), title, text, buttons=self._default_buttons)
            case _:
                pass

# MessageBoxHelper instance
_msg_box_helper = MessageBoxHelper()

def get_msg_box_helper() -> MessageBoxHelper:
    return _msg_box_helper


# Service call callback helpers
def srvcl_failed_show_err(unavail: bool):
    if not unavail:
        get_msg_box_helper().show_msg_box_sig.emit("error", "Error", "ROS service call failed!")
    else:
        get_msg_box_helper().show_msg_box_sig.emit("warn", "Warning", "ROS service unavailable!")
