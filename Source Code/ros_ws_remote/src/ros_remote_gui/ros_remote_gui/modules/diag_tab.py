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

from datetime import datetime
from enum import Enum
from diagnostic_msgs.msg import DiagnosticStatus


# Individual message object
class DiagMsgObj:
    timestamp: datetime
    level: "DiagMsgObj.Level"
    name: str
    message: str
    hardware_id: str
    key_values: dict

    def from_ros_message(self, report: DiagnosticStatus) -> None:
        self.timestamp = datetime.now()
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
        self.level = data["level"]

    class Level(Enum):
        OK = 0
        WARN = 1
        ERROR = 2
        STALE = 3


# Message display handler
class DiagMsgDisplay:
    show_timestamps: bool
    show_keyvals: bool
    auto_scroll: bool

    def __init__(self):
        self.show_timestamps = False
        self.show_keyvals = False
        self.auto_scroll = False