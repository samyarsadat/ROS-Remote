#  The ROS remote project (GUI package) - QT threaded worker class
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

from PySide6 import QtCore
import traceback
import sys


# ---- Worker signals ----
class WorkerSignals(QtCore.QObject):
    finished = QtCore.Signal()
    result = QtCore.Signal(object)
    progress = QtCore.Signal(float)
    error = QtCore.Signal(tuple)


# ---- Worker object ----
class Worker(QtCore.QRunnable):
    def __init__(self, func, *args, **kwargs):
        super(Worker, self).__init__()

        self.func = func
        self.args = args
        self.kwargs = kwargs
        self.signals = WorkerSignals()
        self.kwargs["prog_callback"] = self.signals.progress

    @QtCore.Slot()
    def run(self):
        try:
            result = self.func(*self.args, **self.kwargs)
            self.signals.result.emit(result)
            self.signals.finished.emit()
        except Exception:
            traceback.print_exc()
            exception_type, exception_value = sys.exc_info()[:2]
            self.signals.error.emit((exception_type, exception_value, traceback.format_exc()))
