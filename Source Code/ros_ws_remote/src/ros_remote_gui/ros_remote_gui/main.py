#  The ROS remote project (GUI package)
#  Main file - everything is ran from here
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

import sys
import threading
from PySide6.QtCore import QProcess
from ros_remote_gui.init import qt_app
from ros_remote_gui.main_window import qt_main_window
from ros_remote_gui.ros_main import ros_executor_thread


# ---- Run the program ----
def main():
    # Start the ROS thread
    stop_ros_thread = False
    ros_thread = threading.Thread(target=ros_executor_thread, args=(lambda: stop_ros_thread, ), name="ros_thread")
    ros_thread.start()

    # Start the application
    qt_main_window.show()
    qt_ret_code = qt_app.exec()

    # Shutdown
    stop_ros_thread = True
    ros_thread.join()

    if qt_main_window.restart_on_quit:
        QProcess.startDetached(qt_app.arguments()[0], qt_app.arguments().pop(0))

    sys.exit(qt_ret_code)


if __name__ == "__main__":
    main()
