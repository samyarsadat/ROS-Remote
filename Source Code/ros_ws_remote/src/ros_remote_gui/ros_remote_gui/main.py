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
from PySide6.QtWidgets import QApplication
from PySide6.QtCore import QProcess, QTimer
from ros_remote_gui.config import ProgramConfig, RosConfig
from ros_remote_pui.ros_main import RemotePuiThread

# ---- QT application ----
qt_app = QApplication(sys.argv)

# Must import main_window after qt_app init!
from ros_remote_gui.main_window import get_main_window


def main():
    from ros_remote_gui.ros_main import ros_executor_thread, is_ros_node_initialized, get_ros_node
    get_main_window().init_ui_handlers()

    # Start the ROS thread
    stop_ros_thread = False
    ros_thread = threading.Thread(target=ros_executor_thread, args=(lambda: stop_ros_thread, ), name=RosConfig.THREAD_NAME)
    ros_thread.start()
    RemotePuiThread.start_thread()

    # Create a timer for checking ROS thread liveliness
    def ros_liveliness_check() -> None:
        if not ros_thread.is_alive():
            if is_ros_node_initialized():
                get_ros_node().get_logger().fatal("The ROS thread has died! Terminating program.")
            else:
                print("The ROS thread has died! Terminating program.")
            qt_app.exit(1)

        if not RemotePuiThread.is_alive():
            if is_ros_node_initialized():
                get_ros_node().get_logger().fatal("The remote PUI ROS thread has died! Terminating program.")
            else:
                print("The remote PUI ROS thread has died! Terminating program.")
            qt_app.exit(1)

    ros_liveliness_timer = QTimer()
    ros_liveliness_timer.timeout.connect(ros_liveliness_check)
    ros_liveliness_timer.start(ProgramConfig.THREADS_LIVELINESS_CHECK_INTERVAL_S * 1000)

    # Start the application
    get_main_window().show()
    qt_ret_code = qt_app.exec()

    # Shutdown
    stop_ros_thread = True
    RemotePuiThread.stop_thread(True)
    ros_thread.join()

    if is_ros_node_initialized():
        get_ros_node().get_logger().info(f"Program exiting with code {qt_ret_code}")

    if get_main_window().restart_on_quit:
        qt_app_args = qt_app.arguments(); qt_app_args.pop(0)
        QProcess.startDetached(qt_app.arguments()[0], qt_app_args)

    sys.exit(qt_ret_code)
