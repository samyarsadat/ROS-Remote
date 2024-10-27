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

import time
import cv2
import array
import numpy as np
from asyncio import Future
from PySide6.QtCore import QTimer, Slot, Qt, QThread, Signal
from PySide6.QtGui import QImage, QPixmap
from ros_remote_gui.main import qt_app
from ros_remote_gui.ros_main import get_ros_node, is_ros_node_initialized
from ros_remote_gui.main_window import get_main_window
from ros_remote_gui.config import ProgramConfig
from ros_remote_gui.utils.gui_utils import srvcl_failed_show_err
from ros_robot_msgs.srv import SetCameraLeds
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
from time import sleep


# ---- Viewport frame preparation thread ----
class ViewportThread(QThread):
    update_pixmap_sig = Signal(QPixmap, float)
    test_sig = Signal(int)
    ros_image_cam: CompressedImage
    ros_image_overlay: Image

    def __init__(self, parent):
        super().__init__(parent)
        self._is_running = False
        self._img_bridge = CvBridge()
        self._last_frame_time = 0
        self._current_fps = 0.0

        # Initialize image messages
        blank_rgb = np.zeros((get_main_window().ui.viewport.height(), get_main_window().ui.viewport.width(), 3), dtype=np.uint8)
        self.gen_info_txt_img(blank_rgb, "NO IMAGE RECEIVED!")
        blank_rgb = cv2.cvtColor(blank_rgb, cv2.COLOR_RGB2BGR)
        self.ros_image_cam = self._img_bridge.cv2_to_compressed_imgmsg(blank_rgb, "jpg")
        self.ros_image_overlay = self._img_bridge.cv2_to_imgmsg(np.zeros((get_main_window().ui.viewport.height(), get_main_window().ui.viewport.width(), 4), dtype=np.uint8), "rgba8")

    @staticmethod
    def gen_info_txt_img(rgb_img: np.ndarray, text: str):
        rgb_img[:, :, 2] = 50
        cv2.putText(rgb_img, text, (30, 50), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2, cv2.LINE_AA)

    def run(self) -> None:
        self._is_running = True

        while self._is_running:
            self._current_fps = 1000000000 / (time.time_ns() - self._last_frame_time)
            self._last_frame_time = time.time_ns()

            rgb_img = np.zeros((get_main_window().ui.viewport.height(), get_main_window().ui.viewport.width(), 3), dtype=np.uint8)

            if get_main_window().ui.viewportSelector.currentIndex() == 0:
                try:
                    rgb_img = self._img_bridge.compressed_imgmsg_to_cv2(self.ros_image_cam, "rgb8")
                except Exception:
                    if is_ros_node_initialized():
                        get_ros_node().get_logger().error("ROS -> OpenCV image conversion failure!")
            elif get_main_window().ui.viewportSelector.currentIndex() == 1:
                try:
                    camera_img = self._img_bridge.compressed_imgmsg_to_cv2(self.ros_image_cam, "rgb8")
                    overlay_img = self._img_bridge.imgmsg_to_cv2(self.ros_image_overlay, "rgba8")
                    rgb, alpha = overlay_img[:, :, :3], overlay_img[:, :, 3]
                    alpha = alpha.astype(float) / 255.0
                    rgb_img = np.zeros_like(rgb, dtype=np.uint8)

                    for c in range(3):
                        rgb_img[:, :, c] = (alpha * rgb[:, :, c] + (1 - alpha) * camera_img[:, :, c]).astype(np.uint8)
                except Exception:
                    if is_ros_node_initialized():
                        get_ros_node().get_logger().error("ROS -> OpenCV image conversion failure!")
            elif get_main_window().ui.viewportSelector.currentIndex() == 2:
                self.gen_info_txt_img(rgb_img, "VIEW NOT IMPLEMENTED")
            else:
                self.gen_info_txt_img(rgb_img, "UNKNOWN VIEW")

            height, width, num_ch = rgb_img.shape
            bytes_per_line = num_ch * width
            convert_to_qt_format = QImage(rgb_img.data, width, height, bytes_per_line, QImage.Format_RGB888)
            q_img = convert_to_qt_format.scaled(get_main_window().ui.viewport.width(), get_main_window().ui.viewport.height(), Qt.KeepAspectRatio)
            self.update_pixmap_sig.emit(QPixmap.fromImage(q_img), self._current_fps)

            # Framerate limiting
            sleep_time_ns = (1000000000 / ProgramConfig.VIEWPORT_TARGET_FPS) - (time.time_ns() - self._last_frame_time)
            sleep((sleep_time_ns / 1000000000) if sleep_time_ns > 0 else 0)

    def stop(self) -> None:
        self._is_running = False
        sleep(0.1)   # Wait for run() to finish.
        self.quit()


# ---- Tab data display update and interaction handler ----
class MainTab:
    batt_voltage: float
    viewport_fps: float

    def __init__(self):
        self.batt_voltage = 0.0
        self.viewport_fps = 0.0

        get_main_window().ui.camLedsBrightnessSlider.valueChanged.connect(self._cam_led_slider_change)
        get_main_window().ui.camLed1Check.stateChanged.connect(self._check_led_1_box)
        get_main_window().ui.camLed2Check.stateChanged.connect(self._check_led_2_box)
        get_main_window().ui.camLed3Check.stateChanged.connect(self._check_led_3_box)
        get_main_window().ui.camLed4Check.stateChanged.connect(self._check_led_4_box)

        # Viewport
        self.viewport_thread = ViewportThread(get_main_window())
        self.viewport_thread.update_pixmap_sig.connect(self._set_viewport_pixmap)
        self.viewport_thread.start()
        qt_app.aboutToQuit.connect(self.viewport_thread.stop)

        # UI data update timer
        self._update_ui_tmr = QTimer()
        self._update_ui_tmr.timeout.connect(self._update_ui_tmr_call)
        self._update_ui_tmr.start(ProgramConfig.UI_DATA_UPDATE_INTERVAL_MS)

    def _update_ui_tmr_call(self) -> None:
        if get_main_window().ui.pages.currentWidget().objectName() == get_main_window().ui.mainTab.objectName():
            get_main_window().ui.viewportFpsValue.setText(str(round(self.viewport_fps, 1)))
            get_main_window().ui.teleBatteryValue.setText(str(round(self.batt_voltage, 2)))

    @Slot(QPixmap, float)
    def _set_viewport_pixmap(self, pixmap: QPixmap, fps: float):
        get_main_window().ui.viewport.setPixmap(pixmap)
        self.viewport_fps = fps

    @staticmethod
    def _cam_srvcl_done_call(future: Future) -> None:
        if future.exception() or not future.result().success:
            get_ros_node().get_logger().warn("ROS service call failed! (Camera LED)")
            srvcl_failed_show_err(False)

    @Slot(int)
    def _cam_led_slider_change(self, value) -> None:
        if get_ros_node().set_camera_leds_srvcl.service_is_ready():
            cam_led_req = SetCameraLeds.Request()
            cam_led_req.set_output_mask = [get_main_window().ui.camLed1Check.isChecked(),
                                           get_main_window().ui.camLed2Check.isChecked(),
                                           get_main_window().ui.camLed3Check.isChecked(),
                                           get_main_window().ui.camLed4Check.isChecked()]
            value = int(value * 655.35)
            cam_led_req.led_outputs = array.array("I", [value, value, value, value])
            future = get_ros_node().set_camera_leds_srvcl.call_async(cam_led_req)
            future.add_done_callback(self._cam_srvcl_done_call)
        else:
            srvcl_failed_show_err(True)

    def _set_led_state_checkbox(self, led_num: int, checkbox_state: Qt.CheckState) -> None:
        if get_ros_node().set_camera_leds_srvcl.service_is_ready():
            cam_led_req = SetCameraLeds.Request()
            cam_led_req.set_output_mask = [False, False, False, False]
            cam_led_req.led_outputs = array.array("I", [0, 0, 0, 0])
            cam_led_req.set_output_mask[led_num] = True

            if checkbox_state == Qt.CheckState.Checked.value:
                cam_led_req.led_outputs[led_num] = int(get_main_window().ui.camLedsBrightnessSlider.value() * 655.35)

            future = get_ros_node().set_camera_leds_srvcl.call_async(cam_led_req)
            future.add_done_callback(self._cam_srvcl_done_call)
        else:
            srvcl_failed_show_err(True)

    @Slot(Qt.CheckState)
    def _check_led_1_box(self, state) -> None:
        self._set_led_state_checkbox(0, state)

    @Slot(Qt.CheckState)
    def _check_led_2_box(self, state) -> None:
        self._set_led_state_checkbox(1, state)

    @Slot(Qt.CheckState)
    def _check_led_3_box(self, state) -> None:
        self._set_led_state_checkbox(2, state)

    @Slot(Qt.CheckState)
    def _check_led_4_box(self, state) -> None:
        self._set_led_state_checkbox(3, state)
