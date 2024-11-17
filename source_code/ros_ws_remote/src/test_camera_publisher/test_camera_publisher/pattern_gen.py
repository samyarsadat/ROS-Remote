#  The ROS remote project (Test Camera Publisher)
#  Test pattern preparation & generation
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

import cv2
import numpy as np
import pkgutil
from numpy import ndarray
from test_camera_publisher.config import ProgramConfig
from datetime import datetime


# ---- Camera image ----
def get_camera_image(camera_name: str, width: int, height: int) -> (ndarray, int, int):
    img_bin = np.frombuffer(pkgutil.get_data(__name__, ProgramConfig.BASE_TEST_PATTERN), np.uint8)
    pattern = cv2.imdecode(img_bin, cv2.IMREAD_COLOR)
    pattern = cv2.resize(pattern, (width, height), interpolation=cv2.INTER_LINEAR)

    txt_font_size = [height * 0.00125, 2]   # Font scale, thickness
    txt_size = cv2.getTextSize(camera_name, ProgramConfig.CV_TEXT_FONT, txt_font_size[0], txt_font_size[1])[0]
    txt_x = int((pattern.shape[1] - txt_size[0]) / 2)
    txt_y = round(height * 0.1416) if not ProgramConfig.PM5644_MODE else round(height * 0.1454)
    cv2.putText(pattern, camera_name, (txt_x, txt_y), ProgramConfig.CV_TEXT_FONT, txt_font_size[0], (255, 255, 255), txt_font_size[1])

    return pattern, pattern.shape[1], pattern.shape[0]


# ---- Camera overlay ----
def get_camera_overlay(width: int, height: int) -> ndarray:
    pattern = np.zeros((height, width, 3), dtype=np.uint8)
    time_txt = datetime.now().strftime("%H:%M:%S.%f")

    txt_font_size = [height * 0.001318, 2]   # Font scale, thickness
    txt_size = cv2.getTextSize(time_txt, ProgramConfig.CV_TEXT_FONT, txt_font_size[0], txt_font_size[1])[0]
    txt_x = int((pattern.shape[1] - txt_size[0]) / 2)
    txt_y = round(height * 0.813) if not ProgramConfig.PM5644_MODE else round(height * 0.809)
    cv2.putText(pattern, time_txt, (txt_x, txt_y), ProgramConfig.CV_TEXT_FONT, txt_font_size[0], (255, 255, 255), txt_font_size[1])

    """alpha_mask = cv2.inRange(pattern, (0, 0, 0), (0, 0 ,0))
    alpha_mask = cv2.bitwise_not(alpha_mask)
    pattern = cv2.cvtColor(pattern, cv2.COLOR_RGB2RGBA)
    pattern[:, :, 3] = alpha_mask"""

    return pattern


# ---- Overlay moving square generation ----
class OverlaySquare:
    def __init__(self, width: int, height: int, color=(255, 0, 0), size_ratio=5, velocity=None, origin=None):
        if not velocity:
            velocity = [8, 8]
        if not origin:
            origin = [0, 0]

        self._position = origin
        self._velocities = velocity
        self._width = width
        self._height = height
        self._size = round(min(width, height) / size_ratio)
        self._color = color

    def draw_square(self, img: ndarray) -> None:
        self._position[0] += self._velocities[0]
        self._position[1] += self._velocities[1]

        if self._position[0] <= 0 or self._position[0] + self._size >= self._width:
            self._velocities[0] = -self._velocities[0]
        if self._position[1] <= 0 or self._position[1] + self._size >= self._height:
            self._velocities[1] = -self._velocities[1]

        pos_1 = (self._position[0], self._position[1])
        pos_2 = (self._position[0] + self._size, self._position[1] + self._size)
        cv2.rectangle(img, pos_1, pos_2, self._color, -1)
