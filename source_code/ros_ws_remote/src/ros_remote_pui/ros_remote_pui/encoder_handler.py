#  The ROS remote project (PUI package)
#  Encoder IO handler. This is for GUI navigation using the encoder.
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

from PySide6.QtCore import QObject, Signal, QTimer
from PySide6.QtWidgets import QApplication, QPushButton, QTabBar
from gpiozero import RotaryEncoder, Button
from ros_remote_gui.main_window import get_main_window
from ros_remote_pui.config import RpiIoConfig, ProgramConfig


# Encoder input handler
class EncoderInput(QObject):
    enc_rotated = Signal(bool)
    btn_pressed = Signal()

    def __init__(self):
        super().__init__()

        self._rotary_enc = RotaryEncoder(RpiIoConfig.ENCODER_PIN_A, RpiIoConfig.ENCODER_PIN_B)
        self._rotary_enc.when_rotated_clockwise = self._enc_rotate_call_cw
        self._rotary_enc.when_rotated_counter_clockwise = self._enc_rotate_call_ccw

        self._rotary_btn = Button(RpiIoConfig.ENCODER_BTN_PIN, pull_up=False, bounce_time=RpiIoConfig.BUTTON_DEBOUNCE_TIME_S)
        self._rotary_btn.when_activated = self._enc_btn_call

    def _enc_btn_call(self) -> None:
        self.btn_pressed.emit()

    def _enc_rotate_call_cw(self) -> None:
        self.enc_rotated.emit(True)

    def _enc_rotate_call_ccw(self) -> None:
        self.enc_rotated.emit(False)


# Encoder UI navigation handler
class EncoderNavHandler:
    def __init__(self):
        self._input_handler = EncoderInput()
        self._input_handler.enc_rotated.connect(self._enc_rotate)
        self._input_handler.btn_pressed.connect(self._enc_btn_press)

        self._highlight_timer = QTimer()
        self._highlight_timer.setInterval(ProgramConfig.ENCODER_HIGHLIGHT_TIMEOUT_MS)
        self._highlight_timer.setSingleShot(True)
        self._highlight_timer.timeout.connect(self._clear_highlight)

        self._highlight_stylesheet = "QWidget:focus { border: 2px solid blue; }"
        self._clear_stylesheet = "QWidget:focus { border: none; }"

    def _enc_btn_press(self) -> None:
        focused_widget = QApplication.focusWidget()

        if isinstance(focused_widget, QPushButton):
            focused_widget.click()
        elif isinstance(focused_widget, QTabBar):
            current_index = focused_widget.currentIndex()
            next_index = (current_index + 1) % focused_widget.count()
            focused_widget.setCurrentIndex(next_index)
        self._apply_highlight()

    def _enc_rotate(self, direction: bool) -> None:
        if direction:
            get_main_window().focusNextChild()
        else:
            get_main_window().focusPreviousChild()
        self._apply_highlight()

    def _apply_highlight(self) -> None:
        if get_main_window().styleSheet().find(self._highlight_stylesheet) == -1:
            get_main_window().setStyleSheet(get_main_window().styleSheet() + self._highlight_stylesheet)

        if self._highlight_timer.isActive():
            self._highlight_timer.stop()
        self._highlight_timer.start()

    def _clear_highlight(self) -> None:
        get_main_window().setStyleSheet(get_main_window().styleSheet().replace(self._highlight_stylesheet, self._clear_stylesheet))
