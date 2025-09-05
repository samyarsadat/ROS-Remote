/*
    The ROS remote project - Remote HID interface ROS package
    Copyright 2025 Samyar Sadat Akhavi.
    Written by Samyar Sadat Akhavi, 2025.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#pragma once
#include <hidapi/hidapi.h>
#include <expected>
#include <atomic>
#include <functional>
#include <future>
#include <mutex>
#include <string>
#include <thread>
#include <rclcpp/logging.hpp>
#include "hid_config.hpp"


// Main HID helper class
class HIDDevice {
    public:
        explicit HIDDevice(rclcpp::Logger logger);
        ~HIDDevice();

        std::expected<void, std::string>
        open_hid(const uint16_t vid, const uint16_t pid, const uint8_t itf_num);
        void close_hid();

        std::expected<void, std::string>
        set_led_state(const uint8_t index, const uint8_t mode, const uint16_t pwm_out);
        std::expected<hid_led_states_report, std::string> get_led_states();

    private:
        rclcpp::Logger logger;

        std::mutex hid_dev_mutex;
        hid_device* hid_dev_handle{nullptr};

        inline static std::string wstr_to_string(const wchar_t* wstr) {
            if (!wstr)
                return {};
            std::wstring ws(wstr);

            // Non-ASCII characters will get lost/mangled, I know.
            return std::string(ws.begin(), ws.end());
        }

        inline static std::string get_last_error(hid_device* device) {
            return wstr_to_string(hid_error(device));
        }
};
