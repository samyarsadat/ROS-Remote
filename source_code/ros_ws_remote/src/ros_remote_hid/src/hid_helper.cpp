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

#include "hid_helper.hpp"
#include <bit>
#include <experimental/scope>
#include <cstring>


// ---- Constructor / Destructor ----
HIDDevice::HIDDevice(rclcpp::Logger logger)
    : logger(logger) {
}

HIDDevice::~HIDDevice() {
    close_hid();
    (void) hid_exit();
}


// ---- Member functions ----
std::expected<void, std::string>
HIDDevice::open_hid(const uint16_t vid, const uint16_t pid, const uint8_t itf_num) {
    std::lock_guard lock(hid_dev_mutex);

    if (hid_dev_handle) {
        return std::unexpected("HID device already open");
    }

    // Enumerate devices
    hid_device_info* devs = hid_enumerate(vid, pid);
    if (!devs)
        return std::unexpected("failed to enumerate HID devices: " + get_last_error(nullptr));
    auto _ = std::experimental::scope_exit([devs] { hid_free_enumeration(devs); });

    const char* itf_path = nullptr;
    for (const auto* cur = devs; cur; cur = cur->next) {
        if (cur->interface_number == itf_num) {
            itf_path = cur->path;
            RCLCPP_INFO_STREAM(logger,
                               "HID device found: " << wstr_to_string(cur->product_string) << " ("
                                                    << wstr_to_string(cur->manufacturer_string)
                                                    << ") - interface: " << cur->interface_number);
            break;
        }
    }

    if (!itf_path) {
        return std::unexpected("HID interface not found");
    }

    // Open interface
    hid_dev_handle = hid_open_path(itf_path);
    if (!hid_dev_handle) {
        return std::unexpected("failed to open HID device interface: " + get_last_error(nullptr));
    }

    if (hid_set_nonblocking(hid_dev_handle, 1) < 0) {
        std::string err_msg = get_last_error(hid_dev_handle);
        hid_close(hid_dev_handle);
        hid_dev_handle = nullptr;
        return std::unexpected("failed to set HID device non-blocking mode: " + err_msg);
    }

    RCLCPP_INFO(logger, "HID device opened: %s", itf_path);
    return {};
}

void HIDDevice::close_hid() {
    std::lock_guard lock(hid_dev_mutex);

    if (hid_dev_handle) {
        hid_close(hid_dev_handle);
        hid_dev_handle = nullptr;
    }
}

std::expected<void, std::string>
HIDDevice::set_led_state(const uint8_t index, const uint8_t mode, const uint16_t pwm_out) {
    if (index >= NUMBER_OF_LEDS)
        return std::unexpected("LED index out of range");
    if (mode >= NUMBER_OF_LED_MODES)
        return std::unexpected("LED mode out of range");

    hid_led_report report;
    report.index = index;
    report.mode = mode;
    report.pwm_out = pwm_out;

    int res = 0;
    std::string err_msg;
    {
        std::lock_guard lock(hid_dev_mutex);
        if (!hid_dev_handle)
            return std::unexpected("HID device not open");
        res = hid_write(hid_dev_handle, reinterpret_cast<uint8_t*>(&report), sizeof(report));
        err_msg = get_last_error(hid_dev_handle);
    }

    if (res < 0)
        return std::unexpected("failed to send set LED HID report: " + err_msg);
    if (res != sizeof(report))
        return std::unexpected("incomplete write of set LED HID report");

    return {};
}

std::expected<hid_led_states_report, std::string> HIDDevice::get_led_states() {
    std::array<uint8_t, sizeof(hid_led_states_report)> buff{};
    buff[0] = LED_STATES_FEATURE_REPORT_ID;

    int res = 0;
    std::string err_msg;
    {
        std::lock_guard lock(hid_dev_mutex);
        if (!hid_dev_handle)
            return std::unexpected("HID device not open");
        res = hid_get_feature_report(hid_dev_handle, buff.data(), buff.size());
        err_msg = get_last_error(hid_dev_handle);
    }

    if (res < 0)
        return std::unexpected("failed to get LED states: " + err_msg);
    if (res < static_cast<int>(sizeof(hid_led_states_report)))
        return std::unexpected("incomplete read of LED states");

    return std::bit_cast<hid_led_states_report>(buff);
}
