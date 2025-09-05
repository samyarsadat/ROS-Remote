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

#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include "hid_helper.hpp"
#include "ros_remote_hid/srv/set_led_state.hpp"
#include "ros_remote_hid/srv/get_led_states.hpp"
#include <std_srvs/srv/trigger.hpp>
#include <rclcpp_components/register_node_macro.hpp>


// Node class
class LEDInterfaceNode : public rclcpp::Node {
    public:
        explicit LEDInterfaceNode(const rclcpp::NodeOptions &options)
            : Node("led_interface_node", options) {
            RCLCPP_INFO(get_logger(), "LED HID interface node initializing...");

            // Parameters
            rcl_interfaces::msg::ParameterDescriptor param_descr_vid;
            param_descr_vid.description = "Vendor ID of the HID device";
            param_descr_vid.read_only = true;
            vid = declare_parameter<uint16_t>("vid", DEFAULT_VID, param_descr_vid);

            rcl_interfaces::msg::ParameterDescriptor param_descr_pid;
            param_descr_pid.description = "Product ID of the HID device";
            param_descr_pid.read_only = true;
            pid = declare_parameter<uint16_t>("pid", DEFAULT_PID, param_descr_pid);

            rcl_interfaces::msg::ParameterDescriptor param_descr_itf_num;
            param_descr_itf_num.description = "Interface number of the HID device";
            param_descr_itf_num.read_only = true;
            itf_num = declare_parameter<uint8_t>("itf_num", ITF_NUM_LEDS_HID, param_descr_itf_num);

            // Open HID device
            auto res = hid_device.open_hid(vid, pid, itf_num);
            if (!res) {
                throw std::runtime_error("Failed to open HID device: " + res.error());
            }

            // Callback groups
            multithread_cb_group = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

            // Service servers
            set_led_state_service = create_service<ros_remote_hid::srv::SetLedState>(
                "~/set_led_state",
                std::bind(&LEDInterfaceNode::handle_set_led_state,
                          this,
                          std::placeholders::_1,
                          std::placeholders::_2),
                rclcpp::ServicesQoS(),
                multithread_cb_group);
            get_led_states_service = create_service<ros_remote_hid::srv::GetLedStates>(
                "~/get_led_states",
                std::bind(&LEDInterfaceNode::handle_get_led_states,
                          this,
                          std::placeholders::_1,
                          std::placeholders::_2),
                rclcpp::ServicesQoS(),
                multithread_cb_group);
            reopen_hid_device_service = create_service<std_srvs::srv::Trigger>(
                "~/reopen_hid_device",
                std::bind(&LEDInterfaceNode::handle_reopen_hid_device,
                          this,
                          std::placeholders::_1,
                          std::placeholders::_2));
        }

        ~LEDInterfaceNode() {
            hid_device.close_hid();
            RCLCPP_INFO(get_logger(), "LED HID interface node stopped.");
        }

    private:
        HIDDevice hid_device{get_logger()};

        uint16_t vid, pid;
        uint8_t itf_num;

        rclcpp::CallbackGroup::SharedPtr multithread_cb_group;
        rclcpp::Service<ros_remote_hid::srv::SetLedState>::SharedPtr set_led_state_service;
        rclcpp::Service<ros_remote_hid::srv::GetLedStates>::SharedPtr get_led_states_service;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reopen_hid_device_service;

        // Callbacks
        void handle_set_led_state(
            const std::shared_ptr<ros_remote_hid::srv::SetLedState::Request> request,
            const std::shared_ptr<ros_remote_hid::srv::SetLedState::Response> response) {
            auto res =
                hid_device.set_led_state(request->index, request->led_mode, request->pwm_output);
            if (!res) {
                response->success = false;
                response->message = res.error();
                RCLCPP_ERROR_STREAM(get_logger(), "Failed to set LED state: " << res.error());
                return;
            }

            RCLCPP_DEBUG_STREAM(get_logger(),
                                "Set LED " << static_cast<int>(request->index) << " to mode "
                                           << static_cast<int>(request->led_mode)
                                           << " with PWM output " << request->pwm_output);

            response->success = true;
        }

        void handle_get_led_states(
            const std::shared_ptr<ros_remote_hid::srv::GetLedStates::Request> request,
            const std::shared_ptr<ros_remote_hid::srv::GetLedStates::Response> response) {
            (void) request;

            auto res = hid_device.get_led_states();
            if (!res) {
                response->success = false;
                response->message = res.error();
                RCLCPP_ERROR_STREAM(get_logger(), "Failed to get LED states: " << res.error());
                return;
            }

            for (size_t i = 0; i < NUMBER_OF_LEDS; i++) {
                response->led_mode[i] = res->mode[i];
                response->pwm_output[i] = res->pwm_out[i];
            }

            response->success = true;
        }

        void
        handle_reopen_hid_device(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                 const std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
            (void) request;
            RCLCPP_WARN(get_logger(), "Reopening HID device...");

            hid_device.close_hid();
            auto res = hid_device.open_hid(vid, pid, itf_num);
            if (!res) {
                response->success = false;
                response->message = res.error();
                RCLCPP_ERROR_STREAM(get_logger(), "Failed to reopen HID device: " << res.error());
                return;
            }

            response->success = true;
        }
};


// Register the node class (composable node)
RCLCPP_COMPONENTS_REGISTER_NODE(LEDInterfaceNode)
