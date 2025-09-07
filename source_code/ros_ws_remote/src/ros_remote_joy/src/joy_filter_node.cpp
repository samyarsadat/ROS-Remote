/*
    The ROS remote project - Remote joystick filtering ROS package
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
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <ros_remote_hid/srv/set_led_state.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include "config.hpp"


#define DECLARE_PARAMETER(param_var_type, param_name, default_value, description_text)             \
    rcl_interfaces::msg::ParameterDescriptor param_descr_##param_name;                             \
    param_descr_##param_name.description = description_text;                                       \
    param_descr_##param_name.read_only = true;                                                     \
    param_name =                                                                                   \
        declare_parameter<param_var_type>(#param_name, default_value, param_descr_##param_name);


// Node class
class JoyFilterNode : public rclcpp::Node {
    public:
        explicit JoyFilterNode(const rclcpp::NodeOptions &options)
            : Node("joy_filter_node", options) {
            RCLCPP_INFO(get_logger(), "Joystick filter node initializing!");

            // Parameters
            DECLARE_PARAMETER(float, max_lin_vel_mult, 1.0f, "Maximum linear velocity multiplier");
            DECLARE_PARAMETER(float, min_lin_vel_mult, 0.0f, "Minimum linear velocity multiplier");
            DECLARE_PARAMETER(float, max_ang_vel_mult, 1.0f, "Maximum angular velocity multiplier");
            DECLARE_PARAMETER(float, min_ang_vel_mult, 0.0f, "Minimum angular velocity multiplier");
            DECLARE_PARAMETER(uint8_t,
                              joy_active_led_num,
                              DEFAULT_JOY_ACTIVE_LED_NUM,
                              "LED number for joystick active indication");
            DECLARE_PARAMETER(uint8_t,
                              joy_active_btn_num,
                              DEFAULT_JOY_ACTIVE_BTN_NUM,
                              "Joystick enable button number");
            DECLARE_PARAMETER(uint8_t,
                              joy_override_btn_num,
                              DEFAULT_JOY_OVERRIDE_BTN_NUM,
                              "Joystick control override button number");
            DECLARE_PARAMETER(uint8_t,
                              remote_lock_btn_num,
                              DEFAULT_REMOTE_LOCK_BTN_NUM,
                              "Key-switch lock button number");

            uint16_t joy_mux_lock_pub_rate;
            DECLARE_PARAMETER(uint16_t,
                              joy_mux_lock_pub_rate,
                              DEFAULT_JOY_MUX_LOCK_PUB_RATE,
                              "Default rate (in Hz) to publish joy_mux_lock status");
            DECLARE_PARAMETER(
                bool, is_cmd_vel_source, false, "Whether this node is the source of cmd_vel_joy");

            if (is_cmd_vel_source) {
                RCLCPP_INFO(get_logger(), "This node is the source of cmd_vel_joy.");
                DECLARE_PARAMETER(float,
                                  joy_lin_vel_scale,
                                  1.0f,
                                  "Linear velocity scale from joystick -> cmd_vel_joy");
                DECLARE_PARAMETER(float,
                                  joy_ang_vel_scale,
                                  1.0f,
                                  "Angular velocity scale from joystick -> cmd_vel_joy");
            }

            // Timers
            joy_mux_lock_pub_timer = create_wall_timer(
                std::chrono::milliseconds(1000 / joy_mux_lock_pub_rate), [this]() {
                    std_msgs::msg::Bool msg;
                    msg.data = override_active && joy_active && !remote_locked;
                    joy_mux_lock_pub->publish(msg);
                });

            // Subscriptions
            joy_sub = create_subscription<sensor_msgs::msg::Joy>(
                "joy", 10, std::bind(&JoyFilterNode::joy_callback, this, std::placeholders::_1));

            if (!is_cmd_vel_source) {
                multithread_cb_group = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
                rclcpp::SubscriptionOptions sub_options_multithread;
                sub_options_multithread.callback_group = multithread_cb_group;

                cmd_vel_sub = create_subscription<geometry_msgs::msg::Twist>(
                    "cmd_vel_joy",
                    10,
                    std::bind(&JoyFilterNode::cmd_vel_callback, this, std::placeholders::_1),
                    sub_options_multithread);
            }

            // Publishers
            const char* joy_cmd_topic = is_cmd_vel_source ? "cmd_vel_joy" : "cmd_vel_joy/filtered";
            cmd_vel_pub = create_publisher<geometry_msgs::msg::Twist>(joy_cmd_topic, 10);
            joy_mux_lock_pub = create_publisher<std_msgs::msg::Bool>("joy_mux_lock", 10);

            // Service clients
            set_led_state_client = create_client<ros_remote_hid::srv::SetLedState>("set_led_state");
        }

    private:
        // Configuration parameters
        uint8_t joy_active_led_num, joy_active_btn_num, remote_lock_btn_num, joy_override_btn_num;
        float max_lin_vel_mult, min_lin_vel_mult, max_ang_vel_mult, min_ang_vel_mult;

        bool is_cmd_vel_source;
        float joy_lin_vel_scale, joy_ang_vel_scale;

        bool remote_locked = true, override_active = false, joy_active = false;
        float potentiometer_value = 0.0f;
        struct led_state {
                LED_MODE mode;
                uint16_t pwm_value;
                auto operator<=>(const led_state &) const = default;
        } joy_active_led_state;
        bool joy_last_zeroed = false;

        // ROS interfaces
        rclcpp::TimerBase::SharedPtr joy_mux_lock_pub_timer;
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr joy_mux_lock_pub;
        rclcpp::Client<ros_remote_hid::srv::SetLedState>::SharedPtr set_led_state_client;

        rclcpp::CallbackGroup::SharedPtr multithread_cb_group;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;

        // Callbacks
        void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
            joy_active = msg->buttons.at(joy_active_btn_num);
            remote_locked = msg->buttons.at(remote_lock_btn_num);
            override_active = msg->buttons.at(joy_override_btn_num);
            potentiometer_value = msg->axes.at(POTENTIOMETER_AXIS);

            // Publish cmd_vel if we are the source
            if (is_cmd_vel_source) {
                geometry_msgs::msg::Twist joy_cmd{};

                if (joy_active && !remote_locked) {
                    const float lin_factor =
                        lin_interp(
                            potentiometer_value, -1.0f, 1.0f, min_lin_vel_mult, max_lin_vel_mult)
                        * joy_lin_vel_scale;
                    const float ang_factor =
                        lin_interp(
                            potentiometer_value, -1.0f, 1.0f, min_ang_vel_mult, max_ang_vel_mult)
                        * joy_ang_vel_scale;

                    joy_cmd.linear.x = lin_factor * msg->axes.at(JOYSTICK_X_AXIS);
                    joy_cmd.linear.y = lin_factor * msg->axes.at(JOYSTICK_Y_AXIS);
                    joy_cmd.angular.z = ang_factor * msg->axes.at(JOYSTICK_RZ_AXIS);

                    joy_last_zeroed = false;
                } else if (!joy_last_zeroed) {
                    joy_last_zeroed = true;
                } else {
                    goto proc_leds;
                }

                cmd_vel_pub->publish(joy_cmd);
            }

        proc_leds:
            // Joystick active LED indicator
            led_state new_led_state = {LED_SOLID_PWM, 0};
            if (joy_active) {
                new_led_state.pwm_value = 65535;

                if (remote_locked) {
                    new_led_state.mode = LED_SLOW_FLASH; // Active but locked
                } else {
                    new_led_state.mode = LED_SLOW_FADE; // Active and unlocked
                }
            }

            if (new_led_state != joy_active_led_state) {
                if (!set_led_state_client->service_is_ready()) {
                    RCLCPP_ERROR(get_logger(), "Set LED state service not available!");
                    return;
                }

                auto request = std::make_shared<ros_remote_hid::srv::SetLedState::Request>();
                request->index = joy_active_led_num;
                request->led_mode = new_led_state.mode;
                request->pwm_output = new_led_state.pwm_value;

                // We don't care about old requests as we're about to send a new one.
                (void) set_led_state_client->prune_pending_requests();
                (void) set_led_state_client->async_send_request(
                    request,
                    std::bind(
                        &JoyFilterNode::set_led_state_resp_handler, this, std::placeholders::_1));

                joy_active_led_state = new_led_state;
            }
        }

        void set_led_state_resp_handler(
            rclcpp::Client<ros_remote_hid::srv::SetLedState>::SharedFuture future) {
            auto response = future.get();
            if (!response->success) {
                RCLCPP_ERROR_STREAM(get_logger(), "Failed to set LED state: " << response->message);
            }
        }

        // Only when we are not the source of cmd_vel
        void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
            geometry_msgs::msg::Twist filtered_cmd = *msg;

            if (joy_active && !remote_locked) {
                // Only linear.x, linear.y, and angular.z are sent by the joystick.
                // No need to filter other channels.
                filtered_cmd.linear.x *= lin_interp(
                    potentiometer_value, -1.0f, 1.0f, min_lin_vel_mult, max_lin_vel_mult);
                filtered_cmd.linear.y *= lin_interp(
                    potentiometer_value, -1.0f, 1.0f, min_lin_vel_mult, max_lin_vel_mult);
                filtered_cmd.angular.z *= lin_interp(
                    potentiometer_value, -1.0f, 1.0f, min_ang_vel_mult, max_ang_vel_mult);

                joy_last_zeroed = false;
            } else if (!joy_last_zeroed) {
                filtered_cmd = geometry_msgs::msg::Twist{};
                joy_last_zeroed = true;
            } else {
                return;
            }

            cmd_vel_pub->publish(filtered_cmd);
        }

        // Linear interpolation utility
        inline float
        lin_interp(const float x, const float x0, const float x1, const float y0, const float y1) {
            return std::lerp(y0, y1, (x - x0) / (x1 - x0));
        }
};


// Register the node class (composable node)
RCLCPP_COMPONENTS_REGISTER_NODE(JoyFilterNode)
