// Copyright (c) 2024, omar.salem
// Copyright (c) 2024, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <limits>
#include <vector>

#include "ugv_chassis_firmware/ugv_chassis_hardware.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ugv_chassis_firmware
{
        UGVChassisHardware::UGVChassisHardware() : node_(std::make_shared<rclcpp::Node>("ugv_motors_hw_interface_node"))
        {
                odomSubscription = node_->create_subscription<ugv_interfaces::msg::MotorsOdom>(
                    "ugv/motors_state", 10,
                    [this](const ugv_interfaces::msg::MotorsOdom::SharedPtr motorsOdom)
                    {
                            this->readOdom(motorsOdom);
                    });
                velocityPublisher = node_->create_publisher<ugv_interfaces::msg::MotorsOdom>("ugv/motors_cmd",
                                                                                             10);
        }

        CallbackReturn UGVChassisHardware::on_init(
            const HardwareInfo &info)
        {
                RCLCPP_INFO(get_logger("UGVChassisHardware"), "on_init ...please wait...");

                if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
                {
                        return CallbackReturn::ERROR;
                }
                leftWheel = make_unique<Wheel>("front_left_wheel_joint");
                rightWheel = make_unique<Wheel>("front_right_wheel_joint");
                return CallbackReturn::SUCCESS;
        }

        CallbackReturn UGVChassisHardware::on_configure(
            const State & /*previous_state*/)
        {
                RCLCPP_INFO(get_logger("UGVChassisHardware"), "on_configure ...please wait...");
                return CallbackReturn::SUCCESS;
        }

        vector<StateInterface> UGVChassisHardware::export_state_interfaces()
        {
                RCLCPP_INFO(get_logger("UGVChassisHardware"), "export_state_interfaces ...please wait...");
                vector<StateInterface> state_interfaces;

                state_interfaces.emplace_back(
                    leftWheel->name, HW_IF_POSITION, &leftWheel->position_state);

                state_interfaces.emplace_back(
                    rightWheel->name, HW_IF_POSITION, &rightWheel->position_state);

                return state_interfaces;
        }

        vector<CommandInterface> UGVChassisHardware::export_command_interfaces()
        {
                RCLCPP_INFO(get_logger("UGVChassisHardware"), "export_command_interfaces ...please wait...");
                vector<CommandInterface> command_interfaces;
                command_interfaces.emplace_back(
                    leftWheel->name, HW_IF_VELOCITY, &leftWheel->velocity_command);
                command_interfaces.emplace_back(
                    rightWheel->name, HW_IF_VELOCITY, &rightWheel->velocity_command);

                return command_interfaces;
        }

        CallbackReturn UGVChassisHardware::on_activate(
            const State & /*previous_state*/)
        {
                RCLCPP_INFO(get_logger("UGVChassisHardware"), "on_activate ...please wait...");
                return CallbackReturn::SUCCESS;
        }

        CallbackReturn UGVChassisHardware::on_deactivate(
            const State & /*previous_state*/)
        {
                RCLCPP_INFO(get_logger("UGVChassisHardware"), "on_deactivate ...please wait...");
                setMotorsVelocity(0, 0);
                return CallbackReturn::SUCCESS;
        }

        return_type UGVChassisHardware::read(
            const Time & /*time*/, const Duration & /*period*/)
        {
                rclcpp::spin_some(node_);
                return return_type::OK;
        }

        return_type UGVChassisHardware::write(
            const Time & /*time*/, const Duration & /*period*/)
        {

                //        RCLCPP_INFO(get_logger("UGVChassisHardware"), "frontLeftWheel->velocity_command %f",frontLeftWheel->velocity_command);
                //        RCLCPP_INFO(get_logger("UGVChassisHardware"), "frontRightWheel->velocity_command %f",frontRightWheel->velocity_command);
                //        RCLCPP_INFO(get_logger("UGVChassisHardware"), "rearLeftWheel->velocity_command %f",rearLeftWheel->velocity_command);
                //        RCLCPP_INFO(get_logger("UGVChassisHardware"), "rearRightWheel->velocity_command %f",rearRightWheel->velocity_command);

                setMotorsVelocity(leftWheel->velocity_command,
                                  rightWheel->velocity_command);
                return return_type::OK;
        }

        void UGVChassisHardware::setMotorsVelocity(double left,
                                                   double right)
        {
                auto cmd_msg = std::make_shared<ugv_interfaces::msg::MotorsOdom>();
                cmd_msg->left = left;
                cmd_msg->right = right;
                velocityPublisher->publish(*cmd_msg);
        }

        void UGVChassisHardware::readOdom(const ugv_interfaces::msg::MotorsOdom::SharedPtr motorsOdom)
        {
                leftWheel->position_state = motorsOdom->left;
                rightWheel->position_state = motorsOdom->right;
        }

} // namespace ugv_chassis_firmware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    ugv_chassis_firmware::UGVChassisHardware, hardware_interface::SystemInterface)
