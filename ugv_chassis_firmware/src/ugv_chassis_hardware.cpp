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

namespace ugv_chassis_firmware {
    UGVChassisHardware::UGVChassisHardware() : node_(std::make_shared<rclcpp::Node>("ugv_motors_hw_interface_node")) {
        odomSubscription = node_->create_subscription<ugv_interfaces::msg::MotorsOdom>(
                "ugv/motors_state", 10,
                [this](const ugv_interfaces::msg::MotorsOdom::SharedPtr motorsOdom) {
                    this->readOdom(motorsOdom);
                });
        velocityPublisher = node_->create_publisher<ugv_interfaces::msg::MotorsOdom>("ugv/motors_cmd",
                                                                                     10);
    }

    CallbackReturn UGVChassisHardware::on_init(
            const HardwareInfo &info) {
        RCLCPP_INFO(get_logger("UGVChassisHardware"), "on_init ...please wait...");

        if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
            return CallbackReturn::ERROR;
        }
        frontLeftWheel = make_unique<Wheel>("front_left_wheel_joint");
        frontRightWheel = make_unique<Wheel>("front_right_wheel_joint");
        rearLeftWheel = make_unique<Wheel>("rear_left_wheel_joint");
        rearRightWheel = make_unique<Wheel>("rear_right_wheel_joint");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn UGVChassisHardware::on_configure(
            const State & /*previous_state*/) {
        RCLCPP_INFO(get_logger("UGVChassisHardware"), "on_configure ...please wait...");
        return CallbackReturn::SUCCESS;
    }

    vector <StateInterface> UGVChassisHardware::export_state_interfaces() {
        RCLCPP_INFO(get_logger("UGVChassisHardware"), "export_state_interfaces ...please wait...");
        vector <StateInterface> state_interfaces;

//        frontLeftWheel
        state_interfaces.emplace_back(
                frontLeftWheel->name, HW_IF_POSITION, &frontLeftWheel->position_state);

//        state_interfaces.emplace_back(
//                frontLeftWheel->name, HW_IF_VELOCITY, &frontLeftWheel->velocity_state);

//        frontRightWheel
        state_interfaces.emplace_back(
                frontRightWheel->name, HW_IF_POSITION, &frontRightWheel->position_state);

//        state_interfaces.emplace_back(
//                frontRightWheel->name, HW_IF_VELOCITY, &frontRightWheel->velocity_state);

//        rearLeftWheel
        state_interfaces.emplace_back(
                rearLeftWheel->name, HW_IF_POSITION, &rearLeftWheel->position_state);

//        state_interfaces.emplace_back(
//                rearLeftWheel->name, HW_IF_VELOCITY, &rearLeftWheel->velocity_state);

//        rearRightWheel
        state_interfaces.emplace_back(
                rearRightWheel->name, HW_IF_POSITION, &rearRightWheel->position_state);

//        state_interfaces.emplace_back(
//                rearRightWheel->name, HW_IF_VELOCITY, &rearRightWheel->velocity_state);

        return state_interfaces;
    }

    vector <CommandInterface> UGVChassisHardware::export_command_interfaces() {
        RCLCPP_INFO(get_logger("UGVChassisHardware"), "export_command_interfaces ...please wait...");
        vector <CommandInterface> command_interfaces;
        command_interfaces.emplace_back(
                frontLeftWheel->name, HW_IF_VELOCITY, &frontLeftWheel->velocity_command);
        command_interfaces.emplace_back(
                frontRightWheel->name, HW_IF_VELOCITY, &frontRightWheel->velocity_command);
        command_interfaces.emplace_back(
                rearLeftWheel->name, HW_IF_VELOCITY, &rearLeftWheel->velocity_command);
        command_interfaces.emplace_back(
                rearRightWheel->name, HW_IF_VELOCITY, &rearRightWheel->velocity_command);

        return command_interfaces;
    }

    CallbackReturn UGVChassisHardware::on_activate(
            const State & /*previous_state*/) {
        RCLCPP_INFO(get_logger("UGVChassisHardware"), "on_activate ...please wait...");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn UGVChassisHardware::on_deactivate(
            const State & /*previous_state*/) {
        RCLCPP_INFO(get_logger("UGVChassisHardware"), "on_deactivate ...please wait...");
        setMotorsVelocity(0, 0, 0, 0);
        return CallbackReturn::SUCCESS;
    }

    return_type UGVChassisHardware::read(
            const Time & /*time*/, const Duration & /*period*/) {
        rclcpp::spin_some(node_);
        return return_type::OK;
    }

    return_type UGVChassisHardware::write(
            const Time & /*time*/, const Duration & /*period*/) {

//        RCLCPP_INFO(get_logger("UGVChassisHardware"), "frontLeftWheel->velocity_command %f",frontLeftWheel->velocity_command);
//        RCLCPP_INFO(get_logger("UGVChassisHardware"), "frontRightWheel->velocity_command %f",frontRightWheel->velocity_command);
//        RCLCPP_INFO(get_logger("UGVChassisHardware"), "rearLeftWheel->velocity_command %f",rearLeftWheel->velocity_command);
//        RCLCPP_INFO(get_logger("UGVChassisHardware"), "rearRightWheel->velocity_command %f",rearRightWheel->velocity_command);

        setMotorsVelocity(frontLeftWheel->velocity_command,
                          frontRightWheel->velocity_command,
                          rearLeftWheel->velocity_command,
                          rearRightWheel->velocity_command);
        return return_type::OK;
    }

    void UGVChassisHardware::setMotorsVelocity(double frontLeft,
                                               double frontRight,
                                               double rearLeft,
                                               double rearRight) {
        auto cmd_msg = std::make_shared<ugv_interfaces::msg::MotorsOdom>();
        cmd_msg->front_left.velocity = frontLeft;
        cmd_msg->front_right.velocity = frontRight;
        cmd_msg->rear_left.velocity = rearLeft;
        cmd_msg->rear_right.velocity = rearRight;
        velocityPublisher->publish(*cmd_msg);
    }

    void UGVChassisHardware::readOdom(const ugv_interfaces::msg::MotorsOdom::SharedPtr motorsOdom) {
        frontLeftWheel->position_state = motorsOdom->front_left.position;
//        frontLeftWheel->velocity_state = motorsOdom->front_left.velocity;

        frontRightWheel->position_state = motorsOdom->front_right.position;
//        frontRightWheel->velocity_state = motorsOdom->front_right.velocity;

        rearLeftWheel->position_state = motorsOdom->rear_left.position;
//        rearLeftWheel->velocity_state = motorsOdom->rear_left.velocity;

        rearRightWheel->position_state = motorsOdom->rear_right.position;
//        rearRightWheel->velocity_state = motorsOdom->rear_right.velocity;
    }

}  // namespace ugv_chassis_firmware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
        ugv_chassis_firmware::UGVChassisHardware, hardware_interface::SystemInterface
)
