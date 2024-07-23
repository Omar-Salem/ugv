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

#ifndef UGV_CHASSIS_FIRMWARE__UGV_CHASSIS_HARDWARE_HPP_
#define UGV_CHASSIS_FIRMWARE__UGV_CHASSIS_HARDWARE_HPP_

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "ugv_interfaces/msg/motors_odom.hpp"
#include "Wheel.h"

using namespace std;
using namespace rclcpp;
using namespace rclcpp_lifecycle;
using namespace hardware_interface;
namespace ugv_chassis_firmware
{
    class UGVChassisHardware : public SystemInterface
    {
    public:
        UGVChassisHardware();

        CallbackReturn on_init(
            const HardwareInfo &info) override;

        CallbackReturn on_configure(
            const State &previous_state) override;

        vector<StateInterface> export_state_interfaces() override;

        vector<CommandInterface> export_command_interfaces() override;

        CallbackReturn on_activate(
            const State &previous_state) override;

        CallbackReturn on_deactivate(
            const State &previous_state) override;

        return_type read(
            const Time &time, const Duration &period) override;

        return_type write(
            const Time &time, const Duration &period) override;

    private:
        shared_ptr<Wheel> frontLeftWheel;
        shared_ptr<Wheel> frontRightWheel;
        shared_ptr<Wheel> rearLeftWheel;
        shared_ptr<Wheel> rearRightWheel;
        vector<shared_ptr<Wheel>> motors;

        shared_ptr<Node> node_;
        Subscription<ugv_interfaces::msg::MotorsOdom>::SharedPtr odomSubscription;
        Publisher<ugv_interfaces::msg::MotorsOdom>::SharedPtr velocityPublisher;

        void setMotorsVelocity(double frontLeft, double frontRight, double rearLeft, double rearRight);

        void readOdom(const ugv_interfaces::msg::MotorsOdom::SharedPtr motorsOdom);
    };

} // namespace ugv_chassis_firmware

#endif // UGV_CHASSIS_FIRMWARE__UGV_CHASSIS_HARDWARE_HPP_
