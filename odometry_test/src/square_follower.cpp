#include <chrono>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

using namespace std::chrono_literals;
using geometry_msgs::msg::TwistStamped;

class SquareFollower : public rclcpp::Node
{
public:
  SquareFollower()
      : Node("square_follower"), count_(0)
  {
    velocity_publisher_ = this->create_publisher<TwistStamped>("/diff_drive_controller/cmd_vel", 10);
    auto timer_callback =
        [this]() -> void
    {
      count_++;
      auto message = TwistStamped();
      message.header.stamp = this->now();
      message.twist.linear.x = 0.1;
      // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      this->velocity_publisher_->publish(message);
    };
    timer_ = this->create_wall_timer(100ms, timer_callback);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<TwistStamped>::SharedPtr velocity_publisher_;
  size_t count_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SquareFollower>());
  rclcpp::shutdown();
  return 0;
}
