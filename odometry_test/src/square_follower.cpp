#include <chrono>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
using namespace std::chrono_literals;
using geometry_msgs::msg::TwistStamped;
using nav_msgs::msg::Odometry;

class SquareFollower : public rclcpp::Node
{
public:
  SquareFollower()
      : Node("square_follower"), count_(0)
  {
    velocityPublisher_ = this->create_publisher<TwistStamped>("/diff_drive_controller/cmd_vel", 10);
    auto timer_callback =
        [this]() -> void
    {
      count_++;
      auto message = TwistStamped();
      message.header.stamp = this->now();

      if (count_ % 100 == 0)
      {
        turning_ = true;
      }
      if (turning_)
      {
        message.twist.angular.z = 0.1;
      }
      else
      {
        message.twist.linear.x = 0.1;
      }
      this->velocityPublisher_->publish(message);

      auto turnAngle = abs(startYaw_ - currentYaw_);
      RCLCPP_INFO(this->get_logger(), "turnAngle: '%f'", turnAngle);
      RCLCPP_INFO(this->get_logger(), "currentYaw_: '%f'", currentYaw_);
      if (turning_ && turnAngle >= 1.5708)
      {
        turning_ = false;
        startYaw_ = currentYaw_;
      }
    };

    auto topic_callback =
        [this](Odometry::UniquePtr odom) -> void
    {
      auto orientation = odom->pose.pose.orientation;
      tf2::Quaternion quat_tf;
      tf2::fromMsg(orientation, quat_tf);
      double r{}, p{}, y{};
      tf2::Matrix3x3 m(quat_tf);
      m.getRPY(r, p, y);
      if (!yawInitialized_)
      {
        startYaw_ = y;
        yawInitialized_ = true;
      }
      currentYaw_ = y;
      // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    };
    odometrySubscription_ =
        this->create_subscription<Odometry>("/diff_drive_controller/odom", 10, topic_callback);

    timer_ = this->create_wall_timer(100ms, timer_callback);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<TwistStamped>::SharedPtr velocityPublisher_;
  rclcpp::Subscription<Odometry>::SharedPtr odometrySubscription_;
  double startYaw_;
  double currentYaw_;
  bool yawInitialized_;
  bool turning_;
  size_t count_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SquareFollower>());
  rclcpp::shutdown();
  return 0;
}