#include <chrono>
#include <memory>
#include <string>
#include <math.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
using namespace std::chrono_literals;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::TwistStamped;
using nav_msgs::msg::Odometry;

class SquareFollower : public rclcpp::Node
{
public:
  SquareFollower()
      : Node("square_follower")
  {
    velocityPublisher_ = this->create_publisher<TwistStamped>("/diff_drive_controller/cmd_vel", 10);
    auto timer_callback =
        [this]() -> void
    {
      auto message = TwistStamped();
      message.header.stamp = this->now();
      auto remainingDistance = calculateDistance(startPosition_, currentPosition_);
      RCLCPP_INFO(this->get_logger(), "remainingDistance: '%f'", remainingDistance);
      if (remainingDistance < 1)
      {
        message.twist.linear.x = 0.1;
      }
      else
      {
        message.twist.angular.z = 0.1;
        auto turnAngle = abs(startYaw_ - currentYaw_);
        if (turnAngle >= 1.5708)
        {
          startYaw_ = currentYaw_;
          startPosition_ = currentPosition_;
        }
      }
      this->velocityPublisher_->publish(message);
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
        yawInitialized_ = true;
        startYaw_ = y;
        startPosition_ = odom->pose.pose.position;
      }
      currentYaw_ = y;
      currentPosition_ = odom->pose.pose.position;
      // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    };
    odometrySubscription_ =
        this->create_subscription<Odometry>("/diff_drive_controller/odom", 10, topic_callback);

    timer_ = this->create_wall_timer(50ms, timer_callback);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<TwistStamped>::SharedPtr velocityPublisher_;
  rclcpp::Subscription<Odometry>::SharedPtr odometrySubscription_;

  double startYaw_;
  Point startPosition_;

  double currentYaw_;
  Point currentPosition_;

  bool yawInitialized_;
  bool turning_;

  double calculateDistance(Point p1, Point p2)
  {
    double x = p1.x - p2.x;
    double y = p1.y - p2.y;

    auto dist = pow(x, 2) + pow(y, 2);
    return sqrt(dist);
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SquareFollower>());
  rclcpp::shutdown();
  return 0;
}
