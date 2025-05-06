#include <memory>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class ThrusterController : public rclcpp::Node
{
public:
  ThrusterController()
  : Node("thrust_allocator")
  {
    // Declare & get parameters (remappable via CLI or launch)
    this->declare_parameter<int>("throttle_axis", 1);
    this->declare_parameter<int>("steering_axis", 3);
    this->declare_parameter<double>("max_thrust", 100.0);

    this->get_parameter("throttle_axis", throttle_axis_);
    this->get_parameter("steering_axis", steering_axis_);
    this->get_parameter("max_thrust",    max_thrust_);

    // Publisher: two-element array [left_force, right_force]
    thruster_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/blueboat/thruster_forces", 10);

    // Subscriber: /joy input
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10,
      std::bind(&ThrusterController::joy_callback, this, std::placeholders::_1));
  }

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
  {
    double throttle = 0.0;
    double steering = 0.0;

    throttle = joy_msg->axes[throttle_axis_];
    steering = joy_msg->axes[steering_axis_];

    double left  = throttle + steering;
    double right = throttle - steering;

    left  = std::clamp(left,  -1.0, 1.0);
    right = std::clamp(right, -1.0, 1.0);

    left  *= max_thrust_;
    right *= max_thrust_;

    auto out = std::make_unique<std_msgs::msg::Float64MultiArray>();
    out->data = {left, right}; 
    //RCLCPP_INFO(this->get_logger(), "Publishing: left: %f, right: %f", left, right);
    thruster_pub_->publish(std::move(out));
  }

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr thruster_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr         joy_sub_;

  int    throttle_axis_, steering_axis_;
  double max_thrust_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ThrusterController>());
  rclcpp::shutdown();
  return 0;
}
