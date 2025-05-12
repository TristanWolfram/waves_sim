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
    this->declare_parameter<double>("max_forward_thrust", 150.0);
    this->declare_parameter<double>("max_steering_thrust", 75.0);

    this->get_parameter("throttle_axis", throttle_axis_);
    this->get_parameter("steering_axis", steering_axis_);
    this->get_parameter("max_forward_thrust", max_thrust_);
    this->get_parameter("max_steering_thrust", max_steering_);

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
    double throttle_in = 0.0;
    double steering_in = 0.0;

    throttle_in = joy_msg->axes[throttle_axis_];
    steering_in = joy_msg->axes[steering_axis_];

    double throttle = throttle_in * max_thrust_;
    double steering = steering_in * max_steering_;

    double left  = throttle + steering;
    double right = throttle - steering;

    double max_possible = max_thrust_ + max_steering_;
    left  = std::clamp(left,  -max_possible, max_possible);
    right = std::clamp(right, -max_possible, max_possible);

    auto out = std::make_unique<std_msgs::msg::Float64MultiArray>();
    out->data = {left, right}; 
    //RCLCPP_INFO(this->get_logger(), "Publishing: left: %f, right: %f", left, right);
    thruster_pub_->publish(std::move(out));
  }

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr thruster_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr         joy_sub_;

  int    throttle_axis_, steering_axis_;
  double max_thrust_;
  double max_steering_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ThrusterController>());
  rclcpp::shutdown();
  return 0;
}
