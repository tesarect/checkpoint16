#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using namespace std::chrono_literals;

class WheelVelocitiesPublisher : public rclcpp::Node {
public:
  WheelVelocitiesPublisher() : Node("wheel_velocities_publisher_node") {
    publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "/wheel_speed", 10);

    timer_ = this->create_wall_timer(
        100ms, std::bind(&WheelVelocitiesPublisher::start_motion_sequence, this));
    
    RCLCPP_INFO(this->get_logger(), "Initialized wheel velocities publisher node");
  }

private:
  void start_motion_sequence() {
    // Moving Forward
    RCLCPP_INFO(this->get_logger(), "Move forward");
    set_wheel_speeds(1.0, 1.0, 1.0, 1.0);

    // Moving Backward
    RCLCPP_INFO(this->get_logger(), "Move backward");
    set_wheel_speeds(-1.0, -1.0, -1.0, -1.0);

    // Moving Left side
    RCLCPP_INFO(this->get_logger(), "Move sideways-left");
    set_wheel_speeds(-1.0, 1.0, -1.0, 1.0);

    // Moving Right side
    RCLCPP_INFO(this->get_logger(), "Move sideways-right");
    set_wheel_speeds(1.0, -1.0, 1.0, -1.0);

    // Moving Turn Clockwise
    RCLCPP_INFO(this->get_logger(), "Move clockwise");
    set_wheel_speeds(1.0, -1.0, -1.0, 1.0);

    // Moving Turn Counter Clock wise
    RCLCPP_INFO(this->get_logger(), "Move counter-clockwise");
    set_wheel_speeds(-1.0, 1.0, 1.0, -1.0);
  }

  void set_wheel_speeds(float fl, float fr, float rl, float rr) {
    // {front_left, front_right, rear_left, rear_right}
    msg_.data = {fl, fr, rl, rr};
    publisher_->publish(msg_);
  }

  std_msgs::msg::Float32MultiArray msg_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelVelocitiesPublisher>());
  rclcpp::shutdown();
  return 0;
}
