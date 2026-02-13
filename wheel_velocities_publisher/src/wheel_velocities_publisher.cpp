#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <chrono>

using namespace std::chrono_literals;

class WheelVelocitiesPublisher : public rclcpp::Node {
public:
  WheelVelocitiesPublisher() : Node("wheel_velocities_publisher") {
    publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "/wheel_speed", 10);

    // Wait for subscriber to connect
    while (publisher_->get_subscription_count() == 0 && rclcpp::ok()) {
      RCLCPP_INFO(this->get_logger(), "Waiting for /wheel_speed subscriber");
      rclcpp::sleep_for(std::chrono::milliseconds(200));
    }

    RCLCPP_INFO(this->get_logger(),
                "Initialized wheel velocities publisher node");

    timer_ = this->create_wall_timer(
        20ms, std::bind(&WheelVelocitiesPublisher::timer_callback, this));

    motion_index_ = 0;
    // Record start time
    motion_start_time_ = this->now();
    set_current_motion(0);
  }

private:
  void timer_callback() {
    // Check if current motion is running for 3 seconds
    auto elapsed = (this->now() - motion_start_time_).seconds();

    if (elapsed >= 3.0) {
      rclcpp::sleep_for(2000ms);
      motion_index_++;

      // End of motion sequence
      if (motion_index_ >= 7) {
        timer_->cancel();
        return;
      }

      // Reset timer and set next motion
      motion_start_time_ = this->now();
      set_current_motion(motion_index_);
    }

    // Publish current wheel speeds
    publisher_->publish(current_msg_);
  }

  void set_current_motion(int index) {
    switch (index) {
    case 0:
      RCLCPP_INFO(this->get_logger(), "Move forward");
      current_msg_.data = {2.0, 2.0, 2.0, 2.0};
      break;
    case 1:
      RCLCPP_INFO(this->get_logger(), "Move backward");
      current_msg_.data = {-2.0, -2.0, -2.0, -2.0};
      break;
    case 2:
      RCLCPP_INFO(this->get_logger(), "Move left");
      current_msg_.data = {-2.0, 2.0, -2.0, 2.0};
      break;
    case 3:
      RCLCPP_INFO(this->get_logger(), "Move right");
      current_msg_.data = {2.0, -2.0, 2.0, -2.0};
      break;
    case 4:
      RCLCPP_INFO(this->get_logger(), "Turn clockwise");
      current_msg_.data = {2.0, -2.0, -2.0, 2.0};
      break;
    case 5:
      RCLCPP_INFO(this->get_logger(), "Turn counter-clockwise");
      current_msg_.data = {-2.0, 2.0, 2.0, -2.0};
      break;
    case 6:
      RCLCPP_INFO(this->get_logger(), "Stop");
      current_msg_.data = {0.0, 0.0, 0.0, 0.0};
      break;
    }
  }

  std_msgs::msg::Float32MultiArray current_msg_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time motion_start_time_;
  int motion_index_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelVelocitiesPublisher>());
  rclcpp::shutdown();
  return 0;
}