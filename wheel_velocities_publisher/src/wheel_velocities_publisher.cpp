#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using namespace std::chrono_literals;

class WheelVelocitiesPublisher : public rclcpp::Node {
public:
  WheelVelocitiesPublisher() : Node("wheel_velocities_publisher_node") {
    publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "/wheel_speed", 10);

    timer_ = this->create_wall_timer(
        100ms, std::bind(&WheelVelocitiesPublisher::publish_message, this));
  }

private:
  void publish_message() {
    // {front_left, front_right, rear_left, rear_right}
    msg.data = {1.0f, 1.0f, 1.0f, 1.0f};

    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published wheel velocities");
  }

  std_msgs::msg::Float32MultiArray msg;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelVelocitiesPublisher>());
  rclcpp::shutdown();
  return 0;
}
