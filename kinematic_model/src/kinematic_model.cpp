#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "std_msgs/msg/detail/float32_multi_array__struct.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using std::placeholders::_1;

using namespace std::chrono_literals;

class KinematicModel : public rclcpp::Node {
public:
  KinematicModel() : Node("kinematic_model_node") {
    subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "wheel_speed", 10,
        std::bind(&KinematicModel::wheel_speed_callback, this, _1));

    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    RCLCPP_INFO(this->get_logger(), "Kinematic_model initiated\nSubscribed to "
                                    "`/wheel_speed`\nPublishing to `/cmd_vel`");

    r_ = 0.100 / 2.0;
    l_ = 0.170 / 2.0;
    w_ = 0.269 / 2.0;
  }

private:
  void
  wheel_speed_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    // Check if we have 4 wheel speeds
    if (msg->data.size() != 4) {
      RCLCPP_WARN(this->get_logger(), "Expected 4 wheel speeds, got %zu",
                  msg->data.size());
      return;
    }

    // Extract wheel speeds [fl, fr, rl, rr]
    double w_fl = msg->data[0];
    double w_fr = msg->data[1];
    double w_rl = msg->data[2];
    double w_rr = msg->data[3];

    // Using the pseudo-inverse of the forward kinematics matrix
    double vx = r_ / 4.0 * (w_fl + w_fr + w_rl + w_rr);
    double vy = r_ / 4.0 * (-w_fl + w_fr - w_rl + w_rr);
    double wz = r_ / (4.0 * (l_ + w_)) * (-w_fl + w_fr + w_rl - w_rr);

    // Create and publish Twist message
    auto twist_msg = geometry_msgs::msg::Twist();
    twist_msg.linear.x = vx;
    twist_msg.linear.y = vy;
    twist_msg.linear.z = 0.0;
    twist_msg.angular.x = 0.0;
    twist_msg.angular.y = 0.0;
    twist_msg.angular.z = wz;

    publisher_->publish(twist_msg);

    RCLCPP_DEBUG(this->get_logger(),
                 "Published twist: vx=%.2f, vy=%.2f, wz=%.2f", vx, vy, wz);
  }


  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  double r_; // wheel radius
  double l_; // half wheel base
  double w_; // half track width
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KinematicModel>());
  rclcpp::shutdown();
  return 0;
}