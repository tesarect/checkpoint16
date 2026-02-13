#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include <cmath>
#include <cstddef>
#include <vector>

using namespace std::chrono_literals;

class EightTrajectory : public rclcpp::Node {
public:
  EightTrajectory() : Node("eight_trajectory") {

    wheel_speed_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "/wheel_speed", 10);

    // Wait for subscriber to connect
    while (wheel_speed_pub_->get_subscription_count() == 0 && rclcpp::ok()) {
      RCLCPP_INFO(this->get_logger(), "Waiting for /wheel_speed subscriber");
      rclcpp::sleep_for(std::chrono::milliseconds(200));
    }

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", 10,
        std::bind(&EightTrajectory::odom_callback, this,
                  std::placeholders::_1));

    timer_ = this->create_wall_timer(
        50ms, std::bind(&EightTrajectory::control_loop, this));

    r_ = 0.100 / 2.0; // wheel radius
    l_ = 0.170 / 2.0; // half wheel base
    w_ = 0.269 / 2.0; // half track width

    // Initialize waypoints [dphi, dx, dy]
    waypoints_ = {
        {0.0, 1.0, -1.0},      // w1
        {0.0, 1.0, 1.0},       // w2
        {0.0, 1.0, 1.0},       // w3
        {-1.5708, 1.0, -1.0},  // w4 (-90 degrees)
        {-1.5708, -1.0, -1.0}, // w5 (-90 degrees)
        {0.0, -1.0, 1.0},      // w6
        {0.0, -1.0, 1.0},      // w7
        {0.0, -1.0, -1.0}      // w8
    };

    // Current & Target position
    current_x_ = 0.0;
    current_y_ = 0.0;
    current_yaw_ = 0.0;
    target_x_ = 0.0;
    target_y_ = 0.0;
    target_yaw_ = 0.0;

    update_target_from_waypoint(0); // updating the first waypoint position

    odom_received_ = false;

    RCLCPP_INFO(this->get_logger(), "Initialized eight_trajectory node");
    RCLCPP_INFO(this->get_logger(),
                "Moving to waypoint %zu: target=[%.2f, %.2f, %.2f rad]",
                waypoint_index_ + 1, target_x_, target_y_, target_yaw_);
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Current x, y
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;

    // Current yaw
    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    current_yaw_ = yaw;

    odom_received_ = true;
  }

  void control_loop() {
    // Odometry availability
    if (!odom_received_) {
      return;
    }

    // Error calculation
    float error_x = target_x_ - current_x_;
    float error_y = target_y_ - current_y_;
    float error_yaw = normalize_angle(target_yaw_ - current_yaw_);

    // Check if waypoint reached
    float position_error = std::sqrt(error_x * error_x + error_y * error_y);
    bool waypoint_reached =
        (position_error < 0.05) && (std::abs(error_yaw) < 0.1);

    if (waypoint_reached) {
      rclcpp::sleep_for(2000ms);
      waypoint_index_++;

      // Check if all waypoints are completed
      if (waypoint_index_ >= waypoints_.size()) {
        RCLCPP_INFO(this->get_logger(),
                    "All waypoints reached! Stopping robot.");
        stop_robot();
        timer_->cancel();
        return;
      }

      // Move to next waypoint
      update_target_from_waypoint(waypoint_index_);
      RCLCPP_INFO(this->get_logger(),
                  "Moving to waypoint %zu: target=[%.2f, %.2f, %.2f rad]",
                  waypoint_index_ + 1, target_x_, target_y_, target_yaw_);
    }

    // Error correction
    float K = 1.5; // proportional gain
    float dphi = K * error_yaw;
    float dx = K * error_x;
    float dy = K * error_y;

    // Transform velocities from global frame to robot frame
    auto twist = global_to_robot_frame(dphi, dx, dy);

    // Convert twist to wheel speeds
    auto wheel_speeds = twist_to_wheel_speeds(twist[0], twist[1], twist[2]);

    // Publish wheel speeds
    auto msg = std_msgs::msg::Float32MultiArray();
    msg.data = wheel_speeds;
    wheel_speed_pub_->publish(msg);
  }

  void update_target_from_waypoint(int index) {
    // Updating to next waypoint
    target_yaw_ += waypoints_[index][0];
    target_x_ += waypoints_[index][1];
    target_y_ += waypoints_[index][2];

    target_yaw_ = normalize_angle(target_yaw_);
  }

  double normalize_angle(double angle) {
    // Normalize angle to [-pi, pi]
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  std::vector<float> global_to_robot_frame(float dphi, float dx, float dy) {
    // Rotation matrix to transform from global frame to robot frame
    // [wz]   [1    0          0       ] [dphi]
    // [vx] = [0  cos(yaw)  sin(yaw)  ] [dx  ]
    // [vy]   [0 -sin(yaw)  cos(yaw)  ] [dy  ]

    float wz = dphi;
    float vx = dx * std::cos(current_yaw_) + dy * std::sin(current_yaw_);
    float vy = -dx * std::sin(current_yaw_) + dy * std::cos(current_yaw_);

    return {wz, vx, vy};
  }

  std::vector<float> twist_to_wheel_speeds(float wz, float vx, float vy) {
    // Forward kinematics matrix H
    // For mecanum wheels:
    // [w_fl]   [-(l+w)   1   -1] [wz]
    // [w_fr] = [ (l+w)   1    1] [vx] * (1/r)
    // [w_rl]   [ (l+w)   1   -1] [vy]
    // [w_rr]   [-(l+w)   1    1]

    float lw = l_ + w_;

    float w_fl = (1.0 / r_) * (-lw * wz + vx - vy);
    float w_fr = (1.0 / r_) * (lw * wz + vx + vy);
    float w_rl = (1.0 / r_) * (lw * wz + vx - vy);
    float w_rr = (1.0 / r_) * (-lw * wz + vx + vy);

    return {w_fl, w_fr, w_rl, w_rr};
  }

  void stop_robot() {
    auto msg = std_msgs::msg::Float32MultiArray();
    msg.data = {0.0, 0.0, 0.0, 0.0};
    wheel_speed_pub_->publish(msg);
  }

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr
      wheel_speed_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<std::vector<double>> waypoints_;
  size_t waypoint_index_;
  double r_, l_, w_;
  double current_x_, current_y_, current_yaw_;
  double target_x_, target_y_, target_yaw_;
  bool odom_received_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EightTrajectory>());
  rclcpp::shutdown();
  return 0;
}