#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <array>
#include <chrono>
#include <cmath>
#include <vector>

using namespace std::chrono_literals;

class SinglePointTrajectory : public rclcpp::Node {
public:
  SinglePointTrajectory()
      : Node("single_point_trajectory"), initial_pose_set_(false),
        reached_waypoint_(false), k_rho_(0.3), k_alpha_(0.8), k_beta_(-0.15),
        v_const_(0.3), current_waypoint_idx_(0) {
    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", 10,
        std::bind(&SinglePointTrajectory::odom_callback, this,
                  std::placeholders::_1));
    wheel_speed_publisher_ =
        this->create_publisher<std_msgs::msg::Float32MultiArray>("/wheel_speed",
                                                                 10);

    timer_ = this->create_wall_timer(
        100ms, std::bind(&SinglePointTrajectory::move_to_point, this));

    waypoints_ = {{0.0, 1, -1},    {0.0, 1, 1},       {0.0, 1, 1},
                  {1.5708, 1, -1}, {-3.1415, -1, -1}, {0.0, -1, 1},
                  {0.0, -1, 1},    {0.0, -1, -1}};
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (!initial_pose_set_) {
      initial_x_ = msg->pose.pose.position.x;
      initial_y_ = msg->pose.pose.position.y;
      initial_yaw_ = get_yaw_from_pose(msg->pose.pose);
      initial_pose_set_ = true;
    }

    current_pose_ = msg->pose.pose;
    reached_waypoint_ = check_if_reached_waypoint();
    if (reached_waypoint_ && current_waypoint_idx_ < waypoints_.size() - 1) {
      current_waypoint_idx_++;
      reached_waypoint_ = false; // Proceed to next waypoint
    }
  }

  bool check_if_reached_waypoint() {
    double current_x = current_pose_.position.x;
    double current_y = current_pose_.position.y;

    double global_target_x = initial_x_ + waypoints_[current_waypoint_idx_][1];
    double global_target_y = initial_y_ + waypoints_[current_waypoint_idx_][2];

    double distance = std::sqrt(std::pow(global_target_x - current_x, 2) +
                                std::pow(global_target_y - current_y, 2));

    return distance < 0.1;
  }

  void move_to_point() {
    if (current_waypoint_idx_ >= waypoints_.size()) {
      RCLCPP_INFO(this->get_logger(), "Reached all waypoints");
      return;
    }

    double global_target_x = initial_x_ + waypoints_[current_waypoint_idx_][1];
    double global_target_y = initial_y_ + waypoints_[current_waypoint_idx_][2];

    double current_x = current_pose_.position.x;
    double current_y = current_pose_.position.y;

    double error_x = global_target_x - current_x;
    double error_y = global_target_y - current_y;

    double current_yaw = get_yaw_from_pose(current_pose_);
    double target_yaw = initial_yaw_ + waypoints_[current_waypoint_idx_][0];

    // Calculate polar coordinates
    double rho = std::sqrt(std::pow(error_x, 2) + std::pow(error_y, 2));
    double alpha = normalize_angle(std::atan2(error_y, error_x) - current_yaw);
    double beta = normalize_angle(target_yaw - std::atan2(error_y, error_x));

    // Calculate control signals
    double v = k_rho_ * rho;
    double w = 0; // Default to no rotation

    if (std::abs(waypoints_[current_waypoint_idx_][0]) >
        0.1) { // Only rotate if significant rotation is needed
      w = k_alpha_ * alpha + k_beta_ * beta;
    }

    // Normalize velocity to maintain constant linear velocity
    double s = v_const_ / std::abs(v);
    v *= s;
    w *= s;

    // Since this is an omnidirectional robot, we can use vx and vy directly
    double vx = v * std::cos(current_yaw + alpha);
    double vy = v * std::sin(current_yaw + alpha);

    auto wheel_speeds = compute_wheel_speeds(vx, vy, w);

    auto message = std_msgs::msg::Float32MultiArray();
    message.layout.data_offset = 0;
    message.data = {static_cast<float>(wheel_speeds[0]),
                    static_cast<float>(wheel_speeds[1]),
                    static_cast<float>(wheel_speeds[2]),
                    static_cast<float>(wheel_speeds[3])};

    wheel_speed_publisher_->publish(message);
    RCLCPP_INFO(
        this->get_logger(),
        "Publishing wheel speeds - FL: %.2f, FR: %.2f, RL: %.2f, RR: %.2f",
        wheel_speeds[0], wheel_speeds[1], wheel_speeds[2], wheel_speeds[3]);
  }

  std::array<double, 4> compute_wheel_speeds(double vx, double vy,
                                             double omega) {
    double r = 0.1;
    double L = 0.17;
    double w = 0.135;

    std::array<double, 4> wheel_speeds;
    wheel_speeds[0] = vx - vy - (L + w) * omega;
    wheel_speeds[1] = vx + vy + (L + w) * omega;
    wheel_speeds[2] = vx + vy - (L + w) * omega;
    wheel_speeds[3] = vx - vy + (L + w) * omega;

    for (auto &speed : wheel_speeds) {
      speed /= r;
    }

    RCLCPP_INFO(this->get_logger(),
                "Wheel speeds - FL: %.2f, FR: %.2f, RL: %.2f, RR: %.2f",
                wheel_speeds[0], wheel_speeds[1], wheel_speeds[2],
                wheel_speeds[3]);

    return wheel_speeds;
  }

  double get_yaw_from_pose(const geometry_msgs::msg::Pose &pose) {
    tf2::Quaternion q(pose.orientation.x, pose.orientation.y,
                      pose.orientation.z, pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
  }

  double normalize_angle(double angle) {
    while (angle > M_PI)
      angle -= 2.0 * M_PI;
    while (angle < -M_PI)
      angle += 2.0 * M_PI;
    return angle;
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr
      wheel_speed_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::Pose current_pose_;
  double initial_x_ = 0.0;
  double initial_y_ = 0.0;
  double initial_yaw_ = 0.0;
  bool initial_pose_set_;
  bool reached_waypoint_;

  double k_rho_;
  double k_alpha_;
  double k_beta_;
  double v_const_;

  size_t current_waypoint_idx_;
  std::vector<std::array<double, 3>> waypoints_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SinglePointTrajectory>());
  rclcpp::shutdown();
  return 0;
}
