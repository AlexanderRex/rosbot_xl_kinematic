#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <array>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

class SinglePointTrajectory : public rclcpp::Node {
public:
  SinglePointTrajectory()
      : Node("single_point_trajectory"), reached_waypoint_(false), kp_lin_(1.0),
        ki_lin_(0.0), kd_lin_(0.0), prev_error_x_(0.0), prev_error_y_(0.0),
        integral_x_(0.0), integral_y_(0.0), kp_ang_(1.0), ki_ang_(0.0),
        kd_ang_(0.0), prev_error_ang_(0.0), integral_ang_(0.0),
        initial_pose_set_(false) {
    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", 10,
        std::bind(&SinglePointTrajectory::odom_callback, this,
                  std::placeholders::_1));
    wheel_speed_publisher_ =
        this->create_publisher<std_msgs::msg::Float32MultiArray>("/wheel_speed",
                                                                 10);
    marker_publisher_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/trajectory_markers", 10);

    timer_ = this->create_wall_timer(
        100ms, std::bind(&SinglePointTrajectory::move_to_point, this));
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
  }

  bool check_if_reached_waypoint() {
    double current_x = current_pose_.position.x;
    double current_y = current_pose_.position.y;

    double global_target_x = initial_x_ + target_point_[1];
    double global_target_y = initial_y_ + target_point_[2];

    double distance = std::sqrt(std::pow(global_target_x - current_x, 2) +
                                std::pow(global_target_y - current_y, 2));

    return distance < 0.1;
  }

  void move_to_point() {
    if (reached_waypoint_) {
      RCLCPP_INFO(this->get_logger(),
                  "Reached the waypoint and target orientation");
      return;
    }

    double global_target_x = initial_x_ + target_point_[1];
    double global_target_y = initial_y_ + target_point_[2];

    double current_x = current_pose_.position.x;
    double current_y = current_pose_.position.y;

    double error_x = global_target_x - current_x;
    double error_y = global_target_y - current_y;

    double current_yaw = get_yaw_from_pose(current_pose_);
    double target_yaw = initial_yaw_ + target_point_[0];
    double error_ang = normalize_angle(target_yaw - current_yaw);

    double control_signal_x = pid_control(error_x, prev_error_x_, integral_x_,
                                          kp_lin_, ki_lin_, kd_lin_);
    double control_signal_y = pid_control(error_y, prev_error_y_, integral_y_,
                                          kp_lin_, ki_lin_, kd_lin_);
    double control_signal_ang = pid_control(
        error_ang, prev_error_ang_, integral_ang_, kp_ang_, ki_ang_, kd_ang_);

    auto wheel_speeds = compute_wheel_speeds(control_signal_x, control_signal_y,
                                             control_signal_ang);

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

    publish_markers(global_target_x, global_target_y);
  }

  std::array<double, 4> compute_wheel_speeds(double dx, double dy,
                                             double dtheta) {
    double r = 0.1;
    double L = 0.17;
    double w = 0.135;

    double vx = dx * cos(initial_yaw_) - dy * sin(initial_yaw_);
    double vy = dx * sin(initial_yaw_) + dy * cos(initial_yaw_);
    double omega = dtheta;

    RCLCPP_INFO(this->get_logger(),
                "Control signals - vx: %.2f, vy: %.2f, omega: %.2f", vx, vy,
                omega);

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

  double pid_control(double error, double &prev_error, double &integral,
                     double kp, double ki, double kd) {
    integral += error * 0.1;
    double derivative = (error - prev_error) / 0.1;
    prev_error = error;
    return kp * error + ki * integral + kd * derivative;
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

  void publish_markers(double target_x, double target_y) {
    visualization_msgs::msg::MarkerArray marker_array;

    // Current position marker
    visualization_msgs::msg::Marker current_pos_marker;
    current_pos_marker.header.frame_id = "odom";
    current_pos_marker.header.stamp = this->now();
    current_pos_marker.ns = "robot_position";
    current_pos_marker.id = 0;
    current_pos_marker.type = visualization_msgs::msg::Marker::SPHERE;
    current_pos_marker.action = visualization_msgs::msg::Marker::ADD;
    current_pos_marker.pose.position.x = current_pose_.position.x;
    current_pos_marker.pose.position.y = current_pose_.position.y;
    current_pos_marker.pose.position.z = 0.0;
    current_pos_marker.pose.orientation.w = 1.0;
    current_pos_marker.scale.x = 0.1;
    current_pos_marker.scale.y = 0.1;
    current_pos_marker.scale.z = 0.1;
    current_pos_marker.color.a = 1.0;
    current_pos_marker.color.r = 0.0;
    current_pos_marker.color.g = 1.0;
    current_pos_marker.color.b = 0.0;

    marker_array.markers.push_back(current_pos_marker);

    // Target position marker
    visualization_msgs::msg::Marker target_pos_marker;
    target_pos_marker.header.frame_id = "odom";
    target_pos_marker.header.stamp = this->now();
    target_pos_marker.ns = "target_position";
    target_pos_marker.id = 1;
    target_pos_marker.type = visualization_msgs::msg::Marker::SPHERE;
    target_pos_marker.action = visualization_msgs::msg::Marker::ADD;
    target_pos_marker.pose.position.x = target_x;
    target_pos_marker.pose.position.y = target_y;
    target_pos_marker.pose.position.z = 0.0;
    target_pos_marker.pose.orientation.w = 1.0;
    target_pos_marker.scale.x = 0.1;
    target_pos_marker.scale.y = 0.1;
    target_pos_marker.scale.z = 0.1;
    target_pos_marker.color.a = 1.0;
    target_pos_marker.color.r = 1.0;
    target_pos_marker.color.g = 0.0;
    target_pos_marker.color.b = 0.0;

    marker_array.markers.push_back(target_pos_marker);

    // Trajectory line marker
    visualization_msgs::msg::Marker trajectory_marker;
    trajectory_marker.header.frame_id = "odom";
    trajectory_marker.header.stamp = this->now();
    trajectory_marker.ns = "trajectory";
    trajectory_marker.id = 2;
    trajectory_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    trajectory_marker.action = visualization_msgs::msg::Marker::ADD;
    trajectory_marker.scale.x = 0.05;
    trajectory_marker.color.a = 1.0;
    trajectory_marker.color.r = 0.0;
    trajectory_marker.color.g = 0.0;
    trajectory_marker.color.b = 1.0;

    geometry_msgs::msg::Point p;
    p.z = 0.0;
    p.x = initial_x_;
    p.y = initial_y_;
    trajectory_marker.points.push_back(p);

    p.x = target_x;
    p.y = target_y;
    trajectory_marker.points.push_back(p);

    marker_array.markers.push_back(trajectory_marker);

    marker_publisher_->publish(marker_array);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr
      wheel_speed_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::Pose current_pose_;
  double initial_x_ = 0.0;
  double initial_y_ = 0.0;
  double initial_yaw_ = 0.0;
  bool initial_pose_set_;
  bool reached_waypoint_;

  double kp_lin_;
  double ki_lin_;
  double kd_lin_;
  double prev_error_x_;
  double prev_error_y_;
  double integral_x_;
  double integral_y_;

  double kp_ang_;
  double ki_ang_;
  double kd_ang_;
  double prev_error_ang_;
  double integral_ang_;

  std::array<double, 3> target_point_ = {1.5708, 1,
                                         -1}; // Set target point here
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SinglePointTrajectory>());
  rclcpp::shutdown();
  return 0;
}
