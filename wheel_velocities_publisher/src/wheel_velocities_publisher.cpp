#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class WheelVelocitiesPublisher : public rclcpp::Node {
public:
  WheelVelocitiesPublisher()
      : Node("wheel_velocities_publisher"), motion_index_(0) {
    publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "/wheel_speed", 10);
    timer_ = this->create_wall_timer(
        3s, std::bind(&WheelVelocitiesPublisher::publish_wheel_speeds, this));
    RCLCPP_INFO(this->get_logger(),
                "Initialized wheel velocities publisher node");
  }

private:
  void publish_wheel_speeds() {
    auto message = std_msgs::msg::Float32MultiArray();
    message.layout.data_offset = 0;
    message.data.resize(4);

    switch (motion_index_) {
    case 0:
      RCLCPP_INFO(this->get_logger(), "Move forward");
      message.data = {1.0, 1.0, 1.0, 1.0};
      break;
    case 1:
      RCLCPP_INFO(this->get_logger(), "Move backward");
      message.data = {-1.0, -1.0, -1.0, -1.0};
      break;
    case 2:
      RCLCPP_INFO(this->get_logger(), "Move left");
      message.data = {-1.0, 1.0, 1.0, -1.0};
      break;
    case 3:
      RCLCPP_INFO(this->get_logger(), "Move right");
      message.data = {1.0, -1.0, -1.0, 1.0};
      break;
    case 4:
      RCLCPP_INFO(this->get_logger(), "Turn clockwise");
      message.data = {1.0, -1.0, 1.0, -1.0};
      break;
    case 5:
      RCLCPP_INFO(this->get_logger(), "Turn counter-clockwise");
      message.data = {-1.0, 1.0, -1.0, 1.0};
      break;
    default:
      RCLCPP_INFO(this->get_logger(), "Stop");
      message.data = {0.0, 0.0, 0.0, 0.0};
      timer_->cancel();
      break;
    }

    publisher_->publish(message);
    motion_index_++;
  }

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  int motion_index_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelVelocitiesPublisher>());
  rclcpp::shutdown();
  return 0;
}
