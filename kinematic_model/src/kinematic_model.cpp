#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <memory>
#include <vector>

class KinematicModel : public rclcpp::Node {
public:
  KinematicModel() : Node("kinematic_model") {
    subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/wheel_speed", 10,
        std::bind(&KinematicModel::topic_callback, this,
                  std::placeholders::_1));
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  }

private:
  void topic_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    if (msg->data.size() != 4) {
      RCLCPP_ERROR(this->get_logger(), "Expected 4 wheel speeds, received %zu",
                   msg->data.size());
      return;
    }

    double fl = msg->data[0]; // front left wheel speed
    double fr = msg->data[1]; // front right wheel speed
    double bl = msg->data[2]; // back left wheel speed
    double br = msg->data[3]; // back right wheel speed

    // Kinematic model for holonomic robot with omni wheels
    double L =
        0.17; // Distance from the robot center to the wheel, adjust as needed
    double w = 0.135; // Half of the track width, adjust as needed
    double r = 0.1;   // Radius of the wheel, adjust as needed

    // Calculate the chassis velocities
    double vx = r * (fl + fr + bl + br) / 4.0;
    double vy = r * (-fl + fr + bl - br) / 4.0;
    double omega = r * (-fl + fr - bl + br) / (4.0 * (L + w));

    auto twist = geometry_msgs::msg::Twist();
    twist.linear.x = vx;
    twist.linear.y = vy;
    twist.angular.z = omega;

    publisher_->publish(twist);
  }

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr
      subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KinematicModel>());
  rclcpp::shutdown();
  return 0;
}
