#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <random>

class NoiseNode : public rclcpp::Node {
public:
  NoiseNode() : Node("noise_node") {
    // Initialize publisher for odom_noise
    odom_noise_pub_ = create_publisher<geometry_msgs::msg::Pose2D>(
        "/odom_noise",
        10
    );

    // Initialize random number generator
    std::random_device rd;
    random_generator_ = std::mt19937(rd());
    distribution_ = std::normal_distribution<double>(0.0, 0.1); // Mean = 0.0, Standard deviation = 0.1

    // Set the update rate for publishing the noisy data
    timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&NoiseNode::publishNoisyData, this));
  }

private:
  void publishNoisyData() {
    // Generate random Gaussian noise
    double noise_x = distribution_(random_generator_);
    double noise_y = distribution_(random_generator_);

    // Create a new noisy odom message
    auto odom_noise_msg = std::make_unique<geometry_msgs::msg::Pose2D>();
    odom_noise_msg->x += noise_x; // Add noise to the x position
    odom_noise_msg->y += noise_y; // Add noise to the y position

    // Publish the noisy odom data
    odom_noise_pub_->publish(std::move(odom_noise_msg));
  }

  rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr odom_noise_pub_;
  std::mt19937 random_generator_;
  std::normal_distribution<double> distribution_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NoiseNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}