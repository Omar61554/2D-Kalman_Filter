#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <eigen3/Eigen/Dense>

class KalmanFilterNode : public rclcpp::Node {
public:
  KalmanFilterNode() : Node("kalman_filter_node") {
    // Initialize subscriber to odom_noise and publisher for estimated position
    odom_noise_sub_ = create_subscription<geometry_msgs::msg::Pose2D>(
        "/odom_noise",
        10,
        std::bind(&KalmanFilterNode::odomNoiseCallback, this, std::placeholders::_1)
    );

    estimated_position_pub_ = create_publisher<geometry_msgs::msg::Pose2D>(
        "/estimated_position",
        10
    );

    // Initialize other variables and matrices
    is_initialized_ = false;

    // State vector [x, y, vx, vy]
    x_ = Eigen::VectorXd(4);
    x_ << 0.0, 0.0, 0.0, 0.0;

    // State covariance matrix
    P_ = Eigen::MatrixXd(4, 4);
    P_ << 1.0, 0.0, 0.0, 0.0,
          0.0, 1.0, 0.0, 0.0,
          0.0, 0.0, 1.0, 0.0,
          0.0, 0.0, 0.0, 1.0;

    // Process noise covariance matrix
    Q_ = Eigen::MatrixXd(4, 4);
    Q_ << 0.1, 0.0, 0.0, 0.0,
          0.0, 0.1, 0.0, 0.0,
          0.0, 0.0, 0.1, 0.0,
          0.0, 0.0, 0.0, 0.1;

    // Measurement matrix
    H_ = Eigen::MatrixXd(2, 4);
    H_ << 1.0, 0.0, 0.0, 0.0,
          0.0, 1.0, 0.0, 0.0;

    // Measurement noise covariance matrix
    R_ = Eigen::MatrixXd(2, 2);
    R_ << 0.1, 0.0,
          0.0, 0.1;

    // Set the update rate for publishing the estimated position
    timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&KalmanFilterNode::publishEstimatedPosition, this));
  }

private:
  void odomNoiseCallback(const geometry_msgs::msg::Pose2D::SharedPtr msg) {
    // Extract the noisy measurements
    double z_x = msg->x;
    double z_y = msg->y;

    if (!is_initialized_) {
      // Initialize the state using the first measurement
      x_(0) = z_x;
      x_(1) = z_y;
      is_initialized_ = true;
    } else {
      // Predict step
      Eigen::VectorXd x_pred = projectState(x_);
      Eigen::MatrixXd P_pred = projectCovariance(P_);

      // Update step
      Eigen::VectorXd y = Eigen::VectorXd(2);
      y << z_x - x_pred(0), z_y - x_pred(1);

      Eigen::MatrixXd S = H_ * P_pred * H_.transpose() + R_;
      Eigen::MatrixXd K = P_pred * H_.transpose() * S.inverse();

      x_ = x_pred + K * y;
      P_ = (Eigen::MatrixXd::Identity(4, 4) - K * H_) * P_pred;
    }
  }

  Eigen::VectorXd projectState(const Eigen::VectorXd& x) {
    // Implement the state projection based on the system dynamics
    // ...

    return x; // Return the projected state
  }

  Eigen::MatrixXd projectCovariance(const Eigen::MatrixXd& P) {
    // Implement the covariance projection based on the system dynamics
    // ...

    return P; // Return the projected covariance
  }

  void publishEstimatedPosition() {
    // Create a new estimated position message
    auto estimated_msg = std::make_unique<geometry_msgs::msg::Pose2D>();
    estimated_msg->x = x_(0);
    estimated_msg->y = x_(1);

    // Publish the estimated position
    estimated_position_pub_->publish(std::move(estimated_msg));
  }

  bool is_initialized_;
  Eigen::VectorXd x_; // State vector
  Eigen::MatrixXd P_; // StateApologies for the incomplete response! Here's the continuation of the code:


  Eigen::MatrixXd Q_; // Process noise covariance matrix
  Eigen::MatrixXd H_; // Measurement matrix
  Eigen::MatrixXd R_; // Measurement noise covariance matrix

  rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr odom_noise_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr estimated_position_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KalmanFilterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}