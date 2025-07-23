#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class VelocityEstimatorNode : public rclcpp::Node
{
public:
  VelocityEstimatorNode() : Node("velocity_estimator_node")
  {
    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&VelocityEstimatorNode::joint_callback, this, std::placeholders::_1));

    velocity_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "/joint_states_with_velocity", 10);

    RCLCPP_INFO(this->get_logger(), "Velocity estimator node started.");
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr velocity_pub_;

  sensor_msgs::msg::JointState last_msg_;
  rclcpp::Time last_time_;

  void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (last_msg_.position.empty()) {
      last_msg_ = *msg;
      last_time_ = this->now();
      return;
    }

    rclcpp::Duration dt = this->now() - last_time_;
    if (dt.seconds() <= 0.0) return;

    sensor_msgs::msg::JointState vel_msg;
    vel_msg.header.stamp = msg->header.stamp;
    vel_msg.name = msg->name;
    vel_msg.position = msg->position;
    vel_msg.velocity.resize(msg->position.size(), 0.0);

    for (size_t i = 0; i < msg->position.size(); ++i) {
      double delta = msg->position[i] - last_msg_.position[i];
      vel_msg.velocity[i] = delta / dt.seconds();
    }

    last_msg_ = *msg;
    last_time_ = this->now();

    velocity_pub_->publish(vel_msg);
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VelocityEstimatorNode>());
  rclcpp::shutdown();
  return 0;
}

