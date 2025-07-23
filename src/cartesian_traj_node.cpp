#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

class CartesianPlanner : public rclcpp::Node
{
public:
  CartesianPlanner() : Node("cartesian_traj_node") {}

  void initialize()
  {
    RCLCPP_INFO(this->get_logger(), "Waiting 2s for RViz/MoveIt to be ready...");
    rclcpp::sleep_for(std::chrono::seconds(2));

    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "ur_manipulator");

    move_group_->setMaxVelocityScalingFactor(0.2);
    move_group_->setMaxAccelerationScalingFactor(0.1);
    plan_and_execute();
  }

private:
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

  void plan_and_execute()
  {
    std::vector<geometry_msgs::msg::Pose> waypoints;

    auto start_pose = move_group_->getCurrentPose().pose;
    waypoints.push_back(start_pose);

    geometry_msgs::msg::Pose p1 = start_pose;
    p1.position.x += 0.15;
    waypoints.push_back(p1);

    geometry_msgs::msg::Pose p2 = p1;
    p2.position.y += 0.1;
    waypoints.push_back(p2);

    geometry_msgs::msg::Pose p3 = p2;
    p3.position.x -= 0.15;
    waypoints.push_back(p3);

    geometry_msgs::msg::Pose p4 = p3;
    p4.position.y -= 0.1;
    waypoints.push_back(p4);

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double eef_step = 0.01;
    const double jump_threshold = 0.0;

    double fraction = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    RCLCPP_INFO(this->get_logger(), "Planned %.2f%% of the path.", fraction * 100.0);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;

    move_group_->execute(plan);
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CartesianPlanner>();
  node->initialize();  // <- safe to use shared_from_this() here
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

