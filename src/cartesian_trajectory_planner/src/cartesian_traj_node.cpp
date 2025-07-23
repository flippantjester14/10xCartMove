#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <cmath>

class CartesianPlanner : public rclcpp::Node
{
public:
  CartesianPlanner(const rclcpp::NodeOptions &options)
      : Node("cartesian_traj_node", options)
  {
    this->declare_parameter("velocity_scale", 0.3);
    this->declare_parameter("compliance_enabled", false);
  }

  void initialize()
  {
    wait_for_robot_description();

    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), "panda_arm");

    move_group_->setPlanningTime(10.0);
    move_group_->setNumPlanningAttempts(5);

    plan_and_execute_spiral();
  }

private:
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

  void wait_for_robot_description()
  {
    auto param_client = std::make_shared<rclcpp::SyncParametersClient>(
        shared_from_this(), "robot_state_publisher");

    RCLCPP_INFO(this->get_logger(), "Waiting for 'robot_description' parameter...");
    while (!param_client->wait_for_service(std::chrono::seconds(1)))
    {
      RCLCPP_WARN(this->get_logger(), "Waiting for robot_state_publisher service...");
    }

    while (!param_client->has_parameter("robot_description"))
    {
      RCLCPP_WARN(this->get_logger(), "Waiting for 'robot_description' to become available...");
      rclcpp::sleep_for(std::chrono::milliseconds(500));
    }

    RCLCPP_INFO(this->get_logger(), "'robot_description' is now available.");
  }

  void plan_and_execute_spiral()
  {
    double velocity_scale = this->get_parameter("velocity_scale").as_double();
    bool compliance = this->get_parameter("compliance_enabled").as_bool();

    move_group_->setMaxVelocityScalingFactor(velocity_scale);
    move_group_->setMaxAccelerationScalingFactor(velocity_scale / 2.0);

    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose start_pose = move_group_->getCurrentPose().pose;
    waypoints.push_back(start_pose);

    // Smooth spiral path in XY plane
    double x = start_pose.position.x;
    double y = start_pose.position.y;
    double r = 0.01;
    for (double theta = 0.0; theta <= 4 * M_PI; theta += 0.05)
    {
      geometry_msgs::msg::Pose p = start_pose;
      p.position.x = x + r * theta * cos(theta);
      p.position.y = y + r * theta * sin(theta);
      waypoints.push_back(p);
    }

    const double eef_step = 0.001;
    const double jump_threshold = 0.0;

    moveit_msgs::msg::RobotTrajectory ros_traj;
    double fraction = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, ros_traj);
    RCLCPP_INFO(this->get_logger(), "Planned %.2f%% of the spiral path", fraction * 100.0);

    if (fraction < 0.85)
    {
      RCLCPP_ERROR(this->get_logger(), "Path fraction too low — skipping execution.");
      return;
    }
    else if (fraction < 0.90)
    {
      RCLCPP_WARN(this->get_logger(), "Path planned only %.2f%% — below 90%% accuracy!", fraction * 100.0);
    }

    robot_trajectory::RobotTrajectory robot_traj(
        move_group_->getCurrentState()->getRobotModel(),
        move_group_->getName());
    robot_traj.setRobotTrajectoryMsg(*move_group_->getCurrentState(), ros_traj);

    trajectory_processing::TimeOptimalTrajectoryGeneration totg;
    if (!totg.computeTimeStamps(robot_traj))
    {
      RCLCPP_ERROR(this->get_logger(), "Time parameterization failed.");
      return;
    }

    moveit_msgs::msg::RobotTrajectory timed_traj;
    robot_traj.getRobotTrajectoryMsg(timed_traj);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = timed_traj;

    if (compliance)
    {
      RCLCPP_WARN(this->get_logger(), "Compliance enabled: checking fake force (wall sim)");
      for (const auto &point : timed_traj.joint_trajectory.points)
      {
        double fake_force = 2.5; // fake contact force sim
        if (fake_force > 2.0)
        {
          RCLCPP_WARN(this->get_logger(), "Force exceeds wall limit. Skipping execution.");
          return;
        }
      }
    }

    move_group_->execute(plan);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"use_sim_time", rclcpp::ParameterValue(true)}});
  auto node = std::make_shared<CartesianPlanner>(options);
  node->initialize();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

