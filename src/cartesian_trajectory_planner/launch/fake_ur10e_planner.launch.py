from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": LaunchConfiguration("robot_description")
            }],
            remappings=[("/joint_states", "/joint_states")]
        ),

        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            output="screen"
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen"
        ),

        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[{
                "robot_description": LaunchConfiguration("robot_description"),
                "use_sim_time": True
            }]
        ),
    ])
