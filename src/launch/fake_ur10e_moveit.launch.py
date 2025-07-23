from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='ur_robot_driver', executable='ur_control.launch.py', name='ur_control',
             arguments=['ur_type:=ur10e', 'robot_ip:=127.0.0.1',
                        'use_fake_hardware:=true', 'use_sim_time:=true',
                        'launch_rviz:=true']),
    ])

