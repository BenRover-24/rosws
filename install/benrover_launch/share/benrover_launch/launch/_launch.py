import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    
    rover_params = os.path.join(
        get_package_share_directory('benrover_launch'),
        'config',
        'benrover_params.yaml'
    )

    launchDescription = LaunchDescription()
    launchDescription.add_action(
       Node(
            package='benrover_navigation',
            executable='navigation',
            name='benrover_navigation',
            output='screen',
            emulate_tty=True,
            parameters=[rover_params]
       )
    )    
    
    launchDescription.add_action(
       Node(
            package='joy',
            executable='joy_node',
            name='joy',
            output='screen',
            emulate_tty=True,
            respawn=True,
            parameters=[
                {"autorepeat_rate": 5.0},
                {"device_id": 0},
            ]
       )
    )

    launchDescription.add_action(
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy',
            output='screen',
            emulate_tty=True,
            respawn=True,
            parameters=[
                # {"scale_linear.x": 0.4},  # scale to apply to drive speed, in m/s: drive_motor_rpm * 2pi / 60 * wheel radius * slowdown_factor
                {"scale_linear.x": -0.4},  # scale to apply to drive speed, in m/s: drive_motor_rpm * 2pi / 60 * wheel radius * slowdown_factor
                # {"axis_linear.x": 4},
                {"axis_linear.x": 3},
                # {"axis_angular.yaw": 0},  # which joystick axis to use for driving
                {"axis_angular.yaw": 2},  # which joystick axis to use for driving
                # {"scale_angular.yaw": 1.25},  # scale to apply to angular speed, in rad/s: scale_linear / min_radius(=0.45m)
                {"axis_angular.pitch": 0},  # axis to use for in-place rotation
                {"scale_angular.yaw": -1.25},  # scale to apply to angular speed, in rad/s: scale_linear / min_radius(=0.45m)
                {"scale_angular.pitch": 0.25},  # scale to apply to angular speed, in rad/s: scale_linear / min_radius(=0.45m)
                {"scale_angular_turbo.yaw": 3.95},  # scale to apply to angular speed, in rad/s: scale_linear_turbo / min_radius
                {"scale_linear_turbo.x": 1.78},  # scale to apply to linear speed, in m/s
                # {"enable_button": 4},  # which button to press to enable movement
                {"enable_button": 0},  # which button to press to enable movement
                # {"enable_turbo_button": 5}  # -1 to disable turbo
                {"enable_turbo_button": -1}  # -1 to disable turbo
            ],
            remappings=[
                ('/cmd_vel', '/cmd_vel_intuitive')
            ]
        )
    )    
    return launchDescription