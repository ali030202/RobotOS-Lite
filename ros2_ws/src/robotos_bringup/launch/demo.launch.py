from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('enable_fault_injection', default_value='false'),
        Node(package='robotos_core', executable='sim_node', output='screen',
             parameters=[{'enable_fault_injection': ParameterValue(
                 LaunchConfiguration('enable_fault_injection'), value_type=bool)}]),
        Node(package='robotos_core', executable='control_node', output='screen'),
    ])
