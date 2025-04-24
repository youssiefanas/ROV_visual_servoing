from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    plot_flag = LaunchConfiguration('plot')

    return LaunchDescription([
        # Declare the launch argument
        DeclareLaunchArgument(
            'plot',
            default_value='false',
            description='Launch PlotJuggler if true'
        ),

        # ROS 2 nodes
        Node(
            package='object_tracking',
            executable='buoy_tracker',
            name='buoy_tracker',
            output='screen'
        ),
        Node(
            package='object_tracking',
            executable='video_publisher',
            name='video_publisher',
            output='log',
        ),

        # Conditional execution of PlotJuggler
        ExecuteProcess(
            cmd=['plotjuggler', '--layout', './visual_tracking_layout.xml'],
            output='screen',
            condition=IfCondition(plot_flag)
        )
    ])
