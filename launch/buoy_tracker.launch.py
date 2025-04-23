from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Your ROS 2 node
        Node(
            package='object_tracking',
            executable='buoy_tracker',
            name='buoy_tracker',
            output='screen'
        ),

        # PlotJuggler as a process
        ExecuteProcess(
            cmd=['plotjuggler', '--streaming'],
            output='screen'
        )
    ])


