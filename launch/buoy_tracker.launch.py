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

        Node(
            #             'video_publisher = object_tracking.video_publisher:main',  # ✅ Ensure this line is here
            package='object_tracking',
            executable='video_publisher',
            name='video_publisher',
            output='log',
        ),
        # PlotJuggler as a process
        ExecuteProcess(
            cmd=['plotjuggler', '--layout', '/home/hans/ros2/task2/src/ROV_visual_servoing/visual_tracking_layout.xml'],
            output='screen'
        )
    ])
