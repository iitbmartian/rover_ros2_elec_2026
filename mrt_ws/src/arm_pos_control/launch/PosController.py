from launch import LaunchDescription
from launch_ros.actions import Node

this_pkg = 'arm_pos_control'

def generate_launch_description():
    return LaunchDescription([
        Node(
            package=this_pkg,
            executable='queue',
            name='queue'
        ),
        Node(
            package=this_pkg,
            executable='pid_controller',
            name='controller'
        )
    ])