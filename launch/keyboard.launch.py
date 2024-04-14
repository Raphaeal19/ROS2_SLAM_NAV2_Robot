# Import necessary modules
import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Set the parameters for the teleop_twist_keyboard node
    teleop_twist_keyboard_params = {
        "scale_linear": 0.5,    # Linear velocity scaling factor
        "scale_angular": 0.5    # Angular velocity scaling factor
    }

    # Create a node to launch teleop_twist_keyboard
    teleop_node = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name="teleop_node",
        output="screen",
        parameters=[teleop_twist_keyboard_params],
        emulate_tty=True,   # Emulate a TTY terminal
        prefix="gnome-terminal --",  # Use xterm to launch the node
        remappings=[('cmd_vel','/cmd_vel_key')]
    )

    # Create the launch description and add the teleop_twist_keyboard node

    return LaunchDescription([
        teleop_node
    ])
