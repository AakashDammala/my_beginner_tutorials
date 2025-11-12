from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():    
    # 1. Declare the launch argument
    oven_name_arg = DeclareLaunchArgument(
        'oven_name',
        default_value='main_oven',
        description='The name for the oven.'
    )

    # 2. Define the oven_publisher node
    oven_publisher_node = Node(
        package='beginner_tutorials',
        executable='oven_publisher',
        name='oven_publisher',
        parameters=[
            {'name': LaunchConfiguration('oven_name')}
        ]
    )

    # 4. Define the oven_subscriber node (unchanged)
    oven_subscriber_node = Node(
        package='beginner_tutorials',
        executable='oven_subscriber',
        name='oven_subscriber'
    )

    # 5. Return the LaunchDescription to run
    return LaunchDescription([
        oven_name_arg,  # Add the argument action
        oven_publisher_node,
        oven_subscriber_node
    ])