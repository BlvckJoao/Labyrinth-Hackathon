from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    motor_node = Node(
        package='maze_robot',
        executable='motor_node',
        name='motor_node',
        output='screen'
    )

    navigation_node = Node(
        package='maze_robot',
        executable='navigation_node',
        name='navigation_node',
        output='screen'
    )

    perception_node = Node(
        package='maze_robot',
        executable='perception_node',
        name='perception_node',
        output='screen'
    )

    return LaunchDescription([
        motor_node,
        navigation_node,
        perception_node
    ])