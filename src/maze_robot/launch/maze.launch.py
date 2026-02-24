from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

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
        navigation_node,
        perception_node
    ])