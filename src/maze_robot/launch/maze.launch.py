from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    navigation_node = Node(
        package='seu_pacote',
        executable='navigation_node',
        name='navigation_node',
        output='screen'
    )

    perception_node = Node(
        package='seu_pacote',
        executable='perception_node',
        name='perception_node',
        output='screen'
    )

    return LaunchDescription([
        navigation_node,
        perception_node
    ])