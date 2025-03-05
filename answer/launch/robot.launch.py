from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    
    # 视觉处理节点
    vision_node = Node(
        package='answer',
        executable='vision_node',
        name='vision_node',
        output='screen'
    )
    ld.add_action(vision_node)
    
    # 导航节点
    navigation_node = Node(
        package='answer',
        executable='navigation_node',
        name='navigation_node',
        output='screen'
    )
    ld.add_action(navigation_node)
    
    return ld 