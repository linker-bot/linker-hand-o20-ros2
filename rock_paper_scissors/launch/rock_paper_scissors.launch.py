from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rock_paper_scissors',
            executable='rock_paper_scissors',
            name='rock_paper_scissors',
            output='screen',
            parameters=[{
                'hand_type': 'right', # 配置Linker Hand灵巧手类型 left | right 字母为小写
                'hand_joint': "O20", # O20 字母为大写
            }],
        ),
    ])