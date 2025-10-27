#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gui_control',
            executable='gui_control',
            name='left_hand_control_node',
            output='screen',
            parameters=[{
                'hand_type': 'left',  # 配置Linker Hand灵巧手类型 left | right
                'hand_joint': "O20",
                'topic_hz': 20, # topic发布频率
                'is_touch': False, # 是否有压力传感器
                'is_arc': False, # 是否发布弧度值topic
            }],
        ),
        # # 配置第二个Linker Hand灵巧手, 注意这里的name属性要和第一个节点的name属性不同
        # Node(
        #     package='gui_control',
        #     executable='gui_control',
        #     name='right_hand_control_node',
        #     output='screen',
        #     parameters=[{
        #         'hand_type': 'right',  # 配置Linker Hand灵巧手类型 left | right
        #         'hand_joint': "O20",
        #         'topic_hz': 20, # topic发布频率
        #         'is_touch': False, # 是否有压力传感器
        #         'is_arc': False, # 是否发布弧度值topic
        #     }],
        # ),
    ])
