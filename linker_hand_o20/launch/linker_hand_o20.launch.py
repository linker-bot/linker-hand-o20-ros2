#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='linker_hand_o20',
            executable='linker_hand_o20',
            name='linker_hand_o20',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'hand_type': 'right',  # 配置Linker Hand灵巧手类型 left | right
                'hand_joint': "O20",
                'topic_hz': 10, # topic发布频率
                'is_touch': False, # 是否有压力传感器
                'is_angle': False, # 是否发布弧度值topic
                "is_slave": False, # 是否为从机模式 从机模式会关闭所有状态数据获取发布，只有遥操作模式下有效
            }],
        ),
        # 配置第二个Linker Hand灵巧手, 只需要修改serial_port参数即可
        # Node(
        #     package='linker_hand_o20',
        #     executable='linker_hand_o20',
        #     name='linker_hand_o20',
        #     output='screen',
        #     parameters=[{
        #         'serial_port': '/dev/O20_right',
        #         'hand_type': 'right',  # 配置Linker Hand灵巧手类型 left | right
        #         'hand_joint': "O20",
        #         'topic_hz': 10, # topic发布频率
        #         'is_touch': False, # 是否有压力传感器
        #         'is_angle': False, # 是否发布弧度值topic
        #         "is_slave": False, # 是否为从机模式 从机模式会关闭所有状态数据获取发布，只有遥操作模式下有效
        #     }],
        # ),
    ])
