#!/usr/bin/env python3 
# -*- coding: utf-8 -*-
'''
编译: colcon build --symlink-install
启动命令:ros2 run linker_hand_ros2_sdk linker_hand_sdk
'''
import rclpy,math,os,sys,subprocess,time,platform,threading,json,csv,pprint
import serial.tools
import serial.tools.list_ports
import numpy as np
from rclpy.node import Node                      # ROS2 节点类
from std_msgs.msg import String, Header, Float32MultiArray
from sensor_msgs.msg import JointState
from datetime import datetime
from threading import Lock

from .LinkerHandO20API.linker_hand_o20_api import LinkerHandO20API
from .LinkerHandO20API.utils.colos_msg import ColorMsg
IDX_MAP = [17, 14, 10, 6, 1, 18, 15, 11, 7, 2,
           19, 13, 9, 5, 3, 16, 12, 8, 4, 0]

IDX_NAMES = ["Thumb_Base","Index_Finger_Base","Middle_Finger_Base","Ring_Finger_Base","Little_Finger_Base",
"Thumb_Abduction","Index_Finger_Abduction","Middle_Finger_Abduction","Ring_Finger_Abduction","Little_Finger_Abduction",
"Thumb_Rotation","Index_Finger_Middle","Middle_Finger_Middle","Ring_Finger_Middle","Little_Finger_Rotation",
"Thumb_Tip","Index_Finger_Distal","Middle_Finger_Distal","Ring_Finger_Distal","Little_Finger_Tip"]

IDX_NAMES_CN = ["拇指根部", "食指根部", "中指根部", "无名指根部","小指根部",
 "拇指侧摆","食指侧摆","中指侧摆","无名指侧摆","小指侧摆",
 "拇指旋转","食指中部","中指中部","无名指中部","小拇指旋转",
 "拇指尖部","食指末端","中指末端","无名指末端","小指末端"]
class LinkerHandO20TeleoperatedMaster(Node):
    def __init__(self,name):
        super().__init__(name)
        # 声明参数（带默认值）
        self.declare_parameter('serial_port', '/dev/O20_FTAA08AW')
        self.declare_parameter('hand_type', 'right')
        self.declare_parameter('hand_joint', 'O20')
        self.declare_parameter('is_touch', False)
        self.declare_parameter('topic_hz', 30)
        self.declare_parameter('is_angle', False)
        
        
        # 获取参数值
        self.serial_port = self.get_parameter('serial_port').value
        self.hand_type = self.get_parameter('hand_type').value
        self.hand_joint = self.get_parameter('hand_joint').value
        self.is_touch = self.get_parameter('is_touch').value
        self.topic_hz = self.get_parameter('topic_hz').value
        self.is_angle = self.get_parameter('is_angle').value


        self.is_commond = False
        self.all_current = [-1] * 20
        self.all_temperature = [-1] * 20
        self.all_voltage = [-1] * 20
        self.all_torque = [-1] * 20
        self.hz = 1.0 / int(self.topic_hz)
        self.count = 0
        self._init_hand()
        self._init_pose()
        time.sleep(1)
        ColorMsg(msg=f"{self.hand_type}_{self.hand_joint}已失能,当前处于遥操主动端模式", color="yellow")


        
    def _init_hand(self):
        if self.is_angle == True:
            self.hand_cmd_pub_angle = self.create_publisher(JointState, f'/cb_{self.hand_type}_hand_control_cmd_angle', 10)
        else:
            self.hand_cmd_pub = self.create_publisher(JointState, f'/cb_{self.hand_type}_hand_control_cmd', 1)

        self.hand=LinkerHandO20API(port=self.serial_port)
        

    def _init_pose(self):
        poseition = [255, 255, 255, 255, 255, 108, 122, 246, 156, 191, 255, 255, 254, 255, 88, 255, 255, 255, 255, 255]
        pose_dic = {IDX_MAP[i] + 1: poseition[i] for i in range(20)}
        final_angles = {}          # motor_id -> 限幅后的实际角度
        for motor_id, deg in pose_dic.items():
            limited = self.hand.map_to_limit_angle(motor_id, deg, dir=True)
            final_angles[motor_id] = round(float(limited), 2)
        try:
            ordered_angles = dict(sorted(final_angles.items()))
            self.hand.set_position(ordered_angles)
        except Exception as e:
            print(e)
        time.sleep(1)
        self.hand.motor_disable_all()

    def joint_state_msg(self, pose, vel=[]) -> JointState:
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = IDX_NAMES
        joint_state.position = [float(x) for x in pose]
        if len(vel) > 1:
            joint_state.velocity = [float(x) for x in vel]
        else:
            joint_state.velocity = [0.0] * len(pose)
        joint_state.effort = [0.0] * len(pose)
        return joint_state


    def hand_control_cb_angle(self, msg):
        pass

    def run(self):
        while rclpy.ok():
            try:
                t1 = time.time()
                # 获取手指状态角度值和范围值
                state_angle, state_range = self.hand.get_status(range=True)
                if self.hand.is_teleoperated == False:
                    continue
                aligned_angle = [state_angle[k+1] for k in IDX_MAP]
                aligned_range   = [state_range[k+1]   for k in IDX_MAP]
                if self.is_angle == True:
                    msg = self.joint_state_msg(pose=aligned_angle)
                    self.hand_cmd_pub_angle.publish(msg)
                else:
                    msg = self.joint_state_msg(pose=aligned_range)
                    self.hand_cmd_pub.publish(msg)
                
            except:
                pass
        
    


    def close_sdk(self) -> None:
        # 断开连接
        self.hand.disconnect()
        time.sleep(0.1)
        ColorMsg(msg=f"正在终止程序....", color="red")




def main(args=None) -> None:
    rclpy.init(args=args)
    node = LinkerHandO20TeleoperatedMaster("linker_hand_o20_teleoperated_master")
    try:
        node.run()
        #rclpy.spin(node)         # 主循环，监听 ROS 回调
    except KeyboardInterrupt:
        print("收到 Ctrl+C，准备退出...")
    finally:
        node.close_sdk()         # 关闭 CAN 或其他硬件资源
        node.destroy_node()      # 销毁 ROS 节点
        if rclpy.ok():
            rclpy.shutdown()
        print("程序已退出。")
