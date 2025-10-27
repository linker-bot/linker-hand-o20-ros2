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

IDX_P = 1000   # 默认值1200
IDX_I = 30  # 默认值0
IDX_D = 250  # 默认值0
class LinkerHandO20(Node):
    def __init__(self,name):
        super().__init__(name)
        # 声明参数（带默认值）
        self.declare_parameter('serial_port', '/dev/O20_FTAA08AW')
        self.declare_parameter('hand_type', 'right')
        self.declare_parameter('hand_joint', 'O20')
        self.declare_parameter('is_touch', False)
        self.declare_parameter('topic_hz', 30)
        self.declare_parameter('is_angle', False)
        self.declare_parameter('is_slave', False)
        
        
        # 获取参数值
        self.serial_port = self.get_parameter('serial_port').value
        self.hand_type = self.get_parameter('hand_type').value
        self.hand_joint = self.get_parameter('hand_joint').value
        self.is_touch = self.get_parameter('is_touch').value
        self.topic_hz = self.get_parameter('topic_hz').value
        self.is_angle = self.get_parameter('is_angle').value
        self.is_slave = self.get_parameter('is_slave').value
        self.hand_lock = threading.Lock()   # 1. 创建锁

        self.is_commond = False # 当前是否有setting命令进入
        self.all_current = [-1] * 20
        self.all_temperature = [-1] * 20
        self.all_voltage = [-1] * 20
        self.all_torque = [-1] * 20
        self.hz = 1.0 / int(self.topic_hz)
        self.count = 0
        self._init_hand()
        self._init_pose()



        

        
    def _init_hand(self):
        if self.is_angle == True:
            self.hand_state_pub_angle = self.create_publisher(JointState, f'/cb_{self.hand_type}_hand_state_angle', 10)
            self.hand_cmd_sub_angle = self.create_subscription(JointState, f"/cb_{self.hand_type}_hand_control_cmd_angle", self.hand_control_cb_angle, 10)
        else:
            self.hand_state_pub = self.create_publisher(JointState, f'/cb_{self.hand_type}_hand_state', 10)
            self.hand_cmd_sub = self.create_subscription(JointState, f"/cb_{self.hand_type}_hand_control_cmd", self.hand_control_cb, 1)
        self.hand_info_pub = self.create_publisher(String, f'/cb_{self.hand_type}_hand_info', 10)
        self.hand_setting_sub = self.create_subscription(String, f"/cb_hand_setting", self.hand_setting_cb, 10)
        self.hand=LinkerHandO20API(port=self.serial_port, hand_type=self.hand_type)
        time.sleep(1)

        
        self.thread_get_state = threading.Thread(target=self.pub_joint_state)
        self.thread_get_state.daemon = True
        self.thread_get_state.start()
        ColorMsg(msg=f"{self.hand_type}_{self.hand_joint}初始化成功", color="green")
        if self.is_slave == True:
            ColorMsg(msg=f"{self.hand_type}_{self.hand_joint}当前为遥操从动端模式", color="yellow")

    def hand_setting_cb(self, msg):
        # ros2 topic pub --once /cb_hand_setting std_msgs/msg/String "data: '{\"commond\":\"disable\",\"params\":{\"hand_joint\":\"o20\",\"hand_type\":\"right\"}}'"
        """失能命令"""
        # {"commond":"disable","params":{"hand_joint":"o20", "hand_type":"right"}}
        cmd = json.loads(msg.data)
        self.is_commond = True
        time.sleep(0.1)
        if cmd["commond"] == "disable":
            params = cmd["params"]
            if params["hand_joint"].lower() == self.hand_joint.lower() and params["hand_type"].lower() == self.hand_type.lower():
                self.hand.motor_disable_all()
            else:
                ColorMsg(msg=f"失能命令错误:{cmd}", color="red")
                
        """使能命令"""
        # ros2 topic pub --once /cb_hand_setting std_msgs/msg/String "data: '{\"commond\":\"enable\",\"params\":{\"hand_joint\":\"o20\",\"hand_type\":\"right\"}}'"
        if cmd["commond"] == "enable":
            params = cmd["params"]
            if params["hand_joint"].lower() == self.hand_joint.lower() and params["hand_type"].lower() == self.hand_type.lower():
                self.hand.motor_enable_all()
            else:
                ColorMsg(msg=f"使能命令错误:{cmd}", color="red")
        

        self.is_commond = False

      

    def _init_pose(self):
        poseition = [255, 255, 255, 255, 255, 108, 122, 135, 156, 191, 255, 255, 254, 255, 88, 255, 255, 255, 255, 255]
        pose_dic = {IDX_MAP[i] + 1: poseition[i] for i in range(20)}
        final_angles = {}          # motor_id -> 限幅后的实际角度
        for motor_id, deg in pose_dic.items():
            limited = self.hand.map_to_limit_angle(motor_id, deg, dir=True)
            final_angles[motor_id] = round(float(limited), 2)
        try:
            with self.hand_lock:
                ordered_angles = dict(sorted(final_angles.items()))
                self.hand.set_position(ordered_angles)
        except Exception as e:
            print(e)

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

    def hand_control_cb(self,msg):
        if self.is_commond == True: # 如果接收到了命令，则停止当前控制
            return
        poseition = list(msg.position)
        pose_dic = {IDX_MAP[i] + 1: poseition[i] for i in range(20)}
        final_angles = {}          # motor_id -> 限幅后的实际角度
        for motor_id, deg in pose_dic.items():
            limited = self.hand.map_to_limit_angle(motor_id, deg, dir=True)
            final_angles[motor_id] = round(float(limited), 2)
        try:
            with self.hand_lock:
                ordered_angles = dict(sorted(final_angles.items()))
                self.hand.set_position(ordered_angles)
        except Exception as e:
            print(e)
        if self.is_slave == False:
            time.sleep(0.06)


    def hand_control_cb_angle(self, msg):
        pass

    def pub_joint_state(self):
        while rclpy.ok():
            if self.is_commond == True: # 如果接收到了命令，则停止当前控制
                continue
            try:
                with self.hand_lock:
                    t1 = time.time()
                    # 获取手指状态角度值和范围值
                    state_angle, state_range = self.hand.get_status(range=True)
                    aligned_angle = [state_angle[k+1] for k in IDX_MAP]
                    aligned_range   = [state_range[k+1]   for k in IDX_MAP]
                    if self.count % 3 == 0:
                        tmp_speed = self.hand.get_all_velocity()
                        self.all_speed = [round(abs(tmp_speed[k+1]), 2) for k in IDX_MAP]
                    if self.is_angle == True:
                        msg = self.joint_state_msg(pose=aligned_angle, vel=self.all_speed)
                        self.hand_state_pub_angle.publish(msg)
                    else:
                        msg = self.joint_state_msg(pose=aligned_range, vel=self.all_speed)
                        self.hand_state_pub.publish(msg)
                    if self.count % 4 == 0: # 获取手指状态电流值
                        tmp_current = self.hand.get_all_current()
                        self.all_current = [tmp_current[k+1] for k in IDX_MAP]
                    if self.count == 10: # 获取手指状态温度值
                        tmp_temperature = self.hand.get_all_temperature()
                        self.all_temperature = [tmp_temperature[k+1] for k in IDX_MAP]
                    if self.count == 15: # 获取手指状态扭矩值
                        tmp_torque = self.hand.get_all_torque()
                        self.all_torque = [tmp_torque[k+1] for k in IDX_MAP]
                    if self.count == 20: # 获取手指状态电压值
                        tmpl_voltage = self.hand.get_all_voltage()
                        self.all_voltage = [tmpl_voltage[k+1] for k in IDX_MAP]
                        self.count = 0
                    info_msg = String()
                    data = {
                        "current": self.all_current,
                        "temperature": self.all_temperature,
                        "voltage":self.all_voltage,
                        "torque": self.all_torque,

                    }
                    info_msg.data = json.dumps(data)
                    self.hand_info_pub.publish(info_msg)
                    
                    #print(time.time()-t1, flush=True)
            except:
                pass
            self.count += 1
            time.sleep(0.03)
        
    


    def close_sdk(self) -> None:
        # 断开连接
        self.hand.disconnect()
        time.sleep(0.1)
        ColorMsg(msg=f"正在终止程序....", color="red")




def main(args=None) -> None:
    rclpy.init(args=args)
    node = LinkerHandO20("linker_hand_o20")
    try:
        #node.run()
        rclpy.spin(node)         # 主循环，监听 ROS 回调
    except KeyboardInterrupt:
        print("收到 Ctrl+C，准备退出...")
    finally:
        node.close_sdk()         # 关闭 CAN 或其他硬件资源
        node.destroy_node()      # 销毁 ROS 节点
        if rclpy.ok():
            rclpy.shutdown()
        print("程序已退出。")
