#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""

"""
import time,sys,os
from typing import List, Dict, Optional, Union
from .utils.colos_msg import ColorMsg
from .core.linker_hand_o20_u2d2 import LinerHandO20U2D2


SDK_VERSION = "1.1.1" # SDK版本号 新增使用电流位置模式
# 电机ID对应的角度限制 {id: [min,median,max,direction]}  {ID: [最小值, 中间值, 最大值, 旋转方向]}
# 旋转方向只作为标记，不代表实际意义
# 最小值为正转，最大值为负转，中间值为初始点
LIMIT_TYPE = 1 # 0: 最大值限制 1: 中间值值限制
MOTOR_LIMIT_LEFT = {
    1: [89.91, 180, 271.76, 1],
    2: [88.51, 180, 271.32, 1],
    3: [89.21, 180, 192.04, 1],
    4: [272.08, 90, 180.32, 1],
    5: [86.66, 180, 270.35, -1],
    6: [92.22, 180, 270.97, -1],
    7: [272.02, 270, 177.1, -1],
    8: [146.16, 180, 194.68, 1],
    9: [88.68, 180, 272.02, -1],
    10: [89.91, 180, 270.02, -1],
    11: [271.05, 270, 185.1, -1],
    12: [147.74, 180, 212.52, 1],
    13: [88.24, 180, 270.63, -1],
    14: [91.23, 180, 270.09, -1],
    15: [271.49, 270, 178.07, -1],
    16: [165.15, 180, 215.68, 1],
    17: [271.05, 180, 90.18, -1],
    18: [272.11, 180, 89.12, -1],
    19: [238.01, 180, 90.18, -1],
    20: [38.62, 180, 210.67, 1],
}
MOTOR_LIMIT_RIGHT = {
    1: [272.1, 180, 89.3, 1],
    2: [271.3, 180, 89.4, 1],
    3: [271.5, 180, 168.0, 1],
    4: [89.2, 90, 180.6, 1],
    5: [88.4, 180, 270.5, -1],
    6: [90.7, 180, 271.7, -1],
    7: [270.1, 270, 160.5, -1],
    8: [215.0, 180, 164.4, 1],
    9: [88.6, 180, 270.4, -1],
    10: [90.0, 180, 271.6, -1],
    11: [272.3, 270, 185.8, -1],
    12: [213.0, 180, 146.7, 1],
    13: [89.3, 180, 271.2, -1],
    14: [90.1, 180, 272.3, -1],
    15: [270.7, 270, 177.6, -1],
    16: [193.3, 180, 143.0, 1],
    17: [88.07, 180, 271.58, -1],
    18: [87.71, 180, 270.7, -1],
    19: [119.5, 180, 271.5, -1],
    20: [331.0, 180, 149.0, 1],
}
class LinkerHandO20API:
    def __init__(self,port: str,hand_type = "right"):
        self.hand_type = hand_type
        self.version = SDK_VERSION
        if hand_type == "right":
            self.motor_limit = MOTOR_LIMIT_RIGHT
        else:
            self.motor_limit = MOTOR_LIMIT_LEFT
        self.hand = LinerHandO20U2D2(port=port)
        self.is_teleoperated = False
        self._connect()

    def _connect(self)->list:
        ColorMsg(msg=f"{self.hand_type}-扫描电机中....", color="green")
        self.ids = self.hand.scan_ids(print_progress=True)
        if len(self.ids) != 0:
            # 1 电机全部失能
            self.motor_disable_all()
            time.sleep(0.5)
            # 1.1 设置电流，需要断电重启
            self.hand.set_currents_safe()
            time.sleep(0.5)
            # 2 电机切换到电流位置模式 
            self.set_all_current_position_mode()
            time.sleep(0.5)
            # 3 批量设置PID
            pid = {}
            for id in range(len(self.ids)):
                pid[id+1] = {"P": 300, "I": 0, "D": 600}
            self.set_pos_pid(pid)
            time.sleep(1)
            print(self.get_pos_pid(self.ids))
            time.sleep(1)
            # 3 电机全部使能
            self.motor_enable_all()



    def motor_enable_all(self):
        """所有电机使能"""
        self.hand.set_torques(True)
        for id in range(len(self.ids)):
            a = self.hand.read_torque(self.ids[id])
            if a == True:
                ColorMsg(msg=f"电机ID:{self.hand_type}-{self.ids[id]} 以在线，使能状态", color="green")
            else:
                ColorMsg(msg=f"电机ID:{self.hand_type}-{self.ids[id]} 失能状态", color="red")

    def motor_enable_by_id(self,id: int):
        """根据电机ID使能"""
        self.hand.set_torque(id,True)

    def motor_disable_all(self):
        """所有电机失能"""
        self.hand.set_torques(False)
        for id in range(len(self.ids)):
            a = self.hand.read_torque(self.ids[id])
            if a == True:
                ColorMsg(msg=f"电机ID:{self.hand_type}-{self.ids[id]} 以在线，使能状态", color="green")
            else:
                ColorMsg(msg=f"电机ID:{self.hand_type}-{self.ids[id]} 失能状态", color="red")

    def set_pos_pid(self, pid_dict: Dict[int, Dict[str, int]]):
        """
        批量设置位置 PID
        pid_dict = {
            1: {"P": 800, "I": 0, "D": 0},
            2: {"P": 640, "I": 0, "D": 0},
        }
        """
        self.hand.set_pos_pid_sync(pid_dict)

    def get_pos_pid(self, ids: List[int]) -> Dict[int, Dict[str, int]]:
        """
        批量读取位置环 PID 增益
        返回: {id: {"P": int, "I": int, "D": int}}
        使用 GroupSyncRead，自动 ≤16 分段
        """
        return self.hand.get_pos_pid_sync(ids)

    def map_to_limit_angle(self, motor_id: int, degree_0_255: float, dir=True) -> float:
        """根据电机ID和范围值转换为限位内角度值"""
        if motor_id not in self.motor_limit:
            raise ValueError(f"电机 ID {motor_id} 不在限位表 {self.motor_limit} 中")
        lo, mid, hi, direction = self.motor_limit[motor_id]
        degree_0_255 = max(0.0, min(255.0, degree_0_255))
        if LIMIT_TYPE == 1:
            if motor_id == 4 or motor_id == 7 or motor_id == 11 or motor_id == 15 or motor_id == 16 or motor_id == 19 or motor_id == 8 or motor_id == 12:
                hi = hi
            else:
                hi = mid
        if motor_id == 15 or motor_id == 11 or motor_id == 7:
            dir = False

        if dir == True:          # 0→lo，255→hi
            angle = lo + (hi - lo) * degree_0_255 / 255.0
        else:                       # 0→hi，255→lo
            angle = hi - (hi - lo) * degree_0_255 / 255.0

        # 正确 clamp
        lower, upper = sorted((lo, hi))
        angle = max(lower, min(upper, angle))
        return angle
    
    
    def angle_to_0_255(self, motor_id: int, angle: float, dir: bool = True) -> float:
        """
        将实际角度反算回 0-255 的整型范围（浮点返回值，可再 round/int）
        是 map_to_limit_angle 的严格逆运算。
        """
        if motor_id not in self.motor_limit:
            raise ValueError(f"电机 ID {motor_id} 不在限位表 {self.motor_limit} 中")

        lo, mid, hi, direction = self.motor_limit[motor_id]

        # 1. 与 map_to_limit_angle 保持同一套限幅逻辑
        if LIMIT_TYPE == 1:
            if motor_id in {4, 7, 11, 15, 16, 19,8,12}:
                hi = hi          # 保持最大限位
            else:
                hi = mid         # 否则压到 mid

        # 2. 方向翻转特殊电机
        if motor_id in {15, 11, 7}:
            dir = False

        # 3. 角度 clamp 到 [lo, hi]（考虑方向）
        lower, upper = sorted((lo, hi))
        angle = max(lower, min(upper, angle))

        # 4. 反算 0-255
        if dir:                      # 0→lo，255→hi
            ratio = (angle - lo) / (hi - lo)
        else:                        # 0→hi，255→lo
            ratio = (hi - angle) / (hi - lo)

        return max(0.0, min(255.0, ratio * 255.0))
    """===========================设置方法===================================="""
    #set_currents
    # def set_currents(self, currents: Dict[int, int]) -> None:
    #     # if len(currents) < 20:
    #     #     raise ValueError(f"数据长度错误:{currents}")
    #     self.hand.set_currents_sync(curr=currents)
    # def set_currents_safe(self):
    #     """设置默认电流为80% 尽量不让电机过流失能"""
    #     self.hand.set_currents_safe()

    def set_position(self,position: Dict[int, float]) -> None:
        if len(position) < 20:
            raise ValueError(f"数据长度错误:{position}")
        self.hand.set_angles_sync(deg=position)
        time.sleep(0.005)

    def set_position_single(self,position: Dict[int, float]) -> None:
        """单个电机设置位置"""
        self.hand.set_angles(deg=position)


    """===========================获取状态方法===================================="""
    def get_status(self, range=False) -> Dict[int, float]:
        """获取所有电机状态"""
        tmp_state_angle = self.hand.read_all_angle_sync_safe(ids=self.ids)
        if tmp_state_angle[9] > 270 and tmp_state_angle[13] > 270 and tmp_state_angle[5] > 270:
            self.is_teleoperated = True
        if tmp_state_angle[9] > 270 and tmp_state_angle[13] > 270 and tmp_state_angle[5] < 88:
            self.is_teleoperated = False
        tmp_state_range = None
        if range is True:
            tmp_state_range = {id: round(self.angle_to_0_255(id, ang), 1) for id, ang in tmp_state_angle.items()}
        return tmp_state_angle, tmp_state_range

    def get_motor_state_by_id(self,ids: List[int]) -> Dict[int, float]:
        """根据电机ID获取电机当前角度 0°~360°"""
        id_state_dic = self.hand.read_all_angle_sync_safe(ids=ids)
        return id_state_dic
        
    

    def get_all_current(self) -> Dict[int, int]:
        """获取所有电机的电流"""
        current = self.hand.read_all_current_sync(ids=self.ids)
        return current
    def get_all_temperature(self) -> Dict[int, float]:
        """获取所有电机的温度"""
        temperature = self.hand.read_all_temperature_sync(ids=self.ids)
        return temperature
    def get_all_torque(self) -> Dict[int, bool]:
        """获取所有电机的扭矩状态"""
        torque = self.hand.read_all_torque_sync(ids=self.ids)
        return torque
    
    def get_all_voltage(self) -> Dict[int, float]:
        """获取所有电机的电压"""
        voltage = self.hand.read_all_voltage_sync(ids=self.ids)
        return voltage
    
    def get_all_velocity(self) -> Dict[int, float]:
        """获取所有电机的速度"""
        velocity = self.hand.read_all_velocity_sync(ids=self.ids)
        return velocity
    
    # def set_speed(self,speed={}):
    #     self.hand.set_velocity_limits(limit_dict=speed)
    
    def get_touch_type(self):
        """获取压感类型"""
        touch_type = self.hand.read_force_all()
        tmp_count = 0
        for touch in touch_type:
            if touch == -1:
                tmp_count += 1
        if tmp_count > 3:
            return -1
        else:
            return 1
        
    def get_touch(self):
        """获取压感信息"""
        return self.hand.read_force_all()
    


    def clear_error(self, motor_id: int) -> bool:
        
        return self.hand.clear_error(motor_id=motor_id)
    def set_all_current_position_mode(self) -> None:
        """批量切换到电流-位置模式（0x05）"""
        ColorMsg(msg="切换至电流-位置模式", color="yellow")
        self.hand.set_all_current_position_mode()

    def disconnect(self):
        try:
            self.hand.set_torques(False)
        except RuntimeError as e:
            print(f"[WARN] 关扭矩时串口忙，直接关闭端口: {e}")
        self.hand.close()


    



if __name__ == '__main__':
    init_pose = {1: 255.0, 2: 254.7, 3: 255.0, 4: 0.0, 5: 255.0, 6: 255.0, 7: 240.9, 8: 255.0, 9: 249.8, 10: 249.0, 11: 255.0, 12: 235.3, 13: 248.8, 14: 255.0, 15: 255.0, 16: 228.0, 17: 255.0, 18: 255.0, 19: 255.0, 20: 255.0}
    
    O20 = LinkerHandO20API("/dev/O20_right")
    time.sleep(2)
    # 电机失能
    # O20.motor_disable_all()
    # time.sleep(3)
    # # 电机使能
    # O20.motor_enable_all()

    # 将初始位置范围值按照电机限位表转为角度值
    cmd_0_255 = {id: O20.map_to_limit_angle(id, angle) for id, angle in init_pose.items()}
    t1 = time.time()
    O20.set_position(cmd_0_255) # 此时耗时0.0023秒
    state_angle, state_range = O20.get_status(range=True)  # 此时耗时0.039秒
    print(time.time()-t1)

    
    print(f"当前角度状态值:{state_angle}")
    print(f"当前范围状态值:{state_range}")


    # # 打印看看
    # for id, cmd in cmd_0_255.items():
    #     print(f"ID{id}: {cmd:.2f}")
    # tmp = {}
    # for key,value in state.items():
    #     if value > 180:
    #         v = -(value - 360)
    #     else:
    #         if key == 20 or key == 3:
    #             v = 0 - value
    #         else:
    #             v = value - 0
    #     tmp[key] = v
    #     O20.hand.set_angle(key,v)
    #     state = O20.get_status()

    # time.sleep(1)
    # t1 = time.time()
    # state = O20.get_status()
    # print(f"状态：{state}，耗时：{time.time()-t1}")
    # time.sleep(5)
    #O20.motor_disable_all()
