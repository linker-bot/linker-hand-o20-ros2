#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
O20灵巧手上位机控制程序
基于CANFD协议的17自由度灵巧手控制系统

作者: hejianxin
版本: 1.0.2
日期: 2026-03-30
"""

import sys
import os
import time
import threading
import logging
import json
import configparser
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass, asdict
from enum import Enum
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import numpy as np
from datetime import datetime
#from tactile_window import TactileSensorWindow

# 添加python3.11目录到路径
#sys.path.append(os.path.join(os.path.dirname(__file__), 'python3.11'))

# 导入CANFD库
from ctypes import *

# CANFD库常量和结构体定义
STATUS_OK = 0

class CanFD_Config(Structure):
    _fields_ = [("NomBaud", c_uint),
                ("DatBaud", c_uint),
                ("NomPres", c_ushort),
                ("NomTseg1", c_char),
                ("NomTseg2", c_char),
                ("NomSJW", c_char),
                ("DatPres", c_char),
                ("DatTseg1", c_char),
                ("DatTseg2", c_char),
                ("DatSJW", c_char),
                ("Config", c_char),
                ("Model", c_char),
                ("Cantype", c_char)]

class CanFD_Msg(Structure):
    _fields_ = [("ID", c_uint),
                ("TimeStamp", c_uint),
                ("FrameType", c_ubyte),
                ("DLC", c_ubyte),
                ("ExternFlag", c_ubyte),
                ("RemoteFlag", c_ubyte),
                ("BusSatus", c_ubyte),
                ("ErrSatus", c_ubyte),
                ("TECounter", c_ubyte),
                ("RECounter", c_ubyte),
                ("Data", c_ubyte*64)]

class Dev_Info(Structure):
    _fields_ = [("HW_Type", c_char*32),
                ("HW_Ser", c_char*32),
                ("HW_Ver", c_char*32),
                ("FW_Ver", c_char*32),
                ("MF_Date", c_char*32)]

# 协议常量定义
class DeviceID(Enum):
    RIGHT_HAND = 0x01
    LEFT_HAND = 0x02

# 数据单位转换常量 - 根据协议规范v2.0
POSITION_UNIT = 90.0 / 140.0  # 位置单位：140代表90度 (约0.643度/LSB)
VELOCITY_UNIT = 0.732  # 速度单位：0.732RPM/LSB

class RegisterAddress(Enum):
    SYS_DEVICE_INFO = 0x00
    SYS_CALI_MODE = 0x01
    SYS_ERROR_STATUS = 0x02
    SYS_CURRENT_POS = 0x03
    SYS_CURRENT_VEL = 0x04
    SYS_CONFIG_STATUS = 0x05
    SYS_TARGET_POS = 0x06
    SYS_TARGET_VEL = 0x07
    SYS_TARGET_TORQUE = 0x08  # 目标力矩寄存器
    TACTILE_THUMB_DATA1 = 0x09
    TACTILE_THUMB_DATA2 = 0x0A
    TACTILE_INDEX_DATA1 = 0x0B
    TACTILE_INDEX_DATA2 = 0x0C
    TACTILE_MIDDLE_DATA1 = 0x0D
    TACTILE_MIDDLE_DATA2 = 0x0E
    TACTILE_RING_DATA1 = 0x0F
    TACTILE_RING_DATA2 = 0x10
    TACTILE_PINKY_DATA1 = 0x11
    TACTILE_PINKY_DATA2 = 0x12
    SYS_TEMP_DATA = 0x13  # 温度寄存器
    SYS_MOTOR_CURRENT = 0x15  # 电机电流寄存器
    SYS_JOINT_OFFSET = 0x16  # 关节位置偏差寄存器（可读写，写入后自动保存到Flash）
    SYS_OC_PROT = 0x20  # 过流保护值寄存器
    SYS_OC_PROT_TIME = 0x21  # 过流保护时间寄存器
    SYS_SERIAL_NUMBER = 0x6E  # 设备信息修改寄存器（可读写，写入后自动保存到Flash）

# 关节信息定义
@dataclass
class JointInfo:
    id: int
    name: str
    finger: str
    min_pos: int = -32768
    max_pos: int = 32767
    current_pos: int = 0
    target_pos: int = 0
    current_vel: int = 0
    target_vel: int = 0
    target_acc: int = 0
    current_temp: int = 0  # 当前温度
    current_current: int = 0  # 当前电流 (mA)
    joint_offset: int = 0  # 关节位置偏差（单位：0.087度）
    error_status: int = 0
    config_status: int = 0
    oc_prot: int = 220  # 过流保护阈值 (mA)
    oc_prot_time: int = 110  # 过流保护时间 (ms)
    tactile_data = {
            'thumb': np.zeros((12, 6)),
            'index': np.zeros((12, 6)),
            'middle': np.zeros((12, 6)),
            'ring': np.zeros((12, 6)),
            'pinky': np.zeros((12, 6))
        }

# 手指关节定义 - 按照协议规范v2.0的电机ID分配
JOINT_DEFINITIONS = [
    # 拇指 (4 DOF) - 电机ID 1-4
    JointInfo(1, "指根弯曲", "拇指", 0, 120),      # 电机ID:1 THUMB_MCP (代码: MAX=120)
    JointInfo(2, "指尖弯曲", "拇指", 0, 150),       # 电机ID:2 THUMB_IP (代码: MAX=150)
    JointInfo(3, "侧摆", "拇指", 0, 180),          # 电机ID:3 THUMB_ABD (代码: MAX=180)
    JointInfo(4, "旋转", "拇指", 0, 130),          # 电机ID:4 THUMB_CMC (代码: MAX=130)

    # 食指 (3 DOF) - 电机ID 5-7
    JointInfo(5, "侧摆运动", "食指", -30, 30),        # 电机ID:5 INDEX_ABD
    JointInfo(6, "指根弯曲", "食指", 0, 180),   # 电机ID:6 INDEX_MCP
    JointInfo(7, "指尖弯曲", "食指", 0, 180),   # 电机ID:7 INDEX_PIP

    # 中指 (3 DOF) - 电机ID 8-10
    JointInfo(8, "侧摆", "中指", -30, 30),        # 电机ID:8 MIDDLE_ABD
    JointInfo(9, "指根弯曲", "中指", 0, 180),     # 电机ID:9 MIDDLE_MCP
    JointInfo(10, "指尖弯曲", "中指", 0, 180),     # 电机ID:10 MIDDLE_PIP

    # 无名指 (3 DOF) - 电机ID 11-13
    JointInfo(11, "侧摆运动", "无名指", -20, 20),        # 电机ID:11 RING_ABD
    JointInfo(12, "指根弯曲", "无名指", 0, 180),   # 电机ID:12 RING_MCP
    JointInfo(13, "指尖弯曲", "无名指", 0, 180),   # 电机ID:13 RING_PIP

    # 小指 (3 DOF) - 电机ID 14-16
    JointInfo(14, "侧摆", "小指", -20, 20),        # 电机ID:14 PINKY_ABD
    JointInfo(15, "指根弯曲", "小指", 0, 180),    # 电机ID:15 PINKY_MCP
    JointInfo(16, "指尖弯曲", "小指", 0, 180),    # 电机ID:16 PINKY_DIP

    # 手腕 (1 DOF) - 电机ID 17
    JointInfo(17, "俯仰", "手腕", -1000, 1000),        # 电机ID:17 HAND_WRITE
]

class CANFDCommunication:
    """CANFD通信类"""

    def __init__(self,hand_type="right", canfd_device=0):
        self.hand_type=hand_type
        self.canDLL = None
        self.channel = 0
        self.canfd_device = canfd_device
        if self.hand_type == "left":
            self.device_id = DeviceID.LEFT_HAND.value
        else:
            self.device_id = DeviceID.RIGHT_HAND.value
        self.is_connected = False
        self.dlc2len = [0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64]

    def initialize(self) -> bool:
        """初始化CANFD通信"""
        try:
            CDLL("/usr/local/lib/libusb-1.0.so", RTLD_GLOBAL)
            time.sleep(0.1)  # 确保库加载完成
            self.canDLL = cdll.LoadLibrary("/usr/local/lib/libcanbus.so")  #动态库路径
            # 扫描设备（添加超时控制）
            print("=" * 50, flush=True)
            print("开始扫描CANFD设备...")
            start_time = time.time()
            try:
                ret = self.canDLL.CAN_ScanDevice()
                scan_time = time.time() - start_time

                print(f"扫描完成: 找到 {ret} 个设备 (耗时: {scan_time:.3f}s)")
                if ret <= 0:
                    print("❌ 错误: 未找到CANFD设备")
                    print("   请检查:")
                    print("   1. CANFD适配器是否连接")
                    print("   2. 设备驱动是否安装")
                    print("   3. 设备是否被其他程序占用")
                    return False
                else:
                    print(f"✅ 成功找到 {ret} 个CANFD设备")

            except Exception as e:
                scan_time = time.time() - start_time
                print(f"❌ 扫描设备异常 (耗时: {scan_time:.3f}s): {e}")
                return False

            # 打开设备（添加超时控制）
            print(f"正在打开CANFD-{self.canfd_device}号设备通道 {self.channel}...")
            start_time = time.time()

            try:
                ret = self.canDLL.CAN_OpenDevice(self.canfd_device, self.channel)
                # a = self.canDLL.CAN_GetDevID(0, self.channel)
                # print("_-" * 30)
                # print(a)
                # return
                open_time = time.time() - start_time

                if ret != STATUS_OK:
                    print(f"❌ 打开设备失败，错误码: {ret} (耗时: {open_time:.3f}s)")
                    print(f"   可能原因:")
                    print(f"   1. 设备通道 {self.channel} 不存在")
                    print(f"   2. 设备已被其他程序占用")
                    print(f"   3. 设备权限不足")
                    return False
                else:
                    print(f"✅ CANFD-{self.canfd_device}号设备通道 {self.channel} 打开成功 (耗时: {open_time:.3f}s)")
            except Exception as e:
                open_time = time.time() - start_time
                print(f"❌ 打开设备异常 (耗时: {open_time:.3f}s): {e}")
                return False
            # 读取设备信息
            self._read_device_info()
            # 配置CANFD参数
            print("正在配置CANFD参数...")
            print("  仲裁段波特率: 1Mbps")
            print("  数据段波特率: 5Mbps")

            start_time = time.time()

            try:
                # 1Mbps仲裁段，5Mbps数据段
                can_config = CanFD_Config(
                    1000000,  # NomBaud: 仲裁段波特率 1Mbps
                    5000000,  # DatBaud: 数据段波特率 5Mbps
                    0x0,      # NomPres: 仲裁段预分频
                    0x0,      # NomTseg1: 仲裁段时间段1
                    0x0,      # NomTseg2: 仲裁段时间段2
                    0x0,      # NomSJW: 仲裁段同步跳转宽度
                    0x0,      # DatPres: 数据段预分频
                    0x0,      # DatTseg1: 数据段时间段1
                    0x0,      # DatTseg2: 数据段时间段2
                    0x0,      # DatSJW: 数据段同步跳转宽度
                    0x04,      # Config: 配置标志
                    0x0,      # Model: 模式
                    0x1       # Cantype: CANFD类型
                )

                ret = self.canDLL.CANFD_Init(self.canfd_device, self.channel, byref(can_config))
                config_time = time.time() - start_time
                if ret != STATUS_OK:
                    print(f"❌ CANFD初始化失败，错误码: {ret} (耗时: {config_time:.3f}s)")
                    print("   正在关闭设备...")
                    self.canDLL.CAN_CloseDevice(self.canfd_device, self.channel)
                    return False
                else:
                    print(f"✅ CANFD配置成功 (耗时: {config_time:.3f}s)")

            except Exception as e:
                config_time = time.time() - start_time
                print(f"❌ CANFD配置异常 (耗时: {config_time:.3f}s): {e}")
                try:
                    self.canDLL.CAN_CloseDevice(self.canfd_device, self.channel)
                except:
                    pass
                return False
            # 设置接收过滤器（接收所有消息）
            print("正在设置接收过滤器...")
            start_time = time.time()

            try:
                ret = self.canDLL.CAN_SetFilter(self.canfd_device,self.channel, 0, 0, 0, 0, 1)
                filter_time = time.time() - start_time

                if ret != STATUS_OK:
                    print(f"❌ 设置过滤器失败，错误码: {ret} (耗时: {filter_time:.3f}s)")
                    print("   正在关闭设备...")
                    self.canDLL.CAN_CloseDevice(self.canfd_device,self.channel)
                    return False
                else:
                    print(f"✅ 过滤器设置成功 (耗时: {filter_time:.3f}s)")

            except Exception as e:
                filter_time = time.time() - start_time
                print(f"❌ 设置过滤器异常 (耗时: {filter_time:.3f}s): {e}")
                try:
                    self.canDLL.CAN_CloseDevice(self.canfd_device, self.channel)
                except:
                    pass
                return False

            self.is_connected = True
            print("✅ CANFD通信初始化完成")
            print("=" * 50)
            return True

        except OSError as e:
            if "193" in str(e):
                print("错误: DLL架构不匹配")
                print("请确认CANFD库文件与Python架构匹配")
            else:
                print(f"错误: 加载CANFD库失败: {e}")
            return False
        except Exception as e:
            print(f"CANFD初始化失败: {e}")
            return False
        

    def query_device_type(self) -> Optional[str]:
        """查询设备类型（左手/右手）"""
        if not self.is_connected:
            print("❌ 设备未连接，无法查询设备类型")
            return None

        print("🔍 正在查询设备类型...")
        right_hand_response = None
        left_hand_response = None
        # 尝试查询右手设备
        if self.hand_type == "right":
            print("   查询右手设备 (ID: 0x01)...")
            right_hand_response = self._query_single_device(DeviceID.RIGHT_HAND.value)
        
        if self.hand_type == "left":
            # 尝试查询左手设备
            print("   查询左手设备 (ID: 0x02)...")
            left_hand_response = self._query_single_device(DeviceID.LEFT_HAND.value)
        # 分析响应结果
        if right_hand_response and left_hand_response:
            print("⚠️ 检测到左右手设备都有响应，默认选择右手")
            self.device_id = DeviceID.RIGHT_HAND.value
            return "右手"
        elif right_hand_response:
            print("✅ 检测到右手设备")
            self.device_id = DeviceID.RIGHT_HAND.value
            return "右手"
        elif left_hand_response:
            print("✅ 检测到左手设备")
            self.device_id = DeviceID.LEFT_HAND.value
            return "左手"
        else:
            print("❌ 未检测到任何灵巧手设备响应")
            return None

    def _query_single_device(self, device_id: int) -> bool:
        """查询单个设备是否存在"""
        try:
            # 多次尝试查询
            for attempt in range(3):
                print(f"     尝试 {attempt + 1}/3...")

                # 临时设置设备ID用于发送
                original_device_id = self.device_id
                self.device_id = device_id
                # 发送设备信息查询命令
                success = self.send_message(RegisterAddress.SYS_DEVICE_INFO.value, b'', False)
                
                # 恢复原始设备ID
                self.device_id = original_device_id

                if not success:
                    print(f"     发送查询命令失败")
                    continue

                # 等待响应
                import time
                time.sleep(0.1)  # 100ms等待
                # 接收响应 - 不过滤设备ID，接收所有消息
                messages = self.receive_messages(200, filter_device_id=False)
                
                # 检查是否有来自目标设备的响应
                for frame_id, data in messages:
                    response_device_id = (frame_id >> 21) & 0xFF
                    register_addr = (frame_id >> 13) & 0xFF

                    if (response_device_id == device_id and
                        register_addr == RegisterAddress.SYS_DEVICE_INFO.value and
                        len(data) > 0):
                        print(f"     ✅ 设备 0x{device_id:02X} 响应正常 (数据长度: {len(data)})")

                        # 检查数据是否全为0
                        if all(b == 0 for b in data):
                            print(f"     ⚠️ 设备信息数据全为0，可能设备信息未初始化")
                            # 即使数据为0，也认为设备存在并响应
                            # 根据查询的设备ID来判断类型
                            device_type = "右手" if device_id == 0x01 else "左手"
                            print(f"     根据查询ID判断设备类型: {device_type}")
                            return True
                        else:
                            # 解析设备信息
                            try:
                                if len(data) >= 50:
                                    product_model = data[0:10].decode('utf-8', errors='ignore').strip('\x00')
                                    # 手型标志位在第51字节（索引50）：1=右手，2=左手
                                    hand_type = "右手" if len(data) > 50 and data[50] == 1 else "左手"
                                    print(f"     设备信息: {product_model}, 类型: {hand_type}")
                                else:
                                    print(f"     数据长度不足，无法解析设备信息")
                            except Exception as e:
                                print(f"     解析设备信息失败: {e}")
                            return True

                print(f"     第 {attempt + 1} 次查询无响应")
                time.sleep(0.1)  # 重试间隔

            print(f"     ❌ 设备 0x{device_id:02X} 无响应 (已尝试3次)")
            return False

        except Exception as e:
            print(f"     ❌ 查询设备 0x{device_id:02X} 异常: {e}")
            return False

        except OSError as e:
            if "193" in str(e):
                print("错误: DLL架构不匹配")
                print("请确认CANFD库文件与Python架构匹配")
            else:
                print(f"错误: 加载CANFD库失败: {e}")
            return False
        except Exception as e:
            print(f"CANFD初始化失败: {e}")
            return False

    def _read_device_info(self):
        """读取并显示设备信息"""
        try:
            devinfo = Dev_Info()
            
            ret = self.canDLL.CAN_ReadDevInfo(self.canfd_device, byref(devinfo))
            if ret == STATUS_OK:
                print("\n设备信息:")
                print(f"  设备型号: {devinfo.HW_Type.decode('utf-8', errors='ignore').strip()}")
                print(f"  序列号  : {devinfo.HW_Ser.decode('utf-8', errors='ignore').strip()}")
                print(f"  硬件版本: {devinfo.HW_Ver.decode('utf-8', errors='ignore').strip()}")
                print(f"  固件版本: {devinfo.FW_Ver.decode('utf-8', errors='ignore').strip()}")
                print(f"  生产日期: {devinfo.MF_Date.decode('utf-8', errors='ignore').strip()}")
                print()
            else:
                print("警告: 无法读取设备信息")
        except Exception as e:
            print(f"读取设备信息失败: {e}")

    def create_frame_id(self, device_id: int, register_addr: int, is_write: bool) -> int:
        """创建CANFD扩展帧ID
        
        按照协议规范v2.0的29位扩展帧格式：
        - Bit 28-21: 设备ID (Device ID) - 8位
        - Bit 20-13: 寄存器地址 (Register Address) - 8位
        - Bit 12: 读写标志位 (R/W Flag) - 1位 (0=读, 1=写)
        - Bit 11-0: 保留位 (Reserved) - 12位，默认为0
        """
        frame_id = (device_id << 21) | (register_addr << 13) | ((1 if is_write else 0) << 12)
        return frame_id

    def send_message(self, register_addr: int, data: bytes, is_write: bool = True, target_device_id: Optional[int] = None) -> bool:
        """发送CANFD消息"""
        if not self.is_connected:
            print("错误: CANFD未连接")
            return False

        try:
            # 入口调试：确认每次调用 send_message 的参数
            try:
                print(f"     调用 send_message: 寄存器=0x{register_addr:02X}, is_write={is_write}, data_len={len(data)}")
            except Exception:
                print("     调用 send_message: 参数打印失败")
            # 允许调用方覆盖目标 device_id（向后兼容）
            device_for_frame = target_device_id if target_device_id is not None else self.device_id
            frame_id = self.create_frame_id(device_for_frame, register_addr, is_write)

            # 调试：打印构造后的 frame_id 及解码信息，便于定位 ID 错误
            try:
                decoded_dev = (frame_id >> 21) & 0xFF
                decoded_reg = (frame_id >> 13) & 0xFF
                decoded_rw = (frame_id >> 12) & 0x1
                print(f"     调试: 发送帧 -> ID:0x{frame_id:08X}, Decoded Device:0x{decoded_dev:02X}, Reg:0x{decoded_reg:02X}, R/W:{decoded_rw}")
            except Exception:
                pass

            # 对于SYS_SERIAL_NUMBER寄存器，写入操作时
            # 读取操作时允许空数据
            if register_addr == RegisterAddress.SYS_SERIAL_NUMBER.value:
                if is_write:
                    # 写入操作：允许最长64字节
                    data_len = min(len(data), 64)
                    # 计算DLC
                    dlc = self._get_dlc_from_length(data_len)
                    print(f"     📤 写入设备信息: 数据长度={data_len}字节，DLC={dlc}")
                else:
                    # 读取操作：允许空数据
                    data_len = len(data)  # 读取时可以是0字节
                    dlc = self._get_dlc_from_length(data_len) if data_len > 0 else 0
            else:
                # 限制数据长度
                data_len = min(len(data), 64)
                # 计算DLC (Data Length Code)
                dlc = self._get_dlc_from_length(data_len)

            # 创建数据数组并初始化为0
            data_array = (c_ubyte * 64)()
            for i in range(64):
                data_array[i] = 0

            # 填充实际数据
            for i, byte_val in enumerate(data[:data_len]):
                data_array[i] = byte_val

            # 创建消息
            msg = CanFD_Msg(
                frame_id,     # ID
                0,            # TimeStamp
                4,            # FrameType (CANFD)
                dlc,          # DLC
                1,            # ExternFlag (扩展帧)
                0,            # RemoteFlag
                0,            # BusSatus
                0,            # ErrSatus
                0,            # TECounter
                0,            # RECounter
                data_array    # Data
            )

            # 发送消息
            # 调试打印：在发送前显示要发送的帧信息（frame_id, dlc, 数据预览）
            try:
                sent_data_preview = bytes(data_array[:self.dlc2len[dlc]]) if dlc < len(self.dlc2len) else bytes(data_array[:])
            except Exception:
                sent_data_preview = bytes(data_array[:])
            print(f"     调试: 发送帧 -> ID:0x{frame_id:08X}, DLC:{dlc}, 数据预览:{sent_data_preview.hex().upper()}")
            
            ret = self.canDLL.CANFD_Transmit(self.canfd_device, self.channel,  byref(msg), 1, 200)
            
            print(f"     调试: CANFD_Transmit 返回: {ret}")
            if ret >= 1:
                # 根据寄存器类型显示不同的详细信息
                if register_addr == RegisterAddress.SYS_TARGET_POS.value:
                    print(f"     ✅ 位置命令发送成功:")
                    print(f"        CAN ID: 0x{frame_id:08X}")
                    print(f"        设备ID: 0x{self.device_id:02X}")
                    print(f"        寄存器: 0x{register_addr:02X} (目标位置)")
                    print(f"        数据长度: {data_len}字节")
                    print(f"        数据内容: {data.hex().upper()}")
                elif register_addr == RegisterAddress.SYS_TARGET_VEL.value:
                    print(f"     ✅ 速度命令发送成功:")
                    print(f"        CAN ID: 0x{frame_id:08X}")
                    print(f"        设备ID: 0x{self.device_id:02X}")
                    print(f"        寄存器: 0x{register_addr:02X} (目标速度)")
                    print(f"        数据长度: {data_len}字节")
                    print(f"        数据内容: {data.hex().upper()}")
                elif register_addr == RegisterAddress.SYS_TARGET_TORQUE.value:
                    print(f"     ✅ 力矩命令发送成功:")
                    print(f"        CAN ID: 0x{frame_id:08X}")
                    print(f"        设备ID: 0x{self.device_id:02X}")
                    print(f"        寄存器: 0x{register_addr:02X} (目标力矩)")
                    print(f"        数据长度: {data_len}字节")
                    print(f"        数据内容: {data.hex().upper()}")
                elif register_addr == RegisterAddress.SYS_SERIAL_NUMBER.value:
                    print(f"     ✅ 设备信息写入命令发送成功:")
                    print(f"        CAN ID: 0x{frame_id:08X}")
                    print(f"        设备ID: 0x{self.device_id:02X}")
                    print(f"        寄存器: 0x{register_addr:02X} (SYS_SERIAL_NUMBER)")
                    print(f"        DLC: {dlc} (对应数据长度: {self.dlc2len[dlc] if dlc < len(self.dlc2len) else 'N/A'}字节)")
                    print(f"        数据长度: {data_len}字节")
                    print(f"        数据内容 (完整50字节): {data.hex().upper()}")
                    # 验证数据完整性
                    if data_len == 50:
                        print(f"        ✅ 数据长度正确: 50字节")
                    else:
                        print(f"        ⚠️ 警告: 数据长度不是50字节，实际为{data_len}字节")
                else:
                    print(f"     ✅ 消息发送成功:")
                    print(f"        CAN ID: 0x{frame_id:08X}")
                    print(f"        设备ID: 0x{self.device_id:02X}")
                    print(f"        寄存器: 0x{register_addr:02X}")
                    print(f"        数据长度: {data_len}字节")
                    if data_len > 0:
                        print(f"        数据内容: {data.hex().upper()}")
                
                return True
            elif ret < 0:
                print(f"     ❌ 发送失败:")
                print(f"        寄存器: 0x{register_addr:02X}")
                print(f"        返回值: {ret}")
                print(f"        数据长度: {data_len}字节")
                return False
            
        except Exception as e:
            print(f"发送消息异常: {e}")
            return False

    def _get_dlc_from_length(self, length: int) -> int:
        """根据数据长度获取DLC值
        
        CANFD DLC映射表：
        DLC 0-8: 对应0-8字节
        DLC 9: 12字节
        DLC 10: 16字节
        DLC 11: 20字节
        DLC 12: 24字节
        DLC 13: 32字节
        DLC 14: 48字节
        DLC 15: 64字节
        
        注意：对于50字节，必须使用DLC 15（64字节），因为CANFD不支持50字节的DLC
        """
        if length <= 8:
            return length
        elif length <= 12:
            return 9
        elif length <= 16:
            return 10
        elif length <= 20:
            return 11
        elif length <= 24:
            return 12
        elif length <= 32:
            return 13
        elif length <= 48:
            return 14
        else:
            return 15  # 64字节（对于50字节，也必须使用DLC 15）

    def flush_buffer(self):
        """清空接收缓冲区"""
        if not self.is_connected:
            return
            
        try:
            # 创建临时接收缓冲区
            from ctypes import POINTER
            class CanFD_Msg_ARRAY(Structure):
                _fields_ = [('SIZE', c_uint16), ('STRUCT_ARRAY', POINTER(CanFD_Msg))]

                def __init__(self, num_of_structs):
                    # 创建 ctypes 数组
                    self._array = (CanFD_Msg * num_of_structs)()
                    self.STRUCT_ARRAY = cast(pointer(self._array), POINTER(CanFD_Msg))
                    self.SIZE = num_of_structs

            receive_buffer = CanFD_Msg_ARRAY(1000)

            total_flushed = 0
            while True:
                # 0ms超时，非阻塞读取
                ret = self.canDLL.CANFD_Receive(self.canfd_device, self.channel, byref(receive_buffer), 1000, 0)
                
                if ret <= 0:
                    break
                total_flushed += ret
                
            if total_flushed > 0:
                print(f"     已清空接收缓冲区: 丢弃 {total_flushed} 条积压消息")
                
        except Exception as e:
            print(f"     清空缓冲区失败: {e}")

    def receive_messages(self, timeout_ms: int = 100, filter_device_id: bool = True) -> List[Tuple[int, bytes]]:
        """接收CANFD消息

        Args:
            timeout_ms: 超时时间(毫秒)
            filter_device_id: 是否过滤设备ID，False时接收所有消息
        """
        if not self.is_connected:
            return []

        try:
            # 创建接收缓冲区
            from ctypes import POINTER

            class CanFD_Msg_ARRAY(Structure):
                _fields_ = [('SIZE', c_uint16), ('STRUCT_ARRAY', POINTER(CanFD_Msg))]

                def __init__(self, num_of_structs):
                    self.STRUCT_ARRAY = cast((CanFD_Msg * num_of_structs)(), POINTER(CanFD_Msg))
                    self.SIZE = num_of_structs
                    self.ADDR = self.STRUCT_ARRAY[0]

            receive_buffer = CanFD_Msg_ARRAY(2000)

            # 接收消息
            ret = self.canDLL.CANFD_Receive(self.canfd_device, self.channel, byref(receive_buffer.ADDR), 2000, timeout_ms)

            messages = []
            if ret > 0:
                print(f"     接收到 {ret} 条消息")
                for i in range(ret):
                    msg = receive_buffer.STRUCT_ARRAY[i]

                    # 检查消息有效性
                    if msg.DLC >= len(self.dlc2len):
                        print(f"     警告: 无效的DLC值: {msg.DLC}")
                        continue

                    data_len = self.dlc2len[msg.DLC]
                    data = bytes(msg.Data[:data_len])

                    # 解析帧ID获取寄存器地址
                    response_device_id = (msg.ID >> 21) & 0xFF
                    register_addr = (msg.ID >> 13) & 0xFF
                    is_write = (msg.ID >> 12) & 0x1

                    print(f"     消息 {i+1}: ID=0x{msg.ID:08X}, 设备=0x{response_device_id:02X}, 寄存器=0x{register_addr:02X}, 长度={data_len}")
                    print(f"     数据: {data.hex().upper()}")

                    # 根据filter_device_id参数决定是否过滤
                    if not filter_device_id or response_device_id == self.device_id:
                        messages.append((msg.ID, data))
                    else:
                        print(f"     过滤掉设备0x{response_device_id:02X}的消息 (当前目标设备: 0x{self.device_id:02X})")
            else:
                print(f"     未接收到任何消息 (超时: {timeout_ms}ms)")

            return messages

        except Exception as e:
            print(f"接收消息异常: {e}")
            return []

    def close(self):
        """关闭CANFD连接"""
        if self.canDLL and self.is_connected:
            try:
                print("关闭CANFD连接...")
                self.canDLL.CAN_CloseDevice(self.canfd_device, self.channel)
                print("CANFD连接已关闭")
            except Exception as e:
                print(f"关闭CANFD连接失败: {e}")
            finally:
                self.is_connected = False

    def check_connection(self) -> bool:
        """检查连接状态"""
        if not self.is_connected or not self.canDLL:
            return False

        try:
            # 尝试发送一个简单的查询命令来检测连接
            test_result = self.send_message(RegisterAddress.SYS_ERROR_STATUS.value, b'', False)
            return test_result
        except Exception:
            return False

    def reconnect(self) -> bool:
        """重新连接"""
        print("尝试重新连接CANFD设备...")
        self.close()
        time.sleep(1)  # 等待1秒
        return self.initialize()


class SocketCANCommunication(CANFDCommunication):
    """基于 SocketCAN (python-can) 的 CANFD 通信类

    用于“透明塑封 USB 转 CANFD 设备”。该设备在 Linux 下走标准 SocketCAN，
    无需厂商私有库（libcanbus.so），通过内核 can0 接口 + python-can 收发。

    本类继承 CANFDCommunication，复用其纯逻辑方法：
        create_frame_id / _get_dlc_from_length / query_device_type / _query_single_device
    仅重写与底层传输相关的方法（initialize / send_message / receive_messages /
    flush_buffer / close / check_connection / reconnect）。

    协议帧格式与厂商设备完全一致（29 位扩展帧），因此上层 Controller 无需任何改动。
    """

    def __init__(self, hand_type="right", channel="can0",
                 bitrate=500000, dbitrate=2000000, auto_setup=True):
        self.hand_type = hand_type
        self.channel = channel
        self.bitrate = bitrate      # 仲裁段波特率，默认 500Kbps（透明塑封设备文档值）
        self.dbitrate = dbitrate    # 数据段波特率，默认 2Mbps
        self.auto_setup = auto_setup
        self.bus = None
        if self.hand_type == "left":
            self.device_id = DeviceID.LEFT_HAND.value
        else:
            self.device_id = DeviceID.RIGHT_HAND.value
        self.is_connected = False
        self.dlc2len = [0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64]

    def _setup_interface(self):
        """自动配置 can0 接口（需要 sudo 权限）

        等价于手动执行：
            sudo ip link set can0 down
            sudo ip link set can0 type can bitrate <b> dbitrate <d> fd on restart-ms 100
            sudo ip link set can0 up
            sudo ip link set can0 txqueuelen 1000
        逐条容错，失败不中断（接口可能已由用户提前配置）。

        注意：本模块顶部用 `print = lambda` 禁用了普通打印，因此这里的关键错误
        与回读信息直接写到真实 stderr，保证用户能看到（波特率不匹配是常见故障）。
        """
        import subprocess
        import sys as _sys

        def _log(msg):
            try:
                _sys.__stderr__.write(msg + "\n")
                _sys.__stderr__.flush()
            except Exception:
                pass

        cmds = [
            (["sudo", "ip", "link", "set", self.channel, "down"], True),
            (["sudo", "ip", "link", "set", self.channel, "type", "can",
              "bitrate", str(self.bitrate), "sample-point", "0.8",
              "dbitrate", str(self.dbitrate), "dsample-point", "0.75", "fd", "on"], True),
            (["sudo", "ip", "link", "set", self.channel, "up"], True),
        ]
        for c, required in cmds:
            try:
                ret = subprocess.run(c, check=False, timeout=5,
                                     stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                if ret.returncode != 0:
                    err = ret.stderr.decode(errors='ignore').strip()
                    if required:
                        _log(f"[SocketCAN] 接口配置命令失败: {' '.join(c)}\n"
                             f"            原因: {err}")
                    else:
                        _log(f"[SocketCAN] 提示: 该设备不支持的可选项已跳过: {' '.join(c)} ({err})")
                time.sleep(0.1)  # 给内核时间完成状态切换（尤其 down 之后）
            except Exception as e:
                _log(f"[SocketCAN] 接口配置命令异常: {' '.join(c)} -> {e}")

        # 回读实际生效的时序与状态，便于定位波特率不匹配 / BUS-OFF
        try:
            ret = subprocess.run(["ip", "-details", "link", "show", self.channel],
                                 check=False, timeout=5,
                                 stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            out = ret.stdout.decode(errors='ignore')
            timing = next((l.strip() for l in out.splitlines() if "bitrate" in l), "(未读到)")
            state = next((l.strip() for l in out.splitlines() if "state" in l), "")
            _log(f"[SocketCAN] 期望 bitrate={self.bitrate} dbitrate={self.dbitrate}\n"
                 f"            实际生效: {timing}\n"
                 f"            状态行  : {state}")
            if "BUS-OFF" in state or "ERROR-PASSIVE" in state:
                _log("[SocketCAN] ⚠️ 总线处于错误状态，通常是波特率与设备不一致或接线/终端电阻问题。")
        except Exception:
            pass

    def initialize(self) -> bool:
        """初始化 SocketCAN 通信"""
        try:
            import can
        except ImportError:
            print("❌ 缺少 python-can 库，请执行: pip install python-can")
            return False

        try:
            print("=" * 50, flush=True)
            print(f"开始初始化 SocketCAN 设备 (通道: {self.channel})...")

            # 自动拉起接口
            if self.auto_setup:
                print(f"正在自动配置接口 {self.channel} "
                      f"(bitrate={self.bitrate}, dbitrate={self.dbitrate}, fd on)...")
                self._setup_interface()
                time.sleep(0.2)

            # 打开总线前先确认接口是否存在，避免 python-can 抛出难懂的异常
            if not os.path.exists(f"/sys/class/net/{self.channel}"):
                import sys as _sys
                msg = (f"[SocketCAN] ❌ 未找到网络接口 {self.channel}。\n"
                       f"            该透明塑封设备需拨到 Linux 模式才会枚举为原生 CAN 接口；\n"
                       f"            若 lsusb 显示为 'STM32 Virtual ComPort' 则说明仍是串口模式。\n"
                       f"            请：1) 将 type-c 下方开关拨到 Linux 模式  2) 重新插拔USB\n"
                       f"               3) 用 `ip -br link show type can` 确认 {self.channel} 出现")
                try:
                    _sys.__stderr__.write(msg + "\n"); _sys.__stderr__.flush()
                except Exception:
                    pass
                self.is_connected = False
                self.bus = None
                return False

            # 打开 CANFD 总线
            self.bus = can.interface.Bus(channel=self.channel,
                                         interface='socketcan', fd=True)
            self.is_connected = True
            print(f"✅ SocketCAN 通道 {self.channel} 打开成功")
            print("✅ SocketCAN 通信初始化完成")
            print("=" * 50)
            return True

        except Exception as e:
            print(f"❌ SocketCAN 初始化失败: {e}")
            print("   请检查:")
            print(f"   1. 设备是否接入、type-c 下方开关是否拨到 Linux 模式")
            print(f"   2. 接口 {self.channel} 是否存在 (ip -br link show type can)")
            print(f"   3. 是否具备 sudo 权限以自动配置接口")
            self.is_connected = False
            self.bus = None
            return False

    def send_message(self, register_addr: int, data: bytes, is_write: bool = True,
                     target_device_id: Optional[int] = None) -> bool:
        """发送 CANFD 消息（SocketCAN）"""
        if not self.is_connected or self.bus is None:
            print("错误: SocketCAN 未连接")
            return False

        try:
            import can
            # 允许调用方覆盖目标 device_id（向后兼容）
            device_for_frame = target_device_id if target_device_id is not None else self.device_id
            frame_id = self.create_frame_id(device_for_frame, register_addr, is_write)

            # `cansend <id>##01<data>` 中 ## 后的 01 是 CANFD 的 BRS 标志位，
            # 已由下方 bitrate_switch=True 处理，不进入数据。
            data = bytes(data)
            data_len = min(len(data), 64)
            data = data[:data_len]

            # 将数据零填充到合法的 CANFD 帧长度（0-8,12,16,20,24,32,48,64）
            dlc = self._get_dlc_from_length(data_len)
            padded_len = self.dlc2len[dlc]
            payload = data + b'\x00' * (padded_len - data_len)

            msg = can.Message(
                arbitration_id=frame_id,
                is_extended_id=True,
                is_fd=True,
                # 关闭 BRS：实测 BRS 开启 + 5Mbps 数据段会把总线打入 BUS-OFF，
                # 设备无法应答；BRS 关闭时数据段保持仲裁波特率，稳定收发。
                # 对应可用的 `cansend can0 <id>##0<data>`（flags=0 即 BRS off）。
                bitrate_switch=False,
                data=payload,
            )
            self.bus.send(msg, timeout=0.2)

            # 打印发送的帧和数据，格式与 candump 一致，便于核对是否正确
            hex_data = ' '.join(f"{b:02X}" for b in payload)
            print(f"  {self.channel}  {frame_id:08X}  [{padded_len:02d}]  {hex_data}")
            print(f"     ✅ 消息发送成功: ID=0x{frame_id:08X}, 寄存器=0x{register_addr:02X}, "
                  f"长度={data_len}字节")
            return True

        except Exception as e:
            print(f"发送消息异常(SocketCAN): {e}")
            return False

    def receive_messages(self, timeout_ms: int = 100,
                         filter_device_id: bool = True) -> List[Tuple[int, bytes]]:
        """接收 CANFD 消息（SocketCAN）

        首帧使用给定超时等待，随后以非阻塞方式排空缓冲区内现有的所有帧，
        与厂商实现的“批量接收”语义保持一致。
        """
        if not self.is_connected or self.bus is None:
            return []

        messages = []
        try:
            timeout_s = max(0.0, timeout_ms / 1000.0)
            msg = self.bus.recv(timeout=timeout_s)
            while msg is not None:
                response_device_id = (msg.arbitration_id >> 21) & 0xFF
                register_addr = (msg.arbitration_id >> 13) & 0xFF
                data = bytes(msg.data)

                print(f"     消息: ID=0x{msg.arbitration_id:08X}, 设备=0x{response_device_id:02X}, "
                      f"寄存器=0x{register_addr:02X}, 长度={len(data)}")

                if not filter_device_id or response_device_id == self.device_id:
                    messages.append((msg.arbitration_id, data))
                else:
                    print(f"     过滤掉设备0x{response_device_id:02X}的消息 "
                          f"(当前目标设备: 0x{self.device_id:02X})")

                # 继续把缓冲区中现有的帧读完（非阻塞）
                msg = self.bus.recv(timeout=0)

            return messages

        except Exception as e:
            print(f"接收消息异常(SocketCAN): {e}")
            return messages

    def flush_buffer(self):
        """清空接收缓冲区"""
        if not self.is_connected or self.bus is None:
            return
        try:
            total = 0
            while self.bus.recv(timeout=0) is not None:
                total += 1
            if total > 0:
                print(f"     已清空接收缓冲区: 丢弃 {total} 条积压消息")
        except Exception as e:
            print(f"     清空缓冲区失败: {e}")

    def close(self):
        """关闭 SocketCAN 连接"""
        if self.bus is not None:
            try:
                print("关闭 SocketCAN 连接...")
                self.bus.shutdown()
                print("SocketCAN 连接已关闭")
            except Exception as e:
                print(f"关闭 SocketCAN 连接失败: {e}")
        self.is_connected = False
        self.bus = None

    def check_connection(self) -> bool:
        """检查连接状态"""
        return self.is_connected and self.bus is not None

    def reconnect(self) -> bool:
        """重新连接"""
        print("尝试重新连接 SocketCAN 设备...")
        self.close()
        time.sleep(1)
        return self.initialize()


# print = lambda *_, **__: None # 禁用打印
class DexterousHandModel:
    """灵巧手数据模型"""

    def __init__(self):
        self.joints = {joint.id: joint for joint in JOINT_DEFINITIONS}
        self.device_info = None
        self.calibration_mode = 0
        self.tactile_data = {
            'thumb': np.zeros((12, 6)),
            'index': np.zeros((12, 6)),
            'middle': np.zeros((12, 6)),
            'ring': np.zeros((12, 6)),
            'pinky': np.zeros((12, 6))
        }
        self.tactile_status = {
            'thumb': False,
            'index': False,
            'middle': False,
            'ring': False,
            'pinky': False
        }
        self.last_update_time = time.time()
        self.target_torques = [500] * 17 # Add target torques

    def update_joint_positions(self, positions: List[int]):
        """更新关节位置"""
        for i, pos in enumerate(positions[:17]):
            motor_id = i + 1  # 数组索引转换为电机ID (0->1, 1->2, ...)
            if motor_id in self.joints:
                self.joints[motor_id].current_pos = pos
        self.last_update_time = time.time()

    def update_joint_velocities(self, velocities: List[int]):
        """更新关节速度"""
        for i, vel in enumerate(velocities[:17]):
            motor_id = i + 1  # 数组索引转换为电机ID
            if motor_id in self.joints:
                self.joints[motor_id].current_vel = vel

    def update_joint_temperatures(self, temperatures: List[int]):
        """更新关节温度"""
        for i, temp in enumerate(temperatures[:17]):
            motor_id = i + 1  # 数组索引转换为电机ID
            if motor_id in self.joints:
                self.joints[motor_id].current_temp = temp

    def update_error_status(self, errors: List[int]):
        """更新错误状态"""
        for i, error in enumerate(errors[:17]):
            motor_id = i + 1  # 数组索引转换为电机ID
            if motor_id in self.joints:
                self.joints[motor_id].error_status = error

    def update_motor_currents(self, currents: List[int]):
        """更新电机电流"""
        for i, current in enumerate(currents[:17]):
            motor_id = i + 1  # 数组索引转换为电机ID
            if motor_id in self.joints:
                self.joints[motor_id].current_current = current

    def update_joint_offsets(self, offsets: List[int]):
        """更新关节位置偏差"""
        for i, offset in enumerate(offsets[:17]):
            motor_id = i + 1  # 数组索引转换为电机ID
            if motor_id in self.joints:
                self.joints[motor_id].joint_offset = offset

    def update_oc_prot(self, values: List[int]):
        """更新过流保护阈值"""
        for i, val in enumerate(values[:17]):
            motor_id = i + 1
            if motor_id in self.joints:
                self.joints[motor_id].oc_prot = val

    def update_oc_prot_time(self, values: List[int]):
        """更新过流保护时间"""
        for i, val in enumerate(values[:17]):
            motor_id = i + 1
            if motor_id in self.joints:
                self.joints[motor_id].oc_prot_time = val

    def get_all_joint_offsets(self) -> List[int]:
        """获取所有关节偏差"""
        offsets = [0] * 17
        for joint in self.joints.values():
            if 1 <= joint.id <= 17:
                offsets[joint.id - 1] = joint.joint_offset
        return offsets

    def update_tactile_data(self, finger: str, data: np.ndarray):
        """更新触觉传感器数据"""
        if finger in self.tactile_data:
            self.tactile_data[finger] = data.reshape((12, 6))

    def update_tactile_status(self, finger: str, status: bool):
        """更新触觉传感器在线状态"""
        if finger in self.tactile_status:
            self.tactile_status[finger] = status

    def set_target_positions(self, positions: List[int]):
        """设置目标位置"""
        for i, pos in enumerate(positions[:17]):
            motor_id = i + 1  # 数组索引转换为电机ID
            if motor_id in self.joints:
                self.joints[motor_id].target_pos = pos

    def get_joint_by_finger(self, finger: str) -> List[JointInfo]:
        """根据手指名称获取关节"""
        return [joint for joint in self.joints.values() if joint.finger == finger]

    def get_all_current_positions(self) -> List[int]:
        """获取所有关节当前位置"""
        # 按照电机ID 1-17的顺序返回位置
        positions = [0] * 17
        for joint in self.joints.values():
            if 1 <= joint.id <= 17:
                positions[joint.id - 1] = joint.current_pos
        return positions

    def get_all_target_positions(self) -> List[int]:
        """获取所有关节目标位置"""
        # 按照电机ID 1-17的顺序返回位置
        positions = [0] * 17
        for joint in self.joints.values():
            if 1 <= joint.id <= 17:
                positions[joint.id - 1] = joint.target_pos
        return positions

class LinkerHandO20Controller:
    """灵巧手控制器"""

    def __init__(self, hand_type="right", canfd_device=0,
                 comm_type="usb_canfd", can_channel="can0",
                 can_bitrate=500000, can_dbitrate=2000000):
        # 根据 comm_type 选择底层通信后端
        #   "usb_canfd" : 厂商 libcanbus.so 设备（默认，向后兼容）
        #   "socketcan" : 透明塑封 USB 转 CANFD 设备（标准 SocketCAN）
        if comm_type == "socketcan":
            self.comm = SocketCANCommunication(hand_type=hand_type, channel=can_channel,
                                               bitrate=can_bitrate, dbitrate=can_dbitrate)
        else:
            self.comm = CANFDCommunication(hand_type=hand_type, canfd_device=canfd_device)
        self.model = DexterousHandModel()
        self.is_running = False
        self.update_thread = None
        self.receive_thread = None
        self.update_interval = 0.01  # 10ms更新间隔
        # 触觉数据缓冲区，用于拼接DATA1和DATA2
        self.tactile_buffer = {
            'thumb': {'data1': None, 'data2': None},
            'index': {'data1': None, 'data2': None},
            'middle': {'data1': None, 'data2': None},
            'ring': {'data1': None, 'data2': None},
            'pinky': {'data1': None, 'data2': None}
        }
        # 设备信息查询保护锁，防止主动查询与主动上报的竞态条件
        self.device_info_query_lock = threading.Lock()
        self.device_info_query_active = False
        self.device_info_response = None
        self.device_info_event = threading.Event()

    def connect(self) -> Tuple[bool, Optional[str]]:
        """连接灵巧手，返回(连接成功, 设备类型)"""
        print(f"🔗 控制器开始连接")

        try:
            # 初始化CANFD通信
            result = self.comm.initialize()
            if not result:
                print("❌ CANFD通信初始化失败")
                return False, None
            # 查询设备类型
            device_type = self.comm.query_device_type()
            if device_type:
                print(f"✅ 控制器连接成功，检测到设备类型: {device_type}")
                return True, device_type
            else:
                print("❌ 未检测到灵巧手设备")
                self.comm.close()
                return False, None

        except Exception as e:
            print(f"❌ 控制器连接异常: {e}")
            return False, None

    def disconnect(self):
        """断开连接"""
        self.stop_monitoring()
        self.comm.close()

    def start_monitoring(self):
        """开始监控线程"""
        if not self.is_running:
            self.is_running = True
            # 启动接收线程
            self.receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
            self.receive_thread.start()
            # 启动更新线程（发送请求）
            self.update_thread = threading.Thread(target=self._update_loop, daemon=True)
            self.update_thread.start()

    def stop_monitoring(self):
        """停止监控线程"""
        self.is_running = False
        if self.update_thread:
            self.update_thread.join(timeout=1.0)
        if self.receive_thread:
            self.receive_thread.join(timeout=1.0)

    def _receive_loop(self):
        """接收循环：负责监听所有CANFD消息并更新模型"""
        print("📥 接收线程已启动")
        while self.is_running:
            try:
                # 持续接收消息，超时时间设短一点以保持响应
                messages = self.comm.receive_messages(timeout_ms=10)
                if not messages:
                    continue

                for frame_id, data in messages:
                    register_addr = (frame_id >> 13) & 0xFF

                    # 在设备信息主动查询期间，忽略设备信息相关的数据，避免竞态条件
                    if (register_addr == RegisterAddress.SYS_DEVICE_INFO.value or
                        register_addr == RegisterAddress.SYS_SERIAL_NUMBER.value):
                        if self.device_info_query_active:
                            print(f"   [DEBUG] 捕获主动上报的设备信息数据 (寄存器: 0x{register_addr:02X})")
                            self.device_info_response = data
                            self.device_info_event.set()
                            continue

                    # 根据寄存器地址分发处理
                    if register_addr == RegisterAddress.SYS_CURRENT_POS.value:
                        positions = self._parse_position_data(data)
                        if len(positions) == 17:
                            self.model.update_joint_positions(positions)

                    elif register_addr == RegisterAddress.SYS_CURRENT_VEL.value:
                        velocities = self._parse_velocity_data(data)
                        if len(velocities) == 17:
                            self.model.update_joint_velocities(velocities)

                    elif register_addr == RegisterAddress.SYS_ERROR_STATUS.value:
                        if len(data) >= 17:
                            errors = list(data[:17])
                            self.model.update_error_status(errors)

                    elif register_addr == RegisterAddress.SYS_TEMP_DATA.value:
                        temperatures = self._parse_temperature_data(data)
                        if temperatures:
                            self.model.update_joint_temperatures(temperatures)

                    elif register_addr == RegisterAddress.SYS_MOTOR_CURRENT.value:
                        currents = self._parse_current_data(data)
                        if currents:
                            self.model.update_motor_currents(currents)

                    elif register_addr == RegisterAddress.SYS_JOINT_OFFSET.value:
                        offsets = self._parse_offset_data(data)
                        if offsets:
                            self.model.update_joint_offsets(offsets)

                    elif register_addr == RegisterAddress.SYS_OC_PROT.value:
                        values = self._parse_oc_prot_data(data)
                        if values:
                            self.model.update_oc_prot(values)

                    elif register_addr == RegisterAddress.SYS_OC_PROT_TIME.value:
                        values = self._parse_oc_prot_time_data(data)
                        if values:
                            self.model.update_oc_prot_time(values)

                    # 处理触觉数据 (0x09 - 0x12)
                    elif RegisterAddress.TACTILE_THUMB_DATA1.value <= register_addr <= RegisterAddress.TACTILE_PINKY_DATA2.value:
                        self._handle_tactile_message(register_addr, data)


            except Exception as e:
                print(f"接收循环错误: {e}")
                time.sleep(0.01)

    def _handle_tactile_message(self, reg_addr: int, data: bytes):
        """处理触觉传感器消息，拼接DATA1和DATA2"""
        finger_map = {
            # 恢复标准协议定义 (Standard Protocol Mapping)
            # RegisterAddress 命名本身已对应正确的手指
            RegisterAddress.TACTILE_THUMB_DATA1.value: ('thumb', 'data1'),   # 0x09 -> Thumb
            RegisterAddress.TACTILE_THUMB_DATA2.value: ('thumb', 'data2'),   # 0x0A -> Thumb
            RegisterAddress.TACTILE_INDEX_DATA1.value: ('index', 'data1'),   # 0x0B -> Index
            RegisterAddress.TACTILE_INDEX_DATA2.value: ('index', 'data2'),   # 0x0C -> Index
            RegisterAddress.TACTILE_MIDDLE_DATA1.value: ('middle', 'data1'), # 0x0D -> Middle
            RegisterAddress.TACTILE_MIDDLE_DATA2.value: ('middle', 'data2'), # 0x0E -> Middle
            RegisterAddress.TACTILE_RING_DATA1.value: ('ring', 'data1'),     # 0x0F -> Ring
            RegisterAddress.TACTILE_RING_DATA2.value: ('ring', 'data2'),     # 0x10 -> Ring
            RegisterAddress.TACTILE_PINKY_DATA1.value: ('pinky', 'data1'),   # 0x11 -> Pinky
            RegisterAddress.TACTILE_PINKY_DATA2.value: ('pinky', 'data2'),   # 0x12 -> Pinky
        }
        
        if reg_addr in finger_map:
            finger, part = finger_map[reg_addr]
            self.tactile_buffer[finger][part] = data
            
            # 立即更新在线状态 (如果收到的是DATA1)
            # DATA1的第一字节是状态位
            if part == 'data1' and len(data) > 0:
                is_online = (data[0] != 0)
                self.model.update_tactile_status(finger, is_online)
                if is_online:
                     # 仅在在线时打印，或者仅在状态改变时打印会更好，这里先打印所有非零状态以调试
                     print(f"   [DEBUG_TACTILE] Finger:{finger} Reg:0x{reg_addr:02X} StatusByte:0x{data[0]:02X} -> Online")

            # 如果两部分都到齐了，进行拼接和更新
            buf = self.tactile_buffer[finger]
            if buf['data1'] is not None and buf['data2'] is not None:
                combined_data = bytes(buf['data1']) + bytes(buf['data2'])
                
                # 协议说明: 索引0为在线状态，1..72为数据 (共73字节)
                # 实际CAN传输可能是 64 + 8 = 72字节，或者 64 + N > 72字节
                
                if len(combined_data) >= 73:
                    # 包含状态字节
                    is_online = (combined_data[0] != 0)
                    tactile_bytes = combined_data[1:73]
                    
                    self.model.update_tactile_status(finger, is_online)
                    
                    if len(tactile_bytes) == 72:
                        tactile_array = np.frombuffer(tactile_bytes, dtype=np.uint8).reshape((12, 6))
                        self.model.update_tactile_data(finger, tactile_array)
                        
                elif len(combined_data) == 72:
                    # 只有72字节，假设全部是数据，且设备在线
                    # 或者可能是 第1字节是状态，丢失了最后一个数据字节？
                    # 鉴于协议明确提到 1..72 为数据，我们优先尝试解析为数据
                    # 并假定既然收到了数据，设备就是在线的
                    self.model.update_tactile_status(finger, True)
                    
                    tactile_array = np.frombuffer(combined_data, dtype=np.uint8).reshape((12, 6))
                    self.model.update_tactile_data(finger, tactile_array)
                
                # 清除缓冲区等待下一帧
                buf['data1'] = None
                buf['data2'] = None

    def _update_loop(self):
        """更新循环"""
        connection_check_counter = 0
        connection_check_interval = 100  # 每100次循环检查一次连接
        temp_read_counter = 0
        temp_read_interval = 1000  # 每1000次循环读取一次温度 (1000 * 10ms = 10s)

        while self.is_running:
            try:
                # 定期检查连接状态
                '''
                connection_check_counter += 1
                if connection_check_counter >= connection_check_interval:
                    connection_check_counter = 0
                    if not self.comm.check_connection():
                        print("检测到连接断开，尝试重连...")
                        if not self.comm.reconnect():
                            print("重连失败，暂停数据更新")
                            time.sleep(1)
                            continue
                '''
                # 读取当前位置
                self._read_current_positions()

                # 读取当前速度
                self._read_current_velocities()

                # 读取错误状态
                self._read_error_status()

                # 读取电机电流
                self._read_motor_currents()
                
                # 定期读取当前温度 (每10秒)
                temp_read_counter += 1
                if temp_read_counter >= temp_read_interval:
                    temp_read_counter = 0
                    self._read_current_temperatures()

                # 读取触觉传感器数据（由于是主动上报，不再主动轮询）
                # if connection_check_counter % 10 == 0:  # 每10次循环读取一次触觉数据
                #     self._read_tactile_data()

                time.sleep(self.update_interval)

            except Exception as e:
                print(f"更新循环错误: {e}")
                time.sleep(0.1)

    def _read_current_positions(self):
        """读取当前位置（仅发送请求，由接收线程处理响应）"""
        try:
            self.comm.send_message(RegisterAddress.SYS_CURRENT_POS.value, b'', False)
        except Exception as e:
            print(f"发送读取位置请求失败: {e}")

    def _read_current_velocities(self):
        """读取当前速度（仅发送请求）"""
        try:
            self.comm.send_message(RegisterAddress.SYS_CURRENT_VEL.value, b'', False)
        except Exception as e:
            print(f"发送读取速度请求失败: {e}")

    def _read_error_status(self):
        """读取错误状态（仅发送请求）"""
        try:
            self.comm.send_message(RegisterAddress.SYS_ERROR_STATUS.value, b'', False)
        except Exception as e:
            print(f"发送读取错误状态请求失败: {e}")

    def _read_current_temperatures(self):
        """读取当前温度（仅发送请求）"""
        try:
            self.comm.send_message(RegisterAddress.SYS_TEMP_DATA.value, b'', False)
        except Exception as e:
            print(f"发送读取温度请求失败: {e}")

    def _read_motor_currents(self):
        """读取电机电流（仅发送请求）"""
        try:
            self.comm.send_message(RegisterAddress.SYS_MOTOR_CURRENT.value, b'', False)
        except Exception as e:
            print(f"发送读取电流请求失败: {e}")

    def _read_joint_offsets(self):
        """读取关节位置偏差（仅发送请求）"""
        try:
            self.comm.send_message(RegisterAddress.SYS_JOINT_OFFSET.value, b'', False)
        except Exception as e:
            print(f"发送读取偏差请求失败: {e}")

    def _read_oc_protection(self):
        """读取过流保护阈值"""
        try:
            self.comm.send_message(RegisterAddress.SYS_OC_PROT.value, b'', False)
        except Exception as e:
            print(f"发送读取过流保护请求失败: {e}")

    def _read_oc_protection_time(self):
        """读取过流保护时间"""
        try:
            self.comm.send_message(RegisterAddress.SYS_OC_PROT_TIME.value, b'', False)
        except Exception as e:
            print(f"发送读取过流保护时间请求失败: {e}")

    def set_joint_offsets(self, offsets: List[int]) -> bool:
        """设置关节位置偏差并保存到Flash
        
        Args:
            offsets: 17个关节的偏差值列表（单位：0.087度）
        """
        if len(offsets) != 17:
            print(f"❌ 偏差数据长度错误: 期望17个，实际{len(offsets)}个")
            return False

        print(f"📤 设置关节偏差命令:")
        print(f"   输入偏差数组: {offsets}")

        # 构造偏差数据
        data = bytearray()
        for i, offset in enumerate(offsets):
            # 限制偏差范围
            clamped_offset = max(-32768, min(32767, offset))
            # 转换为小端序字节
            offset_bytes = clamped_offset.to_bytes(2, byteorder='little', signed=True)
            data.extend(offset_bytes)
            
            # 打印每个关节的偏差信息
            if i < len(JOINT_DEFINITIONS):
                joint_def = JOINT_DEFINITIONS[i]
                angle_deg = clamped_offset * POSITION_UNIT  # 转换为角度
                print(f"   电机{joint_def.id:2d} ({joint_def.finger}-{joint_def.name}): "
                      f"偏差值={clamped_offset:6d}, 角度={angle_deg:7.2f}°")

        print(f"   数据包大小: {len(data)}字节")
        print(f"   原始数据: {data.hex().upper()}")

        # 发送偏差命令
        success = self.comm.send_message(RegisterAddress.SYS_JOINT_OFFSET.value, bytes(data), True)

        if success:
            print(f"   ✅ 偏差命令发送成功（已保存到Flash）")
        else:
            print(f"   ❌ 偏差命令发送失败")

        return success

    def set_oc_protection(self, values: List[int]) -> bool:
        """设置过流保护阈值"""
        if len(values) != 17: return False
        data = bytearray()
        for val in values:
            clamped = max(0, min(65535, val))
            data.extend(clamped.to_bytes(2, byteorder='little', signed=False))
        print(f"📤 设置过流保护: {values}")
        return self.comm.send_message(RegisterAddress.SYS_OC_PROT.value, bytes(data), True)

    def set_oc_protection_time(self, values: List[int]) -> bool:
        """设置过流保护时间"""
        if len(values) != 17: return False
        data = bytearray()
        for val in values:
            clamped = max(0, min(65535, val))
            data.extend(clamped.to_bytes(2, byteorder='little', signed=False))
        print(f"📤 设置过流保护时间: {values}")
        return self.comm.send_message(RegisterAddress.SYS_OC_PROT_TIME.value, bytes(data), True)

    def clear_error_status(self) -> bool:
        """清除错误状态"""
        # 写入17个0
        data = bytes([0] * 17)
        print(f"🧹 清除错误状态...")
        return self.comm.send_message(RegisterAddress.SYS_ERROR_STATUS.value, data, True)

    def _read_tactile_data(self):


        """读取触觉传感器数据"""
        # 读取各个手指的触觉数据
        tactile_registers = [
            (RegisterAddress.TACTILE_THUMB_DATA1.value, RegisterAddress.TACTILE_THUMB_DATA2.value, 'thumb'),
            (RegisterAddress.TACTILE_INDEX_DATA1.value, RegisterAddress.TACTILE_INDEX_DATA2.value, 'index'),
            (RegisterAddress.TACTILE_MIDDLE_DATA1.value, RegisterAddress.TACTILE_MIDDLE_DATA2.value, 'middle'),
            (RegisterAddress.TACTILE_RING_DATA1.value, RegisterAddress.TACTILE_RING_DATA2.value, 'ring'),
            (RegisterAddress.TACTILE_PINKY_DATA1.value, RegisterAddress.TACTILE_PINKY_DATA2.value, 'pinky'),
        ]

        for reg1, reg2, finger in tactile_registers:
            # 读取第一部分数据
            self.comm.send_message(reg1, b'', False)
            # 读取第二部分数据
            self.comm.send_message(reg2, b'', False)

    def _is_position_response(self, frame_id: int) -> bool:
        """判断是否为位置响应"""
        register_addr = (frame_id >> 13) & 0xFF
        return register_addr == RegisterAddress.SYS_CURRENT_POS.value

    def _is_velocity_response(self, frame_id: int) -> bool:
        """判断是否为速度响应"""
        register_addr = (frame_id >> 13) & 0xFF
        return register_addr == RegisterAddress.SYS_CURRENT_VEL.value

    def _is_error_response(self, frame_id: int) -> bool:
        """判断是否为错误状态响应"""
        register_addr = (frame_id >> 13) & 0xFF
        return register_addr == RegisterAddress.SYS_ERROR_STATUS.value

    def _is_temperature_response(self, frame_id: int) -> bool:
        """判断是否为温度响应"""
        register_addr = (frame_id >> 13) & 0xFF
        return register_addr == RegisterAddress.SYS_TEMP_DATA.value

    def _parse_position_data(self, data: bytes) -> List[int]:
        """解析位置数据

        数据格式：34字节，每2字节对应一个电机的当前位置
        单位：0.087度/LSB (根据协议规范v2.0)
        数据类型：int16_t，小端序，有符号
        """
        positions = []
        for i in range(0, min(34, len(data)), 2):
            if i + 1 < len(data):
                # 解析16位有符号整数，小端序
                pos = int.from_bytes(data[i:i+2], byteorder='little', signed=True)
                # 注意：这里返回原始值，如需要角度值可乘以POSITION_UNIT
                positions.append(pos)
        return positions

    def _parse_velocity_data(self, data: bytes) -> List[int]:
        """解析速度数据

        数据格式：34字节，每2字节对应一个电机的当前速度
        单位：0.732RPM/LSB (根据协议规范v2.0)
        数据类型：int16_t，小端序，有符号
        """
        velocities = []
        for i in range(0, min(34, len(data)), 2):
            if i + 1 < len(data):
                # 解析16位有符号整数，小端序
                vel = int.from_bytes(data[i:i+2], byteorder='little', signed=True)
                # 注意：这里返回原始值，如需要RPM值可乘以VELOCITY_UNIT(0.732)
                velocities.append(vel)
        return velocities

    def _parse_temperature_data(self, data: bytes) -> List[int]:
        """解析温度数据

        数据格式：17字节，每字节对应一个电机的当前温度
        单位：摄氏度
        数据类型：uint8_t
        """
        # 调试打印原始数据
        print(f"      [DEBUG] 原始温度数据 (hex): {data.hex()}")
        
        temperatures = []
        # 协议规定是17个字节，每个字节一个温度
        # 如果收到的是 280000... 这种hex字符串，说明第一个字节是 0x28 (十进制 40)
        for i in range(min(17, len(data))):
            temperatures.append(data[i])
            
        # 如果数据长度不足17，补齐0
        while len(temperatures) < 17:
            temperatures.append(0)
            
        return temperatures

    def _parse_current_data(self, data: bytes) -> List[int]:
        """解析电流数据

        数据格式：34字节，每2字节对应一个电机的电流
        单位：mA
        数据类型：int16_t，小端序，有符号
        """
        # 调试打印原始数据
        print(f"      [DEBUG] 原始电流数据 (hex): {data.hex()}")
        
        currents = []
        for i in range(0, min(34, len(data)), 2):
            if i + 1 < len(data):
                # 解析16位有符号整数，小端序
                current = int.from_bytes(data[i:i+2], byteorder='little', signed=True)
                currents.append(current)
            
        # 如果数据长度不足17，补齐0
        while len(currents) < 17:
            currents.append(0)
            
        return currents

    def _parse_offset_data(self, data: bytes) -> List[int]:
        """解析关节位置偏差数据

        数据格式：34字节，每2字节对应一个关节的偏差
        单位：0.087度
        数据类型：int16_t，小端序，有符号
        """
        # 调试打印原始数据
        print(f"      [DEBUG] 原始偏差数据 (hex): {data.hex()}")
        
        offsets = []
        for i in range(0, min(34, len(data)), 2):
            if i + 1 < len(data):
                # 解析16位有符号整数，小端序
                offset = int.from_bytes(data[i:i+2], byteorder='little', signed=True)
                offsets.append(offset)
            
        # 如果数据长度不足17，补齐0
        while len(offsets) < 17:
            offsets.append(0)
            
        return offsets

    def _parse_oc_prot_data(self, data: bytes) -> List[int]:
        """解析过流保护阈值数据 (uint16_t)"""
        values = []
        for i in range(0, min(34, len(data)), 2):
            if i + 1 < len(data):
                val = int.from_bytes(data[i:i+2], byteorder='little', signed=False)
                values.append(val)
        while len(values) < 17: values.append(220)
        return values

    def _parse_oc_prot_time_data(self, data: bytes) -> List[int]:
        """解析过流保护时间数据 (uint16_t)"""
        values = []
        for i in range(0, min(34, len(data)), 2):
            if i + 1 < len(data):
                val = int.from_bytes(data[i:i+2], byteorder='little', signed=False)
                values.append(val)
        while len(values) < 17: values.append(110)
        return values

    def set_joint_positions(self, positions: List[int]) -> bool:


        """设置关节位置"""
        if len(positions) != 17:
            print(f"❌ 位置数据长度错误: 期望17个，实际{len(positions)}个")
            return False

        print(f"📤 设置关节位置命令:")
        print(f"   输入位置数组: {positions}")

        # 更新模型
        self.model.set_target_positions(positions)

        # 构造位置数据
        data = bytearray()
        actual_positions = []

        for i, pos in enumerate(positions):
            # 限制位置范围 - 使用关节实际限位
            original_pos = pos
            joint = next((j for j in JOINT_DEFINITIONS if j.id == i+1), None)
            if joint:
                clamped_pos = max(joint.min_pos, min(joint.max_pos, pos))
            else:
                clamped_pos = max(-32768, min(32767, pos))  # 默认范围
            actual_positions.append(clamped_pos)

            if original_pos != clamped_pos:
                print(f"   ⚠️ 电机{i+1}: 位置被限制 {original_pos} → {clamped_pos}")

            # 转换为小端序字节
            pos_bytes = clamped_pos.to_bytes(2, byteorder='little', signed=True)
            data.extend(pos_bytes)

            # 详细打印每个关节的信息
            if i < len(JOINT_DEFINITIONS):
                joint_def = JOINT_DEFINITIONS[i]
                angle_deg = clamped_pos * POSITION_UNIT  # 转换为角度
                print(f"   电机{joint_def.id:2d} ({joint_def.finger}-{joint_def.name}): "
                      f"原始值={clamped_pos:6d}, 角度={angle_deg:7.2f}°, "
                      f"字节=[{pos_bytes[0]:02X} {pos_bytes[1]:02X}]")

        print(f"   实际发送位置: {actual_positions}")
        print(f"   数据包大小: {len(data)}字节")
        print(f"   原始数据: {data.hex().upper()}")

        # 发送位置命令
        success = self.comm.send_message(RegisterAddress.SYS_TARGET_POS.value, bytes(data), True)

        if success:
            print(f"   ✅ 位置命令发送成功")
        else:
            print(f"   ❌ 位置命令发送失败")

        return success

    def set_joint_velocities(self, velocities: List[int]) -> bool:
        """设置关节速度"""
        if len(velocities) != 17:
            print(f"❌ 速度数据长度错误: 期望17个，实际{len(velocities)}个")
            return False

        print(f"📤 设置关节速度命令:")
        print(f"   输入速度数组: {velocities}")

        # 构造速度数据
        data = bytearray()
        actual_velocities = []

        for i, vel in enumerate(velocities):
            # 限制速度范围 (无符号16位)
            original_vel = vel
            clamped_vel = max(0, min(65535, vel))
            actual_velocities.append(clamped_vel)

            if original_vel != clamped_vel:
                print(f"   ⚠️ 电机{i+1}: 速度被限制 {original_vel} → {clamped_vel}")

            # 转换为小端序字节 (无符号)
            vel_bytes = clamped_vel.to_bytes(2, byteorder='little', signed=False)
            data.extend(vel_bytes)

            # 详细打印每个关节的信息
            if i < len(JOINT_DEFINITIONS):
                joint_def = JOINT_DEFINITIONS[i]
                rpm_value = clamped_vel * VELOCITY_UNIT  # 转换为RPM
                print(f"   电机{joint_def.id:2d} ({joint_def.finger}-{joint_def.name}): "
                      f"原始值={clamped_vel:6d}, RPM={rpm_value:7.2f}, "
                      f"字节=[{vel_bytes[0]:02X} {vel_bytes[1]:02X}]")

        print(f"   实际发送速度: {actual_velocities}")
        print(f"   数据包大小: {len(data)}字节")
        print(f"   原始数据: {data.hex().upper()}")

        # 发送速度命令
        success = self.comm.send_message(RegisterAddress.SYS_TARGET_VEL.value, bytes(data), True)

        if success:
            print(f"   ✅ 速度命令发送成功")
        else:
            print(f"   ❌ 速度命令发送失败")

        return success

    def set_default_velocity(self, default_vel: int = 100) -> bool:
        """设置默认速度（位置模式下需要设置一次）

        Args:
            default_vel: 默认速度值，单位为原始值（乘以0.732得到RPM）
        """
        print(f"🚀 设置默认速度: {default_vel} (约{default_vel * VELOCITY_UNIT:.1f} RPM)")

        # 为所有17个电机设置相同的默认速度
        default_velocities = [default_vel] * 17
        return self.set_joint_velocities(default_velocities)

    def set_joint_torques(self, torques: List[int]) -> bool:
        """设置关节力矩限制"""
        if len(torques) != 17:
            print(f"❌ 力矩数据长度错误: 期望17个，实际{len(torques)}个")
            return False

        print(f"📤 设置关节力矩命令:")
        print(f"   输入力矩数组: {torques}")

        # 构造力矩数据 - 使用uint16_t[17]，每2字节对应一个电机
        data = bytearray()
        actual_torques = []

        for i, torque in enumerate(torques):
            # 限制力矩范围 (无符号16位: 0-1000)
            original_torque = torque
            clamped_torque = max(0, min(1000, torque))
            actual_torques.append(clamped_torque)

            if original_torque != clamped_torque:
                print(f"   ⚠️ 电机{i+1}: 力矩被限制 {original_torque} → {clamped_torque}")

            # 转换为2字节小端序 (uint16_t)
            torque_bytes = clamped_torque.to_bytes(2, byteorder='little', signed=False)
            data.extend(torque_bytes)

            # 详细打印每个关节的信息
            if i < len(JOINT_DEFINITIONS):
                joint_def = JOINT_DEFINITIONS[i]
                # 力矩单位：6.5mA (根据协议规范v2.0) - 注意：单位可能需要重新确认
                current_ma = clamped_torque * 6.5
                print(f"   电机{joint_def.id:2d} ({joint_def.finger}-{joint_def.name}): "
                      f"原始值={clamped_torque:4d}, 电流={current_ma:6.1f}mA, "
                      f"字节=[{torque_bytes[0]:02X} {torque_bytes[1]:02X}]")

        print(f"   实际发送力矩: {actual_torques}")
        print(f"   数据包大小: {len(data)}字节 (协议要求34字节)")
        print(f"   原始数据: {data.hex().upper()}")

        # 发送力矩命令
        success = self.comm.send_message(RegisterAddress.SYS_TARGET_TORQUE.value, bytes(data), True)

        if success:
            print(f"   ✅ 力矩命令发送成功")
        else:
            print(f"   ❌ 力矩命令发送失败")

        return success

    def set_default_torque(self, default_torque: int = 500) -> bool:
        """设置默认力矩限制

        Args:
            default_torque: 默认力矩值，单位为原始值
        """
        print(f"💪 设置默认力矩: {default_torque}")

        # 为所有17个电机设置相同的默认力矩
        default_torques = [default_torque] * 17
        self.model.target_torques = default_torques
        return self.set_joint_torques(default_torques)




    def set_calibration_mode(self, mode: int) -> bool:
        """设置校准模式"""
        data = bytes([mode])
        return self.comm.send_message(RegisterAddress.SYS_CALI_MODE.value, data, True)

    def read_device_info(self) -> Optional[str]:
        """读取设备信息"""
        with self.device_info_query_lock:
            try:
                print("📋 开始读取设备信息...")
                # 设置主动查询标志，防止接收线程处理主动上报的设备信息
                self.device_info_query_active = True

                # 多次尝试读取设备信息
                for attempt in range(5):  # 增加尝试次数
                    print(f"   尝试 {attempt + 1}/5...")

                    if self.comm.send_message(RegisterAddress.SYS_DEVICE_INFO.value, b'', False):
                        time.sleep(0.05)  # 增加等待时间到50ms

                        # 接收响应，不过滤设备ID
                        messages = self.comm.receive_messages(200, filter_device_id=False)

                        # 收集所有有效的设备信息响应
                        valid_responses = []
                        for frame_id, data in messages:
                            device_id = (frame_id >> 21) & 0xFF
                            register_addr = (frame_id >> 13) & 0xFF

                            # 检查是否是来自目标设备的设备信息响应
                            if (device_id == self.comm.device_id and
                                register_addr == RegisterAddress.SYS_DEVICE_INFO.value):

                                print(f"   收到设备信息响应: 长度={len(data)}")
                                valid_responses.append(data)

                        # 优先处理长度最长的数据（通常是完整的设备信息）
                        if valid_responses:
                            # 按数据长度降序排序，优先处理最长的数据
                            valid_responses.sort(key=len, reverse=True)
                            data = valid_responses[0]

                            print(f"   选择最完整的数据进行解析: 长度={len(data)}")

                            try:
                                # 添加调试输出
                                print(f"   [DEBUG] 开始解析设备信息，数据长度: {len(data)} 字节")
                                print(f"   [DEBUG] 原始数据: {data.hex().upper()}")

                                # 处理实际设备返回的数据格式
                                if len(data) >= 50:
                                    print(f"   [DEBUG] 进入解析分支，数据长度: {len(data)}字节")

                                    try:
                                        # 基于实际数据分析的解析方式
                                        # 数据格式: 产品型号(10字节) + 序列号(24字节) + 软件版本(8字节) + 硬件版本(8字节) + 手型(1字节) + 唯一识别码(12字节)

                                        # 前10字节: 产品型号
                                        product_model = data[0:10].decode('utf-8', errors='ignore').strip('\x00')
                                        print(f"   [DEBUG] 产品型号: '{product_model}'")

                                        # 10-33字节: 序列号 (24字节)
                                        serial_number = data[10:34].decode('utf-8', errors='ignore').strip('\x00')
                                        print(f"   [DEBUG] 序列号: '{serial_number}'")

                                        # 34-41字节: 软件版本 (8字节)
                                        software_version = data[34:42].decode('utf-8', errors='ignore').strip('\x00')
                                        if not software_version: software_version = "未知"
                                        print(f"   [DEBUG] 软件版本: '{software_version}'")

                                        # 42-49字节: 硬件版本 (8字节)
                                        hardware_version = data[42:50].decode('utf-8', errors='ignore').strip('\x00')
                                        if not hardware_version: hardware_version = "未知"
                                        print(f"   [DEBUG] 硬件版本: '{hardware_version}'")

                                        # 50字节: 手型
                                        if 50 < len(data):
                                            hand_type = data[50]
                                        else:
                                            hand_type = 1
                                        print(f"   [DEBUG] 手型: {hand_type}")

                                        # 51-62字节: 唯一识别码
                                        # 如果数据长度够，取后面所有或固定长度
                                        if len(data) >= 63:
                                             unique_id_data = data[51:63]
                                        else:
                                             unique_id_data = data[51:]
                                        
                                        unique_id = unique_id_data.hex().upper()
                                        print(f"   [DEBUG] 唯一识别码: {unique_id}")


                                        hand_type_str = "右手" if hand_type == 1 else ("左手" if hand_type == 2 else f"未知({hand_type})")

                                        info = f"产品型号: {product_model}\n"
                                        info += f"序列号: {serial_number}\n"
                                        info += f"软件版本: {software_version}\n"
                                        info += f"硬件版本: {hardware_version}\n"
                                        info += f"手型: {hand_type_str} ({hand_type})\n"
                                        info += f"唯一识别码: {unique_id}"

                                        print(f"   ✅ 设备信息解析成功 ({len(data)}字节)")
                                        print(f"   [DEBUG] 最终解析结果:\n{info}")
                                        return info

                                    except Exception as e:
                                        print(f"   ❌ 解析设备信息失败: {e}")
                                        print(f"   [DEBUG] 异常类型: {type(e).__name__}")
                                        import traceback
                                        print(f"   [DEBUG] 异常详情:\n{traceback.format_exc()}")
                                        return None

                                elif len(data) >= 50:
                                    # 解析部分设备信息 - 兼容旧格式
                                    print(f"   ⚠️ 数据长度({len(data)})不足63字节，尝试部分解析")

                                    try:
                                        product_model = data[0:10].decode('utf-8', errors='ignore').strip('\x00')
                                        serial_number = data[10:34].decode('utf-8', errors='ignore').strip('\x00')
                                        software_version = data[34:42].decode('utf-8', errors='ignore').strip('\x00')
                                        hardware_version = data[42:50].decode('utf-8', errors='ignore').strip('\x00')
                                        hand_type = data[50] if len(data) > 50 else 1
                                        hand_type_str = "右手" if hand_type == 1 else ("左手" if hand_type == 2 else f"未知({hand_type})")

                                        info = f"产品型号: {product_model}\n"
                                        info += f"序列号: {serial_number}\n"
                                        info += f"软件版本: {software_version}\n"
                                        info += f"硬件版本: {hardware_version}\n"
                                        info += f"手型: {hand_type_str} ({hand_type})\n"
                                        info += f"注意: 数据长度({len(data)})不足63字节，可能缺少唯一识别码"

                                        print(f"   ⚠️ 部分设备信息解析成功 ({len(data)}字节)")
                                        return info
                                    except Exception as e:
                                        print(f"   ❌ 部分解析也失败: {e}")
                                        return None

                                else:
                                    # 数据太短，无法解析
                                    print(f"   ❌ 数据长度({len(data)})太短，无法解析设备信息")
                                    return None

                            except Exception as e:
                                print(f"   ❌ 解析设备信息失败: {e}")
                                print(f"   [DEBUG] 异常详情: {type(e).__name__}: {e}")
                                # 解析失败时返回None，让UI显示失败信息
                                return None
                    else:
                        print(f"   ❌ 发送查询命令失败")

                    time.sleep(0.05)  # 重试间隔

                print("   ❌ 多次尝试后仍无法读取设备信息")
                return None

            except Exception as e:
                print(f"❌ 读取设备信息异常: {e}")
                return None
            finally:
                # 清除主动查询标志
                self.device_info_query_active = False

    def read_serial_number(self) -> Optional[Dict[str, str]]:
        """读取设备信息修改寄存器（0x6E）

        Returns:
            包含设备信息的字典，如果读取失败返回None
        """
        with self.device_info_query_lock:
            try:
                print("📋 开始读取设备信息修改寄存器...")
                # 设置主动查询标志，防止接收线程处理主动上报的设备信息
                self.device_info_query_active = True
                self.device_info_response = None
                self.device_info_event.clear()

                # 多次尝试读取
                for attempt in range(5):
                    print(f"   尝试 {attempt + 1}/5...")

                    if self.comm.send_message(RegisterAddress.SYS_SERIAL_NUMBER.value, b'', False):
                        
                        # 等待接收线程捕获响应
                        if self.device_info_event.wait(timeout=0.5):
                            data = self.device_info_response
                            print(f"   收到设备信息响应: 长度={len(data)}")
                            
                            # 重置事件，以便下一次尝试（虽然这里我们直接处理了）
                            self.device_info_event.clear()
    
                            print(f"   选择数据进行解析: 长度={len(data)}")
    
                            try:
                                if len(data) >= 50:
                                    print(f"   [DEBUG] 解析0x6E寄存器数据，长度: {len(data)}字节")
    
                                    # 数据格式: 产品型号(10字节) + 序列号(24字节) + 软件版本(8字节) + 硬件版本(8字节) + 手型(1字节) + 唯一识别码(12字节)

                                    # 前10字节: 产品型号
                                    product_model = data[0:10].decode('utf-8', errors='ignore').strip('\x00')
                                    print(f"   [DEBUG] 产品型号: '{product_model}'")

                                    # 10-33字节: 序列号 (24字节)
                                    serial_number = data[10:34].decode('utf-8', errors='ignore').strip('\x00')
                                    print(f"   [DEBUG] 序列号: '{serial_number}'")

                                    # 34-41字节: 软件版本 (8字节)
                                    software_version = data[34:42].decode('utf-8', errors='ignore').strip('\x00')
                                    if not software_version: software_version = "未知"
                                    print(f"   [DEBUG] 软件版本: '{software_version}'")

                                    # 42-49字节: 硬件版本 (8字节)
                                    hardware_version = data[42:50].decode('utf-8', errors='ignore').strip('\x00')
                                    if not hardware_version: hardware_version = "未知"
                                    print(f"   [DEBUG] 硬件版本: '{hardware_version}'")

                                    # 50字节: 手型
                                    if 50 < len(data):
                                        hand_type = data[50]
                                    else:
                                        hand_type = 1
                                    print(f"   [DEBUG] 手型: {hand_type}")

                                    # 51-62字节: 唯一识别码
                                    if len(data) >= 63:
                                         unique_id_data = data[51:63]
                                    else:
                                         unique_id_data = data[51:]
                                    
                                    unique_id = unique_id_data.hex().upper()
                                    print(f"   [DEBUG] 唯一识别码: {unique_id}")
    
                                    device_info = {
                                        'product_model': product_model,
                                        'serial_number': serial_number,
                                        'software_version': software_version,
                                        'hardware_version': hardware_version,
                                        'hand_type': hand_type,  # 数字：1=右手，2=左手
                                        'unique_id': unique_id  # 唯一识别码
                                    }
    
                                    print(f"   ✅ 设备信息解析成功 (0x6E, {len(data)}字节)")
                                    return device_info
                                else:
                                    print(f"   ⚠️ 数据长度不足({len(data)}字节)，需要至少50字节")
                                    return None
    
                            except Exception as e:
                                print(f"   ❌ 解析设备信息失败: {e}")
                                return None
                    else:
                        print(f"   ❌ 发送查询命令失败")

                    time.sleep(0.05)

                print("   ❌ 多次尝试后仍无法读取设备信息")
                return None

            except Exception as e:
                print(f"❌ 读取设备信息异常: {e}")
                return None
            finally:
                # 清除主动查询标志
                self.device_info_query_active = False

    def write_serial_number(self, data: bytes) -> bool:
        """写入设备信息到修改寄存器（0x6E）

        Args:
            data: 63字节的设备信息数据
                格式：产品型号(10字节) + 序列号(24字节) + 软件版本(8字节) + 硬件版本(8字节) + 手型标志(1字节) + 唯一识别码(12字节)

        Returns:
            写入是否成功
        """
        try:
            # 确保数据不超过64字节
            if len(data) > 64:
                data = data[:64]
                print(f"   ⚠️ 数据被截断到64字节")
            
            # 如果小于62字节，至少填充到62字节（兼容协议要求的最小长度）
            if len(data) < 62:
                print(f"   ⚠️ 数据长度({len(data)})不足62字节，用0填充")
                data = data + b'\x00' * (62 - len(data))

            print(f"📝 开始写入设备信息到寄存器0x6E...")
            print(f"   寄存器地址: 0x6E (SYS_SERIAL_NUMBER)")
            print(f"   数据长度: {len(data)}字节")
            print(f"   数据内容 (十六进制): {data.hex().upper()}")

            # 解析并显示数据内容（用于验证）
            # 格式: 产品型号(10字节) + 序列号(24字节) + 软件版本(8字节) + 硬件版本(8字节) + 手型(1字节) + 唯一识别码(12字节)
            try:
                product_model = data[0:10].decode('utf-8', errors='ignore').strip('\x00')
                serial_number = data[10:34].decode('utf-8', errors='ignore').strip('\x00')
                software_version = data[34:42].decode('utf-8', errors='ignore').strip('\x00')
                hardware_version = data[42:50].decode('utf-8', errors='ignore').strip('\x00')
                hand_type = data[50]
                unique_id = data[51:63].hex().upper()
                print(f"   产品型号: {product_model}")
                print(f"   序列号: {serial_number}")
                print(f"   软件版本: {software_version}")
                print(f"   硬件版本: {hardware_version}")
                print(f"   手型标志: {hand_type}")
                print(f"   唯一识别码: {unique_id}")
            except Exception as e:
                print(f"   ⚠️ 解析数据内容时出错: {e}")

            # 发送写入命令
            print(f"   正在发送{len(data)}字节数据...")
            success = self.comm.send_message(RegisterAddress.SYS_SERIAL_NUMBER.value, data, True)

            if success:
                print(f"   ✅ 设备信息写入成功（已保存到Flash）")
                print(f"   ✅ {len(data)}字节数据已完整发送")
            else:
                print(f"   ❌ 设备信息写入失败")

            return success

        except Exception as e:
            print(f"❌ 写入设备信息异常: {e}")
            import traceback
            traceback.print_exc()
            return False

    def emergency_stop(self):
        """紧急停止"""
        # 发送所有关节位置为当前位置
        current_positions = self.model.get_all_current_positions()
        self.set_joint_positions(current_positions)

    def reset_to_zero(self):
        """复位到零位"""
        zero_positions = [0] * 17
        self.set_joint_positions(zero_positions)

class DexterousHandGUI:
    """灵巧手图形用户界面"""

    def __init__(self):
        self.controller = LinkerHandO20Controller()
        self.tactile_window = None
        self.root = tk.Tk()
        self.setup_window()
        self.create_widgets()
        self.update_timer = None
        self.status_var = tk.StringVar(value="系统就绪")
        
        # 录制相关变量
        self.recording_data = []
        self.action_sequences = []
        self.is_recording = False
        self.is_playing = False
        self.saved_position = []
        
        # 输入框事件防重复标志
        self.last_entry_values = {}

    def setup_window(self):
        """设置窗口 - 暗黑主题，自适应屏幕"""
        self.root.title("O20灵巧手控制系统 v2.0")
        
        # 获取屏幕尺寸
        screen_width = self.root.winfo_screenwidth()
        screen_height = self.root.winfo_screenheight()
        
        # 根据屏幕大小动态设置窗口尺寸 (占屏幕90%)
        window_width = min(1400, int(screen_width * 0.9))
        window_height = min(1000, int(screen_height * 0.9))
        
        # 计算窗口居中位置
        x = (screen_width - window_width) // 2
        y = (screen_height - window_height) // 2
        
        self.root.geometry(f"{window_width}x{window_height}+{x}+{y}")
        self.root.configure(bg='#1e1e1e')  # 暗黑背景
        
        # 设置窗口最小尺寸
        self.root.minsize(1000, 700)
        
        # 暗黑主题样式配置
        self.setup_dark_theme()


    def setup_dark_theme(self):
        """设置暗黑主题样式"""
        style = ttk.Style()
        style.theme_use('clam')
        
        # 定义暗黑主题颜色
        self.colors = {
            'bg_primary': '#1e1e1e',      # 主背景色
            'bg_secondary': '#2d2d2d',    # 次要背景色
            'bg_tertiary': '#3d3d3d',     # 第三级背景色
            'fg_primary': '#ffffff',      # 主文字色
            'fg_secondary': '#b0b0b0',    # 次要文字色
            'accent_blue': '#0078d4',     # 蓝色强调
            'accent_green': '#107c10',    # 绿色强调
            'accent_orange': '#ff8c00',   # 橙色强调
            'accent_red': '#d13438',      # 红色强调
            'border': '#404040',          # 边框色
            'hover': '#404040',           # 悬停色
            'selected': '#0078d4',        # 选中色
            'accent_orange': '#ff9500'    # 橙色（用于温度）
        }
        colors = self.colors
        
        # 配置Frame样式
        style.configure('Dark.TFrame', 
                       background=colors['bg_primary'],
                       borderwidth=0)
        
        style.configure('Card.TFrame', 
                       background=colors['bg_secondary'],
                       relief='solid',
                       borderwidth=1,
                       bordercolor=colors['border'])
        
        # 配置LabelFrame样式
        style.configure('Dark.TLabelframe', 
                       background=colors['bg_primary'],
                       foreground=colors['fg_primary'],
                       borderwidth=1,
                       relief='solid',
                       bordercolor=colors['border'])
        
        style.configure('Dark.TLabelframe.Label',
                       background=colors['bg_primary'],
                       foreground=colors['fg_primary'],
                       font=('微软雅黑', 10, 'bold'))
        
        # 配置Label样式
        style.configure('Dark.TLabel',
                       background=colors['bg_primary'],
                       foreground=colors['fg_primary'],
                       font=('微软雅黑', 9))
        
        style.configure('Title.TLabel',
                       background=colors['bg_primary'],
                       foreground=colors['fg_primary'],
                       font=('微软雅黑', 16, 'bold'))
        
        style.configure('Header.TLabel',
                       background=colors['bg_primary'],
                       foreground=colors['accent_blue'],
                       font=('微软雅黑', 12, 'bold'))
        
        style.configure('Status.TLabel',
                       background=colors['bg_primary'],
                       foreground=colors['fg_secondary'],
                       font=('微软雅黑', 9))
        
        style.configure('Temp.TLabel',
                       background=colors['bg_primary'],
                       foreground=colors['accent_orange'],
                       font=('微软雅黑', 9, 'bold'))
        
        style.configure('Success.TLabel',
                       background=colors['bg_primary'],
                       foreground=colors['accent_green'],
                       font=('微软雅黑', 9))
        
        style.configure('Error.TLabel',
                       background=colors['bg_primary'],
                       foreground=colors['accent_red'],
                       font=('微软雅黑', 9))
        
        # 配置Button样式
        style.configure('Dark.TButton',
                       background=colors['bg_tertiary'],
                       foreground=colors['fg_primary'],
                       borderwidth=1,
                       focuscolor='none',
                       font=('微软雅黑', 9))
        
        style.map('Dark.TButton',
                 background=[('active', colors['hover']),
                           ('pressed', colors['selected'])])
        
        # 配置不同类型的按钮
        style.configure('Primary.TButton',
                       background=colors['accent_blue'],
                       foreground='white',
                       borderwidth=0,
                       focuscolor='none',
                       font=('微软雅黑', 9),
                       padding=(12, 6))
        
        style.map('Primary.TButton',
                 background=[('active', '#106ebe'),
                           ('pressed', '#005a9e')])
        
        style.configure('Success.TButton',
                       background=colors['accent_green'],
                       foreground='white',
                       borderwidth=0,
                       focuscolor='none',
                       font=('微软雅黑', 9),
                       padding=(12, 6))
        
        style.map('Success.TButton',
                 background=[('active', '#0e6e0e'),
                           ('pressed', '#0c5d0c')])
        
        style.configure('Warning.TButton',
                       background=colors['accent_orange'],
                       foreground='white',
                       borderwidth=0,
                       focuscolor='none',
                       font=('微软雅黑', 9),
                       padding=(12, 6))
        
        style.map('Warning.TButton',
                 background=[('active', '#e67c00'),
                           ('pressed', '#cc6f00')])
        
        style.configure('Danger.TButton',
                       background=colors['accent_red'],
                       foreground='white',
                       borderwidth=0,
                       focuscolor='none',
                       font=('微软雅黑', 9),
                       padding=(12, 6))
        
        style.map('Danger.TButton',
                 background=[('active', '#bc2e32'),
                           ('pressed', '#a6282c')])
        
        # 配置Entry样式
        style.configure('Dark.TEntry',
                       fieldbackground=colors['bg_tertiary'],
                       background=colors['bg_tertiary'],
                       foreground=colors['fg_primary'],
                       borderwidth=1,
                       insertcolor=colors['fg_primary'],
                       font=('微软雅黑', 9))
        
        # 配置Scale样式
        style.configure('Dark.Horizontal.TScale',
                       background=colors['bg_primary'],
                       troughcolor=colors['bg_tertiary'],
                       borderwidth=0,
                       lightcolor=colors['accent_blue'],
                       darkcolor=colors['accent_blue'])
        
        # 配置Treeview样式
        style.configure('Dark.Treeview',
                       background=colors['bg_secondary'],
                       foreground=colors['fg_primary'],
                       fieldbackground=colors['bg_secondary'],
                       borderwidth=1,
                       relief='solid',
                       font=('微软雅黑', 9))
        
        style.configure('Dark.Treeview.Heading',
                       background=colors['bg_tertiary'],
                       foreground=colors['fg_primary'],
                       borderwidth=1,
                       relief='solid',
                       font=('微软雅黑', 9, 'bold'))
        
        style.map('Dark.Treeview',
                 background=[('selected', colors['selected'])],
                 foreground=[('selected', 'white')])
        
        style.map('Dark.Treeview.Heading',
                 background=[('active', colors['hover'])])
        
        # 配置Progressbar样式
        style.configure('Dark.Horizontal.TProgressbar',
                       background=colors['accent_blue'],
                       troughcolor=colors['bg_tertiary'],
                       borderwidth=0,
                       lightcolor=colors['accent_blue'],
                       darkcolor=colors['accent_blue'])

    def create_widgets(self):
        """创建界面组件 - 暗黑主题优化布局"""
        # 主框架 - 使用暗黑主题
        main_frame = ttk.Frame(self.root, padding="5", style='Dark.TFrame')
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        # 配置网格权重
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(0, weight=1)  # 让内容区域可扩展
        main_frame.rowconfigure(1, weight=1)     # 主要内容区域可扩展
        
        # 顶部标题栏
        self.create_title_bar(main_frame)
        
        # 主要内容区域 - 水平布局
        content_frame = ttk.Frame(main_frame, style='Dark.TFrame')
        content_frame.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), pady=(2, 0))
        content_frame.columnconfigure(0, weight=0)  # 左侧面板 (不扩展)
        content_frame.columnconfigure(1, weight=10) # 中间面板 (占满主要空间)
        content_frame.columnconfigure(2, weight=1)  # 右侧面板
        content_frame.rowconfigure(0, weight=1)
        
        # 左侧控制面板
        self.create_enhanced_control_panel(content_frame)

        # 中间关节状态显示
        self.create_enhanced_joint_status_panel(content_frame)

        # 右侧触觉传感器显示
        self.create_enhanced_tactile_panel(content_frame)

        # 底部动作序列编辑区域
        self.create_enhanced_action_sequence_panel(main_frame)

    def create_title_bar(self, parent):
        """创建标题栏"""
        title_frame = ttk.Frame(parent, style='Dark.TFrame')
        title_frame.grid(row=0, column=0, sticky=(tk.W, tk.E), pady=(0, 0))
        title_frame.columnconfigure(0, weight=1)
        title_frame.columnconfigure(2, weight=1)
        
        
        # 状态信息
        info_frame = ttk.Frame(title_frame, style='Dark.TFrame')
        info_frame.grid(row=0, column=2, sticky=tk.E)
        
        # 连接状态指示器
        self.connection_indicator = ttk.Label(info_frame, text="🔴 未连接", style='Status.TLabel')
        self.connection_indicator.grid(row=0, column=0)

    def create_enhanced_control_panel(self, parent):
        """创建增强版控制面板"""
        control_frame = ttk.LabelFrame(parent, text="🎮 设备控制", style='Dark.TLabelframe', padding="15")
        control_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(0, 10))
        control_frame.columnconfigure(0, weight=1)
        
        # 连接控制区域
        conn_frame = ttk.LabelFrame(control_frame, text="🔗 连接管理", style='Dark.TLabelframe', padding="10")
        conn_frame.grid(row=0, column=0, sticky=(tk.W, tk.E), pady=(0, 15))
        conn_frame.columnconfigure(0, weight=1)
        conn_frame.columnconfigure(1, weight=1)
        
        self.connect_btn = ttk.Button(conn_frame, text="🔌 连接设备", 
                                     command=self.connect_device, style='Primary.TButton')
        self.connect_btn.grid(row=0, column=0, sticky=(tk.W, tk.E), padx=(0, 5))
        
        self.disconnect_btn = ttk.Button(conn_frame, text="🔌 断开连接", 
                                        command=self.disconnect_device, style='Danger.TButton', state='disabled')
        self.disconnect_btn.grid(row=0, column=1, sticky=(tk.W, tk.E), padx=(5, 0))
        
        # 设备信息显示
        info_frame = ttk.LabelFrame(control_frame, text="", style='Dark.TLabelframe', padding="10")
        info_frame.grid(row=1, column=0, sticky=(tk.W, tk.E), pady=(0, 15))
        
        self.device_info_text = tk.Text(info_frame, height=6, width=30, 
                                       bg='#2d2d2d', fg='#ffffff', 
                                       font=('Consolas', 9),
                                       insertbackground='#ffffff',
                                       selectbackground='#0078d4',
                                       relief='solid', borderwidth=1)
        self.device_info_text.grid(row=0, column=0, sticky=(tk.W, tk.E))
        info_frame.columnconfigure(0, weight=1)
        
        # 设备信息修改区域
        device_edit_frame = ttk.LabelFrame(control_frame, text="📝 设备信息修改", style='Dark.TLabelframe', padding="10")
        device_edit_frame.grid(row=2, column=0, sticky=(tk.W, tk.E), pady=(0, 15))
        device_edit_frame.columnconfigure(0, weight=1)
        
        ttk.Button(device_edit_frame, text="✏️ 编辑设备信息", command=self.edit_device_info,
                  style='Primary.TButton').grid(row=0, column=0, sticky=(tk.W, tk.E), pady=(0, 5))
        
        # 快速操作
        quick_frame = ttk.LabelFrame(control_frame, text="", style='Dark.TLabelframe', padding="10")
        quick_frame.grid(row=3, column=0, sticky=(tk.W, tk.E))
        quick_frame.columnconfigure(0, weight=1)
        quick_frame.columnconfigure(1, weight=1)
        quick_frame.columnconfigure(2, weight=1)
        quick_frame.columnconfigure(3, weight=1)
        
        ttk.Button(quick_frame, text="五指张开", command=self.open_hand,
                  style='Success.TButton').grid(row=0, column=0, sticky=(tk.W, tk.E), padx=(0, 5))
        
        ttk.Button(quick_frame, text="开始校准", command=self.start_calibration,
                  style='Primary.TButton').grid(row=0, column=1, sticky=(tk.W, tk.E), padx=(5, 0))

        ttk.Button(quick_frame, text="保护设置", command=self.configure_protection,
                  style='Warning.TButton').grid(row=0, column=2, sticky=(tk.W, tk.E), padx=(5, 0))

        ttk.Button(quick_frame, text="清除错误码", command=self.clear_all_errors,
                  style='Danger.TButton').grid(row=0, column=3, sticky=(tk.W, tk.E), padx=(5, 0))

    def create_enhanced_joint_status_panel(self, parent):
        """创建增强版关节状态面板"""
        joint_frame = ttk.LabelFrame(parent, text="🦾 关节状态与控制", style='Dark.TLabelframe', padding="15")
        joint_frame.grid(row=0, column=1, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(5, 5))
        joint_frame.columnconfigure(0, weight=1)
        joint_frame.rowconfigure(0, weight=1)

        # 创建滚动区域
        canvas = tk.Canvas(joint_frame, bg='#1e1e1e', highlightthickness=0)
        scrollbar = ttk.Scrollbar(joint_frame, orient="vertical", command=canvas.yview)
        container = ttk.Frame(canvas, style='Dark.TFrame')
        
        # 绑定配置事件以更新滚动区域
        container.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        
        # 创建窗口
        canvas_window = canvas.create_window((0, 0), window=container, anchor="nw")
        
        # 绑定Canvas大小变化以调整container宽度
        def configure_canvas(event):
            canvas.itemconfig(canvas_window, width=event.width)
        canvas.bind('<Configure>', configure_canvas)

        canvas.configure(yscrollcommand=scrollbar.set)
        
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        
        # 绑定鼠标滚轮 (全局绑定可能会影响其他滚动区域，这里尝试绑定到Canvas)
        def _on_mousewheel(event):
            canvas.yview_scroll(int(-1*(event.delta/120)), "units")
            
        # 绑定鼠标进入/离开事件来控制滚轮绑定的生效范围
        def _bind_mousewheel(event):
            canvas.bind_all("<MouseWheel>", _on_mousewheel)
        def _unbind_mousewheel(event):
            canvas.unbind_all("<MouseWheel>")
            
        canvas.bind('<Enter>', _bind_mousewheel)
        canvas.bind('<Leave>', _unbind_mousewheel)

        self.joint_widgets = {}
        
        # 定义列数
        num_columns = 3
        
        # 配置容器的列权重
        for i in range(num_columns):
            container.columnconfigure(i, weight=1)

        # 动态创建每个关节的控制控件
        for i, joint in enumerate(JOINT_DEFINITIONS):
            row = i // num_columns
            col = i % num_columns
            
            joint_id = joint.id
            
            # 为每个关节创建一个容器
            joint_control_frame = ttk.LabelFrame(container, text=f"{joint.finger} - {joint.name}", style='Dark.TLabelframe', padding="5")
            joint_control_frame.grid(row=row, column=col, sticky=(tk.W, tk.E), pady=2, padx=5)
            joint_control_frame.columnconfigure(1, weight=1)

            # 简化布局：分为两行
            # 第一行：状态信息 + 输入框
            info_frame = ttk.Frame(joint_control_frame, style='Dark.TFrame')
            info_frame.grid(row=0, column=0, sticky=(tk.W, tk.E))
            
            # 使用pack布局以便利用剩余空间 (一行显示所有状态)
            # 当前位置
            current_pos_var = tk.StringVar(value="0")
            ttk.Label(info_frame, textvariable=current_pos_var, style='Status.TLabel', width=8).pack(side=tk.LEFT, padx=1)

            # 当前温度
            current_temp_var = tk.StringVar(value="0°C")
            ttk.Label(info_frame, textvariable=current_temp_var, style='Temp.TLabel', width=5).pack(side=tk.LEFT, padx=1)

            # 当前电流
            current_current_var = tk.StringVar(value="0mA")
            ttk.Label(info_frame, textvariable=current_current_var, style='Status.TLabel', width=6).pack(side=tk.LEFT, padx=1)

            # 错误状态
            error_status_var = tk.StringVar(value="✅")
            error_label = ttk.Label(info_frame, textvariable=error_status_var, style='Status.TLabel', width=3)
            error_label.pack(side=tk.LEFT, padx=1)

            # 关节偏差
            joint_offset_var = tk.StringVar(value="偏:0")
            ttk.Label(info_frame, textvariable=joint_offset_var, style='Status.TLabel', width=6).pack(side=tk.LEFT, padx=1)

            # 目标位置输入框
            target_pos_var = tk.StringVar(value="0")
            target_entry = ttk.Entry(info_frame, textvariable=target_pos_var, width=5, style='Dark.TEntry', justify='center')
            target_entry.pack(side=tk.LEFT, padx=2)
            
            # 第二行：位置滑块
            scale_frame = ttk.Frame(joint_control_frame, style='Dark.TFrame')
            scale_frame.grid(row=1, column=0, sticky=(tk.W, tk.E), pady=(5, 0))
            scale_frame.columnconfigure(0, weight=1)
            
            position_scale = ttk.Scale(scale_frame, from_=joint.min_pos, to=joint.max_pos, orient=tk.HORIZONTAL,
                                     command=lambda v, jid=joint_id: self.on_scale_change(jid, v))
            position_scale.grid(row=0, column=0, sticky=(tk.W, tk.E))
            
            self.joint_widgets[joint_id] = {
                'current_pos': current_pos_var,
                'current_temp': current_temp_var,
                'current_current': current_current_var,
                'error_status': error_status_var,
                'joint_offset': joint_offset_var,
                'target_pos': target_pos_var,
                'scale': position_scale,
                'target_entry': target_entry,
                'error_label': error_label
            }

            
            target_entry.bind('<Return>', lambda e, jid=joint_id: self.on_target_entry_confirm(jid))
            target_entry.bind('<FocusOut>', lambda e, jid=joint_id: self.on_target_entry_confirm(jid))


    def create_enhanced_tactile_panel(self, parent):
        """创建增强版触觉传感器面板"""
        tactile_frame = ttk.LabelFrame(parent, text="👆 触觉传感器", style='Dark.TLabelframe', padding="15")
        tactile_frame.grid(row=0, column=2, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(10, 0))
        tactile_frame.columnconfigure(0, weight=1)
        tactile_frame.rowconfigure(0, weight=1)
        
        # 居中显示开启按钮
        center_frame = ttk.Frame(tactile_frame, style='Dark.TFrame')
        center_frame.grid(row=0, column=0)
        
        btn = ttk.Button(center_frame, text="🔁 打开独立监视窗口", 
                         command=self.open_tactile_window, style='Primary.TButton')
        btn.pack(pady=20, ipadx=10, ipady=5)
        
        ttk.Label(center_frame, text="点击上方按钮打开\n触觉传感器实时热图", 
                  style='Dark.TLabel', justify='center').pack(pady=10)

    def open_tactile_window(self):
        if self.tactile_window is None:
            self.tactile_window = TactileSensorWindow(self.root, self.controller, self.colors, self.on_tactile_window_close)
        else:
            self.tactile_window.deiconify()
            self.tactile_window.lift()

    def on_tactile_window_close(self):
        self.tactile_window = None

    def create_enhanced_action_sequence_panel(self, parent):
        """创建增强版动作序列面板"""
        sequence_frame = ttk.LabelFrame(parent, text="🎬 动作序列编辑器", style='Dark.TLabelframe', padding="15")
        sequence_frame.grid(row=3, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), pady=(5, 0))
        sequence_frame.columnconfigure(0, weight=1)
        
        # 创建动作序列表格
        self.create_enhanced_sequence_table(sequence_frame)
        
        # 创建操作控制区域
        self.create_enhanced_sequence_controls(sequence_frame)

    def create_enhanced_sequence_table(self, parent):
        """创建增强版动作序列表格"""
        # 表格容器
        table_frame = ttk.Frame(parent, style='Card.TFrame')
        table_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), pady=(0, 15))
        table_frame.columnconfigure(0, weight=1)
        table_frame.rowconfigure(0, weight=1)
        
        # 创建表格列
        columns = ['🔢 序号'] + [f"{joint.finger}-{joint.name}" for joint in JOINT_DEFINITIONS]
        self.sequence_tree = ttk.Treeview(table_frame, columns=columns, show='headings',
                                         style='Dark.Treeview', height=6)
        
        # 设置列标题和宽度
        for i, col in enumerate(columns):
            self.sequence_tree.heading(col, text=col)
            if i == 0:
                self.sequence_tree.column(col, width=80, anchor='center')
            else:
                self.sequence_tree.column(col, width=100, anchor='center')
        
        # 添加滚动条
        v_scrollbar = ttk.Scrollbar(table_frame, orient="vertical", command=self.sequence_tree.yview)
        h_scrollbar = ttk.Scrollbar(table_frame, orient="horizontal", command=self.sequence_tree.xview)
        self.sequence_tree.configure(yscrollcommand=v_scrollbar.set, xscrollcommand=h_scrollbar.set)
        
        self.sequence_tree.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        v_scrollbar.grid(row=0, column=1, sticky=(tk.N, tk.S))
        h_scrollbar.grid(row=1, column=0, sticky=(tk.W, tk.E))
        
        # 绑定事件
        self.sequence_tree.bind('<Double-1>', self.on_tree_double_click)
        self.sequence_tree.bind('<Button-3>', self.show_context_menu)

    def create_enhanced_sequence_controls(self, parent):
        """创建增强版序列控制区域"""
        # 创建统一的操作框架
        control_frame = ttk.LabelFrame(parent, text="🎮 操作区域", style='Dark.TLabelframe', padding="10")
        control_frame.grid(row=1, column=0, sticky=(tk.W, tk.E), pady=(15, 0))
        control_frame.columnconfigure(1, weight=1)  # 中间区域可扩展
        
        # 左侧按钮组 - 位置操作（两行布局）
        left_btn_frame = ttk.LabelFrame(control_frame, text="📍 位置操作", style='Dark.TLabelframe', padding="5")
        left_btn_frame.grid(row=0, column=0, sticky=(tk.W, tk.N), padx=(0, 10))
        
        # 第一行按钮
        self.read_current_pos_btn = ttk.Button(left_btn_frame, text="读取位置",
                                              command=self.read_current_position, style='Primary.TButton')
        self.read_current_pos_btn.grid(row=0, column=0, padx=2, pady=2)
        
        self.save_current_pos_btn = ttk.Button(left_btn_frame, text="💾保存",
                                              command=self.save_current_position, style='Success.TButton')
        self.save_current_pos_btn.grid(row=0, column=1, padx=2, pady=2)
        
        self.read_temp_btn = ttk.Button(left_btn_frame, text="读取温度",
                                       command=self.read_temperature, style='Primary.TButton')
        self.read_temp_btn.grid(row=0, column=2, padx=2, pady=2)
        
        self.read_error_btn = ttk.Button(left_btn_frame, text="读取错误码",
                                        command=self.read_error_status, style='Warning.TButton')
        self.read_error_btn.grid(row=0, column=3, padx=2, pady=2)
        
        # 第二行按钮
        self.read_current_btn = ttk.Button(left_btn_frame, text="读取电流",
                                          command=self.read_motor_current, style='Primary.TButton')
        self.read_current_btn.grid(row=1, column=0, padx=2, pady=2)
        
        self.read_offset_btn = ttk.Button(left_btn_frame, text="📐偏差",
                                         command=self.read_joint_offset, style='Primary.TButton')
        self.read_offset_btn.grid(row=1, column=1, padx=2, pady=2)
        
        self.set_offset_btn = ttk.Button(left_btn_frame, text="📝设偏",
                                        command=self.show_offset_editor, style='Warning.TButton')
        self.set_offset_btn.grid(row=1, column=2, padx=2, pady=2)
        
        self.run_selected_btn = ttk.Button(left_btn_frame, text="▶️执行", 
                                          command=self.run_selected_row, style='Primary.TButton')
        self.run_selected_btn.grid(row=1, column=3, padx=2, pady=2)

        
        # 中间参数设置（一行布局）
        param_frame = ttk.LabelFrame(control_frame, text="⚙️ 运行参数", style='Dark.TLabelframe', padding="8")
        param_frame.grid(row=0, column=1, sticky=(tk.W, tk.E, tk.N), padx=(5, 5))
        param_frame.columnconfigure(1, weight=1)
        
        # 所有参数在一行
        ttk.Label(param_frame, text="⏱️ 间隔(秒):", style='Dark.TLabel').grid(row=0, column=0, sticky=tk.W, padx=(0, 5))
        self.interval_time_var = tk.StringVar(value="1.0")
        self.interval_time_entry = ttk.Entry(param_frame, textvariable=self.interval_time_var, 
                                           width=8, font=('微软雅黑', 9), style='Dark.TEntry')
        self.interval_time_entry.grid(row=0, column=1, sticky=tk.W, padx=(0, 15))
        
        ttk.Label(param_frame, text="🔄 循环:", style='Dark.TLabel').grid(row=0, column=2, sticky=tk.W, padx=(0, 5))
        self.loop_count_var = tk.StringVar(value="1")
        self.loop_count_entry = ttk.Entry(param_frame, textvariable=self.loop_count_var, 
                                        width=6, font=('微软雅黑', 9), style='Dark.TEntry')
        self.loop_count_entry.grid(row=0, column=3, sticky=tk.W)
        
        # 右侧按钮组 - 序列控制（一行布局）
        right_btn_frame = ttk.LabelFrame(control_frame, text="🚀 序列控制", style='Dark.TLabelframe', padding="8")
        right_btn_frame.grid(row=0, column=2, sticky=(tk.E, tk.N), padx=(10, 10))
        
        # 所有控件在一行显示
        self.run_all_btn = ttk.Button(right_btn_frame, text="🎬 启动", 
                                      command=self.run_all_sequences, style='Success.TButton')
        self.run_all_btn.grid(row=0, column=0, padx=(0, 5))
        
        self.stop_run_btn = ttk.Button(right_btn_frame, text="⏹️ 停止", 
                                       command=self.stop_running, style='Danger.TButton')
        self.stop_run_btn.grid(row=0, column=1, padx=(0, 10))
        
        # 状态显示也在同一行
        ttk.Label(right_btn_frame, text="状态:", style='Dark.TLabel').grid(row=0, column=2, sticky=tk.W, padx=(0, 5))
        self.loop_status_var = tk.StringVar(value="待机中")
        self.loop_status_label = ttk.Label(right_btn_frame, textvariable=self.loop_status_var, 
                                          style='Status.TLabel')
        self.loop_status_label.grid(row=0, column=3, sticky=tk.W)
        
        # 文件操作区域
        data_mgmt_frame = ttk.LabelFrame(control_frame, text="💾 文件操作", style='Dark.TLabelframe', padding="8")
        data_mgmt_frame.grid(row=0, column=3, sticky=(tk.W, tk.E, tk.N), padx=(10, 0))
        
        # 所有按钮在一行显示
        self.save_data_btn = ttk.Button(data_mgmt_frame, text="💾 保存", 
                                        command=self.save_action_data, style='Success.TButton')
        self.save_data_btn.grid(row=0, column=0, padx=(0, 5))
        
        self.load_data_btn = ttk.Button(data_mgmt_frame, text="📂 读取", 
                                        command=self.load_action_data, style='Primary.TButton')
        self.load_data_btn.grid(row=0, column=1, padx=(0, 5))
        
        self.delete_row_btn = ttk.Button(data_mgmt_frame, text="🗑️ 删除", 
                                         command=self.delete_selected_row, style='Warning.TButton')
        self.delete_row_btn.grid(row=0, column=2, padx=(0, 5))
        
        self.clear_table_btn = ttk.Button(data_mgmt_frame, text="🧹 清空", 
                                          command=self.clear_table, style='Danger.TButton')
        self.clear_table_btn.grid(row=0, column=3)

    def update_tactile_display(self):
        """更新触觉传感器显示（通过独立窗口）"""
        """更新触觉传感器显示（通过独立窗口）"""
        # 独立窗口已有自己的定时刷新循环，此处无需操作
        pass

    def on_sequence_double_click(self, event):
        """序列表格双击事件"""
        pass

    def close_hand(self):
        """握拳动作"""
        # 定义握拳位置 - 根据关节限位设置合理的握拳位置
        fist_positions = [
            400,   # 电机1: 拇指指根弯曲 (THUMB_MCP) -100~500
            500,   # 电机2: 拇指指尖弯曲 (THUMB_IP) -100~600
            300,   # 电机3: 拇指侧摆 (THUMB_ABD) -100~400
            800,   # 电机4: 拇指旋转 (THUMB_CMC) -100~1000
            0,     # 电机5: 无名指侧摆 (RING_ABD) -300~300
            500,   # 电机6: 无名指指尖弯曲 (RING_PIP) -100~600
            500,   # 电机7: 无名指指根弯曲 (RING_MCP) -100~600
            500,   # 电机8: 中指指根弯曲 (MIDDLE_MCP) -100~600
            500,   # 电机9: 中指指尖弯曲 (MIDDLE_PIP) -100~600
            500,   # 电机10: 小指指根弯曲 (PINKY_MCP) -100~600
            500,   # 电机11: 小指指尖弯曲 (PINKY_DIP) -100~600
            0,     # 电机12: 小指侧摆 (PINKY_ABD) -300~300
            0,     # 电机13: 中指侧摆 (MIDDLE_ABD) -300~300
            0,     # 电机14: 食指侧摆 (INDEX_ABD) -300~300
            500,   # 电机15: 食指指根弯曲 (INDEX_MCP) -100~600
            500,   # 电机16: 食指指尖弯曲 (INDEX_PIP) -100~600
            0      # 电机17: 手腕俯仰 (HAND_WRITE) -1000~1000
        ]
        self.controller.set_joint_positions(fist_positions)
        self.status_var.set("执行握拳动作")




    # 事件处理方法
    def connect_device(self):
        """连接设备"""
        # 禁用连接按钮防止重复点击
        self.connect_btn.config(state='disabled')
        self.status_var.set("正在连接设备...")
        self.root.update()  # 立即更新界面

        import time
        start_time = time.time()

        print("\n" + "=" * 60)
        print("🔌 开始连接灵巧手设备")
        print("=" * 60)

        try:
            # 显示连接进度
            self.status_var.set("正在初始化CANFD通信...")
            self.root.update()

            # 尝试连接（添加超时控制）
            connect_result, device_type = self.controller.connect()
            connect_time = time.time() - start_time

            print(f"连接结果: {'成功' if connect_result else '失败'} (总耗时: {connect_time:.3f}s)")

            if connect_result and device_type:
                self.disconnect_btn.config(state='normal')
                self.connection_indicator.config(text=f"🟢 已连接({device_type})", style='Success.TLabel')
                self.status_var.set(f"{device_type}设备连接成功 (耗时: {connect_time:.2f}s)")

                print(f"✅ {device_type}设备连接成功！")
                print("🚀 启动数据监控...")

                # 开始监控
                self.controller.start_monitoring()

                # 已移除全局速度和力矩设置功能

                # 读取设备信息
                print("📋 读取设备信息...")
                self.status_var.set("读取设备信息...")
                self.root.update()

                info_start = time.time()
                device_info = self.controller.read_device_info()
                info_time = time.time() - info_start

                # 准备设备信息显示
                info_display = f"检测到设备类型: {device_type}\n"
                info_display += f"默认速度: 60 (43.9 RPM)\n"
                info_display += f"默认力矩: 500\n"
                info_display += "=" * 30 + "\n"

                if device_info:
                    info_display += device_info
                    self.status_var.set(f"{device_type}设备连接并初始化完成")
                    print(f"✅ 设备信息读取成功 (耗时: {info_time:.3f}s)")
                else:
                    info_display += "设备信息读取失败\n"
                    info_display += "但速度和力矩已设置完成\n"
                    info_display += "设备可以正常使用"
                    self.status_var.set(f"{device_type}设备已连接，信息读取失败但可正常使用")
                    print(f"⚠️ 设备信息读取失败 (耗时: {info_time:.3f}s)")

                self.device_info_text.delete(1.0, tk.END)
                self.device_info_text.insert(1.0, info_display)

                # 开始GUI更新
                print("🖥️ 启动GUI更新...")
                self.start_gui_update()

                print("=" * 60)
                print(f"✅ {device_type}设备连接完成，系统就绪")
                print(f"   - 默认速度: 60 (43.9 RPM)")
                print(f"   - 默认力矩: 500")
                print(f"   - 设备可以正常控制")
                print("=" * 60)

            else:
                # 连接失败，恢复按钮状态
                self.connect_btn.config(state='normal')

                print(f"❌ 设备连接失败 (总耗时: {connect_time:.3f}s)")

                if connect_time > 2.0:
                    self.status_var.set(f"⏰ 连接超时 ({connect_time:.1f}s) - 设备未响应")
                    print("⏰ 连接超时 - 设备未响应")
                else:
                    self.status_var.set("❌ 设备连接失败 - 请检查硬件连接")
                    print("❌ 设备连接失败")

                print("\n🔍 可能的原因:")
                print("   1. 灵巧手设备未连接或未开机")
                print("   2. CANFD适配器未连接到计算机")
                print("   3. 设备驱动程序未正确安装")
                print("   4. 设备被其他程序占用")
                print("   5. USB端口或线缆故障")

                print("\n💡 建议解决方案:")
                print("   1. 检查所有硬件连接")
                print("   2. 重新插拔CANFD适配器")
                print("   3. 重启灵巧手设备")
                print("   4. 关闭其他可能占用设备的程序")
                print("=" * 60)

        except Exception as e:
            # 异常处理，恢复按钮状态
            connect_time = time.time() - start_time
            self.connect_btn.config(state='normal')
            self.status_var.set(f"❌ 连接异常 ({connect_time:.1f}s): {str(e)[:30]}...")

            print(f"❌ 连接过程中发生异常 (耗时: {connect_time:.3f}s)")
            print(f"   异常信息: {e}")
            print("\n💡 建议解决方案:")
            print("   1. 检查CANFD库文件是否正确")
            print("   2. 运行环境检测: python check_environment.py")
            print("   3. 尝试测试版本: python test_gui.py")
            print("   4. 重启程序重试")
            print("=" * 60)

    def disconnect_device(self):
        """断开设备连接"""
        try:
            self.stop_gui_update()
            self.controller.disconnect()

            self.connect_btn.config(state='normal')
            self.disconnect_btn.config(state='disabled')
            self.connection_indicator.config(text="🔴 未连接", style='Status.TLabel')
            self.status_var.set("设备已断开")

        except Exception as e:
            self.status_var.set(f"断开错误: {e}")

    def start_calibration(self):
        """开始校准"""
        if self.controller.set_calibration_mode(1):
            self.status_var.set("校准已开始")
        else:
            self.status_var.set("启动校准失败")

    def reset_to_zero(self):
        """复位到零位"""
        self.controller.reset_to_zero()
        self.status_var.set("已发送复位命令")

    def emergency_stop(self):
        """紧急停止"""
        self.controller.emergency_stop()
        self.status_var.set("紧急停止")

    def make_fist(self):
        """握拳动作"""
        # 定义握拳位置 - 根据关节限位设置合理的握拳位置
        fist_positions = [
            400,   # 电机1: 拇指指根弯曲 (THUMB_MCP) -100~500
            500,   # 电机2: 拇指指尖弯曲 (THUMB_IP) -100~600  
            300,   # 电机3: 拇指侧摆 (THUMB_ABD) -100~400
            800,   # 电机4: 拇指旋转 (THUMB_CMC) -100~1000
            0,     # 电机5: 无名指侧摆 (RING_ABD) -300~300
            500,   # 电机6: 无名指指尖弯曲 (RING_PIP) -100~600
            500,   # 电机7: 无名指指根弯曲 (RING_MCP) -100~600
            500,   # 电机8: 中指指根弯曲 (MIDDLE_MCP) -100~600
            500,   # 电机9: 中指指尖弯曲 (MIDDLE_PIP) -100~600
            500,   # 电机10: 小指指根弯曲 (PINKY_MCP) -100~600
            500,   # 电机11: 小指指尖弯曲 (PINKY_DIP) -100~600
            0,     # 电机12: 小指侧摆 (PINKY_ABD) -300~300
            0,     # 电机13: 中指侧摆 (MIDDLE_ABD) -300~300
            0,     # 电机14: 食指侧摆 (INDEX_ABD) -300~300
            500,   # 电机15: 食指指根弯曲 (INDEX_MCP) -100~600
            500,   # 电机16: 食指指尖弯曲 (INDEX_PIP) -100~600
            0      # 电机17: 手腕俯仰 (HAND_WRITE) -1000~1000
        ]
        self.controller.set_joint_positions(fist_positions)
        self.status_var.set("执行握拳动作")

    def open_hand(self):
        """张开手掌"""
        # 定义张开位置 - 全零位置
        open_positions = [
            0,  # 电机1: 拇指指根弯曲 (THUMB_MCP)
            0,  # 电机2: 拇指指尖弯曲 (THUMB_IP)
            0,  # 电机3: 拇指侧摆 (THUMB_ABD)
            0,  # 电机4: 拇指旋转 (THUMB_CMC)
            0,  # 电机5: 无名指侧摆 (RING_ABD)
            0,  # 电机6: 无名指指尖弯曲 (RING_PIP)
            0,  # 电机7: 无名指指根弯曲 (RING_MCP)
            0,  # 电机8: 中指指根弯曲 (MIDDLE_MCP)
            0,  # 电机9: 中指指尖弯曲 (MIDDLE_PIP)
            0,  # 电机10: 小指指根弯曲 (PINKY_MCP)
            0,  # 电机11: 小指指尖弯曲 (PINKY_DIP)
            0,  # 电机12: 小指侧摆 (PINKY_ABD)
            0,  # 电机13: 中指侧摆 (MIDDLE_ABD)
            0,  # 电机14: 食指侧摆 (INDEX_ABD)
            0,  # 电机15: 食指指根弯曲 (INDEX_MCP)
            0,  # 电机16: 食指指尖弯曲 (INDEX_PIP)
            0   # 电机17: 手腕俯仰 (HAND_WRITE)
        ]
        self.controller.set_joint_positions(open_positions)
        self.status_var.set("执行张开手掌动作")

    def set_single_joint_position(self, joint_id: int):
        """设置单个关节位置

        Args:
            joint_id: 电机ID (1-17)，不是数组索引
        """
        try:
            target_pos = int(self.joint_widgets[joint_id]['target_pos'].get())
            current_positions = self.controller.model.get_all_target_positions()

            # 将电机ID转换为数组索引 (电机ID 1-17 对应索引 0-16)
            array_index = joint_id - 1
            if array_index < 0 or array_index >= len(current_positions):
                print(f"❌ 无效的电机ID: {joint_id}")
                return

            old_pos = current_positions[array_index]
            current_positions[array_index] = target_pos

            # 获取关节信息
            joint_def = None
            for jdef in JOINT_DEFINITIONS:
                if jdef.id == joint_id:  # 直接比较电机ID
                    joint_def = jdef
                    break

            joint_name = f"{joint_def.finger}-{joint_def.name}" if joint_def else f"电机{joint_id}"
            angle_deg = target_pos * POSITION_UNIT

            print(f"\n🎯 单个关节位置设置:")
            print(f"   关节: 电机{joint_id} ({joint_name})")
            print(f"   数组索引: {array_index}")
            print(f"   位置变化: {old_pos} → {target_pos} (变化: {target_pos-old_pos:+d})")
            print(f"   角度: {angle_deg:.2f}°")

            if self.controller.set_joint_positions(current_positions):
                self.status_var.set(f"设置{joint_name}位置: {target_pos} ({angle_deg:.1f}°)")
            else:
                self.status_var.set(f"设置{joint_name}位置失败")

        except ValueError:
            self.status_var.set("位置值无效")

    def clear_joint_position(self, joint_id: int):
        """清零单个关节位置

        Args:
            joint_id: 电机ID (1-17)，不是数组索引
        """
        try:
            # 设置目标位置为0
            self.joint_widgets[joint_id]['target_pos'].set("0")

            # 获取当前所有位置
            current_positions = self.controller.model.get_all_target_positions()

            # 将电机ID转换为数组索引 (电机ID 1-17 对应索引 0-16)
            array_index = joint_id - 1
            if array_index < 0 or array_index >= len(current_positions):
                print(f"❌ 无效的电机ID: {joint_id}")
                return

            old_pos = current_positions[array_index]
            current_positions[array_index] = 0

            # 获取关节信息
            joint_def = None
            for jdef in JOINT_DEFINITIONS:
                if jdef.id == joint_id:
                    joint_def = jdef
                    break

            joint_name = f"{joint_def.finger}-{joint_def.name}" if joint_def else f"电机{joint_id}"

            print(f"\n🔄 关节位置清零:")
            print(f"   关节: 电机{joint_id} ({joint_name})")
            print(f"   位置变化: {old_pos} → 0")

            if self.controller.set_joint_positions(current_positions):
                self.status_var.set(f"清零{joint_name}位置")
            else:
                self.status_var.set(f"清零{joint_name}位置失败")

        except Exception as e:
            print(f"❌ 清零关节位置失败: {e}")
            self.status_var.set("清零位置失败")

    def on_scale_change(self, joint_id: int, value: str):
        """滑块值改变事件 - 更新目标位置输入框并自动发送命令"""
        try:
            pos = int(float(value))
            # 直接更新目标位置输入框
            target_pos_var = self.joint_widgets[joint_id]['target_pos']
            target_pos_var.set(str(pos))
            
            # 自动发送位置命令
            self.set_single_joint_position(joint_id)
        except (ValueError, IndexError):
            pass

    def on_target_entry_confirm(self, joint_id: int):
        """目标位置输入框确认事件 - 回车或失去焦点时触发"""
        try:
            if joint_id in self.joint_widgets:
                target_pos_str = self.joint_widgets[joint_id]['target_pos'].get()
                
                # 检查是否与上次值相同，避免重复处理
                last_value = self.last_entry_values.get(joint_id)
                if target_pos_str == last_value:
                    return
                
                if target_pos_str:  # 只有当输入框不为空时才处理
                    pos = int(target_pos_str)
                    # 限制范围
                    joint = next((j for j in JOINT_DEFINITIONS if j.id == joint_id), None)
                    if joint:
                        pos = max(joint.min_pos, min(joint.max_pos, pos))
                    
                    # 记录当前值
                    self.last_entry_values[joint_id] = str(pos)
                    
                    # 更新输入框和滑块
                    self.joint_widgets[joint_id]['target_pos'].set(str(pos))
                    self.joint_widgets[joint_id]['scale'].set(pos)
                    
                    # 发送位置命令
                    self.set_single_joint_position(joint_id)
        except (ValueError, KeyError):
            pass

    def edit_device_info(self):
        """编辑设备信息"""
        if not self.controller.comm.is_connected:
            messagebox.showwarning("警告", "请先连接设备")
            return
        
        # 读取当前设备信息
        device_info = self.controller.read_serial_number()
        if device_info is None:
            messagebox.showerror("错误", "无法读取设备信息，请检查连接")
            return
        
        # 创建设备信息编辑窗口
        edit_window = tk.Toplevel(self.root)
        edit_window.title("编辑设备信息")
        edit_window.geometry("600x400")
        edit_window.configure(bg='#1e1e1e')
        
        # 创建滚动框架
        canvas = tk.Canvas(edit_window, bg='#1e1e1e', highlightthickness=0)
        scrollbar = ttk.Scrollbar(edit_window, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)
        
        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        
        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        
        # 输入框变量
        product_model_var = tk.StringVar(value=device_info.get('product_model', ''))
        serial_number_var = tk.StringVar(value=device_info.get('serial_number', ''))
        software_version_var = tk.StringVar(value=device_info.get('software_version', ''))
        hardware_version_var = tk.StringVar(value=device_info.get('hardware_version', ''))
        # hand_type是数字：1=右手，2=左手
        hand_type_value = device_info.get('hand_type', 1)  # 默认为1（右手）
        if isinstance(hand_type_value, str):
            # 兼容旧代码：如果是字符串，转换为数字
            hand_type_value = 1 if hand_type_value == '右手' else 2
        elif isinstance(hand_type_value, int) and hand_type_value == 0:
            # 兼容旧代码：如果旧代码返回0，转换为2
            hand_type_value = 2
        hand_type_var = tk.StringVar(value=str(hand_type_value))  # 显示为数字字符串
        
        # 产品型号 (10字节)
        ttk.Label(scrollable_frame, text="产品型号 (最多10字节):", style='Dark.TLabel').grid(row=0, column=0, sticky=tk.W, padx=10, pady=5)
        product_model_entry = ttk.Entry(scrollable_frame, textvariable=product_model_var, width=40, style='Dark.TEntry')
        product_model_entry.grid(row=0, column=1, padx=10, pady=5, sticky=(tk.W, tk.E))
        
        # 序列号 (24字节)
        ttk.Label(scrollable_frame, text="序列号 (最多24字节):", style='Dark.TLabel').grid(row=1, column=0, sticky=tk.W, padx=10, pady=5)
        serial_number_entry = ttk.Entry(scrollable_frame, textvariable=serial_number_var, width=40, style='Dark.TEntry')
        serial_number_entry.grid(row=1, column=1, padx=10, pady=5, sticky=(tk.W, tk.E))
        
        # 软件版本 (8字节)
        ttk.Label(scrollable_frame, text="软件版本 (最多8字节):", style='Dark.TLabel').grid(row=2, column=0, sticky=tk.W, padx=10, pady=5)
        software_version_entry = ttk.Entry(scrollable_frame, textvariable=software_version_var, width=40, style='Dark.TEntry')
        software_version_entry.grid(row=2, column=1, padx=10, pady=5, sticky=(tk.W, tk.E))
        
        # 硬件版本 (8字节)
        ttk.Label(scrollable_frame, text="硬件版本 (最多8字节):", style='Dark.TLabel').grid(row=3, column=0, sticky=tk.W, padx=10, pady=5)
        hardware_version_entry = ttk.Entry(scrollable_frame, textvariable=hardware_version_var, width=40, style='Dark.TEntry')
        hardware_version_entry.grid(row=3, column=1, padx=10, pady=5, sticky=(tk.W, tk.E))
        
        # 手型标志（可编辑，数字：1=右手，2=左手）
        ttk.Label(scrollable_frame, text="手型 (数字):", style='Dark.TLabel').grid(row=4, column=0, sticky=tk.W, padx=10, pady=5)
        hand_type_entry = ttk.Entry(scrollable_frame, textvariable=hand_type_var, width=40, style='Dark.TEntry')
        hand_type_entry.grid(row=4, column=1, padx=10, pady=5, sticky=(tk.W, tk.E))
        # 添加说明标签
        hand_type_label = ttk.Label(scrollable_frame, text="(1=右手, 2=左手)", style='Dark.TLabel', font=('微软雅黑', 8))
        hand_type_label.grid(row=5, column=1, sticky=tk.W, padx=10, pady=(0, 5))

        # 唯一识别码（只读）
        unique_id_var = tk.StringVar(value=device_info.get('unique_id', ''))
        ttk.Label(scrollable_frame, text="唯一识别码 (只读):", style='Dark.TLabel').grid(row=6, column=0, sticky=tk.W, padx=10, pady=5)
        unique_id_entry = ttk.Entry(scrollable_frame, textvariable=unique_id_var, width=40,
                                    state='readonly', style='Dark.TEntry')
        unique_id_entry.grid(row=6, column=1, padx=10, pady=5, sticky=(tk.W, tk.E))
        
        scrollable_frame.columnconfigure(1, weight=1)
        
        def save_device_info():
            """保存设备信息"""
            try:
                # 调试：打印从UI读取的值
                print("🔍 调试 - 从UI读取的值:")
                pm_val = product_model_var.get()
                sn_val = serial_number_var.get()
                sw_val = software_version_var.get()
                hw_val = hardware_version_var.get()
                ht_val = hand_type_var.get()
                print(f"   产品型号: '{pm_val}'")
                print(f"   序列号: '{sn_val}'")
                print(f"   软件版本: '{sw_val}'")
                print(f"   硬件版本: '{hw_val}'")
                print(f"   手型: '{ht_val}'")

                # 获取输入值并截断到指定长度，然后填充到固定长度
                product_model = pm_val[:10].encode('utf-8')
                serial_number = sn_val[:24].encode('utf-8')  # 序列号改为24字节
                software_version = sw_val[:8].encode('utf-8')  # 软件版本改为8字节
                hardware_version = hw_val[:8].encode('utf-8')  # 硬件版本改为8字节

                # 构建63字节数据（根据协议：产品型号10字节 + 序列号24字节 + 软件版本8字节 + 硬件版本8字节 + 手型标志1字节 + 唯一识别码12字节）
                data = bytearray(63)
                # 填充产品型号（10字节，索引0-9）
                data[0:10] = (product_model + b'\x00' * 10)[:10]
                # 填充序列号（24字节，索引10-33）
                data[10:34] = (serial_number + b'\x00' * 24)[:24]
                # 填充软件版本（8字节，索引34-41）
                data[34:42] = (software_version + b'\x00' * 8)[:8]
                # 填充硬件版本（8字节，索引42-49）
                data[42:50] = (hardware_version + b'\x00' * 8)[:8]
                # 手型标志（1字节，索引50）
                data[50] = int(hand_type_var.get()) if hand_type_var.get().isdigit() else 1
                
                # 尝试保留唯一识别码 (索引51开始)
                unique_id_str = unique_id_var.get()
                if unique_id_str:
                    try:
                        unique_id_bytes = bytes.fromhex(unique_id_str)
                        # 最多12字节
                        uid_len = min(12, len(unique_id_bytes))
                        data[51:51+uid_len] = unique_id_bytes[:uid_len]
                    except:
                        pass
                
                # 调试：打印构造的数据
                print(f"🔍 调试 - 构造的数据 ({len(data)}字节):")
                print(f"   十六进制: {data.hex().upper()}")
                print(f"   产品型号 (0-9): {data[0:10].hex().upper()} -> '{data[0:10].decode('utf-8', errors='ignore').strip(chr(0))}'")
                print(f"   序列号 (10-33): {data[10:34].hex().upper()} -> '{data[10:34].decode('utf-8', errors='ignore').strip(chr(0))}'")
                print(f"   软件版本 (34-41): {data[34:42].hex().upper()} -> '{data[34:42].decode('utf-8', errors='ignore').strip(chr(0))}'")
                print(f"   硬件版本 (42-49): {data[42:50].hex().upper()} -> '{data[42:50].decode('utf-8', errors='ignore').strip(chr(0))}'")
                print(f"   手型标志 (50): 0x{data[50]:02X}")
                print(f"   唯一识别码 (51-62): {data[51:63].hex().upper()}")

                # 发送写入命令（发送完整数据）
                if self.controller.write_serial_number(bytes(data)):
                    messagebox.showinfo("成功", "设备信息已保存到Flash\n包含完整的产品型号、序列号、版本信息、手型标志和唯一识别码")
                    edit_window.destroy()
                    # 刷新设备信息显示
                    self.refresh_device_info()
                else:
                    messagebox.showerror("错误", "保存设备信息失败")
            except Exception as e:
                messagebox.showerror("错误", f"保存设备信息时发生错误: {e}")
        
        # 按钮框架
        button_frame = ttk.Frame(scrollable_frame, style='Dark.TFrame')
        button_frame.grid(row=7, column=0, columnspan=2, pady=20)
        
        ttk.Button(button_frame, text="保存", command=save_device_info, 
                  style='Success.TButton').pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="取消", command=edit_window.destroy, 
                  style='Danger.TButton').pack(side=tk.LEFT, padx=5)
        
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        
        edit_window.columnconfigure(0, weight=1)
        edit_window.rowconfigure(0, weight=1)
    
    def refresh_device_info(self):
        """刷新设备信息显示"""
        if self.controller.comm.is_connected:
            device_info = self.controller.read_device_info()
            if device_info:
                self.device_info_text.delete(1.0, tk.END)
                self.device_info_text.insert(1.0, device_info)

    def configure_protection(self):
        """配置过流保护参数 (全局 + 列表详情)"""
        if not self.controller.comm.is_connected:
            messagebox.showwarning("警告", "请先连接设备")
            return
            
        dialog = tk.Toplevel(self.root)
        dialog.title("过流保护配置")
        dialog.geometry("600x600")
        dialog.configure(bg='#1e1e1e')
        
        # 创建标签页控件
        style = ttk.Style()
        style.configure('TNotebook', background='#1e1e1e', borderwidth=0)
        style.configure('TNotebook.Tab', padding=[10, 5], font=('微软雅黑', 9))
        
        notebook = ttk.Notebook(dialog, style='TNotebook')
        notebook.pack(fill='both', expand=True, padx=10, pady=10)
        
        # --- 全局配置标签页 ---
        global_frame = ttk.Frame(notebook, style='Dark.TFrame')
        notebook.add(global_frame, text='全局统一设置')
        
        global_frame.columnconfigure(1, weight=1)
        
        ttk.Label(global_frame, text="快速设置所有关节参数 (区分大小电机)", style='Dark.TLabel').grid(row=0, column=0, columnspan=2, pady=10)

        # 定义小电机ID (2, 4, 7, 10, 13, 16)
        SMALL_MOTORS = [2, 4, 7, 10, 13, 16]

        # 获取默认值 (大电机取电机1，小电机取电机2)
        default_big_prot = 300
        default_small_prot = 220
        default_time = 190
        
        if 1 in self.controller.model.joints:
            val = self.controller.model.joints[1].oc_prot
            if val > 0: default_big_prot = val
            val_time = self.controller.model.joints[1].oc_prot_time
            if val_time > 0: default_time = val_time
            
        if 2 in self.controller.model.joints:
            val = self.controller.model.joints[2].oc_prot
            if val > 0: default_small_prot = val

        # 大电机电流
        ttk.Label(global_frame, text="大电机电流阈值 (mA):", style='Dark.TLabel').grid(row=1, column=0, sticky=tk.E, padx=10, pady=5)
        ttk.Label(global_frame, text="(ID: 1,3,5,6,8,9,11,12,14,15,17)", style='Status.TLabel', font=('微软雅黑', 8)).grid(row=2, column=0, columnspan=2, sticky=tk.W, padx=110)
        
        g_big_prot_var = tk.StringVar(value=str(default_big_prot))
        g_big_prot_entry = ttk.Entry(global_frame, textvariable=g_big_prot_var, style='Dark.TEntry')
        g_big_prot_entry.grid(row=1, column=1, sticky=(tk.W, tk.E), padx=10, pady=5)

        # 小电机电流
        ttk.Label(global_frame, text="小电机电流阈值 (mA):", style='Dark.TLabel').grid(row=3, column=0, sticky=tk.E, padx=10, pady=5)
        ttk.Label(global_frame, text="(ID: 2,4,7,10,13,16)", style='Status.TLabel', font=('微软雅黑', 8)).grid(row=4, column=0, columnspan=2, sticky=tk.W, padx=110)

        g_small_prot_var = tk.StringVar(value=str(default_small_prot))
        g_small_prot_entry = ttk.Entry(global_frame, textvariable=g_small_prot_var, style='Dark.TEntry')
        g_small_prot_entry.grid(row=3, column=1, sticky=(tk.W, tk.E), padx=10, pady=5)
        
        # 保护时间 (统一)
        ttk.Label(global_frame, text="统一保护时间 (ms):", style='Dark.TLabel').grid(row=5, column=0, sticky=tk.E, padx=10, pady=15)
        g_time_var = tk.StringVar(value=str(default_time))
        g_time_entry = ttk.Entry(global_frame, textvariable=g_time_var, style='Dark.TEntry')
        g_time_entry.grid(row=5, column=1, sticky=(tk.W, tk.E), padx=10, pady=15)
        
        # 全局按钮
        g_btn_frame = ttk.Frame(global_frame, style='Dark.TFrame')
        g_btn_frame.grid(row=6, column=0, columnspan=2, pady=20)
        
        def read_global():
            # 发送读取命令
            self.controller._read_oc_protection()
            self.controller._read_oc_protection_time()
            
            def update_ui():
                # 大电机参考电机1
                if 1 in self.controller.model.joints:
                    joint = self.controller.model.joints[1]
                    g_big_prot_var.set(str(joint.oc_prot))
                    g_time_var.set(str(joint.oc_prot_time))
                
                # 小电机参考电机2
                if 2 in self.controller.model.joints:
                    joint = self.controller.model.joints[2]
                    g_small_prot_var.set(str(joint.oc_prot))
                
                dialog.title("过流保护配置 - 读取完成")
            
            dialog.title("过流保护配置 - 读取中...")
            dialog.after(800, update_ui)
            
        def save_global():
            try:
                big_prot = int(g_big_prot_var.get())
                small_prot = int(g_small_prot_var.get())
                time_val = int(g_time_var.get())
                
                prots = []
                times = [time_val] * 17
                
                for i in range(1, 18):
                    if i in SMALL_MOTORS:
                        prots.append(small_prot)
                    else:
                        prots.append(big_prot)
                
                s1 = self.controller.set_oc_protection(prots)
                s2 = self.controller.set_oc_protection_time(times)
                
                if s1 and s2:
                    messagebox.showinfo("成功", "全局参数已下发 (区分大小电机)")
                else:
                    messagebox.showwarning("提示", "参数下发完成，但可能部分写入失败")
            except ValueError:
                messagebox.showerror("错误", "请输入有效的数字")

        ttk.Button(g_btn_frame, text="读取配置", command=read_global, style='Primary.TButton').pack(side=tk.LEFT, padx=5)
        ttk.Button(g_btn_frame, text="应用到所有关节", command=save_global, style='Success.TButton').pack(side=tk.LEFT, padx=5)

        # 自动读取配置
        dialog.after(100, read_global)


        # --- 列表配置标签页 ---
        list_frame = ttk.Frame(notebook, style='Dark.TFrame')
        notebook.add(list_frame, text='各关节详细设置')
        
        # 底部按钮区
        l_btn_frame = ttk.Frame(list_frame, style='Dark.TFrame')
        l_btn_frame.pack(side="bottom", fill="x", pady=10)
        
        # 列表滚动区
        list_container = ttk.Frame(list_frame, style='Dark.TFrame')
        list_container.pack(side="top", fill="both", expand=True)
        
        canvas = tk.Canvas(list_container, bg='#1e1e1e', highlightthickness=0)
        scrollbar = ttk.Scrollbar(list_container, orient="vertical", command=canvas.yview)
        scroll_content = ttk.Frame(canvas, style='Dark.TFrame')
        
        scroll_content.bind("<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.create_window((0, 0), window=scroll_content, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        
        # 列表表头
        ttk.Label(scroll_content, text="关节名称", style='Dark.TLabel', font=('微软雅黑', 9, 'bold')).grid(row=0, column=0, padx=10, pady=5, sticky=tk.W)
        ttk.Label(scroll_content, text="电流阈值 (mA)", style='Dark.TLabel', font=('微软雅黑', 9, 'bold')).grid(row=0, column=1, padx=5, pady=5)
        ttk.Label(scroll_content, text="保护时间 (ms)", style='Dark.TLabel', font=('微软雅黑', 9, 'bold')).grid(row=0, column=2, padx=5, pady=5)
        
        self.list_prot_entries = {}
        self.list_time_entries = {}
        
        # 填充列表
        current_row = 1
        for joint in JOINT_DEFINITIONS:
            motor_id = joint.id
            
            # Label
            name_txt = f"{motor_id}: {joint.finger}-{joint.name}"
            ttk.Label(scroll_content, text=name_txt, style='Dark.TLabel').grid(row=current_row, column=0, padx=10, pady=2, sticky=tk.W)
            
            # Initial values
            pv = 220
            tv = 110
            if motor_id in self.controller.model.joints:
                j = self.controller.model.joints[motor_id]
                pv = j.oc_prot
                tv = j.oc_prot_time
            
            # Prot Entry
            p_var = tk.StringVar(value=str(pv))
            p_entry = ttk.Entry(scroll_content, textvariable=p_var, width=12, style='Dark.TEntry', justify='center')
            p_entry.grid(row=current_row, column=1, padx=5, pady=2)
            self.list_prot_entries[motor_id] = p_var
            
            # Time Entry
            t_var = tk.StringVar(value=str(tv))
            t_entry = ttk.Entry(scroll_content, textvariable=t_var, width=12, style='Dark.TEntry', justify='center')
            t_entry.grid(row=current_row, column=2, padx=5, pady=2)
            self.list_time_entries[motor_id] = t_var
            
            current_row += 1
            
        
        # 列表底部按钮逻辑
        def refresh_list_values():
            self.controller._read_oc_protection()
            self.controller._read_oc_protection_time()
            
            def update_ui():
                count = 0
                for mid, p_var in self.list_prot_entries.items():
                    if mid in self.controller.model.joints:
                        p_var.set(str(self.controller.model.joints[mid].oc_prot))
                        count += 1
                for mid, t_var in self.list_time_entries.items():
                    if mid in self.controller.model.joints:
                        t_var.set(str(self.controller.model.joints[mid].oc_prot_time))
                dialog.title(f"过流保护配置 - 已更新 {count} 个关节数据")
                
            dialog.after(800, update_ui)
            dialog.title("过流保护配置 - 读取中...")
            
        def apply_list_values():
            try:
                # Prepare arrays
                prots = [220] * 17
                times = [110] * 17
                
                # JOINT_DEFINITIONS has 17 items. i=0->motor 1
                for i in range(17):
                    mid = i + 1
                    if mid in self.list_prot_entries:
                        prots[i] = int(self.list_prot_entries[mid].get())
                    if mid in self.list_time_entries:
                        times[i] = int(self.list_time_entries[mid].get())
                
                s1 = self.controller.set_oc_protection(prots)
                s2 = self.controller.set_oc_protection_time(times)
                
                if s1 and s2:
                    messagebox.showinfo("成功", "所有关节参数已下发")
                else:
                    messagebox.showwarning("警告", "参数下发未完全成功")
            except ValueError:
                messagebox.showerror("错误", "输入格式有误，请确保都是数字")

        ttk.Button(l_btn_frame, text="从设备读取最新", command=refresh_list_values, style='Primary.TButton').pack(side=tk.LEFT, padx=10)
        ttk.Button(l_btn_frame, text="应用所有修改", command=apply_list_values, style='Success.TButton').pack(side=tk.LEFT, padx=10)

    def start_gui_update(self):
        """开始GUI更新"""
        self.update_counter = 0
        self.update_gui()

    def stop_gui_update(self):
        """停止GUI更新"""
        if hasattr(self, 'update_timer') and self.update_timer:
            self.root.after_cancel(self.update_timer)
            self.update_timer = None

    def update_gui(self):
        """更新GUI显示"""
        try:
            # 更新关节状态
            self.update_joint_display()

            # 更新触觉传感器显示（降低频率避免卡顿）
            self.update_counter += 1
            if self.update_counter % 5 == 0:
                self.update_tactile_display()

            # 录制数据
            if self.is_recording:
                current_positions = self.controller.model.get_all_current_positions()
                timestamp = time.time() - self.recording_start_time
                self.recording_data.append((current_positions, timestamp))

        except Exception as e:
            print(f"GUI更新错误: {e}")
        finally:
            # 安排下次更新
            self.update_timer = self.root.after(100, self.update_gui)

    def update_joint_display(self):
        """更新关节显示"""
        if not hasattr(self, 'joint_widgets'):
            return

        for joint_id, widgets in self.joint_widgets.items():
            if joint_id in self.controller.model.joints:
                joint = self.controller.model.joints[joint_id]

                # 更新当前位置显示
                pos_text = f"{joint.current_pos:5d} ({joint.current_pos * POSITION_UNIT:.1f}°)"
                widgets['current_pos'].set(pos_text)

                # 更新当前温度显示
                temp_text = f"{joint.current_temp}°C"
                widgets['current_temp'].set(temp_text)
                
                # 根据温度改变颜色（可选）
                if joint.current_temp > 60:
                    widgets['current_temp'].set(f"🔥 {joint.current_temp}°C")
                elif joint.current_temp > 45:
                    widgets['current_temp'].set(f"⚠️ {joint.current_temp}°C")

                # 更新当前电流显示
                if 'current_current' in widgets:
                    current_text = f"{joint.current_current}mA"
                    widgets['current_current'].set(current_text)

                # 更新错误状态显示
                if 'error_status' in widgets:
                    error_code = joint.error_status
                    if error_code == 0:
                        widgets['error_status'].set("✅")
                        # 恢复正常样式 (需要在create_widgets中保存widget引用，目前只能通过父类遍历或重新查找，但这里widget是Label)
                        # 为了支持样式修改，我们需要在joint_widgets中保存label控件的引用
                        # 这里我们假设joint_widgets['error_label']保存了label引用
                        if 'error_label' in widgets:
                             widgets['error_label'].configure(style='Status.TLabel')
                    else:
                        if error_code == 1:
                            widgets['error_status'].set("堵转")
                        elif error_code == 2:
                            widgets['error_status'].set("⚡过流")
                        elif error_code == 3:
                            widgets['error_status'].set("📡通讯")
                        elif error_code == 4:
                            widgets['error_status'].set("🔧未校准")
                        else:
                            widgets['error_status'].set(f"❌{error_code}")
                        
                        # 设置错误样式（红色）
                        if 'error_label' in widgets:
                             widgets['error_label'].configure(style='Error.TLabel')

                # 更新关节偏差显示
                if 'joint_offset' in widgets:
                    offset = joint.joint_offset
                    angle_deg = offset * POSITION_UNIT
                    widgets['joint_offset'].set(f"偏:{offset}({angle_deg:.1f}°)")

    def create_context_menu(self):

        """创建右键菜单"""
        self.context_menu = tk.Menu(self.root, tearoff=0)
        self.context_menu.add_command(label="✏️ 编辑单元格", command=self.edit_selected_cell)
        self.context_menu.add_command(label="▶️ 运行此行", command=self.run_selected_row)
        self.context_menu.add_separator()
        self.context_menu.add_command(label="📋 复制行", command=self.copy_selected_row)
        self.context_menu.add_command(label="📄 粘贴行", command=self.paste_row)
        self.context_menu.add_separator()
        self.context_menu.add_command(label="🗑️ 删除行", command=self.delete_selected_row)
    
    def show_context_menu(self, event):
        """显示右键菜单"""
        try:
            self.context_menu.tk_popup(event.x_root, event.y_root)
        finally:
            self.context_menu.grab_release()
    
    def on_row_select(self, event):
        """行选择变化事件"""
        selection = self.sequence_tree.selection()
        if selection:
            # 高亮选中行
            pass
    
    def on_delete_key(self, event):
        """Delete键删除行"""
        self.delete_selected_row()
    
    def on_f2_key(self, event):
        """F2键编辑单元格"""
        self.edit_selected_cell()
    
    def update_stats(self):
        """更新统计信息"""
        count = len(self.action_sequences)
        if count == 0:
            self.stats_var.set("📊 统计: 暂无动作序列")
        else:
            self.stats_var.set(f"📊 统计: {count} 个动作序列")
    
    def edit_selected_cell(self):
        """编辑选中的单元格"""
        selection = self.sequence_tree.selection()
        if selection:
            # 模拟双击第一个可编辑列
            item = selection[0]
            bbox = self.sequence_tree.bbox(item, '#2')  # 第二列（第一个关节）
            if bbox:
                # 创建一个模拟的双击事件
                event = type('Event', (), {})()
                event.x = bbox[0] + bbox[2] // 2
                event.y = bbox[1] + bbox[3] // 2
                self.on_tree_double_click(event)
    
    def copy_selected_row(self):
        """复制选中行"""
        selection = self.sequence_tree.selection()
        if selection:
            item = selection[0]
            values = self.sequence_tree.item(item, 'values')
            if values and len(values) > 1:
                # 复制关节数据（跳过序号）
                self.clipboard_data = [int(x) for x in values[1:]]
                print(f"已复制行数据: {self.clipboard_data}")
    
    def paste_row(self):
        """粘贴行"""
        if hasattr(self, 'clipboard_data') and self.clipboard_data:
            # 添加新行
            row_data = [len(self.action_sequences) + 1] + self.clipboard_data
            self.action_sequences.append(self.clipboard_data)
            self.sequence_tree.insert('', 'end', values=row_data)
            self.update_stats()
            print(f"已粘贴行数据: {self.clipboard_data}")

    def read_current_position(self):
        """读取当前位置并更新UI滑块"""
        if not self.controller.comm.is_connected:
            self.status_var.set("❌ 设备未连接")
            return
        
        print("📖 发送位置读取请求...")
        self.controller._read_current_positions()
        self.status_var.set("⏳ 正在读取位置...")
        # 200ms后同步滑块（等待接收线程更新模型）
        self.root.after(200, self._sync_sliders_to_current)

    def _sync_sliders_to_current(self):
        """将UI滑块同步到模型中的当前位置"""
        positions = self.controller.model.get_all_current_positions()
        for i, pos in enumerate(positions):
            joint_id = i + 1
            if joint_id in self.joint_widgets:
                self.joint_widgets[joint_id]['target_pos'].set(str(pos))
                self.joint_widgets[joint_id]['scale'].set(pos)
        self.status_var.set("✅ 位置已同步到滑块")

    def read_temperature(self):
        """手动读取当前温度并更新UI"""
        if not self.controller.comm.is_connected:
            self.status_var.set("❌ 设备未连接")
            return
        
        print("🌡️ 发送温度读取请求...")
        self.controller._read_current_temperatures()
        self.status_var.set("⏳ 正在读取温度...")
        # 接收线程会自动更新模型，GUI循环会自动刷新显示

    def read_error_status(self):
        """手动读取错误状态并更新UI"""
        if not self.controller.comm.is_connected:
            self.status_var.set("❌ 设备未连接")
            return
        
        print("⚠️ 发送错误状态读取请求...")
        self.controller._read_error_status()
        self.status_var.set("⏳ 正在读取错误状态...")
        # 接收线程会自动更新模型，GUI循环会自动刷新显示

    def read_motor_current(self):
        """手动读取电机电流并更新UI"""
        if not self.controller.comm.is_connected:
            self.status_var.set("❌ 设备未连接")
            return
        
        print("⚡ 发送电流读取请求...")
        self.controller._read_motor_currents()
        self.status_var.set("⏳ 正在读取电流...")
        # 接收线程会自动更新模型，GUI循环会自动刷新显示

    def read_joint_offset(self):
        """手动读取关节位置偏差并更新UI"""
        if not self.controller.comm.is_connected:
            self.status_var.set("❌ 设备未连接")
            return
        
        print("📐 发送偏差读取请求...")
        self.controller._read_joint_offsets()
        self.status_var.set("⏳ 正在读取偏差...")

        # 接收线程会自动更新模型

    def clear_all_errors(self):
        """清除所有错误"""
        if not self.controller.comm.is_connected:
            self.status_var.set("❌ 设备未连接")
            return
            
        if self.controller.clear_error_status():
            self.status_var.set("🧹 已发送清除错误命令")
            # 200ms后自动读取最新错误状态
            self.root.after(200, self.read_error_status)
        else:
            self.status_var.set("❌ 清除错误失败")

    def show_offset_editor(self):
        """显示关节位置偏差编辑器"""
        if not self.controller.comm.is_connected:
            messagebox.showwarning("警告", "请先连接设备")
            return
        
        # 先读取当前偏差
        print("📐 读取当前偏差...")
        self.controller._read_joint_offsets()
        
        # 等待200ms后显示编辑器
        self.root.after(200, self._show_offset_editor_window)

    def _show_offset_editor_window(self):
        """显示偏差编辑器窗口"""
        # 创建偏差编辑窗口
        edit_window = tk.Toplevel(self.root)
        edit_window.title("关节位置偏差编辑器")
        edit_window.geometry("700x600")
        edit_window.configure(bg='#1e1e1e')
        
        # 创建滚动框架
        canvas = tk.Canvas(edit_window, bg='#1e1e1e', highlightthickness=0)
        scrollbar = ttk.Scrollbar(edit_window, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas, style='Dark.TFrame')
        
        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        
        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        
        # 标题说明
        ttk.Label(scrollable_frame, text="关节位置偏差设置 (单位: 0.087度)", 
                 style='Header.TLabel').grid(row=0, column=0, columnspan=3, pady=10, padx=10)
        
        ttk.Label(scrollable_frame, text="偏差值用于补偿机械公差，写入后自动保存到Flash", 
                 style='Status.TLabel').grid(row=1, column=0, columnspan=3, pady=(0, 10), padx=10)
        
        # 存储偏差输入变量
        offset_vars = {}
        
        # 获取当前偏差
        current_offsets = self.controller.model.get_all_joint_offsets()
        
        # 为每个关节创建偏差输入
        for i, joint in enumerate(JOINT_DEFINITIONS):
            row = i + 2
            
            # 关节名称
            ttk.Label(scrollable_frame, text=f"{joint.finger} - {joint.name}", 
                     style='Dark.TLabel', width=20).grid(row=row, column=0, sticky=tk.W, padx=10, pady=3)
            
            # 当前偏差值
            current_offset = current_offsets[i] if i < len(current_offsets) else 0
            offset_var = tk.StringVar(value=str(current_offset))
            offset_vars[joint.id] = offset_var
            
            offset_entry = ttk.Entry(scrollable_frame, textvariable=offset_var, width=10, 
                                    style='Dark.TEntry', justify='center')
            offset_entry.grid(row=row, column=1, padx=10, pady=3)
            
            # 显示角度值
            angle_deg = current_offset * POSITION_UNIT
            angle_label = ttk.Label(scrollable_frame, text=f"≈ {angle_deg:.2f}°", 
                                   style='Status.TLabel', width=12)
            angle_label.grid(row=row, column=2, sticky=tk.W, padx=10, pady=3)
        
        scrollable_frame.columnconfigure(0, weight=1)
        scrollable_frame.columnconfigure(1, weight=1)
        scrollable_frame.columnconfigure(2, weight=1)
        
        def save_offsets():
            """保存偏差到设备"""
            try:
                offsets = []
                for i in range(17):
                    joint_id = i + 1
                    if joint_id in offset_vars:
                        try:
                            offset = int(offset_vars[joint_id].get())
                        except ValueError:
                            offset = 0
                        offsets.append(offset)
                    else:
                        offsets.append(0)
                
                print(f"📝 准备写入偏差: {offsets}")
                
                if self.controller.set_joint_offsets(offsets):
                    messagebox.showinfo("成功", "关节偏差已保存到Flash")
                    edit_window.destroy()
                else:
                    messagebox.showerror("错误", "保存偏差失败")
            except Exception as e:
                messagebox.showerror("错误", f"保存偏差时发生错误: {e}")
        
        def reset_offsets():
            """重置所有偏差为0"""
            for var in offset_vars.values():
                var.set("0")
        
        # 按钮框架
        button_frame = ttk.Frame(scrollable_frame, style='Dark.TFrame')
        button_frame.grid(row=len(JOINT_DEFINITIONS) + 3, column=0, columnspan=3, pady=20)
        
        ttk.Button(button_frame, text="💾 保存到设备", command=save_offsets, 
                  style='Success.TButton').pack(side=tk.LEFT, padx=10)
        ttk.Button(button_frame, text="🔄 全部归零", command=reset_offsets, 
                  style='Warning.TButton').pack(side=tk.LEFT, padx=10)
        ttk.Button(button_frame, text="❌ 取消", command=edit_window.destroy, 
                  style='Danger.TButton').pack(side=tk.LEFT, padx=10)
        
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

    def save_current_position(self):

        """保存当前位置到动作序列"""
        if not self.controller.comm.is_connected:
            self.status_var.set("❌ 设备未连接")
            return
        
        try:
            print("💾 开始保存当前位置...")
            # 发送读取请求（触发一次即时更新）
            self.controller._read_current_positions()
            self.status_var.set("⏳ 正在获取最新位置...")
            
            # 延迟 200ms 后从模型中提取数据并保存
            # 这样可以确保接收线程有足够时间处理响应
            self.root.after(200, self._do_save_position)
            
        except Exception as e:
            print(f"❌ 保存位置异常: {e}")
            self.status_var.set("❌ 保存位置异常")

    def _do_save_position(self):
        """实际执行保存位置的操作"""
        try:
            positions = self.controller.model.get_all_current_positions()
            if any(p != 0 for p in positions):  # 简单检查是否有有效数据
                self.add_position_to_table(positions)
                self.status_var.set("✅ 当前位置已保存到序列")
                print(f"   📊 已保存位置: {positions}")
            else:
                self.status_var.set("⚠️ 未获取到有效位置数据")
        except Exception as e:
            print(f"❌ 执行保存失败: {e}")
            self.status_var.set("❌ 保存失败")

    def on_double_click(self, event):
        """双击编辑表格数据"""
        region = self.sequence_tree.identify("region", event.x, event.y)
        if region == "cell":
            column = self.sequence_tree.identify_column(event.x)
            item = self.sequence_tree.identify_row(event.y)
            if item and column:
                # 获取当前值
                values = list(self.sequence_tree.item(item, 'values'))
                col_index = int(column[1:]) - 1  # 转换为0基索引（从#1开始，所以#1->0, #2->1...）
                
                if col_index >= 0:  # 允许编辑所有列，但序号列会在后面特殊处理
                    if col_index == 0:  # 序号列不允许编辑
                        return
                    # 创建编辑对话框
                    self.edit_cell_value(item, col_index, values[col_index])
    
    def edit_cell_value(self, item, col_index, current_value):
        """编辑单元格值"""
        # 创建编辑对话框
        edit_window = tk.Toplevel(self.root)
        edit_window.title("编辑数值")
        edit_window.geometry("300x150")
        edit_window.transient(self.root)
        edit_window.grab_set()
        
        # 居中显示
        edit_window.geometry("+%d+%d" % (self.root.winfo_rootx() + 50, self.root.winfo_rooty() + 50))
        
        # 创建输入框
        tk.Label(edit_window, text="请输入新数值:").pack(pady=10)
        entry = tk.Entry(edit_window, width=20)
        entry.insert(0, str(current_value))
        entry.pack(pady=10)
        entry.focus()
        
        def save_value():
            try:
                new_value = int(entry.get())
                # 更新表格数据
                values = list(self.sequence_tree.item(item, 'values'))
                values[col_index] = new_value
                self.sequence_tree.item(item, values=values)
                
                # 更新内部数据
                sequence_index = int(values[0]) - 1
                if 0 <= sequence_index < len(self.action_sequences):
                    self.action_sequences[sequence_index][col_index - 1] = new_value
                
                edit_window.destroy()
            except ValueError:
                pass
        
        def cancel_edit():
            edit_window.destroy()
        
        # 创建按钮
        button_frame = tk.Frame(edit_window)
        button_frame.pack(pady=10)
        
        tk.Button(button_frame, text="确定", command=save_value).pack(side=tk.LEFT, padx=5)
        tk.Button(button_frame, text="取消", command=cancel_edit).pack(side=tk.LEFT, padx=5)
        
        # 绑定回车键
        entry.bind('<Return>', lambda e: save_value())
        entry.bind('<Escape>', lambda e: cancel_edit())

    def add_position_to_table(self, positions):
        """添加位置到表格"""
        if len(positions) != 17:
            pass
            return
        
        # 创建新行数据
        row_data = [len(self.action_sequences) + 1] + positions
        self.action_sequences.append(positions)
        
        # 添加到Treeview
        item = self.sequence_tree.insert('', 'end', values=row_data)
        
        # 选中新添加的行
        self.sequence_tree.selection_set(item)
        self.sequence_tree.see(item)
        
        # 更新统计信息
        self.update_stats()
        
        # 更新状态
        self.loop_status_var.set(f"✅ 已添加第 {len(self.action_sequences)} 个动作")

    def run_selected_row(self):
        """运行选中行"""
        if not self.controller.comm.is_connected:
            return
        
        selection = self.sequence_tree.selection()
        if not selection:
            return
        
        try:
            item = selection[0]
            values = self.sequence_tree.item(item, 'values')
            if values and len(values) > 1:
                positions = [int(x) for x in values[1:]]  # 跳过序号
                success = self.controller.set_joint_positions(positions)
                if success:
                    pass
                else:
                    pass
            else:
                pass
        except Exception as e:
            pass

    def run_all_sequences(self):
        """运行所有序列"""
        if not self.controller.comm.is_connected:
            return
        
        if not self.action_sequences:
            return
        
        try:
            self.loop_count = int(self.loop_count_var.get())
            self.current_loop = 0
            self.is_running_sequences = True
            self.run_all_btn.config(state='disabled')
            self.stop_run_btn.config(state='normal')
            
            # 启动序列运行
            self.run_sequence_loop()
        except ValueError:
            pass
        except Exception as e:
            pass

    def run_sequence_loop(self):
        """运行序列循环"""
        if not self.is_running_sequences:
            return
        
        try:
            interval_time = float(self.interval_time_var.get())
            
            # 运行当前循环
            for i, positions in enumerate(self.action_sequences):
                if not self.is_running_sequences:
                    break
                
                success = self.controller.set_joint_positions(positions)
                if not success:
                    pass
                    break
                
                # 更新状态
                self.loop_status_var.set(f"循环次数:{self.loop_count}/当前循环:{self.current_loop + 1}")
                self.root.update()
                
                # 等待间隔时间
                if i < len(self.action_sequences) - 1:  # 不是最后一个位置
                    time.sleep(interval_time)
            
            self.current_loop += 1
            
            # 检查是否需要继续循环
            if self.current_loop < self.loop_count and self.is_running_sequences:
                # 继续下一个循环
                self.root.after(int(interval_time * 1000), self.run_sequence_loop)
            else:
                # 完成所有循环
                self.is_running_sequences = False
                self.run_all_btn.config(state='normal')
                self.stop_run_btn.config(state='disabled')
                self.loop_status_var.set(f"循环次数:{self.loop_count}/当前循环:{self.current_loop}")
                pass
                
        except Exception as e:
            self.is_running_sequences = False
            self.run_all_btn.config(state='normal')
            self.stop_run_btn.config(state='disabled')
            pass

    def stop_running(self):
        """停止运行"""
        self.is_running_sequences = False
        self.run_all_btn.config(state='normal')
        self.stop_run_btn.config(state='disabled')
        pass

    def save_action_data(self):
        """保存动作数据"""
        if not self.action_sequences:
            return
        
        try:
            from tkinter import filedialog
            filename = filedialog.asksaveasfilename(
                defaultextension=".json",
                filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
                title="保存动作数据"
            )
            
            if filename:
                import json
                data = {
                    'action_sequences': self.action_sequences,
                    'interval_time': self.interval_time_var.get(),
                    'loop_count': self.loop_count_var.get()
                }
                
                with open(filename, 'w', encoding='utf-8') as f:
                    json.dump(data, f, ensure_ascii=False, indent=2)
                
                pass
        except Exception as e:
            pass

    def load_action_data(self):
        """读取动作数据"""
        try:
            from tkinter import filedialog
            filename = filedialog.askopenfilename(
                filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
                title="读取动作数据"
            )
            
            if filename:
                import json
                with open(filename, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                
                # 清空当前表格
                self.sequence_tree.delete(*self.sequence_tree.get_children())
                self.action_sequences = []
                
                # 加载数据
                if 'action_sequences' in data:
                    self.action_sequences = data['action_sequences']
                    for i, positions in enumerate(self.action_sequences):
                        row_data = [i + 1] + positions
                        self.sequence_tree.insert('', 'end', values=row_data)
                
                if 'interval_time' in data:
                    self.interval_time_var.set(data['interval_time'])
                
                if 'loop_count' in data:
                    self.loop_count_var.set(data['loop_count'])
                
                pass
        except Exception as e:
            pass

    def delete_selected_row(self):
        """删除选中行"""
        selection = self.sequence_tree.selection()
        if not selection:
            self.loop_status_var.set("⚠️ 请先选择要删除的行")
            return
        
        try:
            item = selection[0]
            values = self.sequence_tree.item(item, 'values')
            if values:
                index = int(values[0]) - 1  # 获取序号
                
                # 删除数据
                if 0 <= index < len(self.action_sequences):
                    del self.action_sequences[index]
                
                # 删除表格行
                self.sequence_tree.delete(item)
                
                # 重新编号
                self.renumber_sequences()
                
                # 更新统计信息
                self.update_stats()
                
                # 更新状态
                self.loop_status_var.set(f"🗑️ 已删除行，剩余 {len(self.action_sequences)} 个动作")
                
                pass
        except Exception as e:
            self.loop_status_var.set("❌ 删除失败")
            pass

    def clear_table(self):
        """清空表格"""
        if not self.action_sequences:
            self.loop_status_var.set("⚠️ 表格已经是空的")
            return
        
        # 确认对话框
        import tkinter.messagebox as msgbox
        if msgbox.askyesno("确认清空", "确定要清空所有动作序列吗？\n此操作不可撤销！", icon='warning'):
            self.sequence_tree.delete(*self.sequence_tree.get_children())
            self.action_sequences = []
            
            # 更新统计信息
            self.update_stats()
            
            # 更新状态
            self.loop_status_var.set("🧹 表格已清空")
        else:
            self.loop_status_var.set("❌ 取消清空操作")
        pass

    def renumber_sequences(self):
        """重新编号序列"""
        items = self.sequence_tree.get_children()
        for i, item in enumerate(items):
            values = list(self.sequence_tree.item(item, 'values'))
            values[0] = i + 1  # 更新序号
            self.sequence_tree.item(item, values=values)

    def on_tree_double_click(self, event):
        """处理表格双击事件"""
        # 获取点击的位置
        item = self.sequence_tree.selection()[0] if self.sequence_tree.selection() else None
        if not item:
            return
        
        # 获取点击的列
        column = self.sequence_tree.identify_column(event.x)
        if not column:
            return
        
        # 转换列标识符为索引
        column_index = int(column.replace('#', '')) - 1
        
        # 序号列不允许编辑
        if column_index == 0:
            return
        
        # 获取单元格的位置和大小
        bbox = self.sequence_tree.bbox(item, column)
        if not bbox:
            return
        
        # 获取当前值
        values = self.sequence_tree.item(item, 'values')
        current_value = values[column_index] if column_index < len(values) else ""
        
        # 创建编辑框
        self.edit_item = item
        self.edit_column = column_index
        
        # 创建Entry控件
        self.edit_entry = tk.Entry(self.sequence_tree, justify='center')
        self.edit_entry.place(x=bbox[0], y=bbox[1], width=bbox[2], height=bbox[3])
        self.edit_entry.insert(0, str(current_value))
        self.edit_entry.select_range(0, tk.END)
        self.edit_entry.focus()
        
        # 绑定事件
        self.edit_entry.bind('<Return>', self.finish_edit)
        self.edit_entry.bind('<Escape>', self.cancel_edit)
        self.edit_entry.bind('<FocusOut>', self.finish_edit)

    def finish_edit(self, event=None):
        """完成编辑"""
        if not self.edit_entry or not self.edit_item:
            return
        
        try:
            # 获取新值
            new_value = self.edit_entry.get().strip()
            
            # 验证数值
            if new_value:
                try:
                    int_value = int(new_value)
                    # 检查范围（可选）
                    if int_value < -32768 or int_value > 32767:
                        pass
                        self.cancel_edit()
                        return
                except ValueError:
                    pass
                    self.cancel_edit()
                    return
            else:
                int_value = 0
            
            # 更新表格显示
            values = list(self.sequence_tree.item(self.edit_item, 'values'))
            values[self.edit_column] = int_value
            self.sequence_tree.item(self.edit_item, values=values)
            
            # 更新数据
            row_index = int(values[0]) - 1  # 动作序号转换为索引
            if 0 <= row_index < len(self.action_sequences):
                joint_index = self.edit_column - 1  # 跳过动作序号列
                if 0 <= joint_index < 17:
                    self.action_sequences[row_index][joint_index] = int_value
            
        except Exception as e:
            pass
        
        finally:
            self.cleanup_edit()

    def cancel_edit(self, event=None):
        """取消编辑"""
        self.cleanup_edit()

    def cleanup_edit(self):
        """清理编辑状态"""
        if self.edit_entry:
            self.edit_entry.destroy()
            self.edit_entry = None
        self.edit_item = None
        self.edit_column = None

    def run(self):
        """运行GUI"""
        try:
            self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
            self.root.mainloop()
        except KeyboardInterrupt:
            self.on_closing()

    def on_closing(self):
        """关闭窗口事件"""
        try:
            print("正在关闭程序...")
            self.stop_gui_update()
            self.controller.disconnect()
            print("程序已安全关闭")
        except Exception as e:
            print(f"关闭程序时发生错误: {e}")
        finally:
            self.root.quit()
            self.root.destroy()



def setup_logging():
    """设置日志记录"""
    import logging
    from datetime import datetime

    # 创建logs目录
    log_dir = "logs"
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)

    # 设置日志文件名
    log_filename = os.path.join(log_dir, f"dexterous_hand_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log")

    # 配置日志
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s',
        handlers=[
            logging.FileHandler(log_filename, encoding='utf-8'),
            logging.StreamHandler()
        ]
    )

    return logging.getLogger(__name__)

def main():
    """主函数"""
    try:
        # 设置日志
        logger = setup_logging()
        logger.info("O20灵巧手上位机控制程序启动")

        # 检查Python架构
        import platform
        arch = platform.architecture()[0]
        logger.info(f"Python架构: {arch}")
        logger.info("使用64位CANFD库")

        # 检查依赖
        try:
            import matplotlib
            import matplotlib.pyplot as plt
            matplotlib.use('TkAgg')  # 设置matplotlib后端

            # 配置中文字体支持，避免字体警告
            try:
                plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'DejaVu Sans']
                plt.rcParams['axes.unicode_minus'] = False
            except:
                pass

            logger.info("matplotlib库加载成功")
        except ImportError:
            logger.warning("matplotlib库未安装，触觉传感器显示将不可用")

        try:
            import numpy
            logger.info("numpy库加载成功")
        except ImportError:
            logger.error("numpy库未安装")
            print("错误: 缺少numpy库")
            print("请安装: pip install numpy")
            return

        # 创建并运行GUI
        logger.info("启动GUI界面")
        app = DexterousHandGUI()
        app.run()

        logger.info("程序正常退出")

    except ImportError as e:
        print(f"缺少依赖库: {e}")
        print("请安装所需的库:")
        print("pip install matplotlib numpy")
    except Exception as e:
        print(f"程序运行错误: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
