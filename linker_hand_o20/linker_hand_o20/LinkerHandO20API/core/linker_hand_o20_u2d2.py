#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
xc330_strict_id.py  ——  “必须显式传 motor_id”的 all-in-one 类
依赖: pip install pyserial dynamixel-sdk
"""
import time
from dynamixel_sdk import *
from dynamixel_sdk import COMM_SUCCESS
from concurrent.futures import ThreadPoolExecutor, as_completed
from typing import List, Dict, Optional, Union

# -------------------- 手册常数 --------------------
ADDR_TORQUE_ENABLE      = 64
ADDR_GOAL_POSITION      = 116
ADDR_GOAL_VELOCITY      = 104
ADDR_GOAL_CURRENT       = 102
ADDR_PRESENT_POSITION   = 132
ADDR_PRESENT_VELOCITY   = 128
ADDR_PRESENT_CURRENT    = 126
ADDR_PRESENT_VOLTAGE    = 144
ADDR_PRESENT_TEMP       = 146
ADDR_MOVING             = 122
ADDR_MIN_VOLTAGE_LIMIT  = 34
ADDR_PWM_LIMIT = 36
ADDR_MAX_VOLTAGE_LIMIT  = 32
ADDR_CURRENT_LIMIT      = 38
ADDR_VELOCITY_LIMIT     = 44
ADDR_MIN_POS_LIMIT      = 52
ADDR_MAX_POS_LIMIT      = 48
ADDR_HOMING_OFFSET   = 20     # 4 字节，单位：脉冲

PROTOCOL                = 2
DEFAULT_BAUD            = 1000000
TORQUE_ON               = 1
TORQUE_OFF              = 0
PULSE_PER_REV           = 4096
CURRENT_UNIT            = 1.0
VELOCITY_UNIT           = 0.229
VOLTAGE_UNIT            = 0.1
TEMP_UNIT               = 1.0
MAX_ID_PER_FRAME = 16  # Dynamixel 协议上限

# PID 范围 XC330为电机内部PID控制，只能通过读写寄存器进行电机内部PID调节
ADDR_POS_P_GAIN = 84
ADDR_POS_I_GAIN = 82
ADDR_POS_D_GAIN = 80
ADDR_VEL_P_GAIN = 78
ADDR_VEL_I_GAIN = 76
ADDR_CUR_P_GAIN = 74
ADDR_CUR_I_GAIN = 72

 # 固定帧（21-25），直接下发
RAW_FRAMES = {
    21: bytes.fromhex('FF FF FD 00 21 07 00 02 90 00 02 00 D7 11'),
    22: bytes.fromhex('FF FF FD 00 22 07 00 02 90 00 02 00 DD 21'),
    23: bytes.fromhex('FF FF FD 00 23 07 00 02 90 00 02 00 DB 31'),
    24: bytes.fromhex('FF FF FD 00 24 07 00 02 90 00 02 00 C9 41'),
    25: bytes.fromhex('FF FF FD 00 25 07 00 02 90 00 02 00 CF 51'),
}


# ===================================================================
class LinerHandO20U2D2:
    """
    用法：任何操作都必须显式传 motor_id
    例：
        xc = LinerHandO20U2D2(port="/dev/ttyUSB0", baud=1_000_000)
        ids = xc.scan_ids()                     # 扫号
        xc.set_torque(5, True)                  # ID=5 上电
        xc.set_angle(5, 90)                     # 转到 90°
        data = xc.read_all(ids)                 # 批量读
    """


   

    def __init__(self, port: str = "/dev/ttyUSB0", baud: int = DEFAULT_BAUD):
        self.port_h = PortHandler(port)
        self.pack_h = PacketHandler(PROTOCOL)
        self.bulk_read = GroupBulkRead(self.port_h, self.pack_h)
        if not self.port_h.openPort():
            raise RuntimeError(f"无法打开串口 {port}")
        if not self.port_h.setBaudRate(baud):
            raise RuntimeError(f"波特率 {baud} 设置失败")


    # ------------------------------------------------------
    #  底层通信
    # ------------------------------------------------------
    def _write1(self, motor_id: int, addr: int, val: int):
        res, err = self.pack_h.write1ByteTxRx(self.port_h, motor_id, addr, val)
        if res != COMM_SUCCESS:
            raise RuntimeError(f"通信错误：{self.pack_h.getTxRxResult(res)}")
        if err != 0:
            raise RuntimeError(f"电机返回错误：{self.pack_h.getRxPacketError(err)}")

    def _write2(self, motor_id: int, addr: int, val: int):
        res, err = self.pack_h.write2ByteTxRx(self.port_h, motor_id, addr, val)
        if res != COMM_SUCCESS:
            raise RuntimeError(f"通信错误：{self.pack_h.getTxRxResult(res)}")
        if err != 0:
            raise RuntimeError(f"电机返回错误：{self.pack_h.getRxPacketError(err)}")

    def _write4(self, motor_id: int, addr: int, val: int):
        res, err = self.pack_h.write4ByteTxRx(self.port_h, motor_id, addr, val)
        if res != COMM_SUCCESS:
            raise RuntimeError(f"通信错误：{self.pack_h.getTxRxResult(res)}")
        if err != 0:
            raise RuntimeError(f"电机返回错误：{self.pack_h.getRxPacketError(err)}")

    def _read1(self, motor_id: int, addr: int) -> int:
        data, res, err = self.pack_h.read1ByteTxRx(self.port_h, motor_id, addr)
        if res != COMM_SUCCESS:
            raise RuntimeError(f"通信错误：{self.pack_h.getTxRxResult(res)}")
        if err != 0:
            raise RuntimeError(f"电机返回错误：{self.pack_h.getRxPacketError(err)}")
        return data

    def _read2(self, motor_id: int, addr: int) -> int:
        data, res, err = self.pack_h.read2ByteTxRx(self.port_h, motor_id, addr)
        if res != COMM_SUCCESS:
            raise RuntimeError(f"通信错误：{self.pack_h.getTxRxResult(res)}")
        if err != 0:
            raise RuntimeError(f"电机返回错误：{self.pack_h.getRxPacketError(err)}")
        return data

    def _read4(self, motor_id: int, addr: int) -> int:
        data, res, err = self.pack_h.read4ByteTxRx(self.port_h, motor_id, addr)
        if res != COMM_SUCCESS:
            raise RuntimeError(f"通信错误：{self.pack_h.getTxRxResult(res)}")
        if err != 0:
            raise RuntimeError(f"电机返回错误：{self.pack_h.getRxPacketError(err)}")
        return data

    # ------------------------------------------------------
    #  PID 增益读写（单个电机）
    # ------------------------------------------------------
    def set_pos_pid(self, motor_id: int, p: int, i: int, d: int):
        self._write2(motor_id, ADDR_POS_P_GAIN, p)
        self._write2(motor_id, ADDR_POS_I_GAIN, i)
        self._write2(motor_id, ADDR_POS_D_GAIN, d)

    def get_pos_pid(self, motor_id: int) -> Dict[str, int]:
        return {
            "P": self._read2(motor_id, ADDR_POS_P_GAIN),
            "I": self._read2(motor_id, ADDR_POS_I_GAIN),
            "D": self._read2(motor_id, ADDR_POS_D_GAIN),
        }

    def set_vel_pi(self, motor_id: int, p: int, i: int):
        self._write2(motor_id, ADDR_VEL_P_GAIN, p)
        self._write2(motor_id, ADDR_VEL_I_GAIN, i)

    def get_vel_pi(self, motor_id: int) -> Dict[str, int]:
        return {
            "P": self._read2(motor_id, ADDR_VEL_P_GAIN),
            "I": self._read2(motor_id, ADDR_VEL_I_GAIN),
        }

    def set_cur_pi(self, motor_id: int, p: int, i: int):
        self._write2(motor_id, ADDR_CUR_P_GAIN, p)
        self._write2(motor_id, ADDR_CUR_I_GAIN, i)

    def get_cur_pi(self, motor_id: int) -> Dict[str, int]:
        return {
            "P": self._read2(motor_id, ADDR_CUR_P_GAIN),
            "I": self._read2(motor_id, ADDR_CUR_I_GAIN),
        }
    def set_pos_pid_sync(self, pid_dict: Dict[int, Dict[str, int]]):
        """
        批量设置位置 PID
        pid_dict = {
            1: {"P": 800, "I": 0, "D": 0},
            2: {"P": 640, "I": 0, "D": 0},
        }
        """
        p_vals = {i: pid_dict[i]["P"] for i in pid_dict}
        i_vals = {i: pid_dict[i]["I"] for i in pid_dict}
        d_vals = {i: pid_dict[i]["D"] for i in pid_dict}
        self._sync_write_dict(p_vals, ADDR_POS_P_GAIN, 2)
        self._sync_write_dict(i_vals, ADDR_POS_I_GAIN, 2)
        self._sync_write_dict(d_vals, ADDR_POS_D_GAIN, 2)

    def get_pos_pid_sync(self, ids: List[int]) -> Dict[int, Dict[str, int]]:
        """
        批量读取位置环 PID 增益
        返回: {id: {"P": int, "I": int, "D": int}}
        使用 GroupSyncRead，自动 ≤16 分段
        """
        MAX_ID = 20
        results: Dict[int, Dict[str, int]] = {}

        # 一次读 3 个寄存器，每个 2 字节
        addr_map = {
            "P": ADDR_POS_P_GAIN,
            "I": ADDR_POS_I_GAIN,
            "D": ADDR_POS_D_GAIN,
        }

        for i in range(0, len(ids), MAX_ID):
            chunk = ids[i:i + MAX_ID]

            # 为 P/I/D 分别建 SyncRead 对象
            readers = {
                key: GroupSyncRead(self.port_h, self.pack_h, addr, 2)
                for key, addr in addr_map.items()
            }

            # 添加参数
            for key in readers:
                for cid in chunk:
                    if not readers[key].addParam(cid):
                        raise RuntimeError(f"SyncRead 添加 ID{cid} 失败（{key}）")

            # 发送并接收
            for key in readers:
                res = readers[key].txRxPacket()
                if res != COMM_SUCCESS:
                    lost = [c for c in chunk if not readers[key].isAvailable(c, addr_map[key], 2)]
                    raise RuntimeError(f"SyncRead {key} 丢包 ID={lost} 错误：{self.pack_h.getTxRxResult(res)}")

            # 取数据
            for cid in chunk:
                results[cid] = {
                    "P": readers["P"].getData(cid, ADDR_POS_P_GAIN, 2),
                    "I": readers["I"].getData(cid, ADDR_POS_I_GAIN, 2),
                    "D": readers["D"].getData(cid, ADDR_POS_D_GAIN, 2),
                }

        return results
    # ------------------------------------------------------
    #  扭矩
    # ------------------------------------------------------
    def set_torque(self, motor_id: int, on: bool):
        """根据电机ID控制电机使能/失能 params: id, True/False"""
        self._write1(motor_id, ADDR_TORQUE_ENABLE, TORQUE_ON if on else TORQUE_OFF)

    def read_torque(self, motor_id: int) -> bool:
        """返回 True 表示扭矩开"""
        return self._read1(motor_id, ADDR_TORQUE_ENABLE) == TORQUE_ON

    # ------------------------------------------------------
    #  角度
    # ------------------------------------------------------
    def set_angle(self, motor_id: int, degree: float, blocking: bool = False, timeout: float = 2.0):
        pulse = int(round((degree % 360) / 360.0 * PULSE_PER_REV)) & 0xFFF
        self._write4(motor_id, ADDR_GOAL_POSITION, pulse)
        if blocking:
            t0 = time.time()
            while self.is_moving(motor_id):
                if time.time() - t0 > timeout:
                    raise RuntimeError("角度运动超时")
                time.sleep(0.001)

    def read_angle(self, motor_id: int) -> float:
        pulse = self._read4(motor_id, ADDR_PRESENT_POSITION)
        return (pulse % PULSE_PER_REV) / PULSE_PER_REV * 360.0

    # ------------------------------------------------------
    #  速度
    # ------------------------------------------------------
    def set_velocity(self, motor_id: int, rpm: float):
        pulse = int(round(abs(rpm) / VELOCITY_UNIT))
        if rpm < 0:
            pulse = -pulse
        self._write4(motor_id, ADDR_GOAL_VELOCITY, pulse & 0xFFFFFFFF)

    def read_velocity(self, motor_id: int) -> float:
        raw = self._read4(motor_id, ADDR_PRESENT_VELOCITY)
        if raw & 0x80000000:
            raw -= 0x100000000
        return raw * VELOCITY_UNIT

    # ------------------------------------------------------
    #  电流
    # ------------------------------------------------------
    def set_current(self, motor_id: int, ma: int):
        self._write2(motor_id, ADDR_GOAL_CURRENT, ma)

    def read_current(self, motor_id: int) -> int:
        return self._read2(motor_id, ADDR_PRESENT_CURRENT)

    # ------------------------------------------------------
    #  温度 / 电压 / 运动状态
    # ------------------------------------------------------
    def read_temperature(self, motor_id: int) -> float:
        return self._read1(motor_id, ADDR_PRESENT_TEMP)

    def read_voltage(self, motor_id: int) -> float:
        return self._read2(motor_id, ADDR_PRESENT_VOLTAGE) * VOLTAGE_UNIT

    def is_moving(self, motor_id: int) -> bool:
        return bool(self._read1(motor_id, ADDR_MOVING))

    # ------------------------------------------------------
    #  扫号
    # ------------------------------------------------------
    def scan_ids(self, start: int = 1, end: int = 20, print_progress: bool = False) -> List[int]:
        online = []
        for cid in range(start, end + 1):
            #if print_progress and cid % 20 == 0:
            #print(f"scan ... {cid}", end="\r")
            model, res, err = self.pack_h.ping(self.port_h, cid)
            if res == COMM_SUCCESS and err == 0:
                #print(f"电机ID:{cid} 已在线", flush=True)
                online.append(cid)
            else:
                print(f"电机ID:{cid} 离线中...", flush=True)
        if print_progress:
            print("scan done !        ")
        return online
    
    

    # ------------------------------------------------------
    #  GroupBulkRead 读取方法，用于获取各种状态 经过验证，GroupBulkRead比GroupSyncRead速度快
    # ------------------------------------------------------
    def read_all_angle_bulk_safe(self, ids: List[int]) -> Dict[int, float]:
        """
        用GroupBulkRead 批量读取电机角度
        返回 {id: 角度(0~360)}
        """
        MAX_SEG = 16  # DYNAMIXEL 协议上限
        angles = {}
        for i in range(0, len(ids), MAX_SEG):
            chunk = ids[i:i + MAX_SEG]
            self.bulk_read.clearParam()
            for cid in chunk:
                ok = self.bulk_read.addParam(cid, ADDR_PRESENT_POSITION, 4)
                if not ok:
                    raise RuntimeError(f"无法添加 ID{cid} 到 Bulk")
            res = self.bulk_read.txRxPacket()
            if res != COMM_SUCCESS:
                # 把具体丢包的 ID 打印出来，方便定位
                lost = [c for c in chunk
                        if self.bulk_read.isAvailable(c, ADDR_PRESENT_POSITION, 4) is False]
                raise RuntimeError(f"Bulk 丢包 ID={lost} 错误：{self.pack_h.getTxRxResult(res)}")
            angles.update({cid: self.bulk_read.getData(cid, ADDR_PRESENT_POSITION, 4)
                           for cid in chunk})
        return {i: (p % PULSE_PER_REV) / PULSE_PER_REV * 360.0 for i, p in angles.items()}

    def read_all_current_bulk(self, ids: List[int]) -> Dict[int, int]:
        """同时获取所有电机电流返回 {id: 电流(mA)}"""
        self.bulk_read.clearParam()
        for i in ids:
            ok = self.bulk_read.addParam(i, ADDR_PRESENT_CURRENT, 2)
            if not ok:
                raise RuntimeError(f"Bulk 添加电流 ID{i} 失败")
        res = self.bulk_read.txRxPacket()
        if res != COMM_SUCCESS:
            lost = [c for c in ids if not self.bulk_read.isAvailable(c, ADDR_PRESENT_CURRENT, 2)]
            raise RuntimeError(f"Bulk 电流丢包 ID={lost} 错误：{self.pack_h.getTxRxResult(res)}")
        return {i: self.bulk_read.getData(i, ADDR_PRESENT_CURRENT, 2) for i in ids}

    def read_all_torque_bulk(self, ids: List[int]) -> Dict[int, bool]:
        """同时获取所有电机扭矩返回 {id: True=扭矩开}"""
        self.bulk_read.clearParam()
        for i in ids:
            ok = self.bulk_read.addParam(i, ADDR_TORQUE_ENABLE, 1)
            if not ok:
                raise RuntimeError(f"Bulk 添加扭矩 ID{i} 失败")
        res = self.bulk_read.txRxPacket()
        if res != COMM_SUCCESS:
            lost = [c for c in ids if not self.bulk_read.isAvailable(c, ADDR_TORQUE_ENABLE, 1)]
            raise RuntimeError(f"Bulk 扭矩丢包 ID={lost} 错误：{self.pack_h.getTxRxResult(res)}")
        return {i: self.bulk_read.getData(i, ADDR_TORQUE_ENABLE, 1) == 1 for i in ids}

    def read_all_temperature_bulk(self, ids: List[int]) -> Dict[int, float]:
        """同时获取所有电机温度返回 {id: 温度(℃)}"""
        self.bulk_read.clearParam()
        for i in ids:
            ok = self.bulk_read.addParam(i, ADDR_PRESENT_TEMP, 1)
            if not ok:
                raise RuntimeError(f"Bulk 添加温度 ID{i} 失败")
        res = self.bulk_read.txRxPacket()
        if res != COMM_SUCCESS:
            lost = [c for c in ids if not self.bulk_read.isAvailable(c, ADDR_PRESENT_TEMP, 1)]
            raise RuntimeError(f"Bulk 温度丢包 ID={lost} 错误：{self.pack_h.getTxRxResult(res)}")
        return {i: self.bulk_read.getData(i, ADDR_PRESENT_TEMP, 1) for i in ids}

    def read_all_voltage_bulk(self, ids: List[int]) -> Dict[int, float]:
        """同时获取所有电机电压返回 {id: 电压(V)}"""
        self.bulk_read.clearParam()
        for i in ids:
            ok = self.bulk_read.addParam(i, ADDR_PRESENT_VOLTAGE, 2)
            if not ok:
                raise RuntimeError(f"Bulk 添加电压 ID{i} 失败")
        res = self.bulk_read.txRxPacket()
        if res != COMM_SUCCESS:
            lost = [c for c in ids if not self.bulk_read.isAvailable(c, ADDR_PRESENT_VOLTAGE, 2)]
            raise RuntimeError(f"Bulk 电压丢包 ID={lost} 错误：{self.pack_h.getTxRxResult(res)}")
        return {i: self.bulk_read.getData(i, ADDR_PRESENT_VOLTAGE, 2) * 0.1 for i in ids}

    def read_all_velocity_bulk(self, ids: List[int]) -> Dict[int, float]:
        """同时获取所有电机速度返回 {id: 速度(rpm)，负值表示反向}"""
        self.bulk_read.clearParam()
        for i in ids:
            ok = self.bulk_read.addParam(i, ADDR_PRESENT_VELOCITY, 4)
            if not ok:
                raise RuntimeError(f"Bulk 添加速度 ID{i} 失败")
        res = self.bulk_read.txRxPacket()
        if res != COMM_SUCCESS:
            lost = [c for c in ids if not self.bulk_read.isAvailable(c, ADDR_PRESENT_VELOCITY, 4)]
            raise RuntimeError(f"Bulk 速度丢包 ID={lost} 错误：{self.pack_h.getTxRxResult(res)}")
        raw = {i: self.bulk_read.getData(i, ADDR_PRESENT_VELOCITY, 4) for i in ids}
        return {i: (p if p < 0x80000000 else p - 0x100000000) * VELOCITY_UNIT for i, p in raw.items()}

    # ------------------------------------------------------
    #  GroupBulkWrite 写入，用于各种控制,私有快捷通道：u8/u16/u32
    # ------------------------------------------------------
    def _bulk_write_dict(self, id_val: Dict[int, int], addr: int, length: int) -> None:
        ids = list(id_val.keys())
        for i in range(0, len(ids), 16):
            chunk = ids[i:i + 16]
            # ****** 每次 new ******
            gbw = GroupBulkWrite(self.port_h, self.pack_h)
            for cid in chunk:
                v = id_val[cid]
                if length == 1:
                    gbw.addParam(cid, addr, 1, [v])
                elif length == 2:
                    gbw.addParam(cid, addr, 2, [v & 0xFF, (v >> 8) & 0xFF])
                elif length == 4:
                    gbw.addParam(cid, addr, 4,
                                [v & 0xFF, (v >> 8) & 0xFF,
                                (v >> 16) & 0xFF, (v >> 24) & 0xFF])
            res = gbw.txPacket()
            if res != COMM_SUCCESS:
                raise RuntimeError(f"BulkWrite 错误（addr=0x{addr:02X}）：{self.pack_h.getTxRxResult(res)}")

    # ===================== 写入电流 =====================
    # def set_currents(self, curr: Dict[int, int]) -> None:
    #     """设置电流 params:{3:200, 6:300, 11:250, 18:200, 20:300} 单位 mA"""
    #     #self._bulk_write_dict(curr, ADDR_GOAL_CURRENT, 2)
    #     rated = 1100          # XC330 额定 1.1 A，可按型号改
    #     self._sync_write_dict({i: int(rated * 0.6) for i in ids},
    #                         ADDR_CURRENT_LIMIT, 2)

    # ===================== 写入扭矩 =====================
    def set_torques(self, on: Union[bool, Dict[int, bool]]) -> None:
        """统一/分别控制扭矩
           on=True/False        → 全同开关
           on={id:bool}         → 分别控制"""
        if isinstance(on, bool):
            on = {cid: on for cid in range(1, 21)}  # 1~20 全部统一
        self._bulk_write_dict({i: int(bool(v)) for i, v in on.items()}, ADDR_TORQUE_ENABLE, 1)

    # ===================== 写入电压 =====================
    def set_voltages(self, volt: Dict[int, float]) -> None:
        """设置电压 parmas:{id: 电压(V)}  写 MAX_VOLTAGE_LIMIT 寄存器（0.1 V/单位）"""
        self._bulk_write_dict({i: int(round(v * 10)) for i, v in volt.items()},
                              ADDR_MAX_VOLTAGE_LIMIT, 2)

    # ===================== 设置速度 =====================
    def set_velocities(self, rpm: Dict[int, float]) -> None:
        """设置速度 params:{id: 速度(rpm)}  负值自动变补码"""
        def to_pulse(r: float) -> int:
            p = int(abs(r) / VELOCITY_UNIT)
            return -p if r < 0 else p
        self._bulk_write_dict({i: to_pulse(rpm[i]) & 0xFFFFFFFF for i in rpm},
                              ADDR_GOAL_VELOCITY, 4)

    # ===================== 设置角度 =====================
    def set_angles(self, deg: Dict[int, float]) -> None:
        """设置角度 params:{id: 角度(°)}  0~360°自动取模"""
        def to_pulse(d: float) -> int:
            return int(round((d % 360) / 360.0 * PULSE_PER_REV)) & 0xFFF
        self._bulk_write_dict({i: to_pulse(deg[i]) for i in deg},
                              ADDR_GOAL_POSITION, 4)

    # ------------------------------------------------------
    #  GroupSyncRead读取方法，用于获取各种状态
    # ------------------------------------------------------
    '''统一底层：按 u8/u16/u32 分段 SyncRead'''
    def _sync_read_u8(self, ids: List[int], addr: int) -> Dict[int, int]:
        return self._sync_read_template(ids, addr, 1, lambda v: v)

    def _sync_read_u16(self, ids: List[int], addr: int) -> Dict[int, int]:
        return self._sync_read_template(ids, addr, 2, lambda v: v)

    def _sync_read_u32(self, ids: List[int], addr: int) -> Dict[int, int]:
        return self._sync_read_template(ids, addr, 4, lambda v: v)

    def _sync_read_template(self, ids: List[int], addr: int, length: int, convert) -> Dict[int, int]:
        MAX_ID = 16
        results: Dict[int, int] = {}
        for i in range(0, len(ids), MAX_ID):
            chunk = ids[i:i + MAX_ID]
            reader = GroupSyncRead(self.port_h, self.pack_h, addr, length)
            for cid in chunk:
                if not reader.addParam(cid):
                    raise RuntimeError(f"SyncRead 添加 ID{cid} 失败")
            res = reader.txRxPacket()
            if res != COMM_SUCCESS:
                lost = [c for c in chunk if not reader.isAvailable(c, addr, length)]
                raise RuntimeError(f"SyncRead 丢包 ID={lost} 错误：{self.pack_h.getTxRxResult(res)}")
            for cid in chunk:
                val = reader.getData(cid, addr, length)
                results[cid] = convert(val)
        return results

    # ===================== 读取电流 =====================
    def read_all_current_sync(self, ids: List[int]) -> Dict[int, int]:
        """SyncRead 批量读电流 → {id: 电流(mA)}"""
        return self._sync_read_u16(ids, ADDR_PRESENT_CURRENT)

    # ===================== 读取扭矩状态 =====================
    def read_all_torque_sync(self, ids: List[int]) -> Dict[int, bool]:
        """SyncRead 批量读扭矩使能 → {id: True=扭矩开}"""
        raw = self._sync_read_u8(ids, ADDR_TORQUE_ENABLE)
        return {i: v == 1 for i, v in raw.items()}

    # ===================== 读取温度 =====================
    def read_all_temperature_sync(self, ids: List[int]) -> Dict[int, float]:
        """SyncRead 批量读温度 → {id: 温度(℃)}"""
        return self._sync_read_u8(ids, ADDR_PRESENT_TEMP)

    # ===================== 读取电压 =====================
    def read_all_voltage_sync(self, ids: List[int]) -> Dict[int, float]:
        """SyncRead 批量读电压 → {id: 电压(V)}"""
        raw = self._sync_read_u16(ids, ADDR_PRESENT_VOLTAGE)
        return {i: v * 0.1 for i, v in raw.items()}

    # ===================== 读取速度 =====================
    def read_all_velocity_sync(self, ids: List[int]) -> Dict[int, float]:
        """SyncRead 批量读速度 → {id: 速度(rpm)，负值表示反向}"""
        raw = self._sync_read_u32(ids, ADDR_PRESENT_VELOCITY)
        return {i: (v if v < 0x80000000 else v - 0x100000000) * VELOCITY_UNIT
                for i, v in raw.items()}

    # ===================== 读取角度 =====================
    def read_all_angle_sync_safe(self, ids: List[int]) -> Dict[int, float]:
        """
        用 GroupSyncRead 批量读角度，保持与 read_all_angle_bulk_safe 相同接口
        返回 {id: 角度(0~360)}
        """
        angles: Dict[int, float] = {}
        for i in range(0, len(ids), MAX_ID_PER_FRAME):
            chunk = ids[i:i + MAX_ID_PER_FRAME]
            # 1. 新建 SyncRead 对象（每帧重新创建，避免 ID 复用冲突）
            sync_reader = GroupSyncRead(self.port_h, self.pack_h,
                                        ADDR_PRESENT_POSITION, 4)
            # 2. 添加参数
            for cid in chunk:
                if not sync_reader.addParam(cid):
                    raise RuntimeError(f"无法添加 ID{cid} 到 SyncRead")
            # 3. 发送并接收
            res = sync_reader.txRxPacket()
            if res != COMM_SUCCESS:
                lost = [c for c in chunk
                        if not sync_reader.isAvailable(c, ADDR_PRESENT_POSITION, 4)]
                raise RuntimeError(f"SyncRead 丢包 ID={lost} 错误：{self.pack_h.getTxRxResult(res)}")
            # 4. 取数据
            for cid in chunk:
                pos = sync_reader.getData(cid, ADDR_PRESENT_POSITION, 4)
                angles[cid] = round((pos % PULSE_PER_REV) / PULSE_PER_REV * 360.0, 2)
        return angles

    # -------------------------------------------------
    # GroupSyncWrite 写入方法，用于各种控制
    # -------------------------------------------------
    # def _sync_write_dict(self, id_val: Dict[int, int], addr: int, length: int) -> None:
    #     """内部通用：字典驱动 SyncWrite，自动 ≤16 分段"""
    #     ids = list(id_val.keys())
    #     for i in range(0, len(ids), 16):
    #         chunk = ids[i:i + 16]
    #         # ****** 关键：每次都 new ******
    #         gsw = GroupSyncWrite(self.port_h, self.pack_h, addr, length)
    #         for cid in chunk:
    #             v = id_val[cid]
    #             param = []
    #             for n in range(length):
    #                 param.append((v >> (8 * n)) & 0xFF)
    #             if not gsw.addParam(cid, param):
    #                 raise RuntimeError(f"SyncWrite 添加 ID{cid} 失败")
    #         res = gsw.txPacket()
    #         if res != COMM_SUCCESS:
    #             raise RuntimeError(f"SyncWrite 错误（addr=0x{addr:02X}）：{self.pack_h.getTxRxResult(res)}")

    def _sync_write_dict(self, id_val: Dict[int, int], addr: int, length: int) -> None:
        ids = list(id_val.keys())
        for i in range(0, len(ids), 16):
            chunk = ids[i:i + 16]
            gsw = GroupSyncWrite(self.port_h, self.pack_h, addr, length)  # 局部
            for cid in chunk:
                param = [(id_val[cid] >> (8 * n)) & 0xFF for n in range(length)]
                if not gsw.addParam(cid, param):
                    raise RuntimeError(f"SyncWrite 添加 ID{cid} 失败")
            res = gsw.txPacket()
            if res != COMM_SUCCESS:
                raise RuntimeError(f"SyncWrite 错误（addr=0x{addr:02X}）：{self.pack_h.getTxRxResult(res)}")
            # 无需保留，直接销毁

    # ===================== 设置电流 sync =====================

    def set_currents_safe(self, ids: Optional[List[int]] = None, cu: int = 450):
        """
        批量把 Current Limit 设为额定 mA 0-1100 mA
        默认 1-20 号，可传自己的列表
        """
        if ids is None:
            ids = list(range(1, 21))          # 整手

        RATED_MA = 1100                       # XC330 额定 1.1 A，按型号改
        limit_ma = int(cu)        # 110 mA

        # 16 个一组，避免协议帧上限
        for i in range(0, len(ids), 16):
            chunk = ids[i:i + 16]
            self._sync_write_dict({cid: limit_ma for cid in chunk},
                                ADDR_CURRENT_LIMIT, 2)
        #print(f"Current Limit 60 % → {limit_ma} mA 已下发到 ID {ids}", flush=True)

        

    # ===================== 设置扭矩 sync =====================
    def set_torques_sync(self, on: Union[bool, Dict[int, bool]]) -> None:
        """sync 设置扭矩 params: True or False or {id: True,id: False,id: True,id: False} 统一 or 分别控制扭矩
           on=True/False        → 全同开关
           on={id:bool}         → 分别控制"""
        if isinstance(on, bool):
            on = {cid: on for cid in range(1, 21)}  # 1~20 全部统一
        self._sync_write_dict({i: int(bool(v)) for i, v in on.items()}, ADDR_TORQUE_ENABLE, 1)

    # ===================== 设置电压 sync =====================
    def set_voltages_sync(self, volt: Dict[int, float]) -> None:
        """sync 设置电压 parmas:{id: 电压(V),id: 电压(V),id: 电压(V),id: 电压(V)}  写 MAX_VOLTAGE_LIMIT 寄存器（0.1 V/单位）"""
        self._sync_write_dict({i: int(round(v * 10)) for i, v in volt.items()},
                              ADDR_MAX_VOLTAGE_LIMIT, 2)

    # ===================== 设置速度 sync =====================
    def set_velocities_sync(self, rpm: Dict[int, float]) -> None:
        """sync 设置速度 params:{id: 速度(rpm),id: 速度(rpm),id: 速度(rpm),id: 速度(rpm)}  负值自动变补码"""
        def to_pulse(r: float) -> int:
            p = int(abs(r) / VELOCITY_UNIT)
            return -p if r < 0 else p
        self._sync_write_dict({i: to_pulse(rpm[i]) & 0xFFFFFFFF for i in rpm},
                              ADDR_GOAL_VELOCITY, 4)

    # ===================== 设置角度 sync =====================
    def set_angles_sync(self, deg: Dict[int, float]) -> None:
        """sync 设置角度 params:{id: 角度(°),id: 角度(°),id: 角度(°)}  0~360°自动取模"""
        def to_pulse(d: float) -> int:
            return int(round((d % 360) / 360.0 * PULSE_PER_REV)) & 0xFFF
        self._sync_write_dict({i: to_pulse(deg[i]) for i in deg},
                              ADDR_GOAL_POSITION, 4)


    



    def set_velocity_limits(self, limit_dict: Dict[int, float]) -> None:  
        """  
        批量设置多个电机的速度阈值  
        :param limit_dict: {电机ID: 最大速度阈值(RPM)}，如 {1: 300, 2: 250}  
        """  
        def rpm_to_unit(rpm: float) -> int:  
            if rpm < 0:  
                raise ValueError(f"速度阈值不能为负数（ID={motor_id}）")  # 修正变量名  
            return int(round(rpm / VELOCITY_UNIT)) & 0xFFFF  # 16位无符号整数  

        unit_dict = {}  
        for motor_id, rpm in limit_dict.items():  # 循环变量为motor_id
            print(f"设置电机速度: {motor_id}-{rpm:.2f} RPM", flush=True)   
            unit_dict[motor_id] = rpm_to_unit(rpm)  

        self._sync_write_dict(unit_dict, ADDR_VELOCITY_LIMIT, 2)  # 同步写入寄存器
        

    def _read2_no_raise(self, motor_id: int, addr: int) -> int:
        """
        与 _read2 相同，但 Hardware-Error 位为 1 时不抛 RxPacketError，
        而是把错误码打印后返回 -1
        """
        from dynamixel_sdk import COMM_SUCCESS
        data, res, err = self.pack_h.read2ByteTxRx(self.port_h, motor_id, addr)
        if res != COMM_SUCCESS:
            print(f"ID{motor_id:02d} 通信失败：{self.pack_h.getTxRxResult(res)}")
            return -1
        # err 的 bit7 是 Hardware Error，我们只关心数据，忽略它
        return data

    
    # ========== 单个/批量重启（协议 0x08） ==========
    def reboot(self, motor_ids: Union[int, list], blocking: bool = True, timeout: float = 1.0) -> None:
        """
        发送 Reboot(0x08) 指令并等待重新上线
        motor_ids : 单个 ID 或 ID 列表
        blocking  : True=阻塞直到 ping 通；False=发完就返回
        timeout   : 单台最长等待（秒）
        """
        if isinstance(motor_ids, int):
            motor_ids = [motor_ids]

        pkt = PacketHandler(2)
        for cid in motor_ids:
            res, err = pkt.reboot(self.port_h, cid)   # 官方已封装
            if res != COMM_SUCCESS:
                raise RuntimeError(f"ID{cid} 重启失败：{pkt.getTxRxResult(res)}")
            if err != 0:
                raise RuntimeError(f"ID{cid} 返回错误：{pkt.getRxPacketError(err)}")

        if not blocking:
            return

        # 阻塞等待全部重新上线
        t0 = time.time()
        remain = set(motor_ids)
        while remain:
            if time.time() - t0 > timeout * len(motor_ids):
                raise RuntimeError(f"ID {sorted(remain)} 重启后未上线")
            for cid in list(remain):
                _, res, err = pkt.ping(self.port_h, cid)
                if res == COMM_SUCCESS and err == 0:
                    remain.discard(cid)
            time.sleep(0.02)


    def set_all_current_position_mode(self, ids: List[int] = None) -> None:
        """
        批量切换到电流-位置模式（0x05）
        默认 1-20 号，可自定义列表
        """
        if ids is None:
            ids = list(range(1, 21))
        self._sync_write_dict({i: 0x05 for i in ids}, 11, 1)
        time.sleep(0.1)
        # 3. 批量重启（软重启）
        self.reboot(ids, blocking=True, timeout=1.2)
        time.sleep(0.1)
    


    def clear_error(self, motor_id: int) -> bool:
        try:
            self._write1(motor_id, ADDR_TORQUE_ENABLE, TORQUE_OFF)
            hw = self._read2(motor_id, 70)
            self._write1(motor_id, ADDR_TORQUE_ENABLE, TORQUE_ON)
            return True
        except:
            return False
        

    # ------------------------------------------------------
    #  O20P 力传感器（1 M，共用总线）—— 零拼接版本
    # ------------------------------------------------------

    def _tx_rx_force(self, frame: bytes) -> bytes:
        #print('TX:', frame.hex(' ').upper())
        self.port_h.clearPort()
        self.port_h.writePort(frame)
        time.sleep(0.02)          # 3 ms 保险
        ans = self.port_h.readPort(30)  # 多读点，防粘包
        time.sleep(0.013)
        #print('RX:', ans.hex(' ').upper() if ans else 'None')
        return ans

    def read_force(self, dev_id: int) -> int:
        '''
        21: bytes.fromhex('FF FF FD 00 21 07 00 02 90 00 02 00 D7 11'),
        22: bytes.fromhex('FF FF FD 00 22 07 00 02 90 00 02 00 DD 21'),
        23: bytes.fromhex('FF FF FD 00 23 07 00 02 90 00 02 00 DB 31'),
        24: bytes.fromhex('FF FF FD 00 24 07 00 02 90 00 02 00 C9 41'),
        25: bytes.fromhex('FF FF FD 00 25 07 00 02 90 00 02 00 CF 51'),
        '''
        if dev_id not in RAW_FRAMES:
            return -1                      # 无固定帧也归 -1
        ans = self._tx_rx_force(RAW_FRAMES[dev_id])
        if not ans or len(ans) < 12:
            #print(f'ID{dev_id} 空包或太短，跳过')
            return -1                      # 空包 → -1
        idx = ans.find(b'\x55\x00')
        if idx == -1 or len(ans) < idx + 4:
            #print(f'ID{dev_id} 无合法状态，跳过')
            return -1                      # 无状态 → -1
        force = int.from_bytes(ans[idx+2:idx+4], 'little', signed=True)
        return force

    def read_force_all(self, ids: List[int] = list(range(21, 26))) -> List[int]:
        """返回 List[int]，顺序与 ids 一致，无应答 = -1"""
        return [self.read_force(i) if i in RAW_FRAMES else -1 for i in ids]

    def set_force_id(self, old_id: int, new_id: int):
        raise NotImplementedError('固定帧版本不提供改 ID 功能')

    

    # ------------------------------------------------------
    #  关闭串口
    # ------------------------------------------------------
    def close(self):
        self.port_h.closePort()
    def __enter__(self):
        return self
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()


if __name__ == "__main__":
    hand = LinerHandO20U2D2(port="/dev/ttyUSB0", baud=1000000)
    ids = hand.scan_ids(print_progress=True)
    angles = hand.read_all_angle_sync_safe(ids)
    print(angles)
    # hand.set_torques(True)
    # t0 = time.time()
    # 1. 读指尖力（传感器 ID=0x02）
    print('ID25 压力:', hand.read_force(25), 'g')
    # 批量读 21-25
    data = hand.read_force_all([21,22,23,24,25])
    # print(data, flush=True)
    for i, g in enumerate(data, 21):
        print(f'ID{i}: {g} g')
    # ==========================GroupBulkRead===================================
    '''
    # ==== GroupBulkRead 读取示例 =======
    angles = hand.read_all_angle_bulk_safe(ids)
    currents = hand.read_all_current_bulk(ids)
    torques = hand.read_all_torque_bulk(ids)
    temps = hand.read_all_temperature_bulk(ids)
    volts = hand.read_all_voltage_bulk(ids)
    vels = hand.read_all_velocity_bulk(ids)
    # ==== GroupBulkRead 设置示例 =======
    # 1. 电流
    hand.set_currents({3: 200, 6: 300, 11: 250, 18: 200, 20: 300})
    # 2. 扭矩（全部上电）
    hand.set_torques(True)
    # 3. 电压限制
    hand.set_voltages({1: 12.0, 2: 11.5, 5: 12.5})
    # 4. 速度（负值=反向）
    hand.set_velocities({1: 30.0, 2: -15.5, 7: 0.0})
    # 5. 角度
    hand.set_angles({1: 0, 2: 45, 3: 90, 16: 180, 20: 270})
    '''
    # ==========================GroupSyncRead===================================
    '''
    # ==== GroupSyncRead 读取示例 =======
    # angles = hand.read_all_angle_sync_safe(ids)
    # currs = hand.read_all_current_sync(ids)
    # torques = hand.read_all_torque_sync(ids)
    # temps = hand.read_all_temperature_sync(ids)
    # volts = hand.read_all_voltage_sync(ids)
    # vels = hand.read_all_velocity_sync(ids)

    # ==== GroupSyncWrite 设置示例 =======
    # 1. 电流
    hand.set_currents_sync({3: 200, 6: 300, 11: 250, 18: 200, 20: 300})
    # 2. 扭矩（全部上电）
    hand.set_torques_sync(True)
    # 3. 电压限制
    hand.set_voltages_sync({1: 12.0, 2: 11.5, 5: 12.5})
    # 4. 速度（负值=反向）
    hand.set_velocities_sync({1: 30.0, 2: -15.5, 7: 0.0})
    # 5. 角度
    hand.set_angles_sync({1: 0, 2: 45, 3: 90, 16: 180, 20: 270})
    '''
    # =========================== PID控制 ===============================
    # 设置 ID=5 的位置 PID
    #hand.set_pos_pid(5, p=800, i=0, d=0)
    # 读取
    # print(hand.get_pos_pid(5))  # {'P': 800, 'I': 0, 'D': 0}



    # 批量读取位置 PID
    



    # for i in ids:
    #     print(f"ID{i} 角度={angles[i]:6.2f}° 电流={currents[i]:4.0f}mA "
    #         f"扭矩={'ON' if torques[i] else 'OFF'} 温度={temps[i]:4.1f}℃ "
    #         f"电压={volts[i]:4.1f}V 速度={vels[i]:6.1f}rpm")
    # print("总耗时:", time.time() - t0)
