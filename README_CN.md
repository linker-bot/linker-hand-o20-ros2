# LinkerHand灵巧手ROS2 SDK

## 概述
LinkerHand O20灵巧手ROS2 SDK 是灵心巧手(北京)科技有限公司开发，用于O20 LinkerHand灵巧手的驱动软件和功能示例源码。可用于真机与仿真器使用。
LinkerHandROS2 SDK当前支持Ubuntu22.04 ROS humble Python3.10 及以上环境

## 安装
&ensp;&ensp;当前系统环境为Ubuntu24 ROS 2 Jazzy Python3.11
- 下载GitHub可能需要梯子

```bash
  $ mkdir -p linker_hand_o20_ros2/src
  $ cd linker_hand_o20_ros2/src
  $ git clone https://github.com/linker-bot/linker-hand-o20-ros2.git
```

- 编译

```bash
  $ pip install dynamixel_sdk --break-system-packages  # U2D2需要系统权限安装，否则会报错
  $ cd linker_hand_o20_ros2/src/
  $ pip install -r requirements.txt
```

## 使用 for Ubuntu
- 查看设备端口,先将USB设备插入Ubuntu设备上，在没有除鼠标键盘以外的其他USB设备的情况下，一般被识别为ttyUSB0
```bash
$ ls /dev/ttyUSB* # 查看USB设备端口
$ udevadm info -a -p $(udevadm info -q path -n /dev/ttyUSB0)
# 找到如下类似信息
ATTRS{idVendor}=="0403" # 将值改为查到的数值
ATTRS{idProduct}=="6014"
ATTRS{serial}=="FT12345678"
# 将设备信息写入udev规则文件
$ sudo vim /etc/udev/rules.d/99-ft232h.rules
# 写入以下内容,将值改为查到的数值 SYMLINK的值改为你期望的字符串，然后保存退出  注意：遥操模式下，应该是两个设备，注意区分SYMLINK名称
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", ATTRS{serial}=="FTAA09QS", MODE="0666",  OWNER="root", SYMLINK+="O20_right"

SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", ATTRS{serial}=="FTAA08AV", MODE="0666",  OWNER="root", SYMLINK+="O20_left"


SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", ATTRS{serial}=="FTAA07LJ", MODE="0666",  OWNER="root", SYMLINK+="O20_right_master"

# 重载 udev 规则
$ sudo udevadm control --reload-rules
$ sudo udevadm trigger
# 验证
$ ls -l /dev/
# 找到如下信息即配置成功
lrwxrwxrwx   1 root       root           7 10月 11 08:50 O20_right -> ttyUSB0
```

&ensp;&ensp; __使用前请先将单手[linker_hand_o20.launch.py](https://github.com/linker-bot/linker-hand-o20-ros2/blob/main/linker_hand_o20/launch/linker_hand_o20.launch.py) or 双手[linker_hand_o20.launch.py](https://github.com/linker-bot/linker-hand-o20-ros2/blob/main/linker_hand_o20/launch/linker_hand_o20.launch.py)文件按照实际灵巧手参数进行配置.__

- 启动SDK单手&ensp;&ensp;&ensp;&ensp;将linker_hand灵巧手的USB设备插入Ubuntu设备上  支持型号:O20
- 启动SDK双手&ensp;&ensp;&ensp;&ensp;先将左手linker_hand灵巧手的USB设备插入Ubuntu设备上，一般被识别为/dev/ttyUSB0。再将右手linker_hand灵巧手的USB设备插入Ubuntu设备上，一般识别为/dev/ttyUSB1.  支持型号:O20
```bash

  $ cd linker_hand_o20_ros2/
  $ colcon build --symlink-install
  $ source ./install/setup.bash
  $ sudo chmod a+x src/linker_hand_o20/linker_hand_o20/linker_hand_o20.py
  $ # 单手
  $ ros2 launch linker_hand_o20 linker_hand_o20.launch.py
  # 显示一下信息即启动成功
  $ [linker_hand_o20-1] 2025-10-11 09:22:20  扫描电机中....
  $ [linker_hand_o20-1] scan done !
  $ [linker_hand_o20-1] 2025-10-11 09:22:20  电机ID:1 以在线，扭矩设置成功
  $ [linker_hand_o20-1] 2025-10-11 09:22:20  电机ID:2 以在线，扭矩设置成功
  $ [linker_hand_o20-1] 2025-10-11 09:22:20  电机ID:3 以在线，扭矩设置成功
  $ [linker_hand_o20-1] 2025-10-11 09:22:20  电机ID:4 以在线，扭矩设置成功
  $ [linker_hand_o20-1] 2025-10-11 09:22:20  电机ID:5 以在线，扭矩设置成功
  $ [linker_hand_o20-1] 2025-10-11 09:22:20  电机ID:6 以在线，扭矩设置成功
  $ [linker_hand_o20-1] 2025-10-11 09:22:20  电机ID:7 以在线，扭矩设置成功
  $ [linker_hand_o20-1] 2025-10-11 09:22:20  电机ID:8 以在线，扭矩设置成功
  $ [linker_hand_o20-1] 2025-10-11 09:22:20  电机ID:9 以在线，扭矩设置成功
  $ [linker_hand_o20-1] 2025-10-11 09:22:20  电机ID:10 以在线，扭矩设置成功
  $ [linker_hand_o20-1] 2025-10-11 09:22:20  电机ID:11 以在线，扭矩设置成功
  $ [linker_hand_o20-1] 2025-10-11 09:22:20  电机ID:12 以在线，扭矩设置成功
  $ [linker_hand_o20-1] 2025-10-11 09:22:20  电机ID:13 以在线，扭矩设置成功
  $ [linker_hand_o20-1] 2025-10-11 09:22:20  电机ID:14 以在线，扭矩设置成功
  $ [linker_hand_o20-1] 2025-10-11 09:22:20  电机ID:15 以在线，扭矩设置成功
  $ [linker_hand_o20-1] 2025-10-11 09:22:20  电机ID:16 以在线，扭矩设置成功
  $ [linker_hand_o20-1] 2025-10-11 09:22:20  电机ID:17 以在线，扭矩设置成功
  $ [linker_hand_o20-1] 2025-10-11 09:22:20  电机ID:18 以在线，扭矩设置成功
  $ [linker_hand_o20-1] 2025-10-11 09:22:20  电机ID:19 以在线，扭矩设置成功
  $ [linker_hand_o20-1] 2025-10-11 09:22:20  电机ID:20 以在线，扭矩设置成功
  $ [linker_hand_o20-1] 2025-10-11 09:22:21  right_O20初始化成功
```

## 使用 for WIN+ROS2

&ensp;&ensp; __使用前请先将 [linker_hand_o20.launch.py](https://github.com/linker-bot/linker-hand-o20-ros2/blob/main/linker_hand_o20/launch/linker_hand_o20.launch.py)文件按照实际灵巧手参数进行配置.__

- 启动SDK&ensp;&ensp;&ensp;&ensp;将linker_hand灵巧手的USB转CAN设备插入WIN系统设备上  支持型号:O20
- 注：安装好USB转CAN驱动后才可使用
```bash
  $ mkdir -p linker_hand_o20_ros2/src
  $ cd linker_hand_o20_ros2/src
  $ git clone git clone https://github.com/linker-bot/linker-hand-o20-ros2.git
  $ cd linker_hand_o20_ros2/
  $ set PYTHONUTF8=1 # 设置环境变量为UTF-8编码
  $ colcon build --symlink-install
  $ call ./install/local_setup.bat
  $ ros2 launch linker_hand_o20 linker_hand_o20.launch.py #先修改launch配置文件的CAN端口名称
  # 显示一下信息即启动成功
  $ [linker_hand_o20-1] 2025-10-11 09:22:20  扫描电机中....
  $ [linker_hand_o20-1] scan done !
  $ [linker_hand_o20-1] 2025-10-11 09:22:20  电机ID:1 以在线，扭矩设置成功
  $ [linker_hand_o20-1] 2025-10-11 09:22:20  电机ID:2 以在线，扭矩设置成功
  $ [linker_hand_o20-1] 2025-10-11 09:22:20  电机ID:3 以在线，扭矩设置成功
  $ [linker_hand_o20-1] 2025-10-11 09:22:20  电机ID:4 以在线，扭矩设置成功
  $ [linker_hand_o20-1] 2025-10-11 09:22:20  电机ID:5 以在线，扭矩设置成功
  $ [linker_hand_o20-1] 2025-10-11 09:22:20  电机ID:6 以在线，扭矩设置成功
  $ [linker_hand_o20-1] 2025-10-11 09:22:20  电机ID:7 以在线，扭矩设置成功
  $ [linker_hand_o20-1] 2025-10-11 09:22:20  电机ID:8 以在线，扭矩设置成功
  $ [linker_hand_o20-1] 2025-10-11 09:22:20  电机ID:9 以在线，扭矩设置成功
  $ [linker_hand_o20-1] 2025-10-11 09:22:20  电机ID:10 以在线，扭矩设置成功
  $ [linker_hand_o20-1] 2025-10-11 09:22:20  电机ID:11 以在线，扭矩设置成功
  $ [linker_hand_o20-1] 2025-10-11 09:22:20  电机ID:12 以在线，扭矩设置成功
  $ [linker_hand_o20-1] 2025-10-11 09:22:20  电机ID:13 以在线，扭矩设置成功
  $ [linker_hand_o20-1] 2025-10-11 09:22:20  电机ID:14 以在线，扭矩设置成功
  $ [linker_hand_o20-1] 2025-10-11 09:22:20  电机ID:15 以在线，扭矩设置成功
  $ [linker_hand_o20-1] 2025-10-11 09:22:20  电机ID:16 以在线，扭矩设置成功
  $ [linker_hand_o20-1] 2025-10-11 09:22:20  电机ID:17 以在线，扭矩设置成功
  $ [linker_hand_o20-1] 2025-10-11 09:22:20  电机ID:18 以在线，扭矩设置成功
  $ [linker_hand_o20-1] 2025-10-11 09:22:20  电机ID:19 以在线，扭矩设置成功
  $ [linker_hand_o20-1] 2025-10-11 09:22:20  电机ID:20 以在线，扭矩设置成功
  $ [linker_hand_o20-1] 2025-10-11 09:22:21  right_O20初始化成功
```


## 主从遥操模式
```bash
# 修改linker_hand_o20.launch.py文件，将is_slave设置为True 被控制的从动端开启被控制模式
$ sudo vim src/linker_hand_o20/launch/linker_hand_o20.launch.py
# 修改linker_hand_o20_teleoperated_master.launch.py文件，按照参数说明修改。默认开启失能模式
$ sudo vim src/linker_hand_o20/launch/linker_hand_o20_teleoperated_master.launch.py
$ source ./install/setup.bash
# 开启从动端灵巧手O20
$ ros2 launch linker_hand_o20 linker_hand_o20.launch.py
# 开启主动遥操端灵巧手O20
$ ros2 launch linker_hand_o20 linker_hand_o20_teleoperated_master.launch.py
# 将主动遥操端食指指尖、中指指尖、无名指指尖手动向手背方向摆动到极限位置，1秒后进入遥操模式。
# 将主动遥操端食指指尖、中指指尖、手动向手背方向摆动到极限位置，将无名指指尖向手心方向摆动到极限位置，1秒后退出遥操模式。
```

## 左右双手控制
```bash
# 新开终端开启GUI控制界面
# 修改linker_hand_o20.launch.py文件，将is_slave设置为False，开启被注销掉的右手灵巧手O20，将is_slave设置为False
$ sudo vim src/linker_hand_o20/launch/linker_hand_o20.launch.py
$ source ./install/setup.bash
# 开启左右双手灵巧手O20
$ ros2 launch linker_hand_o20 linker_hand_o20.launch.py
# gui_control界面控制左右双手灵巧手O20，修改gui_control.launch.py文件中的参数，开启被注销掉的右手gui控制节点。
# 新开终端开启GUI控制界面
$ sudo vim src/gui_control/launch/gui_control.launch.py
$ source ./install/setup.bash
$ ros2 launch gui_control gui_control.launch.py
```


- position手指关节顺序

  O6:  ["拇指根部", "食指根部", "中指根部", "无名指根部", "小指根部", "拇指侧摆", "食指侧摆", "中指侧摆", "无名指侧摆", "小指侧摆", "拇指旋转", "食指中部", "中指中部", "无名指中部", "小拇指旋转", "拇指尖部", "食指末端", "中指末端", "无名指末端", "小指末端"],

## 版本更新

- > ### release_1.1.1
 - 1、支持O20版本灵巧手左右双手控制

- > ### release_1.0.1
 - 1、支持O20版本灵巧手



## [演示](rock_paper_scissors/)
- [rock_paper_scissors(石头、剪刀、布)](rock_paper_scissors/)
需要USB摄像头，通过手势识别控制LinkerHand灵巧手O20进行石头、剪刀、布的动作。修改rock_paper_scissors.launch.py文件中的参数，配置LinkerHand灵巧手类型和关节。
```bash
# 新开终端
$ cd linker_hand_o20_ros2/
$ source ./install/setup.bash
$ ros2 launch rock_paper_scissors rock_paper_scissors.launch.py
```


## 通用
- [gui_control(图形界面控制)](图形界面控制)
图形界面控制可以通过滑动块控制LinkerHand灵巧手O20各个关节独立运动。也可以通过添加按钮记录当前所有滑动块的数值，保存LinkerHand灵巧手当前各个关节运动状态。通过功能性按钮进行动作复现。    

使用gui_control控制LinkerHand灵巧手:
gui_control界面控制灵巧手需要启动linkerhand-ros2-sdk，以topic的形式对LinkerHand灵巧手进行操作
开启ROS2 SDK后

&ensp;&ensp; __使用前请先将 [gui_control.launch.py](https://github.com/linker-bot/linker-hand-o20-ros2/blob/main/gui_control/launch/gui_control.launch.py)文件按照实际灵巧手参数进行配置.__
```bash
# 新开终端
$ cd linker_hand_o20_ros2/
$ source ./install/setup.bash
$ ros2 launch gui_control gui_control.launch.py
```
开启后会弹出UI界面。通过滑动条可控制相应LinkerHand灵巧手关节运动

## WIN+ROS2环境下使用GUI
&ensp;&ensp; __使用前请先将 [gui_control.launch.py](https://github.com/linker-bot/linker-hand-o20-ros2/blob/main/gui_control/launch/gui_control.launch.py)文件按照实际灵巧手参数进行配置.__
```bash
# 新开终端
$ cd linker_hand_o20_ros2/
$ call ./install/setup.bash
$ ros2 launch gui_control gui_control.launch.py
```


## Topic List
```python
/cb_hand_setting_cmd # 设置linkerhand命令话题
/cb_left_hand_control_cmd # 控制左手运动话题 by range 0~255 (范围)
/cb_left_hand_control_cmd_angle # 控制左手运动话题 by angle 0°~360° (角度) 
/cb_left_hand_info  # 左手硬件信息显示话题
/cb_left_hand_state # 左手状态显示话题 范围
/cb_left_hand_state_angle # 左手状态显示话题 角度

/cb_right_hand_control_cmd # 控制右手运动话题 by range 0~255 (范围)
/cb_right_hand_control_cmd_angle # 控制右手运动话题 by angle 0°~360° (角度)
/cb_right_hand_info # 右手硬件信息显示话题
/cb_right_hand_state # 右手状态显示话题 范围
/cb_right_hand_state_angle # 右手状态显示话题 角度
```





