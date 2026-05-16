# ROS 系统实践 — 实验代码仓库

> ROS Noetic | 智行-W2A 机器人平台 | Ubuntu 20.04

## 仓库结构

```
djf_ros_class_ws/
└── src/
    ├── my_class_pkg/               # 第二、三、六周实验代码
    │   ├── msg/MyMessage.msg       # 自定义消息
    │   ├── srv/MyServiceMsg.srv    # 自定义服务
    │   ├── action/MyAction.action  # 自定义动作
    │   ├── cfg/Tutorials.cfg       # 动态参数配置
    │   ├── src/                    # C++ 节点
    │   ├── scripts/                # Python 节点
    │   └── launch/                 # Launch 文件
    ├── djf_robot_description/      # 第四周：两轮小车 URDF/Xacro 建模
    │   ├── urdf/
    │   │   ├── simple_robot.urdf
    │   │   └── simple_robot.xacro
    │   └── launch/
    │       ├── display.launch      # RViz 可视化
    │       └── gazebo.launch       # Gazebo 仿真
    └── zx_description/             # 第四周：W2A 机械臂模型
        ├── urdf/W2A/
        └── launch/
```

---

## 环境准备

```bash
cd ~/djf_ros_class_ws
catkin_make
source devel/setup.bash
```

> 所有传感器实验需先启动硬件通信：`roslaunch upros_bringup bringup_w2a.launch`

---

## 第二周 · 话题、服务与动作通信实验

### 实验内容
- ROS 话题（Topic）发布/订阅通信
- ROS 服务（Service）请求/响应通信
- ROS 动作（Action）异步任务通信
- 自定义消息类型、Launch 文件

### 对应代码

| 实验 | C++ 代码 | Python 代码 |
|------|---------|------------|
| 标准消息 Topic | `ros_publisher.cpp` / `ros_subscriber.cpp` | `ros_publisher_node.py` / `ros_subscriber_node.py` |
| 自定义消息 Topic | `msg_publisher.cpp` / `msg_subscriber.cpp` | — |
| Service 服务 | `ros_server.cpp` / `ros_client.cpp` | `ros_server.py` / `ros_client.py` |
| Action 动作 | `ros_action_server.cpp` / `ros_action_client.cpp` | `ros_action_server.py` / `ros_action_client.py` |

### 启动命令

```bash
# === 话题实验 ===

# C++ 标准消息
roscore
rosrun my_class_pkg ros_publisher_node     # 终端2
rosrun my_class_pkg ros_subscriber_node    # 终端3

# Python 标准消息
rosrun my_class_pkg ros_publisher_node.py
rosrun my_class_pkg ros_subscriber_node.py

# C++ 自定义消息
rosrun my_class_pkg msg_publisher_node
rosrun my_class_pkg msg_subscriber_node

# Launch 一键启动
roslaunch my_class_pkg bringup_topic.launch

# === 服务实验 ===

# C++
rosrun my_class_pkg ros_server_node
rosrun my_class_pkg ros_client_node

# Python
rosrun my_class_pkg ros_server.py
rosrun my_class_pkg ros_client.py

# === 动作实验 ===

# C++
rosrun my_class_pkg ros_action_server
rosrun my_class_pkg ros_action_client

# Python
rosrun my_class_pkg ros_action_server.py
rosrun my_class_pkg ros_action_client.py
```

### 验证

```bash
rosmsg show my_class_pkg/MyMessage
rossrv show my_class_pkg/MyServiceMsg
rosmsg show my_class_pkg/MyActionGoal
```

---

## 第二周 · 传感器实验

### 实验内容
- 碰撞传感器（Bump Sensor）数据订阅与避障
- 超声波传感器（Ultrasonic）距离测量
- TOF 传感器避障

### 对应代码

| 实验 | 代码文件 | 功能 |
|------|---------|------|
| 碰撞传感器 | `ros_bump.cpp` | 订阅打印碰撞数据 |
| 碰撞避障 | `ros_bump_avoid.cpp` | 碰撞后退+转向 |
| 超声波 | `ros_sonic.cpp` | 订阅打印超声波距离 |
| TOF 避障 | `ros_tof_avoid.cpp` | 距离<0.3m 时转向 |

### 启动命令

```bash
# 先启动硬件
roslaunch upros_bringup bringup_w2a.launch

# 碰撞传感器
rosrun my_class_pkg ros_bump_node
rosrun my_class_pkg ros_bump_avoid_node

# 超声波
rosrun my_class_pkg ros_sonic_node

# TOF 避障
rosrun my_class_pkg ros_tof_avoid_node
```

---

## 第三周 · 参数与动态参数实验

### 实验内容
- ROS 参数服务器（get/set/delete/has_param）
- 动态参数配置（dynamic_reconfigure）
- ROS 日志系统（DEBUG/INFO/WARN/ERROR/FATAL）

### 对应代码

| 实验 | C++ 代码 | Python 代码 |
|------|---------|------------|
| 参数服务器 | `ros_param.cpp` | `ros_param.py` |
| 动态参数 | `dynamic_reconfigure.cpp` | — |
| 动态调速 | `ros_dynamic_speed.cpp` | — |
| 日志系统 | `ros_log.cpp` | `ros_log.py` |

### 启动命令

```bash
# 参数实验
roslaunch my_class_pkg parameter.launch

# 动态参数
rosrun my_class_pkg dynamic_reconfigure_node
rosrun rqt_reconfigure rqt_reconfigure   # GUI 调参

# 动态调速（通过 rqt_reconfigure 调节速度）
rosrun my_class_pkg ros_dynamic_speed_node

# 日志
rosrun my_class_pkg ros_log
rosrun my_class_pkg ros_log.py
```

---

## 第四周 · 机器人建模与 Gazebo 仿真

### 实验内容
- URDF 机器人模型描述
- Xacro 参数化建模（含激光雷达）
- RViz 可视化与 Gazebo 物理仿真
- W2A 教具机机械臂模型

### 对应代码

| 包名 | 内容 |
|------|------|
| `djf_robot_description` | 两轮差速小车 URDF/Xacro + Launch |
| `zx_description` | W2A 机械臂完整模型 + Gazebo 世界 |

### 启动命令

```bash
# 两轮小车 RViz 可视化
roslaunch djf_robot_description display.launch

# 两轮小车 Gazebo 仿真
roslaunch djf_robot_description gazebo.launch

# 键盘控制（新终端）
rosrun teleop_twist_keyboard teleop_twist_keyboard.py

# W2A 机械臂 Gazebo 仿真
roslaunch zx_description w2a.launch
```

---

## 第六周 · IMU 惯性测量单元传感器实验

### 实验内容
- 订阅 IMU 数据（加速度、角速度、姿态四元数）
- RViz 中可视化 IMU 姿态
- 利用 IMU 角速度积分实现精确自旋控制（180° 旋转）

### 对应代码

| 实验 | 代码文件 | 功能 |
|------|---------|------|
| IMU 数据订阅 | `ros_imu.cpp` | 打印加速度/角速度/四元数 |
| IMU 自旋控制 | `ros_imu_spin.cpp` | 角速度积分，旋转指定圈数后停止 |

### 启动命令

```bash
# 先启动硬件
roslaunch upros_bringup bringup_w2a.launch

# 订阅 IMU 数据
rosrun my_class_pkg ros_imu_node

# RViz 可视化 IMU
# Fixed Frame 设置为 imu_link，Add → By display type → rviz_imu_plugin

# IMU 自旋控制（默认旋转 1 圈）
rosrun my_class_pkg ros_imu_spin_node

# 指定旋转圈数和速度
rosrun my_class_pkg ros_imu_spin_node _rotations:=0.5 _speed:=0.5
```

---

## 第七周 · 语音交互与大模型实验

### 实验内容
- 麦克风音频获取
- 离线语音识别（ASR）
- 大模型在线问答（Moonshot API）
- 文字转语音（TTS，VITS 模型）

### 对应代码

| 实验 | 代码文件 | 功能 |
|------|---------|------|
| 语音控制 | `voice_control.py` | 语音意图解析，发布控制指令 |
| 分词器 | `tokenizer.py` | 中文分词 + 意图提取 |
| 大模型对话 | `llm_chat.py` | Moonshot API 调用 |
| 语音合成 | `tts_player.py` | VITS 端到端语音合成 |

### 启动命令

```bash
# 先启动硬件 + 语音模块
roslaunch upros_bringup bringup_w2a.launch
roslaunch upros_chat speech_to_word.launch    # 语音识别
roslaunch upros_chat word_to_speech.launch    # 语音合成

# 语音控制节点
rosrun my_class_pkg voice_control.py

# 大模型对话节点
rosrun my_class_pkg llm_chat.py
```

---

## 技术栈

| 组件 | 版本/型号 |
|------|----------|
| ROS | Noetic (Ubuntu 20.04) |
| 机器人平台 | 智行-W2A (双轮差速) |
| 激光雷达 | BlurSea E200 |
| 深度相机 | Orbbec DaBai DCW2 |
| IMU | 板载 MPU |
| 语音合成 | sherpa-onnx VITS |
| 大模型 | Moonshot (Kimi) |
