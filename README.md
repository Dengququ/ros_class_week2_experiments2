# D_ROS_CLASS_WS — 第二周 ROS 实验工作空间

## 工作空间结构

```
d_ros_class_ws/
└── src/
    └── my_class_pkg/
        ├── msg/          # 自定义消息
        ├── srv/          # 自定义服务
        ├── action/       # 自定义动作
        ├── src/          # C++ 节点
        ├── scripts/      # Python 节点
        └── launch/       # Launch 文件
```

---

## 环境准备

```bash
cd ~/d_ros_class_ws
catkin_make
source devel/setup.bash
```

---

## 启动指令

### 5.1 C++ 标准消息（Topic）

```bash
# 终端1 — 启动 ROS Master
roscore

# 终端2 — 发布者
rosrun my_class_pkg ros_publisher_node

# 终端3 — 订阅者
rosrun my_class_pkg ros_subscriber_node

# 或查看话题内容
rostopic echo /my_topic
```

---

### 5.2 Python 标准消息（Topic）

```bash
# 终端1
roscore

# 终端2 — 发布者
rosrun my_class_pkg ros_publisher_node.py

# 终端3 — 订阅者
rosrun my_class_pkg ros_subscriber_node.py
```

---

### 5.3 C++ 自定义消息（MyMessage）

```bash
# 终端1
roscore

# 终端2 — 自定义消息发布者
rosrun my_class_pkg msg_publisher_node

# 终端3 — 自定义消息订阅者
rosrun my_class_pkg msg_subscriber_node

# 或查看话题内容
rostopic echo /my_msg_topic
```

---

### 5.5 Launch 文件一键启动（发布者 + 订阅者）

```bash
roslaunch my_class_pkg bringup_topic.launch
```

> 无需单独启动 roscore，launch 文件会自动处理。

---

### 服务实验（Service）

#### C++ 服务

```bash
# 终端1
roscore

# 终端2 — 服务端
rosrun my_class_pkg ros_server_node

# 终端3 — 客户端（一次性请求）
rosrun my_class_pkg ros_client_node
```

#### Python 服务

```bash
# 终端1
roscore

# 终端2 — 服务端
rosrun my_class_pkg ros_server.py

# 终端3 — 客户端
rosrun my_class_pkg ros_client.py
```

---

### 动作实验（Action）

#### C++ 动作

```bash
# 终端1
roscore

# 终端2 — 动作服务端
rosrun my_class_pkg ros_action_server

# 终端3 — 动作客户端
rosrun my_class_pkg ros_action_client
```

#### Python 动作

```bash
# 终端1
roscore

# 终端2 — 动作服务端
rosrun my_class_pkg ros_action_server.py

# 终端3 — 动作客户端
rosrun my_class_pkg ros_action_client.py
```

---

## 验证自定义消息 / 服务 / 动作

```bash
source ~/d_ros_class_ws/devel/setup.bash

rosmsg show my_class_pkg/MyMessage
rossrv show my_class_pkg/MyServiceMsg
rosmsg show my_class_pkg/MyActionGoal
```

---

## 传感器实验

> 所有传感器节点均需先启动硬件通信：
> ```bash
> roslaunch upros_bringup bringup_w2a.launch
> ```

### 碰撞传感器（Bump Sensor）

```bash
# 订阅并打印碰撞传感器数据
rosrun my_class_pkg ros_bump_node

# 基于碰撞传感器的自动避障（触碰后后退+转向）
rosrun my_class_pkg ros_bump_avoid_node
```

### 超声波 / TOF 传感器

```bash
# 订阅并打印超声波距离数据（左/前/右）
rosrun my_class_pkg ros_sonic_node

# 基于 TOF 传感器的自动避障（距离 < 0.3m 时转向）
rosrun my_class_pkg ros_tof_avoid_node
```

### IMU 传感器

```bash
# 订阅并打印 IMU 数据（加速度 / 角速度 / 姿态四元数）
rosrun my_class_pkg ros_imu_node

# 基于 IMU 自旋控制（旋转精确 180°后自动停止）
rosrun my_class_pkg ros_imu_spin_node
```
