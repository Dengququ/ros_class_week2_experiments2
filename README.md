# D_ROS_CLASS_WS - 第二周 ROS 实验工作空间

## 工作空间结构

```bash
djf_ros_class_ws/
└── src/
    └── my_class_pkg/
        ├── msg/          # 自定义消息
        ├── srv/          # 自定义服务
        ├── action/       # 自定义动作
        ├── src/          # C++ 节点
        ├── scripts/      # Python 节点
        └── launch/       # Launch 文件
```

## 环境准备

```bash
cd ~/djf_ros_class_ws
catkin_make
source devel/setup.bash
```

## 基础通信实验

### 5.1 C++ 标准消息 Topic

```bash
roscore
rosrun my_class_pkg ros_publisher_node
rosrun my_class_pkg ros_subscriber_node
rostopic echo /my_topic
```

### 5.2 Python 标准消息 Topic

```bash
roscore
rosrun my_class_pkg ros_publisher_node.py
rosrun my_class_pkg ros_subscriber_node.py
```

### 5.3 C++ 自定义消息 MyMessage

```bash
roscore
rosrun my_class_pkg msg_publisher_node
rosrun my_class_pkg msg_subscriber_node
rostopic echo /my_msg_topic
```

### 5.5 Launch 文件一键启动

```bash
roslaunch my_class_pkg bringup_topic.launch
```

### 服务实验 Service

```bash
roscore
rosrun my_class_pkg ros_server_node
rosrun my_class_pkg ros_client_node
```

```bash
roscore
rosrun my_class_pkg ros_server.py
rosrun my_class_pkg ros_client.py
```

### 动作实验 Action

```bash
roscore
rosrun my_class_pkg ros_action_server
rosrun my_class_pkg ros_action_client
```

```bash
roscore
rosrun my_class_pkg ros_action_server.py
rosrun my_class_pkg ros_action_client.py
```

## 传感器实验

先启动硬件通信：

```bash
roslaunch upros_bringup bringup_w2a.launch
```

### 碰撞传感器

```bash
rosrun my_class_pkg ros_bump_node
rosrun my_class_pkg ros_bump_avoid_node
```

### 超声波与 TOF

```bash
rosrun my_class_pkg ros_sonic_node
rosrun my_class_pkg ros_tof_avoid_node
```

### IMU

```bash
rosrun my_class_pkg ros_imu_node
rosrun my_class_pkg ros_imu_spin_node
```

## 视觉实验

### 1. 颜色循迹

脚本位置：`src/my_class_pkg/scripts/color_line_follow.py`

功能：基于 HSV 颜色分割提取地面色带，计算重心后控制底盘沿线前进。

运行前建议先完成相机与底盘启动，并根据现场颜色重新标定脚本中的 HSV 阈值。

```bash
roslaunch zoo_bringup bringup_w2c.launch
roslaunch realsense2_camera rs_camera.launch
source ~/djf_ros_class_ws/devel/setup.bash
rosrun my_class_pkg color_line_follow.py
```

默认订阅 `/camera/color/image_raw`，控制输出到 `/cmd_vel`，处理后的图像发布到 `/image_result`。

### 2. AprilTag 识别跟随

脚本位置：`src/my_class_pkg/scripts/apriltag_follow.py`

功能：检测 `ID=1` 的 AprilTag，控制机器人自动转向并向目标靠近，到达设定面积阈值后停止前进。

```bash
roslaunch zoo_bringup bringup_w2c.launch
roslaunch realsense2_camera rs_camera.launch
source ~/djf_ros_class_ws/devel/setup.bash
rosrun my_class_pkg apriltag_follow.py
```

依赖 Python `apriltag` 库和相机图像话题 `/camera/color/image_raw`。

### 3. AprilTag 识别抓取方块

脚本位置：`src/my_class_pkg/scripts/apriltag_grab.py`

启动文件：`src/my_class_pkg/launch/apriltag_grab.launch`

功能：检测 `ID=1` 的 AprilTag，完成底盘靠近、对准、TF 查询和机械臂抓取流程。

方式一，分终端手动启动：

```bash
roslaunch zoo_bringup bringup_w2c.launch
roslaunch realsense2_camera rs_camera.launch
roslaunch upros_arm recognize_apriltag.launch
roslaunch upros_arm control_center.launch
source ~/djf_ros_class_ws/devel/setup.bash
rosrun my_class_pkg apriltag_grab.py
```

方式二，使用 launch 文件启动视觉抓取节点相关组件：

```bash
source ~/djf_ros_class_ws/devel/setup.bash
roslaunch my_class_pkg apriltag_grab.launch
```

抓取实验依赖：

- AprilTag 识别 TF: `tag_1`
- 机械臂服务: `/upros_arm_control/arm_pos_service_open`
- 抓取服务: `/upros_arm_control/grab_service`
- 归零服务: `/upros_arm_control/zero_service`

## 常用验证命令

```bash
source ~/djf_ros_class_ws/devel/setup.bash
rosmsg show my_class_pkg/MyMessage
rossrv show my_class_pkg/MyServiceMsg
rosmsg show my_class_pkg/MyActionGoal
```
