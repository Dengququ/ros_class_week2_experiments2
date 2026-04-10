#!/usr/bin/env python3
# 实验五：视觉抓取实验
# 检测 ID=1 的 AprilTag，机器人自动调整到抓取位置后执行抓取
#
# 运行前需启动：
#   1. roslaunch zoo_bringup bringup_w2c.launch          （底盘）
#   2. roslaunch realsense2_camera rs_camera.launch       （相机）
#   3. roslaunch upros_arm recognize_apriltag.launch      （AprilTag TF）
#   4. roslaunch upros_arm control_center.launch          （机械臂服务）
#
# 运行本节点：
#   rosrun upros_tutorial apriltag_grab.py

import threading
import rospy
import cv2
import numpy as np
import apriltag
import tf2_ros
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from upros_message.srv import ArmPosition, ArmPositionRequest
from std_srvs.srv import Empty

# ── 可调参数 ──────────────────────────────────────
TARGET_ID     = 1       # 目标 AprilTag ID
LINEAR_SPEED  = 0.10    # 靠近速度 (m/s)
KP_ANGULAR    = 0.003   # 转向比例系数
TARGET_AREA   = 8000    # 面积阈值（像素²），到达后停止前进
DEAD_ZONE     = 30      # 横向偏差死区（像素）
ALIGN_FRAMES  = 10      # 连续对准帧数后触发抓取
# ─────────────────────────────────────────────────

# 状态机
STATE_FOLLOW = 0  # 靠近 & 对准阶段
STATE_GRAB   = 1  # 机械臂抓取阶段（后台线程）

bridge    = CvBridge()
detector  = apriltag.Detector(apriltag.DetectorOptions(families='tag36h11'))

state      = STATE_FOLLOW
align_cnt  = 0
state_lock = threading.Lock()

cmd_pub    = None
result_pub = None
tf_buffer  = None
arm_open   = None
arm_grab   = None
arm_zero   = None


def grab_thread():
    """在独立线程中执行机械臂抓取序列，避免阻塞图像回调"""
    global state

    rospy.loginfo("抓取线程启动，查询 TF 坐标...")
    try:
        tfs = tf_buffer.lookup_transform(
            "arm_base_link", "tag_1",
            rospy.Time(0), rospy.Duration(10.0)
        )
    except Exception as e:
        rospy.logerr("TF 获取失败: %s", e)
        rospy.logerr("请确认 recognize_apriltag.launch 已启动且 tag_1 在视野内")
        with state_lock:
            state = STATE_FOLLOW
        return

    # 坐标系转换 (m → mm，与 apriltag_grab.cpp 一致)
    x = -float(tfs.transform.translation.y * 1000)
    y =  float(tfs.transform.translation.x * 1000) + 30.0
    z =  float(tfs.transform.translation.z * 1000) + 40.0
    rospy.loginfo("tag_1 相对机械臂基座: x=%.1f  y=%.1f  z=%.1f (mm)", x, y, z)

    # 1. 移动机械臂到目标位置（夹爪张开）
    req = ArmPositionRequest()
    req.x, req.y, req.z = x, y, z
    rospy.loginfo("移动机械臂到目标位置...")
    arm_open(req)
    rospy.sleep(5.0)

    # 2. 夹取
    rospy.loginfo("夹取中...")
    arm_grab()
    rospy.sleep(5.0)

    # 3. 归零
    rospy.loginfo("归零...")
    arm_zero()
    rospy.sleep(3.0)

    rospy.loginfo("=== 抓取完成，回到跟踪状态 ===")
    with state_lock:
        state = STATE_FOLLOW


def image_callback(msg):
    global state, align_cnt

    with state_lock:
        cur_state = state

    if cur_state != STATE_FOLLOW:
        return  # 抓取阶段不处理图像

    try:
        frame = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    except CvBridgeError as e:
        rospy.logerr(e)
        return

    gray       = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    detections = detector.detect(gray)

    h, w  = frame.shape[:2]
    twist = Twist()
    found = False

    for det in detections:
        if det.tag_id != TARGET_ID:
            continue

        found   = True
        corners = det.corners.astype(int)

        # 绘制标签框和 ID
        for i in range(4):
            cv2.line(frame, tuple(corners[i]), tuple(corners[(i + 1) % 4]),
                     (0, 255, 0), 2)
        cx = int(np.mean(corners[:, 0]))
        cy = int(np.mean(corners[:, 1]))
        cv2.circle(frame, (cx, cy), 6, (0, 0, 255), -1)
        cv2.putText(frame, f"ID:{det.tag_id}",
                    (cx - 20, cy - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)

        area  = cv2.contourArea(corners.reshape(-1, 1, 2).astype(np.float32))
        error = cx - w // 2  # 正 → 标签偏右 → 需左转

        # 转向控制
        twist.angular.z = -KP_ANGULAR * error if abs(error) > DEAD_ZONE else 0.0

        # 距离控制
        if area < TARGET_AREA:
            twist.linear.x = LINEAR_SPEED
            align_cnt = 0
            cv2.putText(frame, "APPROACHING", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 255), 2)
        else:
            twist.linear.x = 0.0
            if abs(error) <= DEAD_ZONE:
                align_cnt += 1
                cv2.putText(frame, f"ALIGNED {align_cnt}/{ALIGN_FRAMES}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                if align_cnt >= ALIGN_FRAMES:
                    # 停止底盘，触发抓取
                    cmd_pub.publish(Twist())
                    rospy.loginfo("位置就绪（对准 %d 帧），开始抓取...", ALIGN_FRAMES)
                    with state_lock:
                        state = STATE_GRAB
                    t = threading.Thread(target=grab_thread, daemon=True)
                    t.start()
                    return
            else:
                align_cnt = 0
                cv2.putText(frame, "ALIGNING", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 165, 255), 2)

        rospy.loginfo_throttle(
            1, "ID=%d | area=%.0f | err=%d | align=%d",
            TARGET_ID, area, error, align_cnt
        )
        break

    if not found:
        rospy.logwarn_throttle(2, "未检测到 AprilTag ID=%d，停止等待", TARGET_ID)
        align_cnt = 0

    cmd_pub.publish(twist)

    try:
        result_pub.publish(bridge.cv2_to_imgmsg(frame, encoding='bgr8'))
    except CvBridgeError:
        pass


def main():
    global cmd_pub, result_pub, tf_buffer, arm_open, arm_grab, arm_zero

    rospy.init_node('apriltag_grab', anonymous=False)

    # TF 监听器
    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer)

    # 等待机械臂服务
    rospy.loginfo("等待机械臂服务...")
    rospy.wait_for_service('/upros_arm_control/arm_pos_service_open')
    rospy.wait_for_service('/upros_arm_control/grab_service')
    rospy.wait_for_service('/upros_arm_control/zero_service')
    arm_open = rospy.ServiceProxy('/upros_arm_control/arm_pos_service_open', ArmPosition)
    arm_grab = rospy.ServiceProxy('/upros_arm_control/grab_service', Empty)
    arm_zero = rospy.ServiceProxy('/upros_arm_control/zero_service', Empty)
    rospy.loginfo("机械臂服务就绪")

    cmd_pub    = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    result_pub = rospy.Publisher('/image_result', Image, queue_size=1)
    rospy.Subscriber('/camera/color/image_raw', Image, image_callback)

    rospy.loginfo("=== AprilTag 抓取节点已启动，目标 ID=%d ===", TARGET_ID)
    rospy.on_shutdown(lambda: cmd_pub.publish(Twist()))
    rospy.spin()


if __name__ == '__main__':
    main()
