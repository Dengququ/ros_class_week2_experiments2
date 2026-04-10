#!/usr/bin/env python3
# 实验四：视觉跟踪实验
# 检测 ID=1 的 AprilTag，控制机器人向其靠近并跟随

import rospy
import cv2
import numpy as np
import apriltag
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()
cmd_pub = None
result_pub = None

# AprilTag 检测器（tag36h11 家族）
detector = apriltag.Detector(apriltag.DetectorOptions(families='tag36h11'))

TARGET_ID = 1          # 跟随的 AprilTag ID
LINEAR_SPEED  = 0.12   # 直线速度 m/s（靠近时使用）
KP_ANGULAR    = 0.003  # 转向比例系数
TARGET_AREA   = 8000   # 目标面积（像素²），达到后停止前进
DEAD_ZONE     = 30     # 角度死区（像素）

def image_callback(msg):
    try:
        frame = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: %s", e)
        return

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    detections = detector.detect(gray)

    h, w = frame.shape[:2]
    twist = Twist()
    found = False

    for det in detections:
        if det.tag_id != TARGET_ID:
            continue

        found = True
        corners = det.corners.astype(int)

        # 绘制标签边框和 ID
        for i in range(4):
            cv2.line(frame, tuple(corners[i]), tuple(corners[(i + 1) % 4]),
                     (0, 255, 0), 2)
        cx = int(np.mean(corners[:, 0]))
        cy = int(np.mean(corners[:, 1]))
        cv2.circle(frame, (cx, cy), 6, (0, 0, 255), -1)
        cv2.putText(frame, f"ID:{det.tag_id}", (cx - 20, cy - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)

        # 估计标签面积（用外接矩形近似）
        area = cv2.contourArea(corners.reshape(-1, 1, 2).astype(np.float32))

        # 横向误差（正值 -> 标签在右侧 -> 需左转）
        error = cx - w // 2
        twist.angular.z = -KP_ANGULAR * error if abs(error) > DEAD_ZONE else 0.0

        # 距离控制：面积小于目标值则前进
        if area < TARGET_AREA:
            twist.linear.x = LINEAR_SPEED
        else:
            twist.linear.x = 0.0
            rospy.loginfo_throttle(2, "已到达目标距离，停止前进")

        rospy.loginfo_throttle(1, f"检测到 ID={TARGET_ID} | cx={cx} | area={area:.0f} | err={error}")
        break

    if not found:
        rospy.logwarn_throttle(2, f"未检测到 AprilTag ID={TARGET_ID}，停止")

    cmd_pub.publish(twist)

    # 发布标注图像
    try:
        result_pub.publish(bridge.cv2_to_imgmsg(frame, encoding='bgr8'))
    except CvBridgeError:
        pass

def main():
    global cmd_pub, result_pub
    rospy.init_node('apriltag_follow', anonymous=True)
    cmd_pub    = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    result_pub = rospy.Publisher('/image_result', Image, queue_size=1)
    rospy.Subscriber('/camera/color/image_raw', Image, image_callback)
    rospy.loginfo(f"AprilTag 跟踪节点已启动，跟随 ID={TARGET_ID}")
    rospy.on_shutdown(lambda: cmd_pub.publish(Twist()))
    rospy.spin()

if __name__ == '__main__':
    main()
