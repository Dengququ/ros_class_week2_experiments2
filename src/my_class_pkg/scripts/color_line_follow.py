#!/usr/bin/env python3
# 实验二：基于颜色识别的自主巡线实验
# 使用HSV颜色空间提取线条颜色，根据颜色重心控制机器人转向

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()
cmd_pub = None
result_pub = None

# ============================================================
# 根据实验步骤5.1标定的HSV阈值，填入自己的数值
# 默认示例：红色线条（请根据实际标定结果修改）
HSV_LOWER = np.array([12, 80, 100])    # H_min, S_min, V_min  黄色线
HSV_UPPER = np.array([35, 255, 255])   # H_max, S_max, V_max  黄色线
# ============================================================

# 控制参数
LINEAR_SPEED  = 0.15   # 直线速度 m/s
KP            = 0.003  # 转向比例系数（误差像素 -> 角速度）
DEAD_ZONE     = 20     # 中心误差死区（像素），小于此值不转向

def image_callback(msg):
    try:
        frame = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: %s", e)
        return

    h, w = frame.shape[:2]

    # 只取图像下半部分，减少远处干扰
    roi = frame[h // 2:, :]

    # RGB -> HSV
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    # 颜色二值化
    mask = cv2.inRange(hsv, HSV_LOWER, HSV_UPPER)

    # 形态学去噪：先腐蚀后膨胀（开运算）
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # 提取连通域，找最大区域重心
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    twist = Twist()

    if contours:
        # 选面积最大的连通域
        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)

        if area > 500:  # 过滤噪点
            M = cv2.moments(largest)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])

                # 画出重心和外接矩形（用于调试/rqt查看）
                x, y, bw, bh = cv2.boundingRect(largest)
                cv2.rectangle(roi, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
                cv2.circle(roi, (cx, cy), 8, (0, 0, 255), -1)
                cv2.putText(roi, f"cx={cx}", (cx + 10, cy),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

                # 计算横向偏差（正值：线条在右侧，机器人需左转）
                error = cx - w // 2

                twist.linear.x = LINEAR_SPEED
                if abs(error) > DEAD_ZONE:
                    twist.angular.z = -KP * error
                rospy.loginfo_throttle(1, f"巡线中 | 误差={error}px | 角速度={twist.angular.z:.3f}")
        else:
            rospy.logwarn_throttle(2, "线条面积过小，停止移动")
    else:
        rospy.logwarn_throttle(2, "未检测到线条，停止移动")

    cmd_pub.publish(twist)

    # 发布处理后图像供 rqt 查看
    try:
        result_msg = bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        result_pub.publish(result_msg)
    except CvBridgeError:
        pass

def main():
    global cmd_pub, result_pub
    rospy.init_node('color_line_follow', anonymous=True)
    cmd_pub    = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    result_pub = rospy.Publisher('/image_result', Image, queue_size=1)
    rospy.Subscriber('/camera/color/image_raw', Image, image_callback)
    rospy.loginfo("巡线节点已启动，请确保已标定 HSV 阈值")
    rospy.on_shutdown(lambda: cmd_pub.publish(Twist()))  # 退出时停止机器人
    rospy.spin()

if __name__ == '__main__':
    main()
