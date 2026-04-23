#!/usr/bin/env python3

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf_conversions import transformations
from math import pi
from upros_message.msg import TagCommand
from std_srvs.srv import Empty


# ══════════════════════════════════════════════════════════
# 导航目标点坐标表
# 建图完成后用 rosrun upros_transform tf_echo_node 标定，
# 将坐标填入下方：[x(m), y(m), yaw(°)]
# ══════════════════════════════════════════════════════════
WAYPOINTS = {
    1: [0.0, 0.0, 0.0],   # TODO: 建图后替换
    2: [0.0, 0.0, 0.0],   # TODO: 建图后替换
}


class VoiceNavNode:
    def __init__(self):
        rospy.init_node("voice_nav_node")

        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("voice_nav: 等待 move_base 服务器...")
        self.client.wait_for_server()
        rospy.loginfo("voice_nav: move_base 已连接，等待语音指令")

        rospy.Subscriber("/voice_control", TagCommand, self.cmd_callback)

    def _reset_recognizer(self):
        try:
            rospy.wait_for_service("/start_recognize", timeout=1.0)
            rospy.ServiceProxy("/start_recognize", Empty)()
        except Exception:
            pass

    def cmd_callback(self, msg):
        if msg.intent != "go_to":
            return

        target = msg.target
        if target not in WAYPOINTS:
            rospy.logwarn("voice_nav: 未知目标点 %d，仅支持 %s", target, list(WAYPOINTS.keys()))
            self._reset_recognizer()
            return

        wp = WAYPOINTS[target]
        rospy.loginfo("voice_nav: 前往 %d 号点 (x=%.2f y=%.2f yaw=%.1f°)", target, wp[0], wp[1], wp[2])

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = wp[0]
        goal.target_pose.pose.position.y = wp[1]
        q = transformations.quaternion_from_euler(0.0, 0.0, wp[2] / 180.0 * pi)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.client.send_goal(goal)
        reached = self.client.wait_for_result(rospy.Duration(60))

        if not reached:
            self.client.cancel_goal()
            rospy.logwarn("voice_nav: 导航超时，目标 %d 号点", target)
        elif self.client.get_state() == GoalStatus.SUCCEEDED:
            rospy.loginfo("voice_nav: 已到达 %d 号点", target)
        else:
            rospy.logwarn("voice_nav: 导航失败，状态 %s", self.client.get_state())

        self._reset_recognizer()


if __name__ == "__main__":
    try:
        VoiceNavNode()
        rospy.spin()
    except KeyboardInterrupt:
        print("\nCaught Ctrl + C. Exiting")
