#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from std_srvs.srv import Empty
from upros_message.msg import TagCommand

from tokenizer import Tokenizer


class VoiceControlNode:
    def __init__(self):
        rospy.init_node("tokenizer_publisher")
        self.tokenizer = Tokenizer()
        self.tag_cmd_pub = rospy.Publisher("/voice_control", TagCommand, queue_size=10)
        self.talker_sub = rospy.Subscriber("/speech/result", String, self.speech_result_callback)
        rospy.loginfo("voice_control node started, waiting for /speech/result")

    def _reset_recognizer(self):
        try:
            rospy.wait_for_service("/start_recognize", timeout=1.0)
            reset = rospy.ServiceProxy("/start_recognize", Empty)
            reset()
        except Exception:
            pass

    def speech_result_callback(self, msg):
        user_input = msg.data.strip()
        if not user_input:
            rospy.logwarn("Empty speech result received")
            return

        filtered_input = self.tokenizer.pre_process(user_input)
        intent_string = self.tokenizer.extract_intent(filtered_input)

        if not intent_string:
            rospy.logwarn("No valid intent parsed from speech: %s", user_input)
            self._reset_recognizer()
            return

        cmd = TagCommand()
        cmd.intent = intent_string[0]["intent"]
        cmd.target = intent_string[0]["target"]
        self.tag_cmd_pub.publish(cmd)
        rospy.loginfo("Published voice command: intent=%s target=%d", cmd.intent, cmd.target)
        self._reset_recognizer()


if __name__ == "__main__":
    try:
        VoiceControlNode()
        rospy.spin()
    except KeyboardInterrupt:
        print("\nCaught Ctrl + C. Exiting")
