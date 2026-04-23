#!/usr/bin/env python3

import os
import sys

import rospy
from openai import OpenAI
from std_msgs.msg import String
from std_srvs.srv import Empty

sys.path.insert(0, os.path.dirname(__file__))
from tokenizer import Tokenizer


DEFAULT_SYSTEM_PROMPT = (
    "你是 Kimi，由 Moonshot AI 提供的人工智能助手。"
    "你擅长中英文对话，会提供安全、准确、有帮助的回答。"
)


class LLMNode:
    def __init__(self):
        rospy.init_node("robot_voice_llm_node", anonymous=True)

        api_key = self._load_api_key()
        base_url = rospy.get_param("~base_url", "https://api.moonshot.cn/v1")
        self.model = rospy.get_param("~model", "moonshot-v1-8k")
        self.system_role_content = rospy.get_param("~system_prompt", DEFAULT_SYSTEM_PROMPT)

        self.tokenizer = Tokenizer()
        self.client = OpenAI(api_key=api_key, base_url=base_url)
        self.tts_pub = rospy.Publisher("/talk", String, queue_size=10)
        self.speech_sub = rospy.Subscriber("/speech/result", String, self.speech_result_callback)
        rospy.loginfo("llm_chat node started, model=%s", self.model)

    def _reset_recognizer(self):
        try:
            rospy.wait_for_service("/start_recognize", timeout=1.0)
            reset = rospy.ServiceProxy("/start_recognize", Empty)
            reset()
        except Exception:
            pass

    def _is_control_intent(self, text):
        tokens = self.tokenizer.pre_process(text)
        return len(self.tokenizer.extract_intent(tokens)) > 0

    def _load_api_key(self):
        api_key = os.getenv("MOONSHOT_API_KEY", "").strip()
        if api_key:
            return api_key

        default_file = os.path.join(os.path.dirname(__file__), "moonshot_api_key.txt")
        api_key_file = rospy.get_param("~api_key_file", default_file)

        if not os.path.exists(api_key_file):
            raise RuntimeError(
                "Moonshot API key file not found: {}. "
                "Set MOONSHOT_API_KEY or create the file.".format(api_key_file)
            )

        with open(api_key_file, "r", encoding="utf-8") as file_obj:
            api_key = file_obj.read().strip()

        if not api_key:
            raise RuntimeError("Moonshot API key file is empty: {}".format(api_key_file))

        return api_key

    def speech_result_callback(self, msg):
        result = msg.data.strip()
        if not result:
            return

        if self._is_control_intent(result):
            rospy.loginfo("控制指令，LLM 跳过: %s", result)
            return

        rospy.loginfo("speech [%s]", result)

        try:
            chat_response = self.query(result)
            rospy.loginfo("LLM response: %s", chat_response.replace("\n", " "))
            self.tts_pub.publish(String(data=chat_response))
        except Exception as exc:
            if "rate_limit" in str(exc):
                rospy.logwarn("Moonshot request rate limited")
            else:
                rospy.logerr("LLM query failed: %s", exc)
        finally:
            self._reset_recognizer()

    def query(self, user_prompt):
        messages = [
            {"role": "system", "content": self.system_role_content},
            {"role": "user", "content": user_prompt},
        ]

        completion = self.client.chat.completions.create(
            model=self.model,
            messages=messages,
            temperature=0.1,
            stream=False,
        )
        return completion.choices[0].message.content


if __name__ == "__main__":
    try:
        LLMNode()
        rospy.spin()
    except KeyboardInterrupt:
        print("\nCaught Ctrl + C. Exiting")
