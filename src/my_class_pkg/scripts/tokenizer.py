#!/usr/bin/env python3

import re

import jieba


class Tokenizer:
    stopwords = {"的", "是", "啊"}

    json_template = {"intent": "", "target": -1}

    chinese_to_arabic = {
        "零": 0,
        "一": 1,
        "二": 2,
        "三": 3,
        "四": 4,
        "五": 5,
        "六": 6,
        "七": 7,
        "八": 8,
        "九": 9,
        "十": 10,
        "百": 100,
        "千": 1000,
        "万": 10000,
        "亿": 100000000,
    }

    fruit_name_mapping = {
        "苹果": "apple",
        "香蕉": "banana",
        "橙子": "orange",
        "橘子": "orange",
        "草莓": "strawberry",
        "西瓜": "watermelon",
        "菠萝": "pineapple",
    }

    phrase_aliases = {
        "发起": "抓起",
        "歌曲": "抓取",  # ASR 常将"抓取"误识别为"歌曲"（韵母相同）
        "各取": "抓取",
        "各区": "抓取",
    }

    def __init__(self):
        print("Init Tokenizer")

    def chinese_to_arabic_number(self, chinese_num):
        total = 0
        section = 0
        number = 0

        for ch in chinese_num:
            value = self.chinese_to_arabic[ch]

            if value >= 10000:
                section = (section + number) * value
                total += section
                section = 0
                number = 0
            elif value >= 10:
                if number == 0:
                    number = 1
                section += number * value
                number = 0
            else:
                number = value

        return total + section + number

    def replace_chinese_numbers(self, text):
        pattern = re.compile(r"零?[一二三四五六七八九十百千万亿]+")
        matches = pattern.findall(text)
        for match in matches:
            arabic_number = self.chinese_to_arabic_number(match)
            text = text.replace(match, str(arabic_number))
        return text

    def pre_process(self, text):
        text = re.sub(r"\s+", "", text)
        text = self.replace_chinese_numbers(text)
        text = text.lower()

        for src, dst in self.phrase_aliases.items():
            text = text.replace(src, dst)

        tokens = list(jieba.lcut(text))
        filtered_tokens = [token for token in tokens if token not in self.stopwords]

        for i, token in enumerate(filtered_tokens):
            if token in self.fruit_name_mapping:
                filtered_tokens[i] = self.fruit_name_mapping[token]

        return filtered_tokens

    def extract_intent(self, tokens):
        intents = {
            "go_to": ["移动", "去", "前往"],
            "pick": ["抓取", "抓起", "拿起", "抓", "拿", "取"],
            "release": ["放下", "放到", "放", "放置"],
        }
        result = []
        i = 0
        while i < len(tokens):
            for intent, triggers in intents.items():
                if tokens[i] in triggers:
                    j = i + 1
                    while j < len(tokens) and not (tokens[j].isdigit() or tokens[j] == "号"):
                        j += 1

                    target_index = -1
                    if j < len(tokens):
                        for k in range(j, len(tokens)):
                            if tokens[k].isdigit():
                                target_index = int(tokens[k])
                                break

                    json_obj = self.json_template.copy()
                    json_obj["intent"] = intent
                    json_obj["target"] = target_index
                    result.append(json_obj)
                    i = j
                    break
            else:
                i += 1
        return result

    def get_intent_from_text(self, text):
        filtered_input = self.pre_process(text)
        return self.extract_intent(filtered_input)


if __name__ == "__main__":
    tokenizer = Tokenizer()
    user_input = input("请输入指令: ")
    filtered_input = tokenizer.pre_process(user_input)
    intent_string = tokenizer.extract_intent(filtered_input)
    print("Output:", intent_string)
