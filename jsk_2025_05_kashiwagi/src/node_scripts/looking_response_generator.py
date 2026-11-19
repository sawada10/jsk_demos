#!/usr/bin/env python3

import os
import rospy
import rospkg

from std_msgs.msg import String
from openai import AzureOpenAI


class LookingResponseGenerator:
    def __init__(self):
        rospy.init_node("looking_response_generator")

        self.cur_state = "unknown"
        self.descriptions = []
        self.generated = False

        self.path_to_pkg = rospkg.RosPack().get_path(
            "jsk_2025_05_kashiwagi"
        )
        self.text_path = os.path.join(
            self.path_to_pkg, "data/tmp/tmp_response.txt"
        )

        self.client = AzureOpenAI(
            azure_endpoint=os.getenv("AZURE_OPENAI_ENDPOINT"),
            api_key=os.getenv("AZURE_OPENAI_KEY"),
            api_version="2024-12-01-preview"
        )

        self.system_prompt = {
            "role": "system",
            "content": (
                "あなたは「柏木さん」というハトロボットです。"
                "一人称は「ぼく」。方向音痴で好奇心旺盛。"
                "話し方はゆっくりのんびりしたタメ語で、"
                "丁寧語は使わず、自然な言いよどみを含めます。"
                "見た目は柏の葉の羽、梅の飾り、小さな黒いリュック、"
                "首から下げた名札が特徴です。"
                "好きな食べ物は柏餅です。"
                "ただし、自分の設定を無理に話題にしないでください。"
            )
        }

        self.pub_response = rospy.Publisher(
            "/talking_game_response", String, queue_size=1
        )

        rospy.Subscriber(
            "/kashiwagi_state",
            String,
            self.state_callback,
            queue_size=1
        )

        rospy.Subscriber(
            "/vlm/description",
            String,
            self.description_callback,
            queue_size=1
        )

        rospy.loginfo("Looking Response Generator started")
        rospy.spin()

    def state_callback(self, msg):
        new_state = msg.data

        if new_state != self.cur_state:
            self.cur_state = new_state
            self.descriptions = []
            self.generated = False

            rospy.loginfo("State: %s", new_state)

    def description_callback(self, msg):
        if self.cur_state != "looking:thinking":
            return

        if self.generated:
            return

        self.descriptions.append(msg.data)

        rospy.loginfo(
            "Description: %d/5",
            len(self.descriptions)
        )

        if len(self.descriptions) == 5:
            self.generated = True
            self.generate_response()

    def generate_response(self):
        observations = "\n".join(
            f"{i+1}. {desc}"
            for i, desc in enumerate(self.descriptions)
        )

        prompt = f"""
        以下はカメラで5回観察した結果です。

        【観察結果】
        {observations}

        5回の観察結果から、実際に見えていそうなものや
        状況を判断してください。

        - 複数回出てくる物体や状況を優先してください。
        - VLMの誤認識の可能性も考慮してください。
        - 確信が持てないものは断定しないでください。

        その状況を見た「柏木さん」として、
        自然なセリフを日本語で生成してください。
        
        - のんびりしたタメ語で話してください。
        - 「あれぇ」「おぉ」「んー」などを自然に使ってください。
        - 周囲のものへの好奇心を表現してください。
        - 30〜100文字程度、最大200文字以内にしてください。
        - 分析過程は出力せず、セリフだけを出力してください。
        """

        try:
            response = self.client.chat.completions.create(
                model=os.getenv("AZURE_OPENAI_MODEL"),
                messages=[
                    self.system_prompt,
                    {"role": "user", "content": prompt}
                ],
                max_completion_tokens=300,
                temperature=1.0,
                stream=False
            )

            reply = response.choices[0].message.content.strip()
            
            os.makedirs(
                os.path.dirname(self.text_path),
                exist_ok=True
            )

            with open(self.text_path, "w", encoding="utf-8") as f:
                f.write(reply)
            self.pub_response.publish(reply)

            rospy.loginfo("Generated response: %s", reply)

        except Exception as e:
            rospy.logerr("Response generation failed: %s", e)


if __name__ == "__main__":
    try:
        LookingResponseGenerator()
    except rospy.ROSInterruptException:
        pass
