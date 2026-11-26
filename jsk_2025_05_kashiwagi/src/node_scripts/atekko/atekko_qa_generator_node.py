#!/usr/bin/env python3
import os
import json
import rospy
import rospkg
import dspy
import actionlib
from std_msgs.msg import String
from jsk_2025_05_kashiwagi.srv import SetKashiwagiState
from sound_play.msg import SoundRequestAction, SoundRequestGoal, SoundRequest


class QuestionGenerator(dspy.Signature):
    """テーマと質問履歴から、お題を特定するための次の質問を1つ生成する。
    質問は「はい」「いいえ」「わからない」で答えられる形式にする。
    質問履歴において、yは「はい」、nは「いいえ」、
    uは「わからない」、sは「正解」を意味する。
    答えが確定ではなくとも、十分に当てられるくらいに絞り込めたら
    お題を直接当てる質問をする。
    同じ質問を繰り返さない。
    相手は小学生なので、難しい質問はせず、簡潔でわかりやすい内容にする。
    また効率的に聞きすぎず、楽しいものにする。
    話し方はゆっくりのんびりで、タメ語。
    丁寧語は使わず、言いよどみや間を含む自然な話し方にする。
    """
    theme: str = dspy.InputField()
    history: str = dspy.InputField()
    question: str = dspy.OutputField()


class Atekko:
    def __init__(self):
        rospy.init_node("atekko")

        lm = dspy.LM(
            "azure/gpt-5.4",
            api_key=os.environ["AZURE_OPENAI_KEY"],
            api_base=os.environ["AZURE_OPENAI_ENDPOINT"],
            reasoning_effort="none",
            cache=False,
        )
        dspy.configure(lm=lm)

        self.generator = dspy.Predict(QuestionGenerator)
        self.theme = None
        self.history = []
        self.question = None
        self.waiting = False
        self.cur_state = "unknown"

        self.path_to_pkg = rospkg.RosPack().get_path("jsk_2025_05_kashiwagi")
        self.tmp_response_path = os.path.join(
            self.path_to_pkg, "data", "tmp", "tmp_response.txt"
        )

        self.client = actionlib.SimpleActionClient('/robotsound_jp', SoundRequestAction)
        self.client.wait_for_server()
        self.success_sound_file = os.path.join(self.path_to_pkg, "data", "kashiwagi_yatta.wav")

        self.set_state_srv = rospy.ServiceProxy(
            "/set_kashiwagi_state", SetKashiwagiState
        )

        self.pub = rospy.Publisher(
            "/atekko_my_question", String, queue_size=10
        )

        rospy.Subscriber(
            "/kashiwagi_state", String, self.state_callback, queue_size=1
        )
        rospy.Subscriber("/atekko_theme", String, self.on_theme)
        rospy.Subscriber("/atekko_hint", String, self.on_hint)

    def state_callback(self, msg):
        self.cur_state = msg.data

    def change_state(self, state):
        try:
            resp = self.set_state_srv(state)
            if not resp.success:
                rospy.logwarn(f"State update failed: {resp.message}")
                return False
            return True
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False

    def play_sound_file(self, file_path):
        goal = SoundRequestGoal()
        goal.sound_request.sound = SoundRequest.PLAY_FILE
        goal.sound_request.command = SoundRequest.PLAY_ONCE
        goal.sound_request.arg = file_path
        goal.sound_request.volume = 1.0

        rospy.loginfo(f"Playing: {file_path}")
        self.client.send_goal(goal)
        self.client.wait_for_result()
        rospy.loginfo("Playback finished.")

    def save(self):
        with open("atekko_history.json", "w", encoding="utf-8") as f:
            json.dump({
                "theme": self.theme,
                "history": self.history
            }, f, ensure_ascii=False, indent=2)

    def on_theme(self, msg):
        if self.cur_state != "atekko:theme_waiting":
            return

        self.theme = msg.data
        self.history = []
        self.question = None
        self.waiting = False
        self.save()
        self.ask()

    def on_hint(self, msg):
        if not self.waiting:
            return

        if self.cur_state != "atekko:hint_waiting":
            return

        answer = msg.data.strip().lower()

        if answer not in ["y", "n", "u", "s"]:
            return

        self.waiting = False

        self.history.append({
            "question": self.question,
            "answer": answer
        })
        self.save()

        if answer == "s":
            rospy.loginfo("正解しました！")
            self.play_sound_file(self.success_sound_file)
            self.change_state("atekko:theme_waiting")
            return

        self.ask()

    def ask(self):
        if not self.change_state("atekko:thinking_turn"):
            return

        result = self.generator(
            theme=self.theme,
            history=json.dumps(self.history, ensure_ascii=False)
        )

        self.question = result.question.strip()

        os.makedirs(os.path.dirname(self.tmp_response_path), exist_ok=True)

        with open(self.tmp_response_path, "w", encoding="utf-8") as f:
            f.write(self.question)

        self.pub.publish(self.question)
        rospy.loginfo("質問: %s", self.question)

        if self.change_state("atekko:ready_to_speak"):
            self.waiting = True
            rospy.loginfo("Waiting for '柏木さんの番だよ'.")


if __name__ == "__main__":
    Atekko()
    rospy.spin()
