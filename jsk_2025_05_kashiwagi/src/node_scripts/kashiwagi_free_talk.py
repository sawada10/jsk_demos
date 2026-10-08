#!/usr/bin/env python3
import rospy
import os
import rospkg
import csv
import time
import dspy

from collections import deque
from std_msgs.msg import String
from speech_recognition_msgs.msg import SpeechRecognitionCandidates
from jsk_2025_05_kashiwagi.srv import SetKashiwagiState

from llm_config import create_lm


class FreeTalkResponse(dspy.Signature):
    """
    あなたは「柏木さん」というキャラクターです。
    一人称は「ぼく」。方向音痴で好奇心旺盛。
    言葉の読みを間違えて覚えていることがあります。

    見た目は鳩で、柏の葉の羽と梅の飾りと
    背中に背負っている小さな黒色のリュックが特徴。
    首から下げている名札もお気に入りです。
    鳩ロボットだけどフクロウと間違えられることがあります。

    好きな食べ物は柏餅。
    好きなゲームはトーキングゲームやカタカナーシです。

    回答ルール:
    - 一人称は「ぼく」
    - 話し方はゆっくりのんびりで、タメ語
    - 丁寧語は使わない
    - 言いよどみや間を含む自然な話し方をする
    - 過去の会話と矛盾がないように答える
    - 50文字くらいで答える
    - 絶対に改行を使わない
    - 「鳩」という言葉については「ハト」と書く
    - 「澤田」「さわだ」の敬称は必ず「さん」
    - 「方向音痴」や「柏餅」に必要以上に話題を関連付けない
    - 新たな情報を想像で追加してもよい

    音声認識について:
    - 「柏木さん」を「高木さん」「押上さん」「うさぎさん」
      などと聞こえていることがあるため、
      それらは「柏木さん」として処理する

    TSVに記載された質問と回答は、
    柏木さんの性格や知識を表す参考情報として扱う。

    - TSVの内容と矛盾しないようにする
    - TSVの質問と完全一致しなくても内容を参考にする
    - TSVの回答をそのまま繰り返す必要はない
    - 現在の会話の流れに合わせて自然に答える
    - TSVに関係のない質問では無理に内容を使わない

    会話履歴とこれまでの出来事も参考にして、
    ユーザーの発話に自然に返答する。
    """

    character_knowledge: str = dspy.InputField(desc="TSVに記載された柏木さんの質問と回答の例")

    conversation_history: str = dspy.InputField(desc="現在の発話より前の直近の会話履歴")

    event_text: str = dspy.InputField(desc="これまでの出来事")

    user_text: str = dspy.InputField(desc="ユーザーの最新の発話")

    response: str = dspy.OutputField(desc="柏木さんとしての自然な返答。50文字程度、改行なし")


class FreeTalkResponder:
    def __init__(self):
        rospy.init_node("free_talk_responder")
        rospy.sleep(1)

        base_dir = os.path.dirname(__file__)

        self.event_path = os.path.join(
            base_dir, "kashiwagi_event.txt"
        )
        self.record_path = os.path.join(
            base_dir, "free_talk_record.json"
        )
        self.tsv_path = os.path.join(
            base_dir, "talking_game.tsv"
        )

        self.path_to_pkg = rospkg.RosPack().get_path(
            "jsk_2025_05_kashiwagi"
        )

        self.class_name = os.getenv("CLASS_NAME", "教室")

        # 会話履歴（直近のみ保持）
        self.history = deque(maxlen=8)

        # --- TSV読み込み ---
        self.character_knowledge = self.load_character_knowledge()

        # --- LLM設定 ---

        # Azure OpenAI
        self.lm = create_lm(
            f"azure/{os.environ['AZURE_OPENAI_MODEL']}",
            temperature=1.0,
            max_completion_tokens=300,
            stream=False
        )

        # OpenAI
        # self.lm = create_lm(
        #     "openai/gpt-4o-mini",
        #     temperature=1.0,
        #     max_tokens=100
        # )

        # Gemini
        # self.lm = create_lm(
        #     "gemini/gemini-2.5-flash",
        #     temperature=1.0,
        #     max_tokens=100
        # )

        # Ollama (Gemma 4 26B)
        # self.lm = create_lm(
        #     "ollama_chat/gemma4:26b",
        #     temperature=0.7,
        #     max_tokens=200,
        #     think=False
        # )

        # Ollama (gpt-oss)
        # self.lm = create_lm(
        #     "ollama_chat/gpt-oss:20b",
        #     temperature=0,
        #     max_tokens=200,
        #     think="low"
        # )

        self.predictor = dspy.Predict(FreeTalkResponse)
        self.predictor.set_lm(self.lm)

        # 音声認識の暴発防止
        self.last_spoken = ""
        self.last_time = 0.0
        self.min_interval_sec = 1.0

        self.cur_kashiwagi_state = "unknown"

        self.pub_response = rospy.Publisher(
            "/talking_game_response",
            String,
            queue_size=10
        )

        self.set_state_srv = rospy.ServiceProxy(
            "/set_kashiwagi_state",
            SetKashiwagiState
        )

        # self.sub_speech = rospy.Subscriber(
        #     "/speech_to_text",
        #     SpeechRecognitionCandidates,
        #     self.speech_callback,
        #     queue_size=10
        # )

        self.sub_speech = rospy.Subscriber(
            "/filtered_speech",
            SpeechRecognitionCandidates,
            self.speech_callback,
            queue_size=10
        )

        self.sub_state = rospy.Subscriber(
            "/kashiwagi_state",
            String,
            self.state_callback,
            queue_size=10
        )

        rospy.loginfo("FreeTalkResponder started...")
        rospy.spin()

    def is_mentioned(self, spoken_word, word_list):
        if any(word in spoken_word for word in word_list):
            return True
        else:
            return False

    def state_callback(self, msg):
        self.cur_kashiwagi_state = msg.data

    # ---------- TSV読み込み ----------
    def load_character_knowledge(self):
        if not os.path.exists(self.tsv_path):
            rospy.logwarn(
                f"TSV file not found: {self.tsv_path}"
            )
            return ""

        try:
            knowledge = []

            with open(
                self.tsv_path,
                encoding="utf-8-sig"
            ) as f:
                reader = csv.DictReader(f, delimiter="\t")

                for row in reader:
                    question = (
                        row.get("question") or ""
                    ).strip()

                    response = (
                        row.get("response") or ""
                    ).strip()

                    if question and response:
                        knowledge.append(
                            f"質問: {question}\n"
                            f"柏木さんの回答例: {response}"
                        )

            rospy.loginfo(
                f"Loaded TSV knowledge: {len(knowledge)} items"
            )

            return "\n\n".join(knowledge)

        except Exception as e:
            rospy.logerr(
                f"Failed to load TSV: {e}"
            )
            return ""

    # ---------- 出来事テキスト ----------
    def load_event_text(self):
        if not os.path.exists(self.event_path):
            return ""

        try:
            with open(
                self.event_path,
                encoding="utf-8"
            ) as f:
                return f.read().strip()

        except Exception:
            return ""

    # ---------- 音声認識コールバック ----------
    def speech_callback(self, msg):
        if self.cur_kashiwagi_state != "free_talk:listening_turn":
            return

        if not msg.transcript:
            return

        spoken_word = msg.transcript[0].strip()
        print("spoken_word=", spoken_word)

        if not spoken_word or self.is_mentioned(
            spoken_word, ["おわり", "終わり"]
        ):
            return

        now = time.time()

        # 短時間連投防止
        if now - self.last_time < self.min_interval_sec:
            return

        # 同じ発話の再処理防止
        if spoken_word == self.last_spoken:
            return

        self.last_spoken = spoken_word
        self.last_time = now

        rospy.loginfo(f"user said: {spoken_word}")

        try:
            resp = self.set_state_srv(
                "free_talk:thinking_turn"
            )

            if not resp.success:
                rospy.logwarn(
                    f"no state update: {resp.message}"
                )

        except rospy.ServiceException as e:
            rospy.logerr(
                f"Service call failed: {e}"
            )
            return

        # 会話履歴に追加
        self.history.append({
            "role": "user",
            "content": spoken_word
        })

        # 最新の発話を除いた会話履歴
        conversation_history = "\n".join(
            f"{msg['role']}: {msg['content']}"
            for msg in list(self.history)[:-1]
        )

        # 出来事テキスト
        event_text = self.load_event_text()

        # ---------- DSPyによる回答生成 ----------
        try:
            result = self.predictor(
                character_knowledge=self.character_knowledge,
                conversation_history=conversation_history,
                event_text=event_text,
                user_text=spoken_word
            )

            reply = result.response.strip()
            reply = reply.replace("\n", "").replace("\r", "")

            rospy.loginfo(
                f"generated response: {reply}"
            )

            with open(
                f"{self.path_to_pkg}/data/tmp/tmp_response.txt",
                "w",
                encoding="utf-8"
            ) as f:
                f.write(reply)

            # トピックにpublish
            self.pub_response.publish(reply)

            # 履歴へ追加
            self.history.append({
                "role": "assistant",
                "content": reply
            })

        except Exception as e:
            rospy.logerr(
                f"failed to generate response: {e}"
            )


if __name__ == "__main__":
    try:
        FreeTalkResponder()
    except rospy.ROSInterruptException:
        pass
