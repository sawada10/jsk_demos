#!/usr/bin/env python3

import rospy
import os
import json
import random
from std_msgs.msg import String, Float32
from jsk_2025_05_kashiwagi.srv import SetKashiwagiState
from ollama import chat


class ResponseGenerator:
    def __init__(self):
        rospy.init_node("propose_game_response_generator")
        rospy.sleep(1)

        base_dir = os.path.dirname(__file__)
        self.json_path = os.path.join(base_dir, "propose_game_word_card.json")

        self.last_qr_distance = None
        self.qr_distance_threshold = 0.10
        self.scanned_qr_ids = []
        self.required_qr_count = 6
        self.cur_state = "unknown"
        self.model_name = "gemma4:26b"

        self.initial_cards = {
            "group1": [
                {"id": "group1-a", "text": "君にとって"},
                {"id": "group1-b", "text": "君の"},
                {"id": "group1-c", "text": "君と"},
                {"id": "group1-d", "text": "君だけの"},
                {"id": "group1-e", "text": "君を"}
            ],
            "group2": [
                {"id": "group2-a", "text": "僕にとって"},
                {"id": "group2-b", "text": "僕の"},
                {"id": "group2-c", "text": "僕と"},
                {"id": "group2-d", "text": "僕だけの"},
                {"id": "group2-e", "text": "僕を"}
            ],
            "group3": [
                {"id": "group3-a", "text": "僕に"},
                {"id": "group3-b", "text": "僕"},
                {"id": "group3-c", "text": "僕が"},
                {"id": "group3-d", "text": "僕も"},
                {"id": "group3-e", "text": "僕は"}
            ],
            "group4": [
                {"id": "group4-a", "text": "君に"},
                {"id": "group4-b", "text": "君"},
                {"id": "group4-c", "text": "君が"},
                {"id": "group4-d", "text": "君も"},
                {"id": "group4-e", "text": "君は"}
            ],
            "group5": [
                {"id": "group5-a", "text": "だけの"},
                {"id": "group5-b", "text": "に"},
                {"id": "group5-c", "text": "を"},
                {"id": "group5-d", "text": "より"},
                {"id": "group5-e", "text": "が"},
                {"id": "group5-f", "text": "と"},
                {"id": "group5-g", "text": "は"},
                {"id": "group5-h", "text": "の"}
            ],
            "group6": [
                {"id": "group6-a", "text": "大切にするよ"},
                {"id": "group6-b", "text": "愛してる"}
            ]
        }

        self.word_map = {}
        try:
            with open(self.json_path, encoding="utf-8") as f:
                data = json.load(f)
            for item in data:
                self.word_map[int(item["id"])] = item["text"]
            rospy.loginfo(f"succeeded in reading json: {len(self.word_map)} entries")
        except Exception as e:
            rospy.logerr(f"failed to read json: {e}")
            return

        self.initial_card_map = {}
        for cards in self.initial_cards.values():
            for card in cards:
                self.initial_card_map[card["id"]] = card["text"]

        self.pub_response = rospy.Publisher("/propose_game_response", String, queue_size=10)
        self.set_state_srv = rospy.ServiceProxy("/set_kashiwagi_state", SetKashiwagiState)

        rospy.Subscriber("/qr_distance", Float32, self.depth_update_callback, queue_size=1)
        rospy.Subscriber("/qr_data", String, self.response_callback, queue_size=1)
        rospy.Subscriber("/kashiwagi_state", String, self.state_callback, queue_size=1)

        rospy.loginfo("Propose Game Response Generator started")
        rospy.spin()

    def state_callback(self, msg):
        self.cur_state = msg.data

    def depth_update_callback(self, msg):
        self.last_qr_distance = msg.data

    def change_state(self, state):
        try:
            resp = self.set_state_srv(state)
            if resp.success:
                rospy.loginfo(f"state updated: {resp.message}")
                return True
            rospy.logwarn(f"state update failed: {resp.message}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
        return False

    def response_callback(self, msg):
        qr_text = msg.data.strip()
        rospy.loginfo(f"received QR data: {qr_text}")

        if self.last_qr_distance is None:
            rospy.logwarn("QR distance has not been received yet")
            return
        if self.last_qr_distance > self.qr_distance_threshold:
            rospy.logwarn(f"QR code is too far: {self.last_qr_distance:.3f} m")
            return
        if not qr_text.isdigit():
            rospy.logwarn("QR code data is not number")
            return

        qr_id = int(qr_text)

        if qr_id not in self.word_map:
            rospy.logwarn(f"Cannot find corresponding id: {qr_id}")
            return
        if len(self.scanned_qr_ids) >= self.required_qr_count:
            rospy.loginfo("Already collected 6 QR cards")
            return
        if qr_id in self.scanned_qr_ids:
            rospy.loginfo(f"QR {qr_id} has already been scanned")
            return

        self.scanned_qr_ids.append(qr_id)
        rospy.loginfo(f"QR: {qr_id} -> {self.word_map[qr_id]}")
        rospy.loginfo(f"QR count: {len(self.scanned_qr_ids)} / {self.required_qr_count}")

        if len(self.scanned_qr_ids) == self.required_qr_count:
            rospy.loginfo("6 QR cards collected")
            self.change_state("propose_game:thinking_turn")
            self.generate_and_select_proposal()

    def validate_ids(self, ids):
        if not isinstance(ids, list):
            raise ValueError("ids is not a list")

        ids = [str(x) for x in ids]
        required_ids = [str(x) for x in self.scanned_qr_ids]

        for qr_id in required_ids:
            if ids.count(qr_id) != 1:
                raise ValueError(f"required QR id {qr_id} must appear exactly once")

        allowed_ids = set(required_ids) | set(self.initial_card_map.keys())

        for card_id in ids:
            if card_id not in allowed_ids:
                raise ValueError(f"invalid card id: {card_id}")

        used_groups = set()
        for card_id in ids:
            if not card_id.startswith("group"):
                continue
            group = card_id.split("-")[0]
            if group in used_groups:
                raise ValueError(f"multiple cards used from {group}")
            used_groups.add(group)

        if len(ids) > 12:
            raise ValueError("too many cards")

        return ids

    def get_card_text(self, card_id):
        if card_id.startswith("group"):
            return self.initial_card_map[card_id]
        return self.word_map[int(card_id)]

    def ids_to_proposal(self, ids):
        return " ".join(f'"{self.get_card_text(card_id)}"' for card_id in ids)

    def is_sentence_ending_card(self, card_id):
        if card_id.startswith("group6"):
            return True
        if card_id.startswith("group"):
            return False

        text = self.word_map[int(card_id)]
        endings = (
            "だよ", "だね", "だろ？", "かな？", "ないか？", "くれ", "ほしい",
            "したいんだ", "してみせる", "するよ", "愛してる", "なのさ",
            "ですか？", "変かな？", "かもしれない", "離さない", "守るよ",
            "一生一緒だよ", "我慢できないんだ", "耐えられないんだ",
            "失いたくないんだ", "夢中さ", "そう思うだろ？",
            "どうにかなってしまいそうだ", "になってくれないか？",
            "作ってくれないか？", "受け取ってほしい", "いてほしい",
            "暮らさないか？", "埋めてくれるかい？"
        )
        return text.endswith(endings)

    def card_type(self, card_id):
        text = self.get_card_text(card_id)

        if card_id.startswith("group5"):
            return "particle"
        if text in ["僕は", "僕が", "僕も", "君は", "君が", "君も"]:
            return "subject"
        if text in ["僕の", "僕だけの", "君の", "君だけの"]:
            return "possessive"
        if text in ["僕を", "僕に", "君を", "君に", "僕と", "君と"]:
            return "case_phrase"
        if self.is_sentence_ending_card(card_id):
            return "predicate"

        noun_ids = {
            505, 509, 513, 517, 520, 523, 524, 529, 532, 533, 536, 538, 539,
            543, 545, 549, 551, 553, 556, 557, 562, 566, 567, 571, 578, 580,
            582, 586, 592, 594, 605, 609, 612, 615, 623, 626, 632, 636, 639,
            643, 644, 646, 650, 653, 655, 661
        }

        modifier_ids = {
            506, 512, 528, 537, 568, 577, 583, 591, 602, 620, 621, 631, 652, 657
        }

        adverb_ids = {
            501, 504, 508, 511, 541, 555, 561, 584, 585, 590, 599, 619,
            628, 634, 637, 645, 647, 659
        }

        if not card_id.startswith("group"):
            qr_id = int(card_id)
            if qr_id in noun_ids:
                return "noun"
            if qr_id in modifier_ids:
                return "modifier"
            if qr_id in adverb_ids:
                return "adverb"

        return "other"

    def preferred_position(self, card_id):
        card_type = self.card_type(card_id)
        text = self.get_card_text(card_id)

        if card_type == "subject":
            return 0.10
        if card_type == "possessive":
            return 0.25
        if card_type == "case_phrase":
            return 0.40
        if card_type == "modifier":
            return 0.35
        if card_type == "noun":
            return 0.50
        if card_type == "particle":
            return 0.60
        if card_type == "adverb":
            return 0.65
        if card_type == "predicate":
            return 0.90

        if text.endswith("ながら") or text.endswith("想うと") or text.endswith("触れると"):
            return 0.35

        return 0.50

    def pair_score(self, a, b):
        a_text = self.get_card_text(a)
        b_text = self.get_card_text(b)
        a_type = self.card_type(a)
        b_type = self.card_type(b)
        score = 0.0

        if a_type == "possessive" and b_type == "noun":
            score += 10.0
        if a_type == "modifier" and b_type == "noun":
            score += 9.0
        if a_type == "noun" and b_type == "particle":
            score += 7.0
        if a_type == "case_phrase" and b_type == "predicate":
            score += 8.0
        if a_type == "adverb" and b_type == "predicate":
            score += 6.0
        if a_type == "subject" and b_type in ["noun", "case_phrase", "modifier"]:
            score += 4.0

        if a_type == "particle" and b_type == "particle":
            score -= 15.0
        if a_type == "predicate" and b_type == "predicate":
            score -= 8.0
        if a_type == "possessive" and b_type == "possessive":
            score -= 12.0
        if a_type == "subject" and b_type == "subject":
            score -= 10.0
        if a_type == "predicate" and b_type in ["subject", "possessive"]:
            score -= 8.0

        if a_text in ["僕は", "僕が", "僕も"] and b_text in ["僕の", "僕だけの"]:
            score -= 10.0
        if a_text in ["君は", "君が", "君も"] and b_text in ["君の", "君だけの"]:
            score -= 10.0

        if a_text in ["僕は", "僕が", "僕も"] and b_text in ["君の", "君だけの", "君を", "君に"]:
            score += 6.0

        if a_text in ["君の", "僕の", "君だけの", "僕だけの"] and b_type == "noun":
            score += 4.0

        if a_text in ["君を", "君に", "僕を", "僕に"] and b_type == "predicate":
            score += 6.0

        if a_text in ["君と", "僕と"] and b_text in ["一緒に", "手を取り合って", "これからもずっと"]:
            score += 6.0

        return score

    def triple_score(self, a, b, c):
        a_text = self.get_card_text(a)
        a_type = self.card_type(a)
        b_type = self.card_type(b)
        c_type = self.card_type(c)
        score = 0.0

        if a_type in ["possessive", "modifier"] and b_type == "noun" and c_type == "particle":
            score += 12.0

        if a_text in ["僕は", "僕が", "僕も"] and b_type == "possessive" and c_type == "noun":
            score += 10.0

        if a_text in ["僕は", "僕が", "僕も"] and b_type == "case_phrase" and c_type == "predicate":
            score += 12.0

        if a_type == "noun" and b_type == "particle" and c_type == "predicate":
            score += 12.0

        if a_type == "modifier" and b_type == "noun" and c_type == "predicate":
            score += 7.0

        if a_type == "subject" and b_type == "subject":
            score -= 10.0

        if a_type == "predicate" and b_type == "subject":
            score -= 10.0

        return score

    def candidate_score(self, ids):
        score = 0.0
        n = len(ids)

        initial_count = sum(card_id.startswith("group") for card_id in ids)
        score += initial_count * 0.8

        for i, card_id in enumerate(ids):
            pos = i / max(n - 1, 1)
            preferred = self.preferred_position(card_id)
            card_type = self.card_type(card_id)

            score -= abs(pos - preferred) * 3.0

            if card_type == "particle" and (i == 0 or i == n - 1):
                score -= 15.0

            if card_id.startswith("group6"):
                if i == n - 1:
                    score += 12.0
                elif pos > 0.70:
                    score += 5.0
                else:
                    score -= 5.0

            if self.is_sentence_ending_card(card_id) and pos > 0.65:
                score += 3.0

        for i in range(n - 1):
            score += self.pair_score(ids[i], ids[i + 1])

        for i in range(n - 2):
            score += self.triple_score(ids[i], ids[i + 1], ids[i + 2])

        ending_count = sum(self.is_sentence_ending_card(card_id) for card_id in ids)

        if ending_count > 2:
            score -= (ending_count - 2) * 6.0

        for i in range(n - 1):
            if self.is_sentence_ending_card(ids[i]) and i < n - 2:
                score -= 5.0

        return score

    def generate_legal_candidates(self, num_candidates=100):
        required_ids = [str(x) for x in self.scanned_qr_ids]
        group_names = list(self.initial_cards.keys())
        raw_candidates = []
        seen = set()

        for _ in range(15000):
            initial_count = random.choice([2, 3, 3, 4, 4, 4, 5])
            selected_groups = random.sample(group_names, initial_count)

            if "group6" not in selected_groups and random.random() < 0.85:
                replace_group = random.choice(selected_groups)
                selected_groups.remove(replace_group)
                selected_groups.append("group6")

            initial_ids = [random.choice(self.initial_cards[group])["id"] for group in selected_groups]
            ids = required_ids + initial_ids

            ids.sort(key=lambda x: self.preferred_position(x) + random.uniform(-0.25, 0.25))

            for _ in range(random.randint(0, 2)):
                a = random.randrange(len(ids))
                b = random.randrange(len(ids))
                ids[a], ids[b] = ids[b], ids[a]

            key = tuple(ids)

            if key in seen:
                continue

            try:
                ids = self.validate_ids(ids)
            except Exception:
                continue

            seen.add(key)
            raw_candidates.append(ids)

        raw_candidates.sort(key=self.candidate_score, reverse=True)
        candidates = raw_candidates[:num_candidates]

        rospy.loginfo(f"generated {len(raw_candidates)} legal candidates, selected top {len(candidates)}")

        rospy.loginfo("===== TOP 10 HEURISTIC CANDIDATES =====")
        for i, ids in enumerate(candidates[:10]):
            rospy.loginfo(f"{i}: score={self.candidate_score(ids):.2f} {self.ids_to_proposal(ids)}")

        return candidates

    def select_from_batch(self, candidates):
        candidate_data = [{"number": i, "proposal": self.ids_to_proposal(ids)} for i, ids in enumerate(candidates)]

        prompt = f"""
以下のプロポーズ候補から、最も良いものを1つだけ選んでください。

候補:
{json.dumps(candidate_data, ensure_ascii=False)}

評価基準:
1. 日本語として自然で意味が通る
2. 単語同士のつながりが自然
3. 愛情が伝わる
4. ロマンチックで詩的
5. プロポーズとして魅力的
6. 少し変な単語が含まれていても、全体として最も自然な意味に読める

特に、単語がそれぞれ良くても、つなげた日本語として不自然な候補は選ばないでください。

重要:
- 候補を変更してはいけません
- カードの順番を変更してはいけません
- 新しい候補を作ってはいけません
- 存在する候補のnumberを1つだけ選んでください

JSONだけで回答してください。
{{"selected":0}}
"""

        for attempt in range(3):
            try:
                response = chat(
                    model=self.model_name,
                    messages=[
                        {"role": "system", "content": "候補の中から、日本語として最も自然でロマンチックなプロポーズを1つだけ選んでください。文章やカード順は変更しないでください。"},
                        {"role": "user", "content": prompt}
                    ],
                    format="json",
                    think=False,
                    stream=False,
                    options={"temperature": 0.1, "num_predict": 30}
                )

                content = response["message"]["content"]
                rospy.loginfo(f"selection response: {content}")

                result = json.loads(content)

                if "selected" not in result:
                    raise ValueError("'selected' not found")

                selected = int(result["selected"])

                if selected < 0 or selected >= len(candidates):
                    raise ValueError(f"invalid selected candidate: {selected}")

                return candidates[selected]

            except Exception as e:
                rospy.logwarn(f"selection failed ({attempt + 1}/3): {e}")

        raise RuntimeError("failed to select candidate")

    def select_best_candidate(self, candidates):
        batch_size = 20
        winners = []

        for start in range(0, len(candidates), batch_size):
            batch = candidates[start:start + batch_size]
            batch_number = start // batch_size + 1

            rospy.loginfo(f"selecting batch {batch_number}: {len(batch)} candidates")

            winner = self.select_from_batch(batch)
            winners.append(winner)

            rospy.loginfo(f"batch {batch_number} winner: {self.ids_to_proposal(winner)}")

        if len(winners) == 1:
            return winners[0]

        rospy.loginfo(f"selecting final winner from {len(winners)} batch winners")

        for i, winner in enumerate(winners):
            rospy.loginfo(f"final candidate {i}: {self.ids_to_proposal(winner)}")

        return self.select_from_batch(winners)

    def generate_and_select_proposal(self):
        try:
            candidates = self.generate_legal_candidates(100)

            if not candidates:
                raise ValueError("No legal candidates generated")

            rospy.loginfo("Selecting best proposal with LLM")

            best_ids = self.select_best_candidate(candidates)
            best_ids = self.validate_ids(best_ids)

            proposal = self.ids_to_proposal(best_ids)

            final_result = {
                "proposal": proposal,
                "ids": [int(x) if x.isdigit() else x for x in best_ids],
                "heuristic_score": self.candidate_score(best_ids)
            }

            final_json = json.dumps(final_result, ensure_ascii=False)

            rospy.loginfo("===== SELECTED PROPOSAL =====")
            rospy.loginfo(final_json)

            print(json.dumps(final_result, ensure_ascii=False, indent=2))

            self.pub_response.publish(final_json)
            self.change_state("propose_game:speaking_turn")

        except Exception as e:
            rospy.logerr(f"failed to generate/select proposal: {e}")


if __name__ == "__main__":
    try:
        ResponseGenerator()
    except rospy.ROSInterruptException:
        pass
