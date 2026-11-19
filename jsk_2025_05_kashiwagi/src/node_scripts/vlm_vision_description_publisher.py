#!/usr/bin/env python3

import base64
import threading
import time

import cv2
import rospy
import requests

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String


class VLMNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.stop_event = threading.Event()

        self.latest_image = None
        self.latest_received = 0.0
        self.latest_seq = 0

        self.description = ""

        self.model = rospy.get_param(
            "~model", "qwen3-vl:2b-instruct"
        )
        self.image_topic = rospy.get_param(
            "~image_topic", "/camera/color/image_raw"
        )
        self.ollama_url = rospy.get_param(
            "~ollama_url",
            "http://127.0.0.1:11434/api/generate"
        )

        self.prompt = (
            "画像に映っているものを日本語で簡潔に説明してください。"
            "人物、物体、動作を中心に、1文で説明してください。"
            "何かが中心に写っているときは，それについて詳しく教えてください．例えば本が見せられたときは，その中身を詳しく述べたり，ぬいぐるみが見せられたときは，その見た目を詳しく教えてください．"
            "推測はせず、見えている情報だけを答えてください。"
            "必ず日本語で回答してください。"
        )

        self.publisher = rospy.Publisher(
            "/vlm/description",
            String,
            queue_size=1
        )

        self.subscriber = rospy.Subscriber(
            self.image_topic,
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=2**24
        )

        self.worker = threading.Thread(
            target=self.inference_loop,
            daemon=True
        )
        self.worker.start()

        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("ROS 1 VLM node started")

    def image_callback(self, msg):
        # 推論中でも常に最新の画像へ上書き
        with self.lock:
            self.latest_image = msg
            self.latest_received = time.monotonic()
            self.latest_seq += 1

    def inference_loop(self):
        processed_seq = -1

        while not rospy.is_shutdown() and \
                not self.stop_event.is_set():

            with self.lock:
                msg = self.latest_image
                received = self.latest_received
                seq = self.latest_seq

            if msg is None or seq == processed_seq:
                self.stop_event.wait(0.05)
                continue

            processed_seq = seq

            # 古すぎる画像を除外
            if time.monotonic() - received > 1.0:
                continue

            try:
                image = self.bridge.imgmsg_to_cv2(
                    msg, desired_encoding="bgr8"
                )

                start = time.monotonic()
                description = self.run_vlm(image)
                inference_time = time.monotonic() - start

                self.description = description

                self.publisher.publish(
                    String(data=description)
                )

                total_delay = (
                    time.monotonic() - received
                )

                rospy.loginfo(
                    "VLM: %s | inference=%.2fs "
                    "| delay=%.2fs",
                    description,
                    inference_time,
                    total_delay
                )

            except Exception as e:
                rospy.logerr("VLM error: %s", e)
                self.stop_event.wait(0.5)

    def run_vlm(self, image):
        # 画像を最大448pxに縮小
        h, w = image.shape[:2]
        scale = min(1.0, 448.0 / max(w, h))

        new_w = max(1, int(w * scale))
        new_h = max(1, int(h * scale))

        if scale < 1.0:
            image = cv2.resize(
                image, (new_w, new_h),
                interpolation=cv2.INTER_AREA
            )

        success, encoded = cv2.imencode(
            ".jpg",
            image,
            [cv2.IMWRITE_JPEG_QUALITY, 80]
        )

        if not success:
            raise RuntimeError("JPEG encoding failed")

        image_base64 = base64.b64encode(
            encoded.tobytes()
        ).decode("utf-8")

        payload = {
            "model": self.model,
            "prompt": self.prompt,
            "images": [image_base64],
            "stream": False,
            "keep_alive": "30m",
            "options": {
                "num_predict": 32,
                "temperature": 0
            }
        }

        response = requests.post(
            self.ollama_url,
            json=payload,
            timeout=30
        )
        response.raise_for_status()

        return response.json()["response"].strip()

    def shutdown(self):
        self.stop_event.set()
        if self.worker.is_alive():
            self.worker.join(timeout=1.0)


if __name__ == "__main__":
    rospy.init_node("vlm_node")

    node = VLMNode()
    rospy.spin()
