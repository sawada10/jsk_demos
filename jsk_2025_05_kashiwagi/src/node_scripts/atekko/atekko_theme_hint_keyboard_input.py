#!/usr/bin/env python3
import rospy
from std_msgs.msg import String


class AtekkoKeyboardInput:
    def __init__(self):
        rospy.init_node("atekko_keyboard_input")

        self.cur_state = "unknown"

        self.pub_theme = rospy.Publisher("/atekko_theme", String, queue_size=1)
        self.pub_hint = rospy.Publisher("/atekko_hint", String, queue_size=1)

        rospy.Subscriber("/kashiwagi_state", String, self.state_callback, queue_size=1)

        rospy.loginfo("Atekko Keyboard Input started")

    def state_callback(self, msg):
        self.cur_state = msg.data

    def run(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if self.cur_state == "atekko:theme_waiting":
                theme = input("テーマを入力してください: ").strip()

                if theme and self.cur_state == "atekko:theme_waiting":
                    self.pub_theme.publish(theme)
                    rospy.loginfo(f"Theme: {theme}")
                    rospy.sleep(0.5)

            elif self.cur_state == "atekko:hint_waiting":
                hint = input("回答 (y/n/u/s): ").strip().lower()

                if hint not in ["y", "n", "u", "s"]:
                    continue

                if self.cur_state == "atekko:hint_waiting":
                    self.pub_hint.publish(hint)
                    rospy.loginfo(f"Hint: {hint}")
                    rospy.sleep(0.5)

            rate.sleep()


if __name__ == "__main__":
    try:
        AtekkoKeyboardInput().run()
    except (rospy.ROSInterruptException, KeyboardInterrupt, EOFError):
        pass
