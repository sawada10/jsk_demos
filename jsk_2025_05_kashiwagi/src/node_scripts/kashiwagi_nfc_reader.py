#!/usr/bin/env python3
import rospy
import actionlib
import os
import rospkg

from std_msgs.msg import String
from nfc_ros.msg import Tag
from jsk_2025_05_kashiwagi.srv import SetKashiwagiState
from sound_play.msg import SoundRequestAction, SoundRequestGoal, SoundRequest


class NFCReader:
    def __init__(self):
        rospy.init_node("kashiwagi_nfc_reader")
        rospy.sleep(1.0)

        self.path_to_pkg = rospkg.RosPack().get_path("jsk_2025_05_kashiwagi")

        self.sub_nfc = rospy.Subscriber("/nfc_ros_node/tag", Tag, self.nfc_callback, queue_size=1)
        self.set_state_srv = rospy.ServiceProxy("/set_kashiwagi_state", SetKashiwagiState)
        self.client = actionlib.SimpleActionClient("/robotsound_jp", SoundRequestAction)
        self.last_nfc_time = rospy.Time(0)
        self.is_updated = False
        self.cur_kashiwagi_state = "unknown"
        self.sub_state = rospy.Subscriber("/kashiwagi_state", String, self.state_callback, queue_size=1)
        self.client.wait_for_server()
        rospy.loginfo("Launching NFC listener node ....")
        rospy.spin()

    def state_callback(self, msg):
        self.cur_kashiwagi_state = msg.data

    def _feedback_cb(self, feedback):
        if self.is_updated == False:
            if self.cur_kashiwagi_state == "daily:normal":
                next_state = "daily:happy"
            elif self.cur_kashiwagi_state == "talking_game:listening_turn":
                next_state = "talking_game:happy"
            elif self.cur_kashiwagi_state == "katakanashi:playing":
                next_state = "katakanashi:happy"
            elif self.cur_kashiwagi_state == "shiritori:listening_turn":
                next_state = "shiritori:happy"
            elif self.cur_kashiwagi_state == "free_talk:listening_turn":
                next_state = "free_talk:happy"
            elif self.cur_kashiwagi_state == "propose_game:playing":
                next_state = "propose_game:happy"
            else:
                return

            try:
                resp = self.set_state_srv(next_state)
                rospy.loginfo(
                    f"State updated: {resp.message}"
                    if resp.success else f"State update failed: {resp.message}"
                )
                self.is_updated = True
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")

    def play_sound_file(self, file_path):
        self.is_updated = False
        
        goal = SoundRequestGoal()
        goal.sound_request.sound = SoundRequest.PLAY_FILE
        goal.sound_request.command = SoundRequest.PLAY_ONCE
        goal.sound_request.arg = file_path
        goal.sound_request.volume = 1.0

        rospy.loginfo(f"Playing: {file_path}")
        self.client.send_goal(
            goal,
            feedback_cb=self._feedback_cb
        )

        rate = rospy.Rate(10)
        while not self.client.wait_for_result(timeout=rospy.Duration(0.1)):
            rate.sleep()

        rospy.loginfo("Playback finished.")

    def nfc_callback(self, msg):
        rospy.loginfo(msg.identifier)

        if msg.identifier == "0404409a215980":
            now = rospy.Time.now()
            if (now - self.last_nfc_time).to_sec() < 10.0:
                return
            self.last_nfc_time = now            
            self.sound_file = f"{self.path_to_pkg}/data/kashiwagi_fukafuka.wav"
            self.play_sound_file(self.sound_file)


if __name__ == "__main__":
    try:
        NFCReader()
    except rospy.ROSInterruptException:
        pass
