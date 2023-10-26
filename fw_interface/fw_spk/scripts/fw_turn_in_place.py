#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import rospkg
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import subprocess

class FreewayTurnInPlace:
    def __init__(self):
        self.front_obstacle_sub = rospy.Subscriber("freeway/front_obstacle", Bool, self.front_obstacle_cb, queue_size=1)
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel_ui', Twist, queue_size=1)
        self.rospack = rospkg.RosPack()
        self.pkg_path = self.rospack.get_path('freeway_spk')
        self.front_obstacle_flag = Bool()
        self.audio_playing = False
        self.loop_rate = rospy.Rate(10)  # 10 Hz
	self.timer = None
	rospy.on_shutdown(self.stop_robot)  # register stop_robot method to be called on shutdown

    def stop_robot(self):
	rospy.loginfo("Stopping Robot")
	cmd_vel = Twist()
	cmd_vel.angular.z = 0.0
	cmd_vel.linear.x = 0.0
	self.pub_cmd_vel.publish(cmd_vel)

    def play_audio(self, file_path):
        if not self.audio_playing:
	    if self.timer is not None:
	        self.timer.shutdown()
            self.audio_playing = True
            subprocess.call(['mpg123', '-q' ,file_path])
            self.audio_playing = False
	    self.timer = rospy.Timer(rospy.Duration(10), self.play_standard_audio, True)

    def play_standard_audio(self, event=None):
        audio_file = self.pkg_path+'/scripts/source/gn/gangnam.mp3'
        self.play_audio(audio_file)
        rospy.loginfo("Played %s" %audio_file)

    def front_obstacle_cb(self, data):
        self.front_obstacle_flag = data
        #rospy.loginfo("front_obstacle_flag : %s" %self.front_obstacle_flag)

    def main(self):
        while not rospy.is_shutdown():
            cmd_vel = Twist()
            if (self.front_obstacle_flag.data == False):
                cmd_vel.angular.z = 0.275  # rad/s

            else:
                cmd_vel.angular.z = 0.0
                #cmd_vel.linear.x = 0.0
            self.pub_cmd_vel.publish(cmd_vel)
            self.loop_rate.sleep()

if __name__ == '__main__':
    rospy.init_node('freeway_turn_in_place_node')
    FTIP = FreewayTurnInPlace()
    FTIP.play_standard_audio()
    FTIP.main()
