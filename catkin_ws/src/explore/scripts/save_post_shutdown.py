#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import subprocess
import os

def save_map():
    rospy.loginfo("Saving map!")
    path = os.path.join(os.path.dirname(__file__), '../maps/map')
    try:
        subprocess.run(['rosrun','map_server','map_saver','-f',path])
        print(subprocess.list2cmdline(['rosrun','map_server','map_saver','-f',path]))
    except Exception as e:
        print(e)

def log_message_callback(msg):
    rospy.loginfo("Received log message: %s", msg.data)
    save_map()

if __name__=="__main__":
    rospy.init_node("map_saver")
    try:
        rospy.Subscriber('/explore_lite_log', String, log_message_callback)
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Interrupt received!")
