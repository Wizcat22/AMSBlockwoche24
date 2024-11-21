#!/usr/bin/env python3

import rospy
import subprocess

def shutdown_hook():
    rospy.loginfo("Saving map!")
    subprocess.check_call(['rosrun','map_server','map_saver','-f ~/catkin_ws/src/explore/maps/map'])

try:
    rospy.on_shutdown(shutdown_hook)

except rospy.ROSInterruptException:
    rospy.loginfo("Interrupt received!")