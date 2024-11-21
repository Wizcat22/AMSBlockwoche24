#!/usr/bin/env python

import rospy
import subprocess
import os

def shutdown_hook():
    rospy.loginfo("Saving map!")
    subprocess.run(['rosrun','map_server','map_saver','-f','~/ams-bw-ws2425/catkin_ws/src/explore/maps/map'])
    print(subprocess.list2cmdline(['rosrun','map_server','map_saver','-f',os.path.join(os.path.dirname(__file__), '../maps/map')]))

try:
    rospy.on_shutdown(shutdown_hook)

except rospy.ROSInterruptException:
    rospy.loginfo("Interrupt received!")