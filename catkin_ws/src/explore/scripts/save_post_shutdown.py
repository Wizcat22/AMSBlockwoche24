#!/usr/bin/env python

import rospy
import subprocess
import os

def shutdown_hook():
    rospy.loginfo("Saving map!")
    path = os.path.join(os.path.dirname(__file__), '../maps/map')
    subprocess.run(['rosrun','map_server','map_saver','-f',path])
    print(subprocess.list2cmdline(['rosrun','map_server','map_saver','-f',path]))


if __name__=="__main__":
    rospy.on_shutdown(shutdown_hook)
    try:
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Interrupt received!")
