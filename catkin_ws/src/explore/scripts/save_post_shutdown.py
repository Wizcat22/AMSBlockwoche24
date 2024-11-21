#!/usr/bin/env python

import rospy
from std_msgs.msg import String 
from threading import Timer
import subprocess
import os
# Timeout in Sekunden 
TIMEOUT = 10.0 

def save_map():
    rospy.loginfo("Saving map!")
    path = os.path.join(os.path.dirname(__file__), '../maps/map')
    try:
        subprocess.run(['rosrun','map_server','map_saver','-f',path])
        print(subprocess.list2cmdline(['rosrun','map_server','map_saver','-f',path]))
    except Exception as e:
        print(e)

def message_callback(msg): 
    global timer # Timer zurücksetzen 
    timer.cancel() 
    timer = Timer(TIMEOUT, save_map) 
    timer.start()

if __name__=="__main__":
    rospy.init_node("map_saver")
    try:
        rospy.Subscriber('/explore/frontier', String, message_callback)

        # Initialer Timer-Start
        timer = Timer(TIMEOUT, save_map)
        timer.start()

        rospy.spin()
        # Timer stoppen, wenn der Node beendet wird
        timer.cancel()

    except rospy.ROSInterruptException:
        rospy.loginfo("Interrupt received!")
