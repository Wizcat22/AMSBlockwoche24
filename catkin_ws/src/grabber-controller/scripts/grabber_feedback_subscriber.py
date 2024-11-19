#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32  # Replace with the correct message type if needed

def feedback_callback(msg):
    # Log or process the received feedback data
    rospy.loginfo(f"Received force feedback: {msg.data}")
    # You can add further logic here to act on the received feedback

def motor_feedback_listener():
    # Initialize the ROS node
    rospy.init_node('motor_feedback_subscriber', anonymous=True)
    # Create a subscriber object
    rospy.Subscriber('motor_feedback', Float32, feedback_callback)
    # Keep the node running and listening for messages
    rospy.spin()

if __name__ == '__main__':
    try:
        motor_feedback_listener()
    except rospy.ROSInterruptException:
        pass
