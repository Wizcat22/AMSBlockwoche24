#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64  # Replace with your message type as needed

def motor_publisher():
    # Initialize the ROS node
    rospy.init_node('motor_command_publisher', anonymous=True)
    # Create a publisher object
    pub = rospy.Publisher('motor_command', Float64, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        command = 1.0  # Replace with the actual command value you want to publish
        rospy.loginfo(f"Publishing motor command: {command}")
        pub.publish(command)
        rate.sleep()

if __name__ == '__main__':
    try:
        motor_publisher()
    except rospy.ROSInterruptException:
        pass
