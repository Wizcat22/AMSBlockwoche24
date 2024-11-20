#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32  # Replace with your message type as needed

def motor_publisher():
    # Initialize the ROS node
    rospy.init_node('motor_command_publisher', anonymous=True)
    # Create a publisher object
    pub = rospy.Publisher('motor_cmd', Float32, queue_size=10)
    rate = rospy.Rate(5)  # 10 Hz

    #while not rospy.is_shutdown():
    for i in range(100):
        rate.sleep()
        command = 350.0 if i % 2 == 0 else 270.0
        rospy.loginfo(f"Publishing motor command: {command}")
        pub.publish(command)

if __name__ == '__main__':
    try:
        motor_publisher()
    except rospy.ROSInterruptException:
        pass
