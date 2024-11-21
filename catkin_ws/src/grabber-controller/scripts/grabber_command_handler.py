#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, String
import random

class GrabberController:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('motor_command_publisher', anonymous=True)

        # Create a publisher object for the grabber
        self.pub = rospy.Publisher('grabber/angle', Float32, queue_size=10)

        # Create a subscriber object for force feedback
        rospy.Subscriber('grabber/feedback', Float32, self.feedback_callback)

        # Create a subscriber object to listen for commands
        rospy.Subscriber('grabber/command', String, self.handle_command)

        self.rate = rospy.Rate(10)

        # Working variables
        self.force_feedback = 0.0
        self.command = None

    def feedback_callback(self, msg):
        # Process force feedback
        self.force_feedback = msg.data
        rospy.loginfo(f"Received force feedback: {self.force_feedback}")

    def compute_command(self):
        if self.command == "open":
            return 800
        elif self.command == "close":
            return -800
        else:
            return 0.0

    def handle_command(self, msg):
        self.command = msg.data.lower()

    def run(self):
        while not rospy.is_shutdown():
            # Generate a command based on feedback
            command = self.compute_command()

            if command != 0.0:
                self.pub.publish(command)
                rospy.loginfo(f"Publishing command: {command}")

            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = GrabberController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
