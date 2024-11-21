#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Point
import actionlib
import tf
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def goal_position_callback(data):
    goal_x = data.position.x
    goal_y = data.position.y

    # Berechne den neuen Zielpunkt 30cm vor dem eigentlichen Ziel
    offset_distance = 0.3
    angle = tf.transformations.euler_from_quaternion(
        [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
    )[2]
    new_goal_x = goal_x - offset_distance * rospy.cos(angle)
    new_goal_y = goal_y - offset_distance * rospy.sin(angle)

    # Setze den neuen Zielpunkt
    send_goal(new_goal_x, new_goal_y, data.orientation)

def send_goal(x, y, orientation):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position = Point(x, y, 0)
    goal.target_pose.pose.orientation = orientation

    rospy.loginfo("Sending goal: ({}, {})".format(x, y))
    client.send_goal(goal)
    client.wait_for_result()
    result = client.get_result()
    rospy.loginfo("Result: {}".format(result))

def main():
    rospy.init_node('docking_goal_listener', anonymous=True)
    rospy.Subscriber('/docking_node/goal_position', PoseStamped, goal_position_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Docking goal listener node terminated.")
