#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
import sys


arm_pub = rospy.Publisher("/target_pose",
                          PoseStamped,
                          queue_size=10)
rospy.init_node("publish_pose")
goal = PoseStamped()
goal.header.frame_id = "/base_link"
goal.header.stamp = rospy.Time.now()
goal.pose.position.x = -0.134669803705
goal.pose.position.y = -0.0577499004659
goal.pose.position.z = 0.214173560617
goal.pose.orientation.x = -0.260166705782
goal.pose.orientation.y = 0.76466155771
goal.pose.orientation.z = 0.589570102172
goal.pose.orientation.w = 0.00361690306403
arm_pub.publish(goal)