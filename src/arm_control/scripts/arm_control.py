#!/usr/bin/env python

import roslib
import rospy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import DisplayTrajectory
import moveit_commander
import sys
import time

class ArmControl(object):
    def __init__(self,target_pose_topic):
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.group = moveit_commander.MoveGroupCommander("arm")
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                            DisplayTrajectory,
                                                            queue_size=20)
        self.pose_sub=rospy.Subscriber(target_pose_topic,
                                       PoseStamped,
                                       self.callback,
                                       queue_size=1)

    def callback(self,target_pose):
        # print self.group.get_current_pose()
        print target_pose
        start_time=time.time()
        self.group.stop()
        self.group.set_pose_target(target_pose.pose)
        route_plan = self.group.plan()

        ##for visulization
        display_trajectory = DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(route_plan)
        self.display_trajectory_publisher.publish(display_trajectory)

        self.group.execute(route_plan, wait=False)
        print 'execute done'
        self.group.clear_pose_targets()
        print 'processing time: ',time.time()-start_time


if __name__ == '__main__':
    target_pose_topic='/target_pose'
    arm_control=ArmControl(target_pose_topic)
    rospy.init_node('arm_control',anonymous=True)
    rospy.spin()