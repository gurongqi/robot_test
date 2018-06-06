#!/usr/bin/env python

import roslib
import rospy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import DisplayTrajectory
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
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
        self.group.set_random_target()
        # self.group.set_pose_target(target_pose.pose)
        route_plan = self.group.plan()

        new_route_plan = route_plan
        n_joints = len(route_plan.joint_trajectory.joint_names)
        n_points = len(route_plan.joint_trajectory.points)

        spd = 100

        points = list(route_plan.joint_trajectory.points)

        for i in range(n_points):
            point = JointTrajectoryPoint()
            # print 'previous: ',route_plan.joint_trajectory.points[i].time_from_start
            point.time_from_start = route_plan.joint_trajectory.points[i].time_from_start / spd
            # print 'after: ',point.time_from_start
            point.velocities = list(route_plan.joint_trajectory.points[i].velocities)
            point.accelerations = list(route_plan.joint_trajectory.points[i].accelerations)
            point.positions = route_plan.joint_trajectory.points[i].positions

            for j in range(n_joints):
                point.velocities[j] = point.velocities[j] * spd
                point.accelerations[j] = point.accelerations[j] * spd * spd

            points[i] = point


        new_route_plan.joint_trajectory.points = points

        # ##for visulization
        # display_trajectory = DisplayTrajectory()
        # display_trajectory.trajectory_start = self.robot.get_current_state()
        # display_trajectory.trajectory.append(new_route_plan)
        # self.display_trajectory_publisher.publish(display_trajectory)

        self.group.execute(new_route_plan, wait=True)
        print 'execute done'
        self.group.clear_pose_targets()
        print 'processing time: ',time.time()-start_time


if __name__ == '__main__':
    target_pose_topic='/target_pose'
    arm_control=ArmControl(target_pose_topic)
    rospy.init_node('arm_control',anonymous=True)
    rospy.spin()