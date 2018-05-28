#!/usr/bin/env python

import roslib
import rospy
from sensor_msgs.msg import JointState
import time

class JointState2Servo(object):
    def __init__(self,joint_state_topic):
        self.joint_state_sub = rospy.Subscriber(joint_state_topic,
                                                JointState,
                                                self.callback,
                                                queue_size=1)

    def callback(self,joint_state):
        print joint_state.position
        time.sleep(1)


if __name__ == '__main__':
    joint_state_topic='/joint_states'
    jointstate2servo=JointState2Servo(joint_state_topic)
    rospy.init_node('joint_state2servo',anonymous=True)
    rospy.spin()