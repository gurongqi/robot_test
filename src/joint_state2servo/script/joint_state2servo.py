#!/usr/bin/env python

import roslib
import rospy
from sensor_msgs.msg import JointState
import time
from std_msgs.msg import UInt16MultiArray
from std_msgs.msg import MultiArrayDimension
import numpy as np
import copy

class JointState2Servo(object):
    def __init__(self,joint_state_topic):
        self.joint_position=np.zeros((0))
        self.joint_state_sub = rospy.Subscriber(joint_state_topic,
                                                JointState,
                                                self.callback,
                                                queue_size=1)

    def callback(self,joint_state):
        if len(self.joint_position)==0:
            self.joint_position=np.array(joint_state.position)
        else:
            joint_position=joint_state.position
        print joint_position
        mat = UInt16MultiArray()
        mat.layout.dim.append(MultiArrayDimension())
        mat.layout.dim.append(MultiArrayDimension())
        mat.layout.dim[0].label = "servo_number"
        mat.layout.dim[0].size = 6
        mat.layout.dim[0].stride = 3 * 6
        mat.layout.dim[1].label = "single_command"
        mat.layout.dim[1].size = 3
        mat.layout.dim[1].stride = 3
        mat.layout.data_offset = 0
        mat.data = [1, 1500, 1000,
                    2, 1500, 1000,
                    3, 1500, 1000,
                    4, 1500, 1000,
                    5, 1500, 1000,
                    6, 1500, 1000]

        dstride1 = mat.layout.dim[1].stride
        offset = mat.layout.data_offset
        for i in range(6):
            PMW_value = joint_position[i]/1.5708*1000+1500
            mat.data[offset + 1 + i * dstride1]=PMW_value
        print mat
        time.sleep(1)


if __name__ == '__main__':
    joint_state_topic='/joint_states'
    jointstate2servo=JointState2Servo(joint_state_topic)
    rospy.init_node('joint_state2servo',anonymous=True)
    rospy.spin()