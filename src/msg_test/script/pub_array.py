#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt16MultiArray
from std_msgs.msg import MultiArrayDimension
import random

import numpy as np

if __name__ =="__main__":
    pub = rospy.Publisher('toggle_led', UInt16MultiArray, queue_size=0)
    rospy.init_node("publisher")
    r = rospy.Rate(0.1)
    # let's build a 3x3 matrix:
    mat = UInt16MultiArray()
    mat.layout.dim.append(MultiArrayDimension())
    mat.layout.dim.append(MultiArrayDimension())
    mat.layout.dim[0].label = "servo_number"
    mat.layout.dim[0].size = 6
    mat.layout.dim[0].stride = 3*6
    mat.layout.dim[1].label = "single_command"
    mat.layout.dim[1].size = 3
    mat.layout.dim[1].stride = 3
    mat.layout.data_offset = 0
    mat.data = [1,1500,1000,
                2,1500,1000,
                3,1500,1000,
                4,1500,1000,
                5,1500,1000,
                6,1500,1000]

    # save a few dimensions:
    dstride1 = mat.layout.dim[1].stride
    offset = mat.layout.data_offset
    while not rospy.is_shutdown():
        tmpmat = np.zeros((6,3))
        for i in range(6):
            for j in range(3):
                if j==0:
                    num=i+1
                    mat.data[offset+j+i*dstride1] = num
                elif j==1:
                	num = random.randint(0, 2000)+500
                	mat.data[offset+j+i*dstride1] = num
                elif j==2:
                	num=1000
                	mat.data[offset+j+i*dstride1] = num

                tmpmat[i,j] = num
        pub.publish(mat)
        print mat.data
        rospy.loginfo("I'm sending:")
        print tmpmat,"\r\n"
        r.sleep()
        print '*****'