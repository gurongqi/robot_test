#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt16MultiArray
from std_msgs.msg import MultiArrayDimension
import random

import numpy as np

if __name__ =="__main__":
    rospy.init_node("publisher")
    pub = rospy.Publisher('toggle_led', UInt16MultiArray, queue_size=1)
    r = rospy.Rate(0.5)
    # let's build a 3x3 matrix:
    mat = UInt16MultiArray()
    mat.layout.dim.append(MultiArrayDimension())
    mat.layout.dim[0].label = "size"
    mat.layout.dim[0].size = 3
    mat.layout.dim[0].stride = 3
    mat.layout.data_offset = 0
    mat.data = [1,500,2000]

    # save a few dimensions:
    dstride0 = mat.layout.dim[0].stride
    offset = mat.layout.data_offset
    while not rospy.is_shutdown():
        tmpmat = np.zeros((3))
        for i in range(3):
            if i==0:
            	num = random.randrange(0,7)
            	num=int(num)
            	mat.data[offset+i] = num
            elif i==1:
            	num = random.randrange(0,10)
            	num=int(num)*200+500
            	mat.data[offset+i] = num
            elif i==2:
            	num=2000
            	mat.data[offset+i] = num

            tmpmat[i] = num
        pub.publish(mat)
        rospy.loginfo("I'm sending:")
        print tmpmat,"\r\n"
        r.sleep()