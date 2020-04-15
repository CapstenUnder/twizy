#!/usr/bin/env python

import rospy
# import random
from std_msgs.msg import Float64MultiArray
global counter
a = Float64MultiArray()


def talker():
    pub = rospy.Publisher('GPS_pos', Float64MultiArray, queue_size=10)
    rospy.init_node('testGPS', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    counter=0

    while not rospy.is_shutdown():
        rospy.loginfo(a)
        pub.publish(a)
        rate.sleep()
        counter+=1
        b=counter
        c=2
        d=2

	a.data=[b, c, d]

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
