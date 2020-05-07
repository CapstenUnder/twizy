#!/usr/bin/env python

from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
import rospy
msg_to_publish = Float64MultiArray()





def talker():

    pub = rospy.Publisher('GPS_pos', Float64MultiArray , queue_size=2)
    a=  0
    b= 0
    c= 1.5
    while not rospy.is_shutdown():
        a += 0.5
	b += 0.5
	c += 0.1
        msg_to_publish.data = [a, b, c]
        pub.publish(msg_to_publish)
	print([a, b, c])
	rate = rospy.Rate(1)
	rate.sleep()



if __name__ == '__main__':
    rospy.init_node('GPS_test')
    talker()
