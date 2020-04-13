#!/usr/bin/env python

from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
import rospy
msg_to_publish = Float32MultiArray()




     
def talker():

    pub = rospy.Publisher('GPS_pos', Float32MultiArray , queue_size=2)
    while not rospy.is_shutdown():
        msg_to_publish.data = [0.0, 0.0, 0.15]
        pub.publish(msg_to_publish)
	print([0.0,0.0, 0.15])
	rate = rospy.Rate(10)
	rate.sleep()

   
   



 

if __name__ == '__main__':
    rospy.init_node('GPS_test')
    talker()
    

