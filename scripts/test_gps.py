#!/usr/bin/env python

from std_msgs.msg import Float32MultiArray
<<<<<<< HEAD
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
    

=======
import rospy
msg_to_publish = Float32MultiArray()

class Test:

    def talker(self,a,b,c):
        pub = rospy.Publisher('GPS_pos', Float32MultiArray, queue_size=2)
        while not rospy.is_shutdown():
            msg_to_publish.data = [a, b, c]
            pub.publish(msg_to_publish)
            a += 0.01
            b += 0.01
            c+= 0.01
            print(msg_to_publish.data[0], msg_to_publish.data[1], msg_to_publish.data[2])

            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('GPS_test')
    rate= rospy.Rate(10)
    test = Test()
    a= 0
    b= 0
    c= 3.14
    test.talker(a,b,c)
>>>>>>> 912211df84ccb8966ffcda15db88beb92f3e38f4
