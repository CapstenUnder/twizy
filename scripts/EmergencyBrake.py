#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from twizy.msg import car_control


def callback_ultrasonicsensor(data):
    #rospy.loginfo(rospy.get_caller_id() + 'i heard %s', data.data)
    current_distances = data.data #array from ultrasonic is saved in current distances
    dist_file = open("distances.txt", "a")
    dist_file.write(str(current_distances[2]) + ",") #write values from rear sensor to file

def main():
    print('main')
    pub = rospy.Publisher('controls', car_control, queue_size=5)
    rate = rospy.Rate(10) # 10hz
  # initializes the custom message and makes sure the starting values are 0
    msg = car_control()
    speed = 0
    collision_warning = emergency_brake()
    if(collision_warning == True):
        while not rospy.is_shutdown():
			msg.speed = speed
			print("Distance to rear sensor too small! Engaging emergencybrake")
			pub.publish(msg)
			rate.sleep()
			


def emergency_brake(): #read values from file, take mean value of 10, if < 0.3 return true
    dist = 0
    dist_array = []
    with open("distances.txt", "r") as dist_file:
		for line in dist_file.readlines():
        	    f_list = [float(i) for i in line.split(",") if i.strip()]
        	    dist_array += f_list
    
    if len(dist_array) > 15:
	    last_index = len(dist_array)
	    first_index = last_index - 10

	    for x in range(first_index, last_index):
		dist += dist_array[x]
	    print(dist/10)
	    if dist/10 < 0.3:
			return True


def listener():
    pass

if __name__ == '__main__':
    rospy.init_node('emergency_brake_listener', anonymous=True)
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        try:
	    rospy.Subscriber('chatter', Float64MultiArray, callback_ultrasonicsensor)
            main()
	    rate.sleep()
        except rospy.ROSInterruptException:
            pass
