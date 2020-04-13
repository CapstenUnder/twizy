#!/usr/bin/env python

from twizy.msg import path_generator
from twizy.scripts.src import path_planner
import rospy
from std_msgs.msg import Float32MultiArray


class Wrapper:

    def shutdown_hook(self):
        print('Goal reached!')

    def mapper_callback(self, msg):

        if msg.data[3]:
            offset = msg.data[0]
            parking_length = msg.data[1]
            distance = msg.data[2]
            current = path_planner.Coordinate(0, 0)
            goal = path_planner.Coordinate(offset + parking_length - 1.25, distance + 1.25)

            start = 0
            p1 = offset
            p2 = offset + parking_length
            end = offset + parking_length + 3

            parking_map = path_planner.Map(start, p1, p2, end, distance, offset).generateMap()

            a, b, c = path_planner.path(current, goal, parking_map)
            msg_to_publish.a = a
            msg_to_publish.b = b
            msg_to_publish.c = c
            msg_to_publish.done = 1
        else:
            print('Mapping not yet done!')
            msg_to_publish.a = 0
            msg_to_publish.b = 0
            msg_to_publish.c = 0
            msg_to_publish.done = 0


if __name__ == '__main__':

    rospy.init_node('PathPlanner')
    pub = rospy.Publisher('path_planner', Float32MultiArray, queue_size=5)
    msg_to_publish = path_generator
    rate = rospy.Rate(1000)  # Adjust rate?

    while not rospy.is_shutdown():
        #rospy.Subscriber('mapping', Float32MultiArray, Wrapper.mapper_callback)
	print('Mapping not yet done!')            
	msg_to_publish.a = 0
        msg_to_publish.b = 0
        msg_to_publish.c = 0
        msg_to_publish.done = 0

        rate.sleep()
