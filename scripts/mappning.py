#!/usr/bin/env python
# Software License Agreement (BSD License)

## Simple talker demo that listens to std_msgs/Strings published
## to the 'ultrasonic' topic
import math
import numpy

import rospy
from std_msgs.msg import Float64
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float32MultiArray  # Unsure if the gps_calc uses 32 or 64
global GPS_history
global all_distances
global pspot_distances
global startpoint
global endpoint


def callback_ultrasonicsensor(data):
    rospy.loginfo(rospy.get_caller_id() + 'i heard %s', data.data)
    current_distances = data
    mapping(current_distances)
    # Perhaps a need to append values (5 for if-statements) before going into mapping


def callback_gps(data):
    rospy.loginfo(rospy.get_caller_id() + 'i heard %s', data.data)
    GPS_history.append(data.data)


def mapping(current_distances):
    """Function to map the surrounding to the right of the car"""



    all_distances.append(current_distances.data)

    # 3 since 2.5m + 0.5m margin
    if all_distances[len(all_distances)-1][0] > 3 and all_distances[len(all_distances)-2][0] > 3 \
            and all_distances[len(all_distances)-3][0] > 3:
        # Pair the sensordistances with a gps-position (imaginary) if large enough (empty parkingspot)
        if all_distances[len(all_distances)-4][0] <= 3 and all_distances[len(all_distances)-5][0] <= 3 \
                and all_distances[len(all_distances)-6][0] <= 3:
            # If it is the first time, include the previous ok measurements (3 in a row)
            pspot_distances.append([all_distances[len(all_distances)-3][0], GPS_history[len(GPS_history)-3][0]])
            pspot_distances.append([all_distances[len(all_distances)-2][0], GPS_history[len(GPS_history)-2][0]])
            startpoint.append([all_distances[len(all_distances)-3][0], GPS_history[len(GPS_history)-3][0]])
            print(startpoint)
        pspot_distances.append([all_distances[len(all_distances)-1][0], GPS_history[len(GPS_history)-1][0]])

    elif all_Float64MultiArraydistances[len(all_distances)-1][0] <= 3 and all_distances[len(all_distances)-2][0] <= 3\
            and all_distances[len(all_distances)-3][0] <= 3 and \
            all_distances[len(all_distances)-4][0] > 3 and all_distances[len(all_distances)-5][0] > 3 \
            and all_distances[len(all_distances)-6][0] > 3:
        endpoint.append([all_distances[len(all_distances)-4][0], GPS_history[len(GPS_history)-4][0]])
        print(endpoint)
        





def listener():
    rospy.init_node('mapping', anonymous=True)
    rospy.Subscriber('GPS_pos', Float64MultiArray, callback_gps)
    rospy.Subscriber('chatter', Float64MultiArray, callback_ultrasonicsensor)

    rospy.spin()


if __name__ == '__main__':
    GPS_history = []
    all_distances = []
    pspot_distances = []
    startpoint = []
    endpoint = []

    listener()