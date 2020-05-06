#!/usr/bin/env python
# Software License Agreement (BSD License)

## Simple talker demo that listens to std_msgs/Strings published
## to the 'ultrasonic' topic
import math
import numpy as np

import rospy
from std_msgs.msg import Float64
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
# from std_msgs.msg import Float32MultiArray  # Unsure if the gps_calc uses 32 or 64. Edit: NOT used
from twizy.msg import car_control

global GPS_history
global all_distances
global pspot_distances
global startpoint
global endpoint
global parking_length
global offset
global distance_to_car
msg_to_publish = Float64MultiArray()
mapping_variables = Float64MultiArray()


def callback_ultrasonicsensor(data):
    # rospy.loginfo(rospy.get_caller_id() + 'i heard %s', data.data)
    current_distances = data  # [behind, rearwheel, frontwheel] distances
    mappning(current_distances)
    # Perhaps a need to append values before going into mapping


def callback_gps(data):
    # rospy.loginfo(rospy.get_caller_id() + 'i heard %s', data.data)
    GPS_history.append(data.data)  # stores: [local_x, local_y, local_angle]


def mappning(current_distances):
    """Function to map the surrounding to the right of the car"""
    global parking_length
    parking_length = 0
    global offset
    offset = 0
    global distance_to_car
    distance_to_car = 0
    global endpoint
    global startpoint

    all_distances.append(current_distances.data)

    if len(all_distances) > 5:  # Only check mapping after 5 values have been recorded
        controller()  # Controls the speed during the mapping sequence. 0 if parkingspot found (endpoint)

        if all_distances[-1][1] > 300 and all_distances[-2][1] > 300 \
                and all_distances[-3][1] > 300:
            # Pair the sensordistances with a gps-position (imaginary) if large enough (empty parkingspot)
            if all_distances[-4][1] <= 300 and all_distances[-5][1] <= 300 \
                    and all_distances[-6][1] <= 300:
                # If it is the first time, include the previous ok measurements (3 in a row)
                pspot_distances.append([all_distances[-3][1], GPS_history[-3]])
                pspot_distances.append([all_distances[-2][1], GPS_history[-2]])
                startpoint = [all_distances[-3][1], GPS_history[-3]]  # Instead of a list, it is now a value
                # Should only be one anyways
                print("\nSTARTPOINT")
                print(startpoint)
            pspot_distances.append([all_distances[-1][1], GPS_history[-1]])
        # print(all_distances[-1][0])
        # print(all_distances[-1][1])
        # print(GPS_history[-1])
        # print(pspot_distances[-1])

        elif all_distances[-1][1] <= 300 and all_distances[-2][1] <= 300 \
                and all_distances[-3][1] <= 300 and \
                all_distances[-4][1] > 300 and all_distances[-5][1] > 300 \
                and all_distances[-6][1] > 300:

            if endpoint == [0, ()]:  # TEST: Changed [] to [0,()]

                # Check if there is an endpoint
                endpoint = [all_distances[-4][1], GPS_history[-4]]  # Instead of a list, it is now a value
                # Should only be one anyways
                print("\nENDPOINT CHECK")
                print(endpoint)

                if math.hypot(endpoint[1][0]-startpoint[1][0], endpoint[1][1]-startpoint[1][1]) < 500:
                    # Check if the length is enough. If not, reset it
                    endpoint = [0, ()]
                    print("\nENDPOINT SUCCESS")
                    print(endpoint)

        # pythagoras of deltax and deltay first[1] for gps, second [0] or [1] for x or y coordinate
        # Only append one endpoint, overwrites the old endpoint

        # Sensor delay roughly 150 ms, so check every 1.5 sec (To change if not good enough)
        position_change = math.hypot(GPS_history[-10][0] - GPS_history[-1][0], GPS_history[-10][1] - GPS_history[-1][1])

        if endpoint != [0, ()] and startpoint != [0, ()]:
            # Continue to update the offset & distance to car
            parking_length = math.hypot(endpoint[1][0] - startpoint[1][0], endpoint[1][1] - startpoint[1][1])
            offset = math.hypot(GPS_history[-1][0] - endpoint[1][0], GPS_history[-1][1] - endpoint[1][1])
            distance_to_car = all_distances[-1][1]  # Latest distance measured from rearwheel

            if position_change < 5:
                talker(1)  # Index tells talker if the car is standing still or not
            else:
                talker(0)  # Still moving

    # TODO:
    # implement that it is only ok if parking_length > 5 meter
    # now it is dependant on GPS signal, have not tested it outside where GPS works
    # If statement for skipping first 5 values for the startpoint
    # Reset startpoint when needed
    # When 3 values are not ok, then 3 again

    # Of the mapping variables, only the offset is to be updated after an endpoint is found
    # if endpoint != [0, ()]:
    #    talker()  # Added this call to the talker to publish mapping variables []


def controller():
    while():
        global endpoint
        pub = rospy.Publisher('controls', car_control, queue_size=5)
        rate = rospy.Rate(10)
        msg_to_publish = car_control()

        # msg_to_publish.speed = 1  # Commented these two lines
        # msg_to_publish.angle = 20
        # Drive forward 1 km/h until endpoint is found (i.e. done mapping, spot found)
        msg_to_publish.angle = 0
        if endpoint != [0, ()]:
            msg_to_publish.speed = 0
        elif endpoint == [0, ()]:
            msg_to_publish.speed = 1

        pub.publish(msg_to_publish)
        rate.sleep()


# Added the following function monday afternoon
def talker(standstill):
    global parking_length
    global offset
    global distance_to_car

    pub = rospy.Publisher('mappning', Float64MultiArray, queue_size=2)
    while not rospy.is_shutdown():
        mapping_variables.data = [parking_length, offset, distance_to_car, standstill]
        # print([parking_length, offset, distance_to_car])
        pub.publish(mapping_variables)
        break  # I och med rospy.spin() behovs val inte while och break? Har utgatt fran gps_calc.
        # Andra aven dar i sa fall


def listener():
    rospy.init_node('mappning', anonymous=True)
    rospy.Subscriber('GPS_pos', Float64MultiArray, callback_gps)  # Changed here to actual gps topic
    rospy.Subscriber('ultrasonic', Float64MultiArray, callback_ultrasonicsensor)
    rospy.spin()


if __name__ == '__main__':
    GPS_history = []
    all_distances = []
    pspot_distances = []
    startpoint = [0, ()]  # Changed both to None/[] (after changing from list to array)
    endpoint = [0, ()]
    rospy.init_node('mappning', anonymous=True)
    listener()

    # 3 since 2.5m + 0.5m margin
    #    if all_distances[len(all_distances)-1][0] > 3 and all_distances[len(all_distances)-2][0] > 3 \
    #            and all_distances[len(all_distances)-3][0] > 3:
    #        # Pair the sensordistances with a gps-position (imaginary) if large enough (empty parkingspot)
    #        if all_distances[len(all_distances)-4][0] <= 3 and all_distances[len(all_distances)-5][0] <= 3 \
    #                and all_distances[len(all_distances)-6][0] <= 3:
    #            # If it is the first time, include the previous ok measurements (3 in a row)
    #            pspot_distances.append([all_distances[len(all_distances)-3][0], GPS_history[len(GPS_history)-3][0]])
    #            pspot_distances.append([all_distances[len(all_distances)-2][0], GPS_history[len(GPS_history)-2][0]])
    #            startpoint.append([all_distances[len(all_distances)-3][0], GPS_history[len(GPS_history)-3][0]])
    #            print(startpoint)
    #        pspot_distances.append([all_distances[len(all_distances)-1][0], GPS_history[len(GPS_history)-1][0]])

    #    elif all_distances[len(all_distances)-1][0] <= 3 and all_distances[len(all_distances)-2][0] <= 3\
    #            and all_distances[len(all_distances)-3][0] <= 3 and \
    #            all_distances[len(all_distances)-4][0] > 3 and all_distances[len(all_distances)-5][0] > 3 \
    #            and all_distances[len(all_distances)-6][0] > 3:
    #        endpoint.append([all_distances[len(all_distances)-4][0], GPS_history[len(GPS_history)-4][0]])
    #        print(endpoint)

    # The following part is copied from above, but counts backwards instead. all "len(GPS_history)" removed
    # Should be exactly the same, but want to test it first
    # [0] changed to [1] since index 1 is for rearwheel sensor, 0 behind
    # Correct..?
    # ALSO, GPS_history[len(GPS_history)-3][0]] --> GPS_history[-3]] Due to an array of x,y,angle
    # Maybe exclude the angle later on?
    # 3 since 2.5m + 0.5m margin
