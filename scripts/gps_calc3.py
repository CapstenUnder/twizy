#!/usr/bin/env python
import numpy as np
import rospy
import utm
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
import matplotlib.pyplot as plt


msg_to_publish = Float32MultiArray()
angle = 0
gps_v_x, gps_h_x, gps_v_y, gps_h_y = 0, 0, 0, 0
x_ref, y_ref, angle_ref = 0, 0, 0
run_once = 0


# sets the reference point.
def get_ref():
    global gps_v_x, gps_v_y
    global x_ref, y_ref, angle_ref
    x_ref = gps_v_x
    y_ref = gps_v_y
    angle_ref = angle


def calc_angle():
    global angle
    global gps_v_x, gps_v_y, gps_h_x, gps_h_y
    gps_x_diff = gps_v_x - gps_h_x
    gps_y_diff = gps_v_y - gps_h_y
    angle = np.arctan2(gps_y_diff, gps_x_diff) + np.pi
    # print([gps_x_diff, gps_y_diff, np.degrees(angle)])


def talker():
    global angle
    global x_ref, y_ref, angle_ref

    pub = rospy.Publisher('GPS_pos', Float32MultiArray, queue_size=2)
    while not rospy.is_shutdown():

        # moves and rotates the global coordinates to local coordinate determined by the reference point and angle
        x = gps_v_x - x_ref
        y = gps_v_y - y_ref
        local_x = x * np.cos(angle_ref) - y * np.sin(angle_ref)
        local_y = x * np.sin(angle_ref) + y * np.cos(angle_ref)
        local_angle = (angle - angle_ref) % 2*np.pi

        msg_to_publish.data = [local_x, local_y, local_angle]
        pub.publish(msg_to_publish)

	size = 5
	plt.scatter(x, y, color="red")
	plt.scatter(local_x, local_y, color="blue")
	plt.axis([-size, size, -size, size])
	plt.ion()
	plt.show()
	plt.pause(0.01)
	print([local_x, local_y, np.degrees(local_angle), angle,  angle_ref])
        #print([gps_v_x, gps_v_y, angle])
        break


def callback_h(data):
    gps_string_h = data.data
    gps_h = gps_string_h.split(",")
    gps_h_lon = float(gps_h[0])
    gps_h_lat = float(gps_h[1])

    global gps_v_x
    global gps_v_y

    gps_v_pos = utm.from_latlon(gps_h_lat, gps_h_lon)
    gps_v_x = gps_v_pos[0]
    gps_v_y = gps_v_pos[1]
    calc_angle()

    global run_once
    if run_once < 10:
        get_ref()
        run_once += 1

    talker()


def callback_v(data):
    gps_string = data.data
    gps_v = gps_string.split(",")
    gps_v_lon = float(gps_v[0])
    gps_v_lat = float(gps_v[1])

    global gps_h_x
    global gps_h_y

    gps_h_pos = utm.from_latlon(gps_v_lat, gps_v_lon)
    gps_h_x = gps_h_pos[0]
    gps_h_y = gps_h_pos[1]


def listener():
    rospy.init_node('GPS_calc', anonymous=True)

    rospy.Subscriber('GPS_left', String, callback_v, queue_size=1)
    rospy.Subscriber('GPS_right', String, callback_h, queue_size=1)

    rospy.spin()


if __name__ == '__main__':
    listener()
