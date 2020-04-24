#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib as mpl
import string
import math
import glob
import os
import rospy
from std_msgs.msg import Float32Multiarray

x = []
y = []
angle = []
file_name = 'gps_trash.txt'

def new_file():
    global file_name
    new_file_name = raw_input("Enter new file name including .txt: ")
    print(new_file_name)
    with open(new_file_name, 'w+') as f:
        f.write(" ---Remove this line---")
    file_name = new_file_name

def callback(data):
    global x,y, angle
    d = data.data
    gps_pos = d.split(",")
    x = float(gps_pos[0])
    y = float(gps_pos[1])
    angle = float(gps_pos[2]
    with open(file_name, 'a') as f:
        f.write(str(x) + "," + str(y) + "," + str(angle) + "\n")

def listener():
    rospy.init_node('namn', anonymous=True)
    rospy.Subscriber("GPS_calc", Float32MultiArray, callback)
    new_file()
    while(1==1):
        new_file()

    rospy.spin()

if __name__ == '__main__':
	listener()

