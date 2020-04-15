#!/usr/bin/env python
import math
import numpy as np
import rospy
import os
import errno
import utm
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
msg_to_publish = Float32MultiArray()
global qr
qr = 0

def calcangle():
     global angle
     gps_x_diff = gps_h_x - gps_v_x
     gps_y_diff = gps_h_y - gps_v_y
     #print( gps_x_diff)
     #print( gps_y_diff)
     angle = np.arctan2(gps_y_diff, gps_x_diff) + np.pi/2
     
     


     
def talker():

    pub = rospy.Publisher('GPS_pos', Float32MultiArray , queue_size=2)
    while not rospy.is_shutdown():
        msg_to_publish.data = [gps_v_x, gps_v_y, angle]
        pub.publish(msg_to_publish)
	print([gps_v_x, gps_v_y, angle])
	break
        
def callback_h(data):
    gps_string_h = data.data
    GPS_h = gps_string_h.split(",")
    GPS_h_lon = float(GPS_h[0])
    GPS_h_lat = float(GPS_h[1])

    long_start = 11.9776858793
    lat_start = 57.6884871703

    r_earth = 6371000
    
 
    global gps_v_x
    global gps_v_y
    #gps_v_x = r_earth * math.sin(90-lat_start*math.pi/180) * GPS_h_lon*math.pi/180 
    #gps_v_y = r_earth * GPS_h_lat*math.pi/180
    #test utm
    gps_v_pos = utm.from_latlon(GPS_h_lat,GPS_h_lon)
    gps_v_x = gps_v_pos[0]
    gps_v_y = gps_v_pos[1]
    #print(data.data)
    #print('hej')
    #print(GPS_h_lon)
    #print(GPS_h_lat)
    
   
   
    calcangle()
   
    talker()

def callback_v(data):
    gps_string = data.data
    GPS_v = gps_string.split(",")
    GPS_v_lon = float(GPS_v[0])
    GPS_v_lat = float(GPS_v[1])

    long_start = 11.9776858793
    lat_start = 57.6884871703

    r_earth = 6371000
 
    global gps_h_x
    global gps_h_y
    #gps_h_x = r_earth * math.sin(90-lat_start*math.pi/180) * GPS_v_lon*math.pi/180 
    #gps_h_y = r_earth * GPS_v_lat*math.pi/180 
    
    gps_h_pos = utm.from_latlon(GPS_v_lat,GPS_v_lon)
    gps_h_x = gps_h_pos[0] 
    gps_h_y = gps_h_pos[1]
    global qr
    
    qr = 1
    
#test
#def callback_test(data):
#    gps_pooos = data.data
#    global gps_h_x
#    global gps_h_y
#    global gps_v_x
#    global gps_v_y
#    global qr
#    qr = 1
#    gps_h_x = gps_pooos[2]
#    gps_h_y = gps_pooos[3]
#    gps_v_x = gps_pooos[0]
#    gps_v_y = gps_pooos[1]
#    calcangle()
   
#    talker()
    
    
def listener():
    #FIFO = "/tmp/posdata"
    #try:
    #    os.mkfifo(FIFO)
    #except OSError as oe: 
    #    if oe.errno != errno.EEXIST:
    #        raise

    #print("Opening FIFO...")
    #with open(FIFO) as fifo:
    #    print("FIFO opened")
    #    while True:
    #        data = fifo.read()
    #        if len(data) == 0:
    #            print("Writer closed")
    #            break
    #        print('Read: "{0}"'.format(data))
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('GPS_calc', anonymous=True)

    #test
    #rospy.Subscriber('GPS_pos_fake', Float32MultiArray , callback_test)
    
  
    rospy.Subscriber('GPS_left', String, callback_v, queue_size=1)
    rospy.Subscriber('GPS_right', String, callback_h, queue_size=1)

    #spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
