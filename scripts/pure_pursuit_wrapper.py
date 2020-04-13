#!/usr/bin/env python


import pure_pursuit
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
from twizy.msg import car_control
from matplotlib import pyplot as plt
import numpy as np
import math


class Wrapper:
    
    # Generate an int array from a simple string
    def string_to_int_arr(z_string):
        z_int_arr = []
        space_removed = z_string.replace(' ', '')
        z_string_arr = space_removed.split(',')
        for j in range(frame_height):
            for i in range(frame_width):
                #print("length of z_string_arr " + str(len(z_string_arr)))
                #print("frame_width length " + str((j * frame_width + i) * 2 + 1))
                z_int_arr.append(int(calculate_z(z_string_arr[(j * frame_width + i) * 2],
                                         z_string_arr[(j * frame_width + i) * 2 + 1]) * forward_max))
        return z_int_arr


    def __init__(self):
        self.counter = 0
        self.path_is_ready = False
	self.GPS = None

    def shutdown_hook(self):
        print('Goal reached!')

    def ros_plot(self, state, states, target_ind):
        plt.cla()
        pure_pursuit.plot_arrow(state.x, state.y, state.yaw)
        plt.plot(path.cx, path.cy, "-r", label="course")
        plt.plot(states.x, states.y, "-b", label="trajectory")
        plt.plot(path.cx[target_ind], path.cy[target_ind], "xg", label="target")
        plt.axis("equal")
        plt.grid(True)
        plt.title("Speed[km/h]:" + str(state.v * 3.6)[:4])
        plt.pause(0.001)

    def GPS_callback(self, msg):
	print('que')
        if self.path_is_ready and self.GPS != None:
            self.GPS = msg.data

            target_speed = -1 / 3.6  # [m/s]
            time = 0.0

            # initial state
            state = pure_pursuit.State(self.GPS[0], self.GPS[1], self.GPS[2], v=0.0)  # yaw+3.14?
            lastIndex = len(path.cx) - 1
            target_ind, _ = path.search_target_index(state)

            states = pure_pursuit.States()
            states.append(time, state)

            while lastIndex > target_ind:

                # Calc control input
                ai = pure_pursuit.proportional_control(target_speed, state.v)
                di, target_ind = pure_pursuit.pure_pursuit_steer_control(
                    state, path, target_ind)
                state.update_from_gps(self.GPS, ai)
                di = di*180/math.pi         # convert to degrees
                if di < -40:
                    angle = -40
                elif di > 40:
                    angle = 40
                else:
                    angle = di
                msg_to_publish.angle = angle
                msg_to_publish.speed = state.v
                pub.publish(msg_to_publish)

                if lastIndex <= target_ind:
                    #self.ros_plot(state, states, target_ind)
                    rospy.on_shutdown(self.shutdown_hook())

    def path_callback(self, msg):  # fix callback when mapping is done
	print('que2')
        #path.path_generated = msg_arr[3]  # msg.data[3]
        if path.is_path_generated() and self.counter < 1 and self.GPS != None:  # only execute once!
            a = 0.8960  # msg.data[0]
            b = 0.6765  # msg.data[1]
            c = 0  # msg.data[2]
            path.set_path(a, b, c, self.GPS[0], self.GPS[1])
            self.counter += 1
            self.path_is_ready = True

if __name__ == '__main__':

    rospy.init_node('pure_pursuit')
    pub = rospy.Publisher('controls', car_control, queue_size=5)
    msg_to_publish = car_control
    rate = rospy.Rate(1000)  # Adjust rate?
    path = pure_pursuit.TargetCourse()
    wrap = Wrapper()

    while not rospy.is_shutdown():
        rospy.Subscriber('GPS_pos', Float32MultiArray, wrap.GPS_callback)
        rospy.Subscriber('path_planner', Float32MultiArray, wrap.path_callback)

        rate.sleep()
