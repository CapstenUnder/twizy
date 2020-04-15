#!/usr/bin/env python
import pure_pursuit
import rospy
from std_msgs.msg import Float64MultiArray
from twizy.msg import car_control
from matplotlib import pyplot as plt
import numpy as np
import math


class Wrapper:

    def __init__(self):
        self.counter = 0
        self.path_is_ready = False
        self.GPS = None
        self.time = 0
        self.offset = None
        self.parking_length = None
        self.GPS_init_xpos = None
        self.init_counter = 0
        self.backsensor_dist = None

    def shutdown_hook(self):
        print('Goal reached!')


    def ros_plot(self, state, states):
        plt.cla()
        plt.plot(path.cx, path.cy, ".r", label="course")
        plt.plot(states.x, states.y, "-b", label="trajectory")
        plt.legend()
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.axis("equal")
        plt.grid(True)

        plt.subplots(1)
        plt.plot(states.t, [iv * 3.6 for iv in states.v], "-r")
        plt.xlabel("Time[s]")
        plt.ylabel("Speed[km/h]")
        plt.grid(True)
        plt.show()



    def GPS_callback(self, msg):
        if self.init_counter < 1:
            self.GPS_init_xpos = msg.data[0]
            self.init_counter += 1

        print(self.GPS_init_xpos)
        self.GPS = msg.data
        if self.path_is_ready:

            target_speed = -1   # [m/s]
            # initial state
            state.update_from_gps(self.GPS, target_speed)  # yaw+3.14?
            lastIndex = len(path.cx) - 1
            target_ind, _ = path.search_target_index(state)


            states.append(self.time, state)
            self.time += 1
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
            msg_to_publish.speed = target_speed
            print(angle,target_speed)
            pub.publish(msg_to_publish)

            #if lastIndex <= target_ind:

            if self.offset != None and self.parking_length != None:
                dist_traveled_x = np.abs(msg.data[0] - self.GPS_init_xpos)
                if dist_traveled_x > self.offset + self.parking_length:  #TODO: subsribe to arduino aswell and use backsensor to stop
                    print('Goal reached, shutting down')
                    msg_to_publish.angle = 0
                    msg_to_publish.speed = 0
                    print(angle,target_speed)
                    pub.publish(msg_to_publish)

                    self.ros_plot(state, states)
                    rospy.on_shutdown(self.shutdown_hook())
            if self.backsensor_dist <0.3
                print('Goal reached, shutting down')
                msg_to_publish.angle = 0
                msg_to_publish.speed = 0
                print(angle,target_speed)
                pub.publish(msg_to_publish)

                self.ros_plot(state, states)
                rospy.on_shutdown(self.shutdown_hook())


    def path_callback(self, msg):  # fix callback when mapping is done
        #print(msg.data[3])
        path.path_generated =  msg.data[3]
        if path.is_path_generated() == 1 and self.counter < 1 and self.GPS != None:  # TODO: only execute once!
            a =  msg.data[0] # 0.8960
            b =  msg.data[1] # 0.6765
            c =  msg.data[2] # 0
            self.offset = msg.data[4]
            self.parking_length = msg.data[5]
            path.set_path(a, b, c, self.GPS[0], self.GPS[1], self.GPS[2])
            self.counter += 1
            self.path_is_ready = True

    def ultrasonic_callback:(self,msg):
        self.backsensor_dist = msg.data[0]  # 1 ,2 ?

if __name__ == '__main__':

    rospy.init_node('pure_pursuit')
    pub = rospy.Publisher('controls', car_control, queue_size=5)

    msg_to_publish = car_control()

    rate = rospy.Rate(10)  # Adjust rate?
    path = pure_pursuit.TargetCourse()
    wrap = Wrapper()
    state = pure_pursuit.State()
    states = pure_pursuit.States()

    while not rospy.is_shutdown():
        rospy.Subscriber('GPS_pos', Float64MultiArray, wrap.GPS_callback)
        rospy.Subscriber('path_planner', Float64MultiArray, wrap.path_callback)
        rospy.Subscriber('ultrasonic', Float64MultiArray, wrap.ultrasonic_callback)
        rate.sleep()
