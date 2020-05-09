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
        self.collision_warning = False
	self.stop = False
	self.target_ind = None



    def shutdown_hook(self):
        print('Goal reached!')


    def ros_plot(self, states):
        plt.cla()
        plt.plot(path.cx, path.cy, ".r", label="course")
        plt.plot(states.x, states.y, "-b", label="trajectory")
        plt.legend()
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.axis("equal")
        plt.grid(True)
	plt.show()


    def GPS_callback(self, msg):
        self.GPS = msg.data
        if self.path_is_ready and self.stop == False:
	    if self.init_counter < 1:
		self.GPS_init_xpos = msg.data[0]
		self.init_counter += 1
            #print(self.GPS[0], self.GPS[1], np.degrees(self.GPS[3]))
            target_speed = -1   # [m/s]
            # initial state
            state.update_from_gps(self.GPS, target_speed)  # yaw+3.14?
            lastIndex = len(path.cx) - 1

            states.append(self.time, state)
            self.time += 1
            # Calc control input

            di, self.target_ind = pure_pursuit.pure_pursuit_steer_control(
                state, path, self.target_ind)


            di = -di*180/math.pi         # convert to degrees
            if di < -40:
                angle = -40		# capping the steering angle at +-40 degrees
            elif di > 40:
                angle = 40
            else:
                angle = di
            msg_to_publish.angle = angle
            msg_to_publish.speed = target_speed
            print(angle,target_speed)
            pub.publish(msg_to_publish)



          	# stops if:
            dist_traveled_x = np.abs(msg.data[0] - self.GPS_init_xpos)
            if dist_traveled_x > self.offset + self.parking_length - 0.5  or self.collision_warning:
            #if self.target_ind > lastIndex:
	        print('Goal reached, shutting down')
                msg_to_publish.angle = 0
                msg_to_publish.speed = 0
                print(angle,target_speed)
                pub.publish(msg_to_publish)
                self.ros_plot(states)
                #rospy.on_shutdown(self.shutdown_hook())

		self.stop = True
		exit()


    def path_callback(self, msg):
        path.path_generated =  msg.data[3]
        if path.is_path_generated() == 1 and self.counter < 1 and self.GPS != None:   #run once and only if gps signal is active
            a =  msg.data[0] # 0.8960
            b =  msg.data[1] # 0.6765
            c =  msg.data[2] # 0
            self.offset = msg.data[4]
            self.parking_length = msg.data[5]
            path.set_path(a, b, c, self.GPS[0], self.GPS[1], self.GPS[3], self.offset, self.parking_length)	#generate path-------------------
            self.counter += 1
	    state.update_from_gps(self.GPS , -1)
	    self.target_ind, _ = path.search_target_index(state)
	    self.path_is_ready = True


    def ultrasonic_callback(self,msg):
	if self.path_is_ready:
            current_distances = msg.data #array from ultrasonic is saved in current distances
            dist_file.write(str(current_distances[0]) + ",") #write values from rear sensor to file


    def emergency_break(self):
	print ('collsion: ' , self.collision_warning)
        if self.path_is_ready: 
	   # initializes the custom message and makes sure the starting values are 0
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
		    if dist/10 < 30:
			self.collision_warning = True

	   
if __name__ == '__main__':

    rospy.init_node('pure_pursuit')
    pub = rospy.Publisher('controls', car_control, queue_size=5)

    msg_to_publish = car_control()

    rate = rospy.Rate(1)  # Adjust rate?
    path = pure_pursuit.TargetCourse()
    wrap = Wrapper()
    state = pure_pursuit.State()
    states = pure_pursuit.States()
    open('distances.txt', 'w').close()
    dist_file = open("distances.txt", "a")

    while not rospy.is_shutdown():
        rospy.Subscriber('GPS_pos', Float64MultiArray, wrap.GPS_callback)		# x_pos, y_pos, heading_angle
        rospy.Subscriber('path_planner', Float64MultiArray, wrap.path_callback)		# a, b, c, Boolean_done, offset, parking_length 
        rospy.Subscriber('ultrasonic', Float64MultiArray, wrap.ultrasonic_callback)	# back_sensor, side_sensor_back, side_sensor_front
	wrap.emergency_break()
        rate.sleep()
