import numpy as np
import math
import matplotlib.pyplot as plt
#import tkinter
import matplotlib
matplotlib.use('TkAgg')

# Parameters
k = 0.1    # look forward gain
Lfc = 0.30  # [m] look-ahead distance
Kp = 2     # speed proportional gain
dt = 0.1   # [s] time tick
WB = 1.686  # [m] wheel base of vehicle

show_animation = True


class State:

    def __init__(self, x=0, y=0, yaw=0, v=0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))

    def update_from_gps(self, gps_data, v):
        self.x = gps_data[0]
        self.y = gps_data[1]
        self.yaw = gps_data[2] + np.pi
        self.v = v
        self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))


    def update(self, a, delta):
        if delta > 0.7:
            delta = 0.7
        if delta < -0.7:
            delta = -0.7
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.v / WB * math.tan(delta) * dt
        self.v += a * dt
        self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))

    def calc_distance(self, point_x, point_y):
        dx = self.rear_x - point_x
        dy = self.rear_y - point_y
        return math.hypot(dx, dy)


class States:

    def __init__(self):
        self.x = []
        self.y = []
        self.yaw = []
        self.rear_x = []
        self.rear_y = []
        self.v = []
        self.t = []

    def append(self, t, state):
        self.x.append(state.x)
        self.y.append(state.y)
        self.yaw.append(state.yaw)
        self.rear_x.append(state.rear_x)
        self.rear_y.append(state.rear_y)
        self.v.append(state.v)
        self.t.append(t)


def proportional_control(target, current):
    a = Kp * (target - current)

    return a


class TargetCourse:

    def __init__(self, cx=None, cy=None):
        self.path_generated = 0     # False
        self.cx = cx
        self.cy = cy
        self.old_nearest_point_index = None

    def is_path_generated(self):
        if self.path_generated == 1:
            gen = True
        else:
            gen = False

        return gen

    def set_path(self, a, b, c, gps_x, gps_y, yaw):
        rear_x = gps_x - ((WB / 2) * math.cos(yaw))
        rear_y = gps_y - ((WB / 2) * math.sin(yaw))

        self.cx = np.arange(rear_x, rear_x + 20 - 1.25, 0.1)
        self.cy = [rear_y + a * np.arctan((1/b)*((-rear_x) -3*b-c)) + a * np.arctan((1 / b) * ((x-rear_x) - 3 * b - c)) for x in self.cx]
        return self.cx, self.cy


    def search_target_index(self, state):

        # To speed up nearest point search, doing it at only first time.
        if self.old_nearest_point_index is None:
            # search nearest point index
            dx = [state.rear_x - icx for icx in self.cx]
            dy = [state.rear_y - icy for icy in self.cy]
            d = np.hypot(dx, dy)
            ind = np.argmin(d)
            self.old_nearest_point_index = ind
        else:
            ind = self.old_nearest_point_index
            distance_this_index = state.calc_distance(self.cx[ind],
                                                      self.cy[ind])
            while True:
                distance_next_index = state.calc_distance(self.cx[ind + 1],
                                                          self.cy[ind + 1])
                if distance_this_index < distance_next_index:
                    break
                ind = ind + 1 if (ind + 1) < len(self.cx) else ind
                distance_this_index = distance_next_index
            self.old_nearest_point_index = ind

        Lf = k * state.v + Lfc  # update look ahead distance

        # search look ahead target point index
        while Lf > state.calc_distance(self.cx[ind], self.cy[ind]):
            if (ind + 1) >= len(self.cx):
                break  # not exceed goal
            ind += 1

        return ind, Lf


def pure_pursuit_steer_control(state, trajectory, pind):
    ind, Lf = trajectory.search_target_index(state)

    if pind >= ind:
        ind = pind

    if ind < len(trajectory.cx):
        tx = trajectory.cx[ind]
        ty = trajectory.cy[ind]
    else:  # toward goal
        tx = trajectory.cx[-1]
        ty = trajectory.cy[-1]
        ind = len(trajectory.cx) - 1

    alpha = math.atan2(ty - state.rear_y, tx - state.rear_x) - state.yaw

    delta = math.atan2(2.0 * WB * math.sin(alpha) / Lf, 1.0)
    return delta, ind


def plot_arrow(x, y, yaw, length=1.0, width=0.5, fc="black", ec="k"):
    """
    Plot arrow
    """

    if not isinstance(x, float):
        for ix, iy, iyaw in zip(x, y, yaw):
            plot_arrow(ix, iy, iyaw)
    else:
        plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
                  fc=fc, ec=ec, head_width=width, head_length=width)
        plt.plot(x, y)


def main():
    a = 0.8960
    b = 0.6765
    c = 0
    #  target course
    path = TargetCourse()
    cx = np.arange(0, 10, 0.1)
    offset = 1
    parking_length = 5.5
    cx , cy = path.set_path(a, b, c, -6, -5, 3.14)
    target_speed = -5 / 3.6  # [m/s]

    T = 100.0  # max simulation time

    # initial state
    state = State(x=-6, y=-5, yaw=3.14, v=0.0)

    lastIndex = len(cx) - 1
    time = 0.0
    states = States()
    states.append(time, state)
    target_course = TargetCourse(cx, cy)
    target_ind, _ = target_course.search_target_index(state)

    while T >= time and lastIndex > target_ind:

        # Calc control input
        ai = proportional_control(target_speed, state.v)
        di, target_ind = pure_pursuit_steer_control(
            state, target_course, target_ind)

        state.update(ai, di)  # Control vehicle

        time += dt
        states.append(time, state)

        if show_animation:  # pragma: no cover
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            plot_arrow(state.x, state.y, state.yaw)
            plt.plot(cx, cy, "-r", label="course")
            plt.plot(states.rear_x, states.rear_y, "-b", label="trajectory")
            plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
            plt.axis("equal")
            plt.grid(True)
            plt.title("Speed[km/h]:" + str(state.v * 3.6)[:4])
            plt.pause(0.001)

    # Test
    assert lastIndex >= target_ind, "Cannot goal"

    if show_animation:  # pragma: no cover
        plt.cla()
        plt.plot(cx, cy, ".r", label="course")
        plt.plot(states.rear_x, states.rear_y, "-b", label="trajectory")
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



if __name__ == '__main__':
    print("Pure pursuit path tracking simulation start")
    main()
