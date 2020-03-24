from matplotlib import pyplot as plt
import numpy as np


class Coordinate:
    def __init__(self, x, y):
        """first Argument is the x coordinate and second is the y coordinate"""
        self.x = x
        self.y = y


def makeMap():  ## for now it will return a fixed map
    """This method creates a dictionary by combining the GPS-coordinate and
     the ultrasonic sensors"""
    parkmap = {
        0.0: 1,
        0.1: 1,
        0.2: 1,
        0.3: 1,
        0.4: 1,
        0.5: 1,
        0.6: 1,
        0.7: 1,
        0.8: 1,
        0.9: 1,
        1.0: 1,
        1.1: 1,
        1.2: 1,
        1.3: 1,
        1.4: 1,
        1.5: 1,
        1.52: 1.2,
        1.55: 2.4,
        1.57: 3.0,  ## Higher density needed here!
        1.6: 3.5,
        1.7: 3.5,
        1.8: 3.5,
        1.9: 3.5,
        2.0: 3.5,
        2.1: 3.5,
        2.2: 3.5,
        2.3: 3.5,
        2.5: 3.5,
        2.6: 3.5,
        2.7: 3.5,
        2.8: 3.5,
        2.9: 3.5,
        3.0: 3.5,
        3.1: 3.5,
        3.2: 3.5,
        3.3: 3.5,
        3.4: 3.5,
        3.5: 3.5,
        3.6: 3.5,
        3.7: 3.5,
        3.8: 3.5,
        3.9: 3.5,
        4.0: 3.5,
        4.1: 3.5,
        4.2: 3.5,
        4.3: 3.5,
        4.4: 3.5,
        4.5: 3.5,
        4.6: 3.5,
        4.7: 3.5,
        4.8: 3.5,
        4.9: 3.5,
        5.0: 3.5,
        5.1: 3.5,
        5.2: 3.5,
        5.3: 3.5,
        5.4: 3.5,
        5.5: 3.5,
        5.6: 3.5,
        5.7: 3.5,
        5.8: 3.5,
        5.9: 3.5,
        6.0: 3.5,
        6.1: 3.5,
        6.2: 3.5,
        6.3: 3.5,
        6.4: 3.5,
        6.5: 3.5,
        6.6: 3.5,
        6.7: 3.5,
        6.8: 3.5,
        6.9: 3.5,
        7.0: 3.5,
        7.1: 1.0,
        7.2: 1.0,
        7.3: 1.0,
        7.4: 1.0,
        7.5: 1.0,
        7.6: 1.0,
        7.7: 1.0,
        7.8: 1.0,
        7.9: 1.0,
        8.0: 1.0,
        8.1: 1.0,
        8.2: 1.0,
        8.3: 1.0,

    }
    return parkmap


def filter_collision(x_0, y_0, deriv):
    circleRadius = 0.69  ### OBS! Check this value!!!
    parkingmap = makeMap()
    carlength = 2.32  ### OBS! Check this value!!!
    halfCar = carlength / 2
    angle = np.arctan(deriv)
    counter = 0
    if (deriv > 0):
        p1 = Coordinate(x_0 - halfCar * np.cos(angle), y_0 - halfCar * np.sin(angle))
        p2 = Coordinate(x_0 + halfCar * np.cos(angle), y_0 + halfCar * np.sin(angle))
    else:
        p1 = Coordinate(x_0 - halfCar * np.cos(angle), y_0 + halfCar * np.sin(angle))
        p2 = Coordinate(x_0 + halfCar * np.cos(angle), y_0 - halfCar * np.sin(angle))

    tang_linspace = np.linspace(p1.x, p2.x, 20)
    # tangent = deriv * (tang_linspace - x_0) + y_0
    # plt.plot(tang_linspace, tangent)            #check collision instead?
    for x in tang_linspace:
        counter = counter + 1
        if counter > 6 or counter < 16:
            for key in parkingmap:
                tangent = deriv * (x - x_0) + y_0
                dist = distance(x, tangent, key, parkingmap.get(key))
                if dist < circleRadius:
                    return True
    return False


def distance(x1, y1, x2, y2):
    return np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def f_arctan(a, b, c, x):  ## Byt till denna funktionen på metoderna under
    return a * np.arctan(c / b + 3) + a * np.arctan((1 / b) * (x - 3 * b - c))


def f_arctan_d1(a, b, c, x):  # derivative of f_arctan
    # return a / (b * (1 + (np.power(c - x / b, 2))))
    return a / (b * (np.power(x - 3 * b - c, 2) / np.power(b, 2) + 1))


def f_arctan_d2(a, b, c, x):  # second derivative of f_arctan
    # return (2 * a * (c - x / b)) / (np.power(b, 2) * (np.power(np.power(c - x / b, 2) + 1, 2)))
    return (2 * a * (-3 * b - c + x)) / (np.power(b, 3) * np.power(np.power(-3 * b - c + x, 2) / np.power(b, 2) + 1, 2))


def path(current, goal):
    """first argument takes in the current coordinate of the car and the second is the
     coordinate of the goal position this method will return the optimal
     trajectory"""
    Depth = np.abs(goal.y - current.y)  # 2
    Length = np.abs(goal.x - current.x)  # 10
    Period = 2
    phase = 10
    Deptharray = np.linspace(0, Depth, 30)
    Periodarray = np.linspace(0, Period, 20)
    Lengtharray = np.linspace(current.x, current.x + Length, 20)
    Phasearray = np.linspace(0, phase, 20)
    radius = []
    candidates = []
    for a in Deptharray:
        for b in Periodarray:
            for c in Phasearray:
                if a != 0 and b != 0:
                    radius.clear()
                    collision1 = False
                    counter = 0
                    function = f_arctan(a, b, c, Lengtharray)
                    if abs((f_arctan(a, b, c,
                                     goal.x) - goal.y)) > 0.3:  # filter out every function of which distance to
                        break  # to the goal position at goal.x is to big

                    if f_arctan_d1(a, b, c, goal.x) > 0.1:  # filter out every function that ends with a slope
                        break  # larger than 0.1 rad (to ensure that the car is parked horizontally)

                    for x in Lengtharray:  # filter out every function with a radius of curvature smaller
                        if x != 0:  # than that of renault twizy
                            f_deriv = f_arctan_d1(a, b, c, x)  # first derivative
                            f_deriv2 = f_arctan_d2(a, b, c, x)  # second derivative

                            if f_deriv2 != 0:
                                radius.insert(counter, np.absolute(np.power(1 + np.power(f_deriv, 2),
                                                                            3 / 2) / f_deriv2))  # formula for radius of curvature
                                counter = counter + 1

                    if min(
                            radius) < 2.3:  # filter out every function with a radius of curvature smaller            ### OBS! Check this value
                        break  # than that of renault twizy

                    for x in Lengtharray:
                        if x != 0:
                            f_deriv = f_arctan_d1(a, b, c, x)  # first derivative
                            collision = filter_collision(x, f_arctan(a, b, c, x), f_deriv)

                            if collision == True:
                                collision1 = True

                    if collision1 == True:
                        break

                    plt.plot(Lengtharray, function)

                    # TODO: Returnera a,b,c på bästa funktionen!


current = Coordinate(0, 0)
goal = Coordinate(6, 2.25)

a = 2.5
b = 1.4
c = 1
CarLength = 3

# plt.plot(t, f(t), '--b')
plt.plot(list(makeMap().keys()), list(makeMap().values()))
# plt.show()

path(current, goal)
plt.show()
