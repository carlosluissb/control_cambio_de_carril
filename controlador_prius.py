import sys
sys.path.append("../../PathPlanning/CubicSpline/")

import numpy as np
import math
import matplotlib.pyplot as plt
import scipy.linalg as la
import pycubicspline as cubic_spline_planner
import lqr_speed_steer_control as lq

import pygame, sys
import rospy
from prius_msgs.msg import Control
from sensor_msgs.msg import Joy


# LQR parameter
Q = np.eye(5)
R = np.eye(2)

# parameters
dt = 0.1  # time tick[s]
L = 0.5  # Wheel base of the vehicle [m]
max_steer = math.radians(45.0)  # maximum steering angle[rad]

show_animation = True

def main():
    print("LQR steering control tracking start!!")
    ax = [1.0, 2.0, 3.0, 6.0, 7.0, 8.0]
    ay = [1.0, 1.0, 1.0, 5.0, 5.0, 5.0]
    goal = [ax[-1], ay[-1]]

    cx, cy, cyaw, ck, s = lq.cubic_spline_planner.calc_spline_course(
        ax, ay, ds=0.1)
    target_speed = 10.0 / 3.6  # simulation parameter km/h -> m/s
    
    sp = lq.calc_speed_profile(cx, cy, cyaw, target_speed)

    t, x, y, yaw, v, ace_norm = lq.closed_loop_prediction(cx, cy, cyaw, ck, sp, goal)

    print(ace_norm)

    if show_animation:
        plt.close()
        flg, _ = plt.subplots(1)
        plt.plot(ax, ay, "xb", label="waypoints")
        plt.plot(cx, cy, "-r", label="target course")
        plt.plot(x, y, "-g", label="tracking")
        plt.grid(True)
        plt.axis("equal")
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.legend()

        flg, ax = plt.subplots(1)
        plt.plot(s, [math.degrees(iyaw) for iyaw in cyaw], "-r", label="yaw")
        plt.grid(True)
        plt.legend()
        plt.xlabel("line length[m]")
        plt.ylabel("yaw angle[deg]")

        flg, ax = plt.subplots(1)
        plt.plot(s, ck, "-r", label="curvature")
        plt.grid(True)
        plt.legend()
        plt.xlabel("line length[m]")
        plt.ylabel("curvature [1/m]")

        plt.show()


if __name__ == '__main__':
    main()