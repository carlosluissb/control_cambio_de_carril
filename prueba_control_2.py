"""
Path tracking simulation with Stanley steering control and PID speed control.
author: Atsushi Sakai (@Atsushi_twi)
"""
from __future__ import division, print_function

import sys

sys.path.append("../../PathPlanning/CubicSpline/")

import matplotlib.pyplot as plt
import numpy as np
import pycubicspline as cubic_spline_planner

import rospy
from prius_msgs.msg import Control
from sensor_msgs.msg import Joy
import tf
import roslib
import math


rospy.init_node('pius_arrow_controller')

pub = rospy.Publisher('prius', Control, queue_size=1)
command = Control()
listener = tf.TransformListener()

k = 0.3  # control gain
Kp = 1.0  # speed propotional gain
dt = 0.001  # [s] time difference
L = 2.9  # [m] Wheel base of vehicle
max_steer = np.radians(30.0)  # [rad] max steering angle

show_animation = True


class State(object):
    """
    Class representing the state of a vehicle.
    :param x: (float) x-coordinate
    :param y: (float) y-coordinate
    :param yaw: (float) yaw angle
    :param v: (float) speed
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        """Instantiate the object."""
        super(State, self).__init__()
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

    def update(self, velocidad, angle_yaw, position):
        """
        Update the state of the vehicle.
        Stanley Control uses bicycle model.
        :param acceleration: (float) Acceleration
        :param delta: (float) Steering
        """

        self.x = position[0]
        self.y = position[1]
        self.yaw = angle_yaw
        self.yaw = normalize_angle(self.yaw)
        self.v = velocidad


def pid_control(target, current):
    """
    Proportional control for the speed.
    :param target: (float)
    :param current: (float)
    :return: (float)
    """
    return Kp * (target - current)


def stanley_control(state, cx, cy, cyaw, last_target_idx):
    """
    Stanley steering control.
    :param state: (State object)
    :param cx: ([float])
    :param cy: ([float])
    :param cyaw: ([float])
    :param last_target_idx: (int)
    :return: (float, int)
    """
    current_target_idx, error_front_axle = calc_target_index(state, cx, cy)

    if last_target_idx >= current_target_idx:
        current_target_idx = last_target_idx

    # theta_e corrects the heading error
    theta_e = normalize_angle(cyaw[current_target_idx] - state.yaw)
    # theta_d corrects the cross track error
    theta_d = np.arctan2(k * error_front_axle, state.v)
    # Steering control
    delta = theta_e + theta_d

    return delta, current_target_idx


def normalize_angle(angle):
    """
    Normalize an angle to [-pi, pi].
    :param angle: (float)
    :return: (float) Angle in radian in [-pi, pi]
    """
    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle


def calc_target_index(state, cx, cy):
    """
    Compute index in the trajectory list of the target.
    :param state: (State object)
    :param cx: [float]
    :param cy: [float]
    :return: (int, float)
    """
    # Calc front axle position
    fx = state.x + L * np.cos(state.yaw)
    fy = state.y + L * np.sin(state.yaw)

    # Search nearest point index
    dx = [fx - icx for icx in cx]
    dy = [fy - icy for icy in cy]
    d = [np.sqrt(idx ** 2 + idy ** 2) for (idx, idy) in zip(dx, dy)]
    error_front_axle = min(d)
    target_idx = d.index(error_front_axle)

    target_yaw = normalize_angle(np.arctan2(fy - cy[target_idx], fx - cx[target_idx]) - state.yaw)
    if target_yaw > 0.0:
        error_front_axle = - error_front_axle

    return target_idx, error_front_axle


def main():
    """Plot an example of Stanley steering control on a cubic spline."""
    #  target course
    ax = [0.0, 10.0, 20.0, 30.0, 40.0, 50.0, 60.0]
    ay = [0.0, 0.0,  0.0,  30.0, 30.0, 30.0, 30.0]

    ax = []
    ay = []
    path = [[0,0],[1,0],[2,0],[3,0],[4,0],[5,0],[6,0],[7,0],[8,0],[9,0],[10,0],[10.7845909573000,0.0308266627000000],[11.5643446504000,0.123116594000000],[12.3344536386000,0.276300796000000],[13.0901699437000,0.489434837000000],[13.8268343237000,0.761204674900000],[14.5399049974000,1.08993475810000],[15.2249856472000,1.47359835650000],[15.8778525229000,1.90983005630000],[16.4944804833000,2.39594034400000],[17.0710678119000,2.92893218810000],[17.6040596560000,3.50551951670000],[18.0901699437000,4.12214747710000],[18.5264016435000,4.77501435280000],[18.9100652419000,5.46009500260000],[19.2387953251000,6.17316567630000],[19.5105651630000,6.90983005630000],[19.7236992040000,7.66554636140000],[19.8768834060000,8.43565534960000],[19.9691733373000,9.21540904270000],[20,10],[20,11],[20,12],[20,13],[20,14],[20,15],[20,16],[20,17],[20,18],[20,19],[20,20],[19.9691733373000,20.7845909573000],[19.8768834060000,21.5643446504000],[19.7236992040000,22.3344536386000],[19.5105651630000,23.0901699437000],[19.2387953251000,23.8268343237000],[18.9100652419000,24.5399049974000],[18.5264016435000,25.2249856472000],[18.0901699437000,25.8778525229000],[17.6040596560000,26.4944804833000],[17.0710678119000,27.0710678119000],[16.4944804833000,27.6040596560000],[15.8778525229000,28.0901699437000],[15.2249856472000,28.5264016435000],[14.5399049974000,28.9100652419000],[13.8268343237000,29.2387953251000],[13.0901699437000,29.5105651630000],[12.3344536386000,29.7236992040000],[11.5643446504000,29.8768834060000],[10.7845909573000,29.9691733373000],[10,30],[9,30],[8,30],[7,30],[6,30],[5,30],[4,30],[3,30],[2,30],[1,30],[0,30],[-0.784590957300000,29.9691733373000],[-1.56434465040000,29.8768834060000],[-2.33445363860000,29.7236992040000],[-3.09016994370000,29.5105651630000],[-3.82683432370000,29.2387953251000],[-4.53990499740000,28.9100652419000],[-5.22498564720000,28.5264016435000],[-5.87785252290000,28.0901699437000],[-6.49448048330000,27.6040596560000],[-7.07106781190000,27.0710678119000],[-7.60405965600000,26.4944804833000],[-8.09016994370000,25.8778525229000],[-8.52640164350000,25.2249856472000],[-8.91006524190000,24.5399049974000],[-9.23879532510000,23.8268343237000],[-9.51056516300000,23.0901699437000],[-9.72369920400000,22.3344536386000],[-9.87688340600000,21.5643446504000],[-9.96917333730000,20.7845909573000],[-10,20],[-10,19],[-10,18],[-10,17],[-10,16],[-10,15],[-10,14],[-10,13],[-10,12],[-10,11],[-10,10],[-9.96917333730000,9.21540904270000],[-9.87688340600000,8.43565534960000],[-9.72369920400000,7.66554636140000],[-9.51056516300000,6.90983005630000],[-9.23879532510000,6.17316567630000],[-8.91006524190000,5.46009500260000],[-8.52640164350000,4.77501435280000],[-8.09016994370000,4.12214747710000],[-7.60405965600000,3.50551951670000],[-7.07106781190000,2.92893218810000],[-6.49448048330000,2.39594034400000],[-5.87785252290000,1.90983005630000],[-5.22498564720000,1.47359835650000]]
    
    for i in range(len(path)):
        ax.append(path[i][0])
        ay.append(path[i][1])

    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=0.1)
    
    target_speed = 30.0 / 3.6  # [m/s]
    
    trans = [0, 0]
    while trans[0] == 0:
        if not rospy.is_shutdown():
                try:
                    (trans, rot) = listener.lookupTransform("/map", "/chassis2", rospy.Time(0))
                   
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue

    angle_yaw = tf.transformations.euler_from_quaternion(rot) 

    max_simulation_time = 100.0

    # Initial state
    state = State(x=trans[0], y=trans[1], yaw=angle_yaw[2], v=0)

    last_idx = len(cx) - 1
    time = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]
    target_idx, _ = calc_target_index(state, cx, cy)

    print('State ',state.x,' ', state.y,' ', state.yaw)

    while max_simulation_time >= time and last_idx > target_idx and not rospy.is_shutdown():
        ai = pid_control(target_speed, state.v)
        di, target_idx = stanley_control(state, cx, cy, cyaw, target_idx)
        delta = di
        #print('delta ',delta)
        #print('State ',state.x,' ', state.y,' ', state.yaw)
        #print('Index ',target_idx)

        command.throttle = ai/30
        command.shift_gears = Control.FORWARD
        
        if ai >= 0:
            command.throttle = ai/30
            command.shift_gears = Control.FORWARD
                          
        else:
            command.brake = (-1)*ai/30
            command.shift_gears = Control.NEUTRAL
            

        if delta >= max_steer:
            delta = max_steer
        if delta <= - max_steer:
            delta = - max_steer
        if delta > 0:
            command.steer = delta/max_steer           
        else:
            command.steer = delta/max_steer        

        pub.publish(command)
        
        if not rospy.is_shutdown():
            try:
                (trans, rot) = listener.lookupTransform("/map", "/chassis2", rospy.Time(0))
                (vel, ang) = listener.lookupTwist("/map", "/chassis", rospy.Time(0), rospy.Duration(0.0001))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        
        angle_yaw =tf.transformations.euler_from_quaternion(rot)

        state.update(math.sqrt(vel[0]*vel[0]+vel[1]*vel[1]), angle_yaw[2], trans)

        time += dt

        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        t.append(time)

        if show_animation:
            plt.cla()
            plt.plot(cx, cy, ".r", label="course")
            plt.plot(x, y, "-b", label="trajectory")
            plt.plot(cx[target_idx], cy[target_idx], "xg", label="target")
            plt.axis("equal")
            plt.grid(True)
            plt.title("Speed[km/h]:" + str(state.v * 3.6)[:4])
            plt.pause(0.001)

    # Test
    print(time)
    print(last_idx)
    print(target_idx)
    assert last_idx >= target_idx, "Cannot reach goal"

    if show_animation:
        plt.plot(cx, cy, ".r", label="course")
        plt.plot(x, y, "-b", label="trajectory")
        plt.legend()
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.axis("equal")
        plt.grid(True)

        flg, ax = plt.subplots(1)
        plt.plot(t, [iv * 3.6 for iv in v], "-r")
        plt.xlabel("Time[s]")
        plt.ylabel("Speed[km/h]")
        plt.grid(True)
        plt.show()


if __name__ == '__main__':
    main()