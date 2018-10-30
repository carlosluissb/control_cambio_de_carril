import pygame, sys
import rospy
from prius_msgs.msg import Control
from sensor_msgs.msg import Joy


import sys 
sys.path.append("../../PathPlanning/CubicSpline/")

import numpy as np
import math
import matplotlib.pyplot as plt
import scipy.linalg as la
import pycubicspline as cubic_spline_planner
import tf
import roslib

from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Joy
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Quaternion, PoseStamped
from ackermann_msgs.msg import AckermannDriveStamped
import signal
import sys




class O2G:
    def __init__(self):
        self.sub = rospy.Subscriber("/base_pose_ground_truth", Odometry, self.callback)#/prius_grid
        #self.steering_sub = rospy.Subscriber("/teb_local_planner/ackermann_command", AckermannDriveStamped, self.steering_callback)
        self.steering_sub = rospy.Subscriber("/ackermann_cmd", AckermannDriveStamped, self.steering_callback)
        self.pub_joy = rospy.Publisher('/joy', Joy, queue_size=1)
        self.last_steering_diff = 0
        self.steering = 0
        self.speed = 0
        self.controller_speed = 0
        self.current_speed = 0
        self.buttons = [0, 0, 0, 0]
        self.last_x = 0
        self.last_y = 0
        self.last_heading = 0



    def callback(self, message):

        control = Joy()
        angles = euler_from_quaternion([message.pose.pose.orientation.x, message.pose.pose.orientation.y, message.pose.pose.orientation.z, message.pose.pose.orientation.w])

        current_speed = np.sqrt(np.square(message.twist.twist.linear.x) + np.square(message.twist.twist.linear.y))
        velocidad = current_speed
        self.current_speed = current_speed
        speed_signal = (np.absolute(self.controller_speed) - current_speed)*3

        if speed_signal > 0:
            speed_signal = 1
        if speed_signal < -0.9:
            speed_signal = -0.9


        if self.controller_speed > 0:
            control.buttons = [1, 0, 0, 0]
            if speed_signal > 0:
                speed_signal = 1
            if speed_signal < -0.9:
                speed_signal = -0.9
            steering = self.steering

        elif self.controller_speed < 0:
            control.buttons = [0, 0, 1, 0]
            if speed_signal > 0:
                speed_signal = 1
            elif speed_signal < -0.9:
                speed_signal = -0.9
            else:
                speed_signal = -1*speed_signal
            steering =  -1*self.steering

        elif self.controller_speed == 0:
            speed_signal = -1
            control.buttons = [0, 0, 0, 1]
            steering = 0

        self.speed = speed_signal
        self.buttons = control.buttons
        control.axes = [steering, 0, self.speed, 0, 0]
        control.buttons = self.buttons
        self.pub_joy.publish(control)

        return velocidad 
        '''
        print(self.controller_speed)
        print(self.speed)
        print(self.current_speed)
        print(self.steering)
        print('-------')
        '''
      

    def steering_callback(self, message):

        self.steering = message.drive.steering_angle
        control = Joy()

#       self.steering = self.steering*180/3.1415/30
        self.steering = self.steering
        self.controller_speed = 7#message.drive.speed

        if self.controller_speed > 0:
            control.buttons = [1, 0, 0, 0]
        elif self.controller_speed < 0:
            control.buttons = [0, 0, 1, 0]
        elif self.controller_speed == 0:
            self.speed = -1
            control.buttons = [0, 0, 0, 1]
        self.buttons = control.buttons
        control.axes = [self.steering, 0, self.speed, 0, 0]
        self.pub_joy.publish(control)
        print(self.controller_speed)
        print(self.speed)
        print(self.current_speed)
        print(self.steering)
        print('-------')

    def signal_handler(self, signal, frame):
        control = Joy()
        control.axes = [0, 0, 0, 0, 0]
        control.buttons = [0, 0, 0, 1]
        self.pub_joy.publish(control)
        print('Pressed Ctrl+C')
        sys.exit(0)






o2g = O2G()

rospy.init_node('pius_arrow_controller')

pub = rospy.Publisher('prius', Control, queue_size=1)
command = Control()
listener = tf.TransformListener()


# LQR parameter
Q = np.eye(5)
R = np.eye(2)

# parameters
dt = 0.01  # time tick[s]
L = 2.7  # Wheel base of the vehicle [m]
max_steer = math.radians(30.0)  # maximum steering angle[rad]

show_animation = True

#inicialitation ROS

rospy.init_node('pius_arrow_controller')

pub = rospy.Publisher('prius', Control, queue_size=1)
command = Control()


class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v


def update(state, a, delta, angle_yaw, trans, vel):

    delta = 0.3*delta
    if delta >= max_steer:
        delta = max_steer
    if delta <= - max_steer:
        delta = - max_steer
    
    command.steer = delta/max_steer            
    
    pub.publish(command)
    
    state.x = trans[0]
    state.y = trans[1]
    state.yaw = angle_yaw
    
    msg = Odometry()
    
    state.v = o2g.current_speed
    print(state.v)
    #print(math.sqrt(vel[0]*vel[0]+vel[1]*vel[1]))

    return state


def pi_2_pi(angle):
    return (angle + math.pi) % (2*math.pi) - math.pi


def solve_DARE(A, B, Q, R):
    """
    solve a discrete time_Algebraic Riccati equation (DARE)
    """
    X = Q
    maxiter = 150
    eps = 0.01

    for i in range(maxiter):
        Xn = A.T * X * A - A.T * X * B * \
            la.inv(R + B.T * X * B) * B.T * X * A + Q
        if (abs(Xn - X)).max() < eps:
            X = Xn
            break
        X = Xn

    return Xn


def dlqr(A, B, Q, R):
    """Solve the discrete time lqr controller.
    x[k+1] = A x[k] + B u[k]
    cost = sum x[k].T*Q*x[k] + u[k].T*R*u[k]
    # ref Bertsekas, p.151
    """

    # first, try to solve the ricatti equation
    X = solve_DARE(A, B, Q, R)

    # compute the LQR gain
    K = np.matrix(la.inv(B.T * X * B + R) * (B.T * X * A))

    eigVals, eigVecs = la.eig(A - B * K)

    return K, X, eigVals


def lqr_steering_control(state, cx, cy, cyaw, ck, pe, pth_e, sp):
    ind, e = calc_nearest_index(state, cx, cy, cyaw)

    tv = 6

    k = ck[ind]
    v = state.v
    th_e = pi_2_pi(state.yaw - cyaw[ind])

    A = np.matrix(np.zeros((5, 5)))
    A[0, 0] = 1.0
    A[0, 1] = dt
    A[1, 2] = v
    A[2, 2] = 1.0
    A[2, 3] = dt
    A[4, 4] = 1.0
    # print(A)

    B = np.matrix(np.zeros((5, 2)))
    B[3, 0] = v / L
    B[4, 1] = dt

    K, _, _ = dlqr(A, B, Q, R)

    x = np.matrix(np.zeros((5, 1)))

    x[0, 0] = e
    x[1, 0] = (e - pe) / dt
    x[2, 0] = th_e
    x[3, 0] = (th_e - pth_e) / dt
    x[4, 0] = v - tv

    ustar = -K * x

    # calc steering input

    ff = math.atan2(L * k, 1)
    fb = pi_2_pi(ustar[0, 0])

    # calc accel input
    ai = ustar[1, 0]

    delta = ff + fb

    return delta, ind, e, th_e, ai



def calc_nearest_index(state, cx, cy, cyaw):
    dx = [state.x - icx for icx in cx]
    dy = [state.y - icy for icy in cy]

    d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

    mind = min(d)

    ind = d.index(mind)

    mind = math.sqrt(mind)

    dxl = cx[ind] - state.x
    dyl = cy[ind] - state.y

    angle = pi_2_pi(cyaw[ind] - math.atan2(dyl, dxl))
    if angle < 0:
        mind *= -1

    return ind, mind


def closed_loop_prediction(cx, cy, cyaw, ck, speed_profile, goal):
    T = 500.0  # max simulation time
    goal_dis = 0.8
    stop_speed = 2
    trans = [0, 0] 
    while trans[0] == 0:
        if not rospy.is_shutdown():
                try:
                    (trans, rot) = listener.lookupTransform("/map", "/chassis2", rospy.Time(0))
                    print("closed_loop_prediction")
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue

    angle_yaw =tf.transformations.euler_from_quaternion(rot) 
    
    state = State(x=trans[0], y=trans[1], yaw=angle_yaw[2], v=0.0)

    time = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]
    target_ind = calc_nearest_index(state, cx, cy, cyaw)
    delt = [0]

    e, e_th = 0.0, 0.0
    
    ai_max = 0 
    ai_min = 0
    i = 0
    while T >= time and not rospy.is_shutdown():
        dl, target_ind, e, e_th, ai = lqr_steering_control(
            state, cx, cy, cyaw, ck, e, e_th, speed_profile)
 
        if abs(state.v) <= stop_speed:
            target_ind += 1

        #print(dl)
        if not rospy.is_shutdown():
            if ai >= 0:
                command.throttle = ai/2.6
                command.shift_gears = Control.FORWARD
                pub.publish(command)              
            else:
                command.brake = (-1)*ai/10
                command.shift_gears = Control.NEUTRAL
                pub.publish(command)
            try:
                (trans, rot) = listener.lookupTransform("/map", "/chassis2", rospy.Time(0))
                (vel, ang) = listener.lookupTwist("/map", "/chassis2", rospy.Time(0), rospy.Duration(1))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        
        if ai > ai_max:
            ai_max = ai
        if ai < ai_min:
            ai_min = ai     

        angle_yaw = tf.transformations.euler_from_quaternion(rot)


        #print(ace_norm_prius)
        state = update(state, ai, dl, angle_yaw[2], trans, vel)
        
        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        delt.append(command.steer)
        t.append(time)

        time = time + dt

        #print(ai_max," ", ai_min," ", 3.6*math.sqrt(vel[0]*vel[0]+vel[1]*vel[1]+vel[2]*vel[2]))
        
        if target_ind % 1 == 0 and show_animation:
            plt.cla()
            plt.plot(cx, cy, "-r", label="course")
            plt.plot(x, y, "ob", label="trajectory")
            plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
            plt.axis("equal")
            plt.grid(True)
            plt.title("speed[km/h]:" + str(round(state.v * 3.6, 2)) +
                      ",target index:" + str(target_ind))
            plt.pause(0.0001)
    

        # check goal
        dx = state.x - goal[0]
        dy = state.y - goal[1]
        if math.sqrt(dx ** 2 + dy ** 2) <= goal_dis:
            print("Goal")
            break

    #print(ai_max," ", ai_min," ", state.v)       
    return t, x, y, yaw, v, delt


def calc_speed_profile(cx, cy, cyaw, target_speed):
    speed_profile = [target_speed] * len(cx)

    direction = 1.0

    # Set stop point
    for i in range(len(cx) - 1):
        dyaw = abs(cyaw[i + 1] - cyaw[i])
        switch = math.pi / 4.0 <= dyaw < math.pi / 2.0

        if switch:
            direction *= -1

        if direction != 1.0:
            speed_profile[i] = - target_speed
        else:
            speed_profile[i] = target_speed

        if switch:
            speed_profile[i] = 0.0

    # speed down
    for i in range(40):
        speed_profile[-i] = target_speed / (50 - i)
        if speed_profile[-i] <= 1.0 / 3.6:
            speed_profile[-i] = 1.0 / 3.6

    return speed_profile


def main():
    print("LQR steering control tracking start!!")
    ax = [0.0, 10.0, 20.0, 30.0, 60.0, 70.0, 80.0, 90.0, 100.0]
    ay = [0.0, 0.0,  0.0,  0.0, 10.0, 10.0, 10.0, 10.0, 10.0]
    
    ax = []
    ay = []
    path = [[0,0],[1,0],[2,0],[3,0],[4,0],[5,0],[6,0],[7,0],[8,0],[9,0],[10,0],[10.7845909573000,0.0308266627000000],[11.5643446504000,0.123116594000000],[12.3344536386000,0.276300796000000],[13.0901699437000,0.489434837000000],[13.8268343237000,0.761204674900000],[14.5399049974000,1.08993475810000],[15.2249856472000,1.47359835650000],[15.8778525229000,1.90983005630000],[16.4944804833000,2.39594034400000],[17.0710678119000,2.92893218810000],[17.6040596560000,3.50551951670000],[18.0901699437000,4.12214747710000],[18.5264016435000,4.77501435280000],[18.9100652419000,5.46009500260000],[19.2387953251000,6.17316567630000],[19.5105651630000,6.90983005630000],[19.7236992040000,7.66554636140000],[19.8768834060000,8.43565534960000],[19.9691733373000,9.21540904270000],[20,10],[20,11],[20,12],[20,13],[20,14],[20,15],[20,16],[20,17],[20,18],[20,19],[20,20],[19.9691733373000,20.7845909573000],[19.8768834060000,21.5643446504000],[19.7236992040000,22.3344536386000],[19.5105651630000,23.0901699437000],[19.2387953251000,23.8268343237000],[18.9100652419000,24.5399049974000],[18.5264016435000,25.2249856472000],[18.0901699437000,25.8778525229000],[17.6040596560000,26.4944804833000],[17.0710678119000,27.0710678119000],[16.4944804833000,27.6040596560000],[15.8778525229000,28.0901699437000],[15.2249856472000,28.5264016435000],[14.5399049974000,28.9100652419000],[13.8268343237000,29.2387953251000],[13.0901699437000,29.5105651630000],[12.3344536386000,29.7236992040000],[11.5643446504000,29.8768834060000],[10.7845909573000,29.9691733373000],[10,30],[9,30],[8,30],[7,30],[6,30],[5,30],[4,30],[3,30],[2,30],[1,30],[0,30],[-0.784590957300000,29.9691733373000],[-1.56434465040000,29.8768834060000],[-2.33445363860000,29.7236992040000],[-3.09016994370000,29.5105651630000],[-3.82683432370000,29.2387953251000],[-4.53990499740000,28.9100652419000],[-5.22498564720000,28.5264016435000],[-5.87785252290000,28.0901699437000],[-6.49448048330000,27.6040596560000],[-7.07106781190000,27.0710678119000],[-7.60405965600000,26.4944804833000],[-8.09016994370000,25.8778525229000],[-8.52640164350000,25.2249856472000],[-8.91006524190000,24.5399049974000],[-9.23879532510000,23.8268343237000],[-9.51056516300000,23.0901699437000],[-9.72369920400000,22.3344536386000],[-9.87688340600000,21.5643446504000],[-9.96917333730000,20.7845909573000],[-10,20],[-10,19],[-10,18],[-10,17],[-10,16],[-10,15],[-10,14],[-10,13],[-10,12],[-10,11],[-10,10],[-9.96917333730000,9.21540904270000],[-9.87688340600000,8.43565534960000],[-9.72369920400000,7.66554636140000],[-9.51056516300000,6.90983005630000],[-9.23879532510000,6.17316567630000],[-8.91006524190000,5.46009500260000],[-8.52640164350000,4.77501435280000],[-8.09016994370000,4.12214747710000],[-7.60405965600000,3.50551951670000],[-7.07106781190000,2.92893218810000],[-6.49448048330000,2.39594034400000],[-5.87785252290000,1.90983005630000],[-5.22498564720000,1.47359835650000]]
    

    for i in range(len(path)):

        ax.append(path[i][0])
        ay.append(path[i][1])
    
    
    goal = [ax[-1], ay[-1]]

    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=0.1)
    target_speed = 10.0 / 3.6  # simulation parameter km/h -> m/s
    
    sp = calc_speed_profile(cx, cy, cyaw, target_speed)


    t, x, y, yaw, v, delt = closed_loop_prediction(cx, cy, cyaw, ck, sp, goal)
    

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

        flg, ax = plt.subplots(1)
        plt.plot(t, delt, "-r", label="delta")
        plt.grid(True)
        plt.legend()
        plt.xlabel("time")
        plt.ylabel("delta [angle_norm]")

        plt.show()


if __name__ == '__main__':
    main()