#!/usr/bin/env python
# license removed for brevity
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

    delta = 0.35*delta
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

    while o2g.current_speed > 0.2:
        command.brake = 1
        command.shift_gears = Control.NEUTRAL
        pub.publish(command)        
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
    
    #cuadrado
    path = [[0,0],[1,0],[2,0],[3,0],[4,0],[5,0],[6,0],[7,0],[8,0],[9,0],[10,0],[10.7845909573000,0.0308266627000000],[11.5643446504000,0.123116594000000],[12.3344536386000,0.276300796000000],[13.0901699437000,0.489434837000000],[13.8268343237000,0.761204674900000],[14.5399049974000,1.08993475810000],[15.2249856472000,1.47359835650000],[15.8778525229000,1.90983005630000],[16.4944804833000,2.39594034400000],[17.0710678119000,2.92893218810000],[17.6040596560000,3.50551951670000],[18.0901699437000,4.12214747710000],[18.5264016435000,4.77501435280000],[18.9100652419000,5.46009500260000],[19.2387953251000,6.17316567630000],[19.5105651630000,6.90983005630000],[19.7236992040000,7.66554636140000],[19.8768834060000,8.43565534960000],[19.9691733373000,9.21540904270000],[20,10],[20,11],[20,12],[20,13],[20,14],[20,15],[20,16],[20,17],[20,18],[20,19],[20,20],[19.9691733373000,20.7845909573000],[19.8768834060000,21.5643446504000],[19.7236992040000,22.3344536386000],[19.5105651630000,23.0901699437000],[19.2387953251000,23.8268343237000],[18.9100652419000,24.5399049974000],[18.5264016435000,25.2249856472000],[18.0901699437000,25.8778525229000],[17.6040596560000,26.4944804833000],[17.0710678119000,27.0710678119000],[16.4944804833000,27.6040596560000],[15.8778525229000,28.0901699437000],[15.2249856472000,28.5264016435000],[14.5399049974000,28.9100652419000],[13.8268343237000,29.2387953251000],[13.0901699437000,29.5105651630000],[12.3344536386000,29.7236992040000],[11.5643446504000,29.8768834060000],[10.7845909573000,29.9691733373000],[10,30],[9,30],[8,30],[7,30],[6,30],[5,30],[4,30],[3,30],[2,30],[1,30],[0,30],[-0.784590957300000,29.9691733373000],[-1.56434465040000,29.8768834060000],[-2.33445363860000,29.7236992040000],[-3.09016994370000,29.5105651630000],[-3.82683432370000,29.2387953251000],[-4.53990499740000,28.9100652419000],[-5.22498564720000,28.5264016435000],[-5.87785252290000,28.0901699437000],[-6.49448048330000,27.6040596560000],[-7.07106781190000,27.0710678119000],[-7.60405965600000,26.4944804833000],[-8.09016994370000,25.8778525229000],[-8.52640164350000,25.2249856472000],[-8.91006524190000,24.5399049974000],[-9.23879532510000,23.8268343237000],[-9.51056516300000,23.0901699437000],[-9.72369920400000,22.3344536386000],[-9.87688340600000,21.5643446504000],[-9.96917333730000,20.7845909573000],[-10,20],[-10,19],[-10,18],[-10,17],[-10,16],[-10,15],[-10,14],[-10,13],[-10,12],[-10,11],[-10,10],[-9.96917333730000,9.21540904270000],[-9.87688340600000,8.43565534960000],[-9.72369920400000,7.66554636140000],[-9.51056516300000,6.90983005630000],[-9.23879532510000,6.17316567630000],[-8.91006524190000,5.46009500260000],[-8.52640164350000,4.77501435280000],[-8.09016994370000,4.12214747710000],[-7.60405965600000,3.50551951670000],[-7.07106781190000,2.92893218810000],[-6.49448048330000,2.39594034400000],[-5.87785252290000,1.90983005630000],[-5.22498564720000,1.47359835650000],[-4.53990499740000,1.08993475810000],[-3.82683432370000,0.761204674900000],[-3.09016994370000,0.489434837000000],[-2.33445363860000,0.276300796000000],[-1.56434465040000,0.123116594000000],[-0.784590957300000,0.0308266627000000]]
    '''
    #trajectory
    path = [[0,0],[0.854261813100000,0.00428233000000000],[1.18151272870000,-0.0254311810000000],[1.61409517840000,-0.00335998650000000],[1.98991529160000,0.0524205144000000],[2.43274456610000,0.0863447037000000],[2.88797599200000,0.0966397168000000],[3.34859409780000,0.129961160600000],[3.74986560340000,0.150310395800000],[4.17310518230000,0.163200097200000],[4.56699475370000,0.166054860000000],[5.04427415820000,0.265597462900000],[5.53808628600000,0.261320861700000],[6.08598392720000,0.295771775800000],[6.60974552660000,0.338100392600000],[7.14542796440000,0.365391400200000],[7.67505000970000,0.334614065600000],[8.32371422070000,0.364221994700000],[8.93526747430000,0.378767782200000],[9.55360678050000,0.382448014900000],[10.1832079318000,0.399554261900000],[10.8419353774000,0.375267207800000],[11.5092196276000,0.414765432200000],[12.1993525039000,0.416787026500000],[12.8712187964000,0.405023192800000],[13.5502492723000,0.387545277200000],[14.2220738691000,0.390712020700000],[14.9267760867000,0.429061870100000],[15.6661407779000,0.405299028200000],[16.3778032770000,0.432095301700000],[17.1491594888000,0.427719699400000],[17.8821894248000,0.410103327900000],[18.6377589438000,0.409119017900000],[19.3675903807000,0.404160485300000],[20.1194874824000,0.391914879100000],[20.9341513690000,0.421324228500000],[21.7031549449000,0.365039801000000],[22.5212306226000,0.406047630400000],[23.1809814499000,0.383896157500000],[23.9707936821000,0.429163802100000],[24.7655779508000,0.409267937800000],[25.4593100735000,0.438303785700000],[26.2141120671000,0.436390313900000],[26.9707606299000,0.481183596400000],[27.7475524166000,0.468912896800000],[28.5218258537000,0.496159840800000],[29.3433437548000,0.539579462900000],[30.0659374931000,0.560095545800000],[30.7408064828000,0.611705536000000],[31.4906139921000,0.618050231200000],[32.2153414177000,0.625513297700000],[32.9492189177000,0.679105394500000],[33.7987031439000,0.704824621800000],[34.5822469062000,0.741230263600000],[35.3536702202000,0.765502780800000],[36.0316068346000,0.772112007200000],[36.8300680661000,0.803279099900000],[37.6710418533000,0.833239134000000],[38.3956724332000,0.831490556600000],[39.1889366838000,0.810095752500000],[39.8850641967000,0.782096685900000],[40.7318929811000,0.787802511800000],[41.4866871668000,0.799893800500000],[42.2390719508000,0.785780610700000],[42.9293366167000,0.763090173800000],[43.6988944830000,0.797808242000000],[44.4483220258000,0.847236846800000],[45.1712468276000,0.844606040700000],[45.9737480465000,0.888465994400000],[46.7842000311000,0.941445516600000],[47.5463240815000,0.975563835900000],[48.2979578397000,0.977651827200000],[49.0454547677000,1.02314092040000],[49.7658167593000,1.04725696060000],[50.5310158542000,1.13703476660000],[51.3127106317000,1.18590348200000],[52.0650425436000,1.32411136970000],[52.7359962843000,1.28390513300000],[53.3993251610000,1.28339090420000],[54.0965387423000,1.42939638340000],[54.7651632541000,1.46966890740000],[55.5000488367000,1.49170087570000],[56.3593279664000,1.50301842640000],[56.9300004767000,1.50567624510000],[57.6041126015000,1.43059445640000],[58.3792450571000,1.27579965870000],[58.9059643816000,1.14703332900000],[59.4964161822000,0.997842254100000],[60.0662619675000,0.735376193800000],[60.5321736597000,0.549589913400000],[60.9062219030000,0.403470326000000],[61.2001787134000,0.201659025500000],[61.5470970619000,-0.00318511620000000],[61.7880705853000,-0.161196768100000],[62.1127565071000,-0.329067418200000],[62.3624544897000,-0.513764253100000],[62.5827993193000,-0.643823435400000],[62.7363617088000,-0.839582485800000],[62.9377030956000,-0.976222474000000],[63.2438127200000,-1.24831607570000],[63.4762408806000,-1.47624764220000],[63.6704383955000,-1.72513547950000],[63.9501557746000,-2.02265628480000],[64.1648457950000,-2.22924325290000],[64.3608602667000,-2.46173383360000],[64.4944557940000,-2.69415455160000],[64.6515248784000,-2.96449454580000],[64.8402690699000,-3.19825233760000],[65.0025581714000,-3.52650951440000],[65.0589071306000,-3.85305199480000],[65.1786575137000,-4.18521995670000],[65.3110363587000,-4.57748584630000],[65.3525619537000,-5.01810807840000],[65.4306998882000,-5.42523654270000],[65.5560669419000,-5.85819899850000],[65.6704315184000,-6.27571427610000],[65.7077816723000,-6.72614756880000],[65.6703393042000,-7.15827075720000],[65.6385923424000,-7.68314178220000],[65.6469552039000,-8.21817577720000],[65.7097538442000,-8.77433015780000],[65.7835252473000,-9.25343517630000],[65.7460572973000,-9.75899455890000],[65.7480656109000,-10.3103185278000],[65.7827803571000,-10.8906556370000],[65.7931377294000,-11.3889122348000],[65.7839368496000,-11.8937098621000],[65.8451171116000,-12.4190969437000],[65.8796497890000,-12.9064242144000],[65.9142478570000,-13.4163571064000],[65.8747557092000,-13.9966155332000],[65.8927550998000,-14.5171189732000],[65.9550944956000,-14.9959116606000],[65.9328005354000,-15.5077694711000],[65.9108983202000,-16.0755034529000],[65.9605697022000,-16.6149043195000],[65.9891558942000,-17.1866283122000],[65.9781125987000,-17.7100087773000],[65.9809396150000,-18.2362005362000],[65.9895109559000,-18.8082248648000],[66.0232801981000,-19.3574310084000],[66.0274823618000,-19.8745543143000],[66.0504161183000,-20.4225574541000],[66.0642585039000,-20.9431659436000],[66.0800384942000,-21.4652576891000],[66.0956347148000,-21.9409723079000],[66.1112484678000,-22.3263973316000],[66.1176797083000,-22.6430631742000],[66.1318305919000,-22.9820795351000],[66.1662878297000,-23.3303064000000],[66.1986413521000,-23.6669515987000],[66.2097870189000,-23.9134225408000],[66.2120014653000,-24.2075549595000],[66.2711739709000,-24.5705044978000],[66.3070778431000,-24.9450082393000],[66.4171384497000,-25.2860680711000],[66.5074471174000,-25.7358099513000],[66.6343842887000,-26.1888952298000],[66.7995764726000,-26.6475910605000],[66.9481844174000,-27.1021227044000],[67.1244055148000,-27.6255386942000],[67.3241522041000,-28.1743687873000],[67.5146359895000,-28.6269079705000],[67.7049124152000,-29.0898096619000],[67.9815423425000,-29.5769712217000],[68.2802127695000,-30.0318784831000],[68.5851231398000,-30.4812730557000],[68.8065969723000,-30.9266652723000],[69.0097915995000,-31.3250978893000],[69.2113331777000,-31.5931460375000],[69.4393183760000,-31.8708892182000],[69.5555651459000,-32.1032832890000],[69.7124716994000,-32.3028475629000],[69.8909562396000,-32.5440208686000],[70.0810449346000,-32.8607849450000],[70.3774269632000,-33.2085429406000],[70.6385520953000,-33.6658158950000],[70.8779662495000,-34.1090824504000],[71.2178651697000,-34.4938694862000],[71.4903550645000,-34.9389877836000],[71.7935134063000,-35.3841000892000],[72.1523463994000,-35.7663230637000],[72.5336190069000,-36.2395366535000],[72.9174760816000,-36.6684949023000],[73.3361739731000,-37.0619902544000],[73.8549829824000,-37.5242048819000],[74.3766368702000,-37.9961211128000],[74.9338094235000,-38.3600855291000],[75.5453500845000,-38.7916191465000],[76.0563318904000,-39.2366728497000],[76.6526989220000,-39.7287293547000],[77.2233705535000,-40.1621924169000],[77.8114031680000,-40.6038502533000],[78.4387283397000,-40.9753070895000],[79.0985702181000,-41.3115112938000],[79.7415654191000,-41.6483663803000],[80.4049305557000,-41.9608762581000],[81.0911111815000,-42.2415328850000],[81.7296820523000,-42.3729516470000],[82.3541604821000,-42.4811030317000],[82.9532192989000,-42.5586365003000],[83.5153674594000,-42.6069561887000],[83.9934307601000,-42.6950929788000],[84.4655352812000,-42.7405155714000],[84.9335764632000,-42.7619435392000],[85.3563478292000,-42.7493779871000],[85.7880939428000,-42.6574838172000],[86.1599640782000,-42.6201268522000],[86.5207311868000,-42.6075653645000],[86.8136703982000,-42.5970519927000],[87.0711520372000,-42.5484922981000]]
    '''
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