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

k = 0.5  # control gain
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
'''
    ax = []
    ay = []
    path = [[0,0],[0.854261813100000,0.00428233000000000],[1.18151272870000,-0.0254311810000000],[1.61409517840000,-0.00335998650000000],[1.98991529160000,0.0524205144000000],[2.43274456610000,0.0863447037000000],[2.88797599200000,0.0966397168000000],[3.34859409780000,0.129961160600000],[3.74986560340000,0.150310395800000],[4.17310518230000,0.163200097200000],[4.56699475370000,0.166054860000000],[5.04427415820000,0.265597462900000],[5.53808628600000,0.261320861700000],[6.08598392720000,0.295771775800000],[6.60974552660000,0.338100392600000],[7.14542796440000,0.365391400200000],[7.67505000970000,0.334614065600000],[8.32371422070000,0.364221994700000],[8.93526747430000,0.378767782200000],[9.55360678050000,0.382448014900000],[10.1832079318000,0.399554261900000],[10.8419353774000,0.375267207800000],[11.5092196276000,0.414765432200000],[12.1993525039000,0.416787026500000],[12.8712187964000,0.405023192800000],[13.5502492723000,0.387545277200000],[14.2220738691000,0.390712020700000],[14.9267760867000,0.429061870100000],[15.6661407779000,0.405299028200000],[16.3778032770000,0.432095301700000],[17.1491594888000,0.427719699400000],[17.8821894248000,0.410103327900000],[18.6377589438000,0.409119017900000],[19.3675903807000,0.404160485300000],[20.1194874824000,0.391914879100000],[20.9341513690000,0.421324228500000],[21.7031549449000,0.365039801000000],[22.5212306226000,0.406047630400000],[23.1809814499000,0.383896157500000],[23.9707936821000,0.429163802100000],[24.7655779508000,0.409267937800000],[25.4593100735000,0.438303785700000],[26.2141120671000,0.436390313900000],[26.9707606299000,0.481183596400000],[27.7475524166000,0.468912896800000],[28.5218258537000,0.496159840800000],[29.3433437548000,0.539579462900000],[30.0659374931000,0.560095545800000],[30.7408064828000,0.611705536000000],[31.4906139921000,0.618050231200000],[32.2153414177000,0.625513297700000],[32.9492189177000,0.679105394500000],[33.7987031439000,0.704824621800000],[34.5822469062000,0.741230263600000],[35.3536702202000,0.765502780800000],[36.0316068346000,0.772112007200000],[36.8300680661000,0.803279099900000],[37.6710418533000,0.833239134000000],[38.3956724332000,0.831490556600000],[39.1889366838000,0.810095752500000],[39.8850641967000,0.782096685900000],[40.7318929811000,0.787802511800000],[41.4866871668000,0.799893800500000],[42.2390719508000,0.785780610700000],[42.9293366167000,0.763090173800000],[43.6988944830000,0.797808242000000],[44.4483220258000,0.847236846800000],[45.1712468276000,0.844606040700000],[45.9737480465000,0.888465994400000],[46.7842000311000,0.941445516600000],[47.5463240815000,0.975563835900000],[48.2979578397000,0.977651827200000],[49.0454547677000,1.02314092040000],[49.7658167593000,1.04725696060000],[50.5310158542000,1.13703476660000],[51.3127106317000,1.18590348200000],[52.0650425436000,1.32411136970000],[52.7359962843000,1.28390513300000],[53.3993251610000,1.28339090420000],[54.0965387423000,1.42939638340000],[54.7651632541000,1.46966890740000],[55.5000488367000,1.49170087570000],[56.3593279664000,1.50301842640000],[56.9300004767000,1.50567624510000],[57.6041126015000,1.43059445640000],[58.3792450571000,1.27579965870000],[58.9059643816000,1.14703332900000],[59.4964161822000,0.997842254100000],[60.0662619675000,0.735376193800000],[60.5321736597000,0.549589913400000],[60.9062219030000,0.403470326000000],[61.2001787134000,0.201659025500000],[61.5470970619000,-0.00318511620000000],[61.7880705853000,-0.161196768100000],[62.1127565071000,-0.329067418200000],[62.3624544897000,-0.513764253100000],[62.5827993193000,-0.643823435400000],[62.7363617088000,-0.839582485800000],[62.9377030956000,-0.976222474000000],[63.2438127200000,-1.24831607570000],[63.4762408806000,-1.47624764220000],[63.6704383955000,-1.72513547950000],[63.9501557746000,-2.02265628480000],[64.1648457950000,-2.22924325290000],[64.3608602667000,-2.46173383360000],[64.4944557940000,-2.69415455160000],[64.6515248784000,-2.96449454580000],[64.8402690699000,-3.19825233760000],[65.0025581714000,-3.52650951440000],[65.0589071306000,-3.85305199480000],[65.1786575137000,-4.18521995670000],[65.3110363587000,-4.57748584630000],[65.3525619537000,-5.01810807840000],[65.4306998882000,-5.42523654270000],[65.5560669419000,-5.85819899850000],[65.6704315184000,-6.27571427610000],[65.7077816723000,-6.72614756880000],[65.6703393042000,-7.15827075720000],[65.6385923424000,-7.68314178220000],[65.6469552039000,-8.21817577720000],[65.7097538442000,-8.77433015780000],[65.7835252473000,-9.25343517630000],[65.7460572973000,-9.75899455890000],[65.7480656109000,-10.3103185278000],[65.7827803571000,-10.8906556370000],[65.7931377294000,-11.3889122348000],[65.7839368496000,-11.8937098621000],[65.8451171116000,-12.4190969437000],[65.8796497890000,-12.9064242144000],[65.9142478570000,-13.4163571064000],[65.8747557092000,-13.9966155332000],[65.8927550998000,-14.5171189732000],[65.9550944956000,-14.9959116606000],[65.9328005354000,-15.5077694711000],[65.9108983202000,-16.0755034529000],[65.9605697022000,-16.6149043195000],[65.9891558942000,-17.1866283122000],[65.9781125987000,-17.7100087773000],[65.9809396150000,-18.2362005362000],[65.9895109559000,-18.8082248648000],[66.0232801981000,-19.3574310084000],[66.0274823618000,-19.8745543143000],[66.0504161183000,-20.4225574541000],[66.0642585039000,-20.9431659436000],[66.0800384942000,-21.4652576891000],[66.0956347148000,-21.9409723079000],[66.1112484678000,-22.3263973316000],[66.1176797083000,-22.6430631742000],[66.1318305919000,-22.9820795351000],[66.1662878297000,-23.3303064000000],[66.1986413521000,-23.6669515987000],[66.2097870189000,-23.9134225408000],[66.2120014653000,-24.2075549595000],[66.2711739709000,-24.5705044978000],[66.3070778431000,-24.9450082393000],[66.4171384497000,-25.2860680711000],[66.5074471174000,-25.7358099513000],[66.6343842887000,-26.1888952298000],[66.7995764726000,-26.6475910605000],[66.9481844174000,-27.1021227044000],[67.1244055148000,-27.6255386942000],[67.3241522041000,-28.1743687873000],[67.5146359895000,-28.6269079705000],[67.7049124152000,-29.0898096619000],[67.9815423425000,-29.5769712217000],[68.2802127695000,-30.0318784831000],[68.5851231398000,-30.4812730557000],[68.8065969723000,-30.9266652723000],[69.0097915995000,-31.3250978893000],[69.2113331777000,-31.5931460375000],[69.4393183760000,-31.8708892182000],[69.5555651459000,-32.1032832890000],[69.7124716994000,-32.3028475629000],[69.8909562396000,-32.5440208686000],[70.0810449346000,-32.8607849450000],[70.3774269632000,-33.2085429406000],[70.6385520953000,-33.6658158950000],[70.8779662495000,-34.1090824504000],[71.2178651697000,-34.4938694862000],[71.4903550645000,-34.9389877836000],[71.7935134063000,-35.3841000892000],[72.1523463994000,-35.7663230637000],[72.5336190069000,-36.2395366535000],[72.9174760816000,-36.6684949023000],[73.3361739731000,-37.0619902544000],[73.8549829824000,-37.5242048819000],[74.3766368702000,-37.9961211128000],[74.9338094235000,-38.3600855291000],[75.5453500845000,-38.7916191465000],[76.0563318904000,-39.2366728497000],[76.6526989220000,-39.7287293547000],[77.2233705535000,-40.1621924169000],[77.8114031680000,-40.6038502533000],[78.4387283397000,-40.9753070895000],[79.0985702181000,-41.3115112938000],[79.7415654191000,-41.6483663803000],[80.4049305557000,-41.9608762581000],[81.0911111815000,-42.2415328850000],[81.7296820523000,-42.3729516470000],[82.3541604821000,-42.4811030317000],[82.9532192989000,-42.5586365003000],[83.5153674594000,-42.6069561887000],[83.9934307601000,-42.6950929788000],[84.4655352812000,-42.7405155714000],[84.9335764632000,-42.7619435392000],[85.3563478292000,-42.7493779871000],[85.7880939428000,-42.6574838172000],[86.1599640782000,-42.6201268522000],[86.5207311868000,-42.6075653645000],[86.8136703982000,-42.5970519927000],[87.0711520372000,-42.5484922981000]]
    
    for i in range(len(path)):
        ax.append(path[i][0])
        ay.append(path[i][1])
'''
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

    while max_simulation_time >= time and last_idx > target_idx:
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