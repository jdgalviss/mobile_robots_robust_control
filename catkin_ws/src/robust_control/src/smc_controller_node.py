#!/usr/bin/env python
import datetime
import time
import rospy
import math
import functools
import numpy as np
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from soft_trajectory_generation.msg import Trajectory
from controllers import SlidingModeController
from robust_control.msg import Control


def callback_ref(data, smc):
    smc.pos_ref = np.array([data.x, data.y, data.yaw])
    smc.vel_ref = np.array([data.vx, data.vy, data.yaw_speed])
    smc.accel_ref = np.array([data.ax, data.ay, data.yaw_acceleration])
    smc.vx_local_ref = data.vx_local
    #rospy.loginfo(data_ref)


def callback_pose(data, smc):
    #Calculate current yaw from quaternion
    yaw = math.atan2((2*(data.pose.orientation.w*data.pose.orientation.z)),((1-2*(math.pow(data.pose.orientation.z,2)))))
    if(yaw - smc.pos_real[2] > 1):
        yaw = yaw -2*math.pi
    if(yaw - smc.pos_real[2] < -1):
        yaw = yaw + 2*math.pi

    #Calculate speeds
    current_time = rospy.get_time()
    dt = current_time - smc.last_time_pose
    actual_pose = np.array([data.pose.position.x, data.pose.position.y, yaw])
    actual_speed = (actual_pose - smc.pos_real) / dt
    smc.pos_real = np.copy(actual_pose)

    #calculate accelerations
    smc.accel_ref = (actual_speed - smc.vel_real) / dt
    smc.vel_real = np.copy(actual_speed)

    #calculate speed in local frame
    vel_local = smc.vel_real[0]*math.cos(smc.pos_real[2])+smc.vel_real[1]*math.sin(smc.pos_real[2])
    smc.accel_local_real = (vel_local - smc.vel_local_real) / dt
    smc.vel_local_real = vel_local
    smc.last_time_pose = current_time


def smc_controller():
    rospy.init_node('sliding_mode_controller', anonymous=True)
    smc = SlidingModeController(rospy.get_time())
    cmd_pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=50)
    log_pub = rospy.Publisher('control_log', Control, queue_size=50)
    rospy.Subscriber('/trajectory/current', Trajectory, callback_ref, smc)
    rospy.Subscriber('slam_out_pose', PoseStamped, callback_pose, smc)

    rate = rospy.Rate(40) # 50hz
    #log to file to analyse data
    file2 = open('hector_smc.txt', 'w')
    while not rospy.is_shutdown():
        #calculate tracking error
        smc.calculate_error()
        #Calculate derivate of the error
        smc.calculate_d_error()
        #Kinematic controller formula
        smc.calculate_control(rospy.get_time())

        #publish cmd topic
        cmd_msg = Twist()
        cmd_msg.linear.x = smc.u_control[0]
        cmd_msg.linear.y = 0.0
        cmd_msg.angular.z = smc.u_control[1]
        #cmd_pub.publish(cmd_msg)
        #publish control Topic for debug
        control = Control()
        control.header.stamp = rospy.Time.now()
        control.xe = smc.error[0]
        control.ye = smc.error[1]
        control.yaw_e = smc.error[2]
        control.d_xe = smc.d_error[0]
        control.d_ye = smc.d_error[1]
        control.d_yaw_e = smc.d_error[2]
        control.x =  smc.pos_real[0]
        control.y = smc.pos_real[1]
        control.yaw = smc.pos_real[2]
        control.vx_local_ref = smc.vel_local_ref
        control.vx_local = smc.vel_local_real
        control.vc = smc.u_control[0]
        control.wc = smc.u_control[1]
        log_pub.publish(control)
"""
	#log to file to analyse data
	file2.write(str(x) + ' ' + str(y) + ' ' + str(th) + ' ' + str(vx_local) + ' ' + str(om) + ' ' + str(alpha) + ' ' + str(ax) + ' ' + str(x_ref) + ' ' + str(y_ref) + ' ' + str(th_ref) + ' ' + str(vx_ref) + ' ' + str(om_ref) + ' ' + str(alpha_ref) + ' ' + str(ax_ref) + ' ' + str(vx_cmd) + ' ' + str(om_cmd) + ' ' + str(current_time2) + '\n') 
	file2.write('')
        pub.publish(cmd_msg)
        rate.sleep()
    file2.close()
"""

if __name__ == '__main__':
    try:
        smc_controller()
    except rospy.ROSInterruptException:
        passvx_int
        global om_intvx_int
        global om_intvx_int
        #global om_int
