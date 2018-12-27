#!/usr/bin/env python
import datetime
import time
import rospy
import math
import functools
import tf
import numpy as np
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from soft_trajectory_generation.msg import Trajectory
from controllers import SlidingModeController
from robust_control.msg import Control
import csv


def callback_ref(data, smc):
    yaw = data.yaw
    if(yaw > math.pi):
        yaw = yaw - 2*math.pi
    smc.pos_ref = np.array([data.x, data.y, yaw])
    smc.vel_ref = np.array([data.vx, data.vy, data.yaw_speed])
    smc.accel_ref = np.array([data.ax, data.ay, data.yaw_acceleration])
    smc.vel_local_ref = data.vx_local
    smc.accel_local_ref = data.ax_local
    if(not smc.enable and smc.vel_local_ref != 0.0):
        smc.enable = True
        
    #else:
    #    if (smc.vx_local_ref == 0.0):
    #        smc.enable = True
        


# def callback_pose(data, smc):
#     # Calculate current yaw from quaternion
#     yaw = math.atan2((2*(data.pose.orientation.w*data.pose.orientation.z)),
#                      ((1-2*(math.pow(data.pose.orientation.z, 2)))))
#     if(yaw - smc.pos_real[2] > 1):
#         yaw = yaw - 2*math.pi
#     if(yaw - smc.pos_real[2] < -1):
#         yaw = yaw + 2*math.pi

#     # Calculate speeds
#     current_time = rospy.get_time()
#     dt = (current_time - smc.last_time_pose)
#     if dt != 0:
#         previous_pose = smc.pos_real
#         smc.filter_position(
#             np.array([[data.pose.position.x, data.pose.position.y, yaw]]))
#         actual_speed = (smc.pos_real - previous_pose) / dt

#         # calculate accelerations
#         # smc.accel_real = (actual_speed - smc.vel_real) / dt
#         smc.vel_real = np.copy(actual_speed)

#         # calculate speed in local frame
#         vel_local = smc.vel_real[0]*math.cos(smc.pos_real[2]) + \
#                                              smc.vel_real[1] * \
#                                                  math.sin(smc.pos_real[2])
#         smc.accel_local_real = (vel_local - smc.vel_local_real) / dt
#         smc.accel_local_real = 0.0
#         smc.vel_local_real = vel_local
#     else:
#         rospy.loginfo("dt = 0 in callback pose")
#     smc.last_time_pose = current_time

def calculate_pose(trans, rot, smc):
    # Calculate current yaw from quaternion
    #print(rot)
    #Restart current_pose
    yaw = math.atan2((2*(rot[3]*rot[2])),
                     ((1-2*(math.pow(rot[2], 2)))))
    if(not smc.enable):
        #print("calculating offset")
        smc.offset = np.array([trans[0], trans[1], yaw])
    #if(yaw - smc.pos_real[2] > 1):
    #    yaw = yaw - 2*math.pi
    #if(yaw - smc.pos_real[2] < -1):
    #    yaw = yaw + 2*math.pi

    # Calculate speeds
    current_time = rospy.get_time()
    dt = (current_time - smc.last_time_pose)
    if dt != 0:
        previous_pose = np.copy(smc.pos_real)
        smc.filter_position(
            np.array([[trans[0] - smc.offset[0], trans[1] - smc.offset[1], yaw - smc.offset[2]]]))
        
        actual_pose = np.copy(smc.pos_real)
        if(actual_pose[2] - previous_pose[2] > math.pi):
            actual_pose[2] = actual_pose[2] - 2*math.pi
        else:
            if(actual_pose[2] - previous_pose[2] < -math.pi):
                actual_pose[2] = actual_pose[2] + 2*math.pi 

        actual_speed = (actual_pose - previous_pose) / dt

        # calculate accelerations
        # smc.accel_real = (actual_speed - smc.vel_real) / dt
        smc.vel_real = np.copy(actual_speed)

        # calculate speed in local frame
        vel_local = smc.vel_real[0]*math.cos(smc.pos_real[2]) + \
                                             smc.vel_real[1] * \
                                                 math.sin(smc.pos_real[2])
        smc.accel_local_real = (vel_local - smc.vel_local_real) / dt
        #smc.accel_local_real = 0.0
        smc.vel_local_real = vel_local
    else:
        rospy.loginfo("dt = 0 in callback pose")
    smc.last_time_pose = current_time


def smc_controller():
    rospy.init_node('sliding_mode_controller', anonymous=True)
    listener = tf.TransformListener()
    smc = SlidingModeController(rospy.get_time())
    cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=50)
    log_pub = rospy.Publisher('control_log', Control, queue_size=50)
    rospy.Subscriber('/trajectory/current', Trajectory, callback_ref, smc)
    # rospy.Subscriber('slam_out_pose', PoseStamped, callback_pose, smc)

    rate = rospy.Rate(50)  # 50hz
    # log to file to analyse data
    initial_time = rospy.get_time()
    with open('simulation_smc_data.csv', mode='w') as csv_file:
        fieldnames = ['time', 'x', 'y', 'yaw', 'd_x', 'd_y', 'd_yaw', 'x_ref', 'y_ref', 'yaw_ref', 'd_x_ref', 'd_y_ref', 'd_yaw_ref','xe', 'ye', 'yaw_e', 'vx_local', 'vx_local_ref', 'vc', 'wc' ]
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        while not rospy.is_shutdown():
            # Find current robot position using transform
            try:
                (trans, rot) = listener.lookupTransform('/odom', '/chassis', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            calculate_pose(trans,rot,smc) 
            if(smc.enable):
                
                # calculate tracking error
                smc.calculate_error()
                # Calculate derivate of the error
                smc.calculate_d_error()
                # Kinematic controller formula
                smc.calculate_control(rospy.get_time())
            # publish cmd topic
            cmd_msg = Twist()
            if(smc.u_control[0] > 1.0):
                cmd_msg.linear.x = 1.0
            else:
                if(smc.u_control[0] < 0.0):
                    cmd_msg.linear.x = 0.0
                else:
                    cmd_msg.linear.x = smc.u_control[0]
            
            if(smc.u_control[1] > 3.0):
                cmd_msg.angular.z = 3.0
            else:
                if(smc.u_control[1] < -3.0):
                    cmd_msg.angular.z = -3.0
                else:
                    cmd_msg.angular.z = smc.u_control[1]
            cmd_pub.publish(cmd_msg)
            # publish control Topic for debug
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
            control.d_x =  smc.vel_real[0]
            control.d_y = smc.vel_real[1]
            control.d_yaw = smc.vel_real[2]
            control.x_ref =  smc.pos_ref[0]
            control.y_ref = smc.pos_ref[1]
            control.yaw_ref = smc.pos_ref[2]
            control.d_x_ref =  smc.vel_ref[0]
            control.d_y_ref = smc.vel_ref[1]
            control.d_yaw_ref = smc.vel_ref[2]
            control.vx_local_ref = smc.vel_local_ref
            control.vx_local = smc.vel_local_real
            control.vc = cmd_msg.linear.x 
            control.wc = cmd_msg.angular.z 
            log_pub.publish(control)

            #Save info to csv
            if(smc.enable):
                current_time = (rospy.get_time() - initial_time)
                writer.writerow({'time':current_time, 'x': str(control.x), 'y': str(control.y), 'yaw': str(control.yaw),
                                'd_x': str(control.d_x), 'd_y': str(control.d_y), 'd_yaw': str(control.d_yaw),
                                'x_ref': str(control.x_ref), 'y_ref': str(control.y_ref), 'yaw': str(control.yaw_ref),
                                'd_x_ref': str(control.d_x_ref), 'd_y_ref': str(control.d_y_ref), 'd_yaw_ref': str(control.d_yaw_ref), 
                                'xe': str(control.xe), 'ye': str(control.ye), 'yaw_e': str(control.yaw_e),
                                'vx_local': str(control.vx_local), 'vx_local_ref': str(control.vx_local_ref),
                                'vc': str(control.vc),'wc': str(control.wc)})
            rate.sleep()


if __name__ == '__main__':
    try:
        smc_controller()
    except rospy.ROSInterruptException:
        pass
        global om_intvx_int
        global om_intvx_int
        # global om_int
