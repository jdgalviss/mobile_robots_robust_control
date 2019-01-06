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
from controllers import AdaptiveNNController
from robust_control.msg import Control
from robust_control.msg import NetworkIO
from robust_control.msg import NetworkWeights
import csv


def callback_ref(data, adaptive_NN_controller):
    yaw = data.yaw
    if(yaw > math.pi):
        yaw = yaw - 2*math.pi
    adaptive_NN_controller.pos_ref = np.array([data.x, data.y, yaw])
    adaptive_NN_controller.vel_ref = np.array([data.vx, data.vy, data.yaw_speed])
    adaptive_NN_controller.accel_ref = np.array([data.ax, data.ay, data.yaw_acceleration])
    adaptive_NN_controller.vel_local_ref = data.vx_local
    adaptive_NN_controller.accel_local_ref = data.ax_local
    if(not adaptive_NN_controller.enable and adaptive_NN_controller.vel_local_ref != 0.0):
        adaptive_NN_controller.enable = True
        
    #else:
    #    if (adaptive_NN_controller.vx_local_ref == 0.0):
    #        adaptive_NN_controller.enable = True
        


# def callback_pose(data, adaptive_NN_controller):
#     # Calculate current yaw from quaternion
#     yaw = math.atan2((2*(data.pose.orientation.w*data.pose.orientation.z)),
#                      ((1-2*(math.pow(data.pose.orientation.z, 2)))))
#     if(yaw - adaptive_NN_controller.pos_real[2] > 1):
#         yaw = yaw - 2*math.pi
#     if(yaw - adaptive_NN_controller.pos_real[2] < -1):
#         yaw = yaw + 2*math.pi

#     # Calculate speeds
#     current_time = rospy.get_time()
#     dt = (current_time - adaptive_NN_controller.last_time_pose)
#     if dt != 0:
#         previous_pose = adaptive_NN_controller.pos_real
#         adaptive_NN_controller.filter_position(
#             np.array([[data.pose.position.x, data.pose.position.y, yaw]]))
#         actual_speed = (adaptive_NN_controller.pos_real - previous_pose) / dt

#         # calculate accelerations
#         # adaptive_NN_controller.accel_real = (actual_speed - adaptive_NN_controller.vel_real) / dt
#         adaptive_NN_controller.vel_real = np.copy(actual_speed)

#         # calculate speed in local frame
#         vel_local = adaptive_NN_controller.vel_real[0]*math.cos(adaptive_NN_controller.pos_real[2]) + \
#                                              adaptive_NN_controller.vel_real[1] * \
#                                                  math.sin(adaptive_NN_controller.pos_real[2])
#         adaptive_NN_controller.accel_local_real = (vel_local - adaptive_NN_controller.vel_local_real) / dt
#         adaptive_NN_controller.accel_local_real = 0.0
#         adaptive_NN_controller.vel_local_real = vel_local
#     else:
#         rospy.loginfo("dt = 0 in callback pose")
#     adaptive_NN_controller.last_time_pose = current_time

def calculate_pose(trans, rot, adaptive_NN_controller):
    # Calculate current yaw from quaternion
    #print(rot)
    #Restart current_pose
    yaw = math.atan2((2*(rot[3]*rot[2])),
                     ((1-2*(math.pow(rot[2], 2)))))
    if(not adaptive_NN_controller.enable):
        #print("calculating offset")
        adaptive_NN_controller.offset = np.array([trans[0], trans[1], yaw])
    #if(yaw - adaptive_NN_controller.pos_real[2] > 1):
    #    yaw = yaw - 2*math.pi
    #if(yaw - adaptive_NN_controller.pos_real[2] < -1):
    #    yaw = yaw + 2*math.pi

    # Calculate speeds
    current_time = rospy.get_time()
    dt = (current_time - adaptive_NN_controller.last_time_pose)
    if dt != 0:
        previous_pose = np.copy(adaptive_NN_controller.pos_real)
        adaptive_NN_controller.filter_position(
            np.array([[trans[0] - adaptive_NN_controller.offset[0], trans[1] - adaptive_NN_controller.offset[1], yaw - adaptive_NN_controller.offset[2]]]))
        
        actual_pose = np.copy(adaptive_NN_controller.pos_real)
        if(actual_pose[2] - previous_pose[2] > math.pi):
            actual_pose[2] = actual_pose[2] - 2*math.pi
        else:
            if(actual_pose[2] - previous_pose[2] < -math.pi):
                actual_pose[2] = actual_pose[2] + 2*math.pi 

        actual_speed = (actual_pose - previous_pose) / dt

        # calculate accelerations
        # adaptive_NN_controller.accel_real = (actual_speed - adaptive_NN_controller.vel_real) / dt
        adaptive_NN_controller.vel_real = np.copy(actual_speed)

        # calculate speed in local frame
        vel_local = adaptive_NN_controller.vel_real[0]*math.cos(adaptive_NN_controller.pos_real[2]) + \
                                             adaptive_NN_controller.vel_real[1] * \
                                                 math.sin(adaptive_NN_controller.pos_real[2])
        adaptive_NN_controller.accel_local_real = (vel_local - adaptive_NN_controller.vel_local_real) / dt
        #adaptive_NN_controller.accel_local_real = 0.0
        adaptive_NN_controller.vel_local_real = vel_local
    else:
        rospy.loginfo("dt = 0 in callback pose")
    adaptive_NN_controller.last_time_pose = current_time

def callback_weights(data, adaptive_NN_controller):
    adaptive_NN_controller.accuracy = data.accuracy
    if(data.accuracy > 80):
        adaptive_NN_controller.v_weight = np.array([data.weightsV_1,data.weightsV_2,data.weightsV_3])
        adaptive_NN_controller.w_weight = np.transpose(np.array([data.weightsW_1,data.weightsW_2,data.weightsW_3]))
        adaptive_NN_controller.bias_hidden = data.bias_hidden
        adaptive_NN_controller.bias_out = data.bias_out
    else: 
        rospy.loginfo("accuracy too low: " + str(data.accuracy))


def adaptive_NN_controller():
    rospy.init_node('adaptive_NN_controller', anonymous=True)
    listener = tf.TransformListener()
    adaptive_NN_controller = AdaptiveNNController(rospy.get_time())
    cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=50)
    log_pub = rospy.Publisher('/control_log', Control, queue_size=50)
    nn_pub = rospy.Publisher('/network_io', NetworkIO, queue_size=50)
    rospy.Subscriber('/trajectory/current', Trajectory, callback_ref, adaptive_NN_controller)
    rospy.Subscriber('/network_weights', NetworkWeights, callback_weights, adaptive_NN_controller)
    # rospy.Subscriber('slam_out_pose', PoseStamped, callback_pose, adaptive_NN_controller)

    rate = rospy.Rate(50)  # 50hz
    # log to file to analyse data
    initial_time = rospy.get_time()
    with open('simulation_adaptive_NN_controller_data.csv', mode='w') as csv_file:
        fieldnames = ['time', 'x', 'y', 'yaw', 'd_x', 'd_y', 'd_yaw', 'x_ref', 'y_ref', 'yaw_ref', 'd_x_ref', 'd_y_ref', 'd_yaw_ref','xe', 'ye', 'yaw_e', 'vx_local', 'vx_local_ref', 'vc', 'wc' ]
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        while not rospy.is_shutdown():
            # Find current robot position using transform
            try:
                (trans, rot) = listener.lookupTransform('/odom', '/chassis', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            calculate_pose(trans,rot,adaptive_NN_controller) 
            if(adaptive_NN_controller.enable):
                
                # calculate tracking error
                adaptive_NN_controller.calculate_error()
                # Calculate derivate of the error
                adaptive_NN_controller.calculate_d_error()
                # Kinematic controller formula
                adaptive_NN_controller.calculate_control(rospy.get_time())
            # publish cmd topic
            cmd_msg = Twist()
            if(adaptive_NN_controller.u_control[0] > 1.0):
                cmd_msg.linear.x = 1.0
            else:
                if(adaptive_NN_controller.u_control[0] < 0.0):
                    cmd_msg.linear.x = 0.0
                else:
                    cmd_msg.linear.x = adaptive_NN_controller.u_control[0]
            
            if(adaptive_NN_controller.u_control[1] > 3.0):
                cmd_msg.angular.z = 3.0
            else:
                if(adaptive_NN_controller.u_control[1] < -3.0):
                    cmd_msg.angular.z = -3.0
                else:
                    cmd_msg.angular.z = adaptive_NN_controller.u_control[1]
            cmd_pub.publish(cmd_msg)
            # publish control Topic for debug
            control = Control()
            control.header.stamp = rospy.Time.now()
            control.xe = adaptive_NN_controller.error[0]
            control.ye = adaptive_NN_controller.error[1]
            control.yaw_e = adaptive_NN_controller.error[2]
            control.d_xe = adaptive_NN_controller.d_error[0]
            control.d_ye = adaptive_NN_controller.d_error[1]
            control.d_yaw_e = adaptive_NN_controller.d_error[2]
            control.x =  adaptive_NN_controller.pos_real[0]
            control.y = adaptive_NN_controller.pos_real[1]
            control.yaw = adaptive_NN_controller.pos_real[2]
            control.d_x =  adaptive_NN_controller.vel_real[0]
            control.d_y = adaptive_NN_controller.vel_real[1]
            control.d_yaw = adaptive_NN_controller.vel_real[2]
            control.x_ref =  adaptive_NN_controller.pos_ref[0]
            control.y_ref = adaptive_NN_controller.pos_ref[1]
            control.yaw_ref = adaptive_NN_controller.pos_ref[2]
            control.d_x_ref =  adaptive_NN_controller.vel_ref[0]
            control.d_y_ref = adaptive_NN_controller.vel_ref[1]
            control.d_yaw_ref = adaptive_NN_controller.vel_ref[2]
            control.vx_local_ref = adaptive_NN_controller.vel_local_ref
            control.vx_local = adaptive_NN_controller.vel_local_real
            control.vc = cmd_msg.linear.x 
            control.wc = cmd_msg.angular.z 
            log_pub.publish(control)

            #Publish for network training
            nn_io = NetworkIO()
            nn_io.header.stamp = rospy.Time.now()
            nn_io.nn_input.x = adaptive_NN_controller.nn_input[0]
            nn_io.nn_input.y = adaptive_NN_controller.nn_input[1]
            nn_io.nn_input.z = adaptive_NN_controller.nn_input[2]
            nn_io.nn_output.x = adaptive_NN_controller.vel_real[0]
            nn_io.nn_output.y = adaptive_NN_controller.vel_real[1]
            nn_io.nn_output.z = adaptive_NN_controller.vel_real[2]
            nn_pub.publish(nn_io)


            #Save info to csv
            if(adaptive_NN_controller.enable):
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
        adaptive_NN_controller()
    except rospy.ROSInterruptException:
        pass
        global om_intvx_int
        global om_intvx_int
        # global om_int
