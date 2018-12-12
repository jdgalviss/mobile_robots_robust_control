#!/usr/bin/env python
import datetime
import time
import rospy
import math
import functools
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from soft_trajectory_generation.msg import refAGV
from controllers import SlidingModeController


def callback_ref(data, smc):
    smc.pose_ref = np.array([data.x, data.y, data.yaw])
    smc.vel_ref = np.array([data.vx, data.vy, data.yaw_speed])
    smc.accel_ref = np.array([data.ax, data.ay, data.yaw_acceleration])
    smc.vx_local_ref = data.vx_local
    #rospy.loginfo(data_ref)


def callback_pose(data, smc):
    #Calculate current yaw from quaternion
    yaw = math.atan2((2*(data.pose.orientation.w*data.pose.orientation.z)),((1-2*(math.pow(data.pose.orientation.z,2)))))
    if(yaw - smc.pose_real[2] > 1):
        yaw = yaw -2*math.pi
    if(yaw - smc.pose_real[2] < -1):
        yaw = yaw + 2*math.pi

    #Calculate speeds
    current_time = rospy.get_time()
    dt = current_time - smc.last_time
    actual_pose = np.array([data.pose.position.x, data.pose.position.y, yaw])
    actual_speed = (actual_pose - smc.pose_real) / dt
    smc.pose_real = np.copy(actual_pose)

    #calculate accelerations
    smc.accel_ref = (actual_speed - smc.speed_real) / dt
    smc.speed_real = np.copy(actual_speed)

    #calculate speed in local frame
    vel_local = smc.speed_real[0]*math.cos(smc.pose_real[2])+smc.speed_real[1]*math.sin(smc.pose_real[2])
    smc.accel_local_real = (vel_local - smc.vel_local_real) / dt
    smc.vel_local_real = vel_local
    smc.last_time = current_time

def sign(num):
    if num==0:
        return 0
    else:
        if num>0:
            return 1
        else:
            return-1

def smc_controller():

    var = 1
    smc = SlidingModeController()
    rospy.init_node('/sliding_mode_controller', anonymous=True)
    pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=50)
    rospy.Subscriber('/trajectory/current', refAGV, callback_ref, smc)
    rospy.Subscriber('slam_out_pose', PoseStamped, callback_pose, smc)

    rate = rospy.Rate(40) # 50hz
    #log to file to analyse data
    file2 = open('hector_smc.txt', 'w')
    while not rospy.is_shutdown():
        #calculate tracking error
        smc.calculate_error()

        #Calculate derivate of the error
        smc.calculate_d_error()
        

        #Avoid very big vx_dot for square movement (th_e=pi/2 and cos(th_e)= 0)
        #if (abs(th_e) > 1.5 ) & (abs(th_e) < 1.65 ):
	#	th_e2 = 0.0 

        #Avoid problems because of atan2 (pi and -pi) Be careful!! discontinuity
        #if (th_e>5):
	#	th_e = th_e - 3.1416*2
        #if (th_e<-5):
	#	th_e = th_e + 3.1416*2

	#Incorrect differentiating:

	#om = om_ref
	#vx_local = vx_ref
	#ax_ref = 0.0
	#ax_ref = 0.0
	#alpha_ref = 0.0
	#ax = ax_ref
	


        xe_dot = vx_ref - vx_local*math.cos(th_e)+ye*om_ref
        ye_dot = -(vx_local*math.sin(th_e)-xe*om_ref)
        th_e_dot = (om_ref-om)
	#th_e_dot = 0.0


        #Kinematic controller formula
        #Sliding surfaces
        s1 = xe_dot + k1*xe
        ##s2 = ye_dot + ye*(k2*sign(ye)) + th_e*sign(ye)*k0
	s2 = ye_dot + ye*(k2) + th_e*sign(ye)*k0


	

        #Derivative of the cmd
        #vx_dot = (-Q*s1-P*math.copysign(1,s1)-alpha_ref*ye-th_e_dot*ye_dot+ax_ref-k1*xe+vx_local*th_e_dot*math.sin(th_e))/math.cos(th_e)
        #if math.cos(th_e) == 0:
        #    vx_dot = 0
        #else:
            ##vx_dot = (-Q*s1-P*sign(s1)-alpha_ref*ye-om_ref*ye_dot+ax_ref-k1*xe_dot+vx_local*th_e_dot*math.sin(th_e))/1.0
	vx_dot = (ax_ref+ye_dot*om_ref+ye*alpha_ref+vx_local*math.sin(th_e)*th_e_dot+k1*xe_dot+Q*s1+P*sign(s1))/1.0	    
        if (k0*sign(ye)) == 0:
            om_dot = 0
        else:
            ##om_dot = (-Q2*s2-P2*sign(s2)-k2*ye)/(vx_local*math.cos(th_e)+k0*sign(ye))+om_ref
	    om_dot = (-Q2*s2-P2*sign(s2)+xe_dot*om_ref+xe*alpha_ref-ax*math.sin(th_e)-vx_local*math.cos(th_e)*th_e_dot-k2*ye_dot)/(-k0*sign(ye))+om_ref	

        #delta time
        current_time2 = rospy.get_time()
        dt2 = current_time2 - end_time2
        end_time2 = current_time2

	#integration error
	dt2 = 1/50.0

        #Integrate to get cmds
        vx_int = vx_int +vx_dot*dt2
       # om_int = om_int + om_dot*dt
        vy_cmd=0.0

        vx_cmd = vx_int
        om_cmd = om_dot

        #limit commands
	#if vx_cmd > 0.5:
	#	vx_cmd = 0.5

	if vx_cmd < 0.0:
		vx_cmd = 0.0

	#if om_cmd > 1.0:
	#	om_cmd = 1.0

	#if om_cmd < -1.0:
	#	om_cmd = -1.0

        vx_cmd = vx_cmd/2.0
        om_cmd = om_cmd/3.0
	       

        #publish cmd topic
        cmd_msg = Twist()
        cmd_msg.linear.x = vx_cmd
        cmd_msg.linear.y = vx_local
        cmd_msg.angular.z = om_cmd
	#rospy.loginfo("vx: " + str(vx_cmd))
	rospy.loginfo("om: " + str(om_cmd))
 	
        #aux=(-Q2*s2-P2*sign(s2)-k2*ye_dot+alpha_ref*xe+om_ref*xe_dot)
	if var < 20:
		#rospy.loginfo("error")
		#rospy.loginfo(xe)
		#rospy.loginfo(ye)
		#rospy.loginfo(th_e)
		#rospy.loginfo(xe_dot)
		#rospy.loginfo(ye_dot)
		#rospy.loginfo(th_e_dot)
		#rospy.loginfo(s1)
		#rospy.loginfo(s2)
		#rospy.loginfo(vx_dot)
		#rospy.loginfo(vx_cmd)
		#rospy.loginfo(om_cmd)
		#rospy.loginfo(dt)
		#var = var + 1
		if abs(om_cmd) > 0.001:
			var = var +1
	#log to file to analyse data
	file2.write(str(x) + ' ' + str(y) + ' ' + str(th) + ' ' + str(vx_local) + ' ' + str(om) + ' ' + str(alpha) + ' ' + str(ax) + ' ' + str(x_ref) + ' ' + str(y_ref) + ' ' + str(th_ref) + ' ' + str(vx_ref) + ' ' + str(om_ref) + ' ' + str(alpha_ref) + ' ' + str(ax_ref) + ' ' + str(vx_cmd) + ' ' + str(om_cmd) + ' ' + str(current_time2) + '\n') 
	file2.write('')
        pub.publish(cmd_msg)
        rate.sleep()
    file2.close()

if __name__ == '__main__':
    try:
        smc_controller()
    except rospy.ROSInterruptException:
        passvx_int
        global om_intvx_int
        global om_intvx_int
        #global om_int
