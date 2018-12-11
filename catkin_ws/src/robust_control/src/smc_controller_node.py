#!/usr/bin/env python

import datetime
import time
import rospy
import math
import functools
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from agv_prueba.msg import refAGV
from controllers import SlidingModeController


def callback_ref(data_ref, smc):
    global x_ref
    global y_ref
    global th_ref
    global om_ref
    global alpha_ref
    global vx_ref
    global ax_ref


    x_ref = data_ref.x
    y_ref = data_ref.y
    th_ref = data_ref.th
    om_ref = data_ref.w
    alpha_ref = data_ref.al
    vx_ref = data_ref.vx
    ax_ref = data_ref.ax
    #rospy.loginfo(data_ref)


def callback_pose(data_pose, smc):
    global x
    global y
    global th
    global om
    global alpha
    global vx_local
    global ax

    global x_prev
    global y_prev
    global th_prev
    global vx_prev
    global vy_prev
    global om_prev
    global current_time
    global end_time

    global vx_acc
    global vy_acc
    global om_acc
    global ax_acc
    global alpha_acc

    global vx2
    global vy2
    global om2
    global ax2
    global alpha2

    global count_filter
    global count_filter2
    #get the data from the topic
    x=data_pose.pose.position.x
    y=data_pose.pose.position.y
    w=data_pose.pose.orientation.w
    z=data_pose.pose.orientation.z
    #th = math.atan2((2*w*z),(1-2*(math.pow(z,2))))
    aux = (1-2*(math.pow(z,2)))
    th = math.atan2((2*(w*z)),((1-2*(math.pow(z,2))))) 
    if aux == 0:
	th = 1.5708
    if(th-th_prev > 1):
	th = th - 2*math.pi

    if(th-th_prev < -1):
	th = th + 2*math.pi
 
    #th = data_pose.pose.orientation.x
    #rospy.loginfo(th)
    #delta time
    current_time = rospy.get_time()
    dt = current_time - end_time
    #rospy.loginfo(dt)
    #dt = 1/50.0

    #differentiate position and get velocities in global frame
    if(count_filter < 20):
        vx_acc = vx_acc + (x-x_prev)/dt
        vy_acc = vy_acc + (y-y_prev)/dt
        om_acc = om_acc + (th-th_prev)/dt
        count_filter = count_filter + 1
    else:
        vx2 = vx_acc/20.0
        vy2 = vy_acc/20.0
        om2 = om_acc/20.0

        vx_acc = 0.0
        vy_acc = 0.0
        om_acc = 0.0
        count_filter = 0
    
    #transform velocity to local frame
    vx_local = vx2*math.cos(th)+vy2*math.sin(th)
    vy_local = -vx2*math.sin(th)+vy2*math.cos(th)
    om = om2
    #rospy.loginfo("om:" + str(om) + "vx_local: " + str(vx_local))

    #Get accelerations in local frame
    if(count_filter2 < 20):
        ax_acc = ax_acc + (vx_local-vx_prev)/dt
        alpha_acc = alpha_acc + (om-om_prev)/dt
        count_filter2 = count_filter2 + 1
    else:
        ax2 = ax_acc/20.0
        alpha2 = alpha_acc/20.0
        ax_acc = 0.0
        alpha_acc = 0.0
        count_filter2 = 0

    ax = ax2
    alpha = alpha2

    #Save prev values
    x_prev=x
    y_prev=y
    th_prev=th
    vx_prev=vx_local
    vy_prev=vy_local
    om_prev=om
    end_time = rospy.get_time()
    #rospy.loginfo(data_pose)

def sign(num):
    if num==0:
        return 0
    else:
        if num>0:
            return 1
        else:
            return-1

def smc_controller():
    global x_ref
    global y_ref
    global th_ref
    global om_ref
    global alpha_ref
    global vx_ref
    global ax_ref

    global x
    global y
    global th
    global om
    global alpha
    global vx_local
    global ax

    global x_prev
    global y_prev
    global th_prev
    global vx_prev
    global vy_prev
    global om_prev
    global current_time2
    global end_time2
    global vx_int
    global om_int


    #sign = functools.partial(math.copysign, 1) # either of these

    var = 1
    smc = SlidingModeController()
    rospy.init_node('/sliding_mode_controller', anonymous=True)
    pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=50)
    rospy.Subscriber('refAGV', refAGV, callback_ref, smc)
    rospy.Subscriber('slam_out_pose', PoseStamped, callback_pose, smc)

    rate = rospy.Rate(50) # 50hz

    #log to file to analyse data
    file2 = open('hector_smc.txt', 'w')
    while not rospy.is_shutdown():
        #Kinematic controller parameters
        k1 = 3.0/10.0
        P = 1.0/10.0
        Q = 1.0/10.0
        P2 = 0.05
        Q2 = 7.0
        k0=10.0*10.0
        k2=5.0*2.0
        #ky1 = 1.0
        #kt1 = 1.0
        

        #Calculate error
        xe = math.cos(th_ref)*(x_ref-x)+math.sin(th_ref)*(y_ref-y)
        ye = -(-math.sin(th_ref)*(x_ref-x)+math.cos(th_ref)*(y_ref-y))
        th_e = (th_ref-th)
        th_e2 = th_e
        
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
