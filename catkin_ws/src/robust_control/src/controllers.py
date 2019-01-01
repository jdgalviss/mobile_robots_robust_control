import numpy as np
import math
import rospy

class KinematicController(object):
    def __init__(self, initial_time):
        self.error = np.zeros((3,), dtype=float)
        self.d_error = np.zeros((3,), dtype=float)
        self.pos_real = np.zeros((3,), dtype=float)
        self.pos_ref = np.zeros((3,), dtype=float)
        self.vel_real = np.zeros((3,), dtype=float)
        self.vel_ref = np.zeros((3,), dtype=float)
        self.accel_real = np.zeros((3,), dtype=float)
        self.accel_ref = np.zeros((3,), dtype=float)
        self.vel_local_real = 0.0     
        self.vel_local_ref = 0.0 
        self.accel_local_real = 0.0     
        self.accel_local_ref = 0.0 
        self.T_local_glob = np.zeros((3,3), dtype=float)
        self.u_control = np.zeros((2,), dtype=float)
        self.last_time_pose = initial_time
        self.last_time_control = initial_time
        self.enable = False
        self.filter_pos = np.empty((0,3), float)
        self.offset = np.zeros((6,), dtype=float)

    def filter_position(self, actual_position):
        if(abs(actual_position[0][2]-self.pos_real[2]) > math.pi): #from pi to -pi
                self.filter_pos = np.copy(actual_position) #reset filter
        else:
            self.filter_pos = np.append(self.filter_pos, actual_position, axis=0)
            if(self.filter_pos.shape[0] > 4):
                self.filter_pos = np.delete(self.filter_pos, (0), axis=0)
        self.pos_real = np.average(self.filter_pos, axis=0)

    def calculate_error(self):
        #yaw = self.pos_real[2]
        #yaw_ref = self.pos_ref[2]
        if(self.pos_ref[2] - self.pos_real[2] > 1.0*3.1416):
            print("yaw_real negative, yaw_ref positive" + str(self.pos_real[2]) + "  " + str(self.pos_ref[2]) )
            self.pos_real[2]= self.pos_real[2] + 2*3.1416
        else:
            if(self.pos_real[2]- self.pos_ref[2] > 1.0 * 3.1416):
                print("yaw_real positive, yaw_ref negative" + str(self.pos_real[2]) + "  " + str(self.pos_ref[2]) )
                self.pos_real[2]= self.pos_real[2] - 2*3.1416

        error_q = self.pos_ref - self.pos_real
        T = np.array([[math.cos(self.pos_real[2]), math.sin(self.pos_real[2]), 0], [-math.sin(self.pos_real[2]), math.cos(self.pos_real[2]), 0], [0, 0, 1]])
        self.error = np.transpose(np.matmul(T,np.transpose(error_q)))

    def calculate_d_error(self):
        xe_dot = self.vel_local_ref - self.vel_local_real*math.cos(self.error[2])+self.error[1]*self.vel_ref[2]
        ye_dot = -(self.vel_local_real*math.sin(self.error[2])-self.error[0]*self.vel_ref[2])
        yaw_e_dot = (self.vel_ref[2]-self.vel_real[2])
        self.d_error = np.array([xe_dot, ye_dot, yaw_e_dot])


class SlidingModeController(KinematicController):
    def __init__(self, initial_time):
        super(SlidingModeController, self).__init__(initial_time)
        self.k0 = 20.0
        self.k1 = 5.0
        self.k2 = 25.0
        self.P = np.array([0.1, 0.1])
        self.Q = np.array([2.0, 2.0])
        self.s = np.zeros((2,), dtype=float)


    def sign(self,num):
        if num == 0:
            return 0
        if num > 0:
            return 1
        else:
            return-1

    def calculate_control(self, current_time):
        s1 = self.d_error[0] + self.k1*self.error[0]
        s2 = self.d_error[1] + self.k2*self.error[1] + self.error[2]*self.sign(self.error[1])*self.k0
        
        if(math.cos(self.error[2]) != 0.0 ):
            d_vc = ( (self.Q[0]*s1 + self.P[0]*self.sign(s1) + self.accel_ref[2]*self.error[1] + \
                    self.vel_ref[2]*self.d_error[1] + self.accel_local_ref + self.k1*self.d_error[0] + \
                    self.vel_local_real*self.d_error[2]*math.sin(self.error[2])) / (math.cos(self.error[2])) )
        else:
            d_vc = ( (self.Q[0]*s1 + self.P[0]*self.sign(s1) + self.accel_ref[2]*self.error[1] + 
                    self.vel_ref[2]*self.d_error[1] + self.accel_local_ref + self.k1*self.d_error[0] + 
                    self.vel_local_real*self.d_error[2]*math.sin(self.error[2])) / (1.0) )
        
        omega_c_num = self.Q[1]*s2 + self.P[1]*self.sign(s2) + self.accel_local_real*math.sin(self.error[2]) - self.d_error[0]*self.vel_ref[2] - self.error[0]*self.accel_ref[2] + self.k2*self.d_error[1]
        omega_c_div = self.vel_local_real*math.cos(self.error[2]) + self.k0*self.sign(self.error[1])


        if omega_c_div != 0:
            omega_c = omega_c_num / omega_c_div + self.vel_ref[2]
        else:
            omega_c = self.vel_ref[2]
        dt = (current_time - self.last_time_control)
        self.last_time_control = current_time
        if dt != 0 and dt < 0.1:
            self.u_control[0] = self.u_control[0] + d_vc*dt
            #self.u_control[0] = d_vc
            self.u_control[1] = omega_c
        #print("dt =" + str(dt) + " in calculate control " + str(current_time) + " and "  + str(self.last_time_control))

class StableController(KinematicController):
    def __init__(self, initial_time):
        super(StableController, self).__init__(initial_time)
        self.k_yaw = 2.0
        self.k_x = 0.5
        self.k_y = 5.0 

    def calculate_control(self, current_time):
        vc = self.vel_local_ref*math.cos(self.error[2]) + self.k_x * self.error[0]
        wc = self.vel_ref[2] + self.k_y*self.vel_local_ref*self.error[1] + self.k_yaw*self.vel_local_ref*math.sin(self.error[2])
        self.u_control = np.array([vc, wc])

class AdaptiveNNController(KinematicController):
    def __init__(self, initial_time):
        super(StableController, self).__init__(initial_time)
        self.k_yaw = 2.0
        self.k_x = 0.5
        self.k_y = 5.0 

    def calculate_control(self, current_time):
        vc = self.vel_local_ref*math.cos(self.error[2]) + self.k_x * self.error[0]
        wc = self.vel_ref[2] + self.k_y*self.vel_local_ref*self.error[1] + self.k_yaw*self.vel_local_ref*math.sin(self.error[2])
        self.u_control = np.array([vc, wc])
        