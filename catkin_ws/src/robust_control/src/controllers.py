import numpy as np
import math

class KinematicController():
    def __init__(self):
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


class SlidingModeController(KinematicController):
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
        self.k0 = 100.0
        self.k1 = 0.1
        self.k2 = 10.0
        self.P = np.array([0.1, 0.05])
        self.Q = np.array([0.01, 7.0])
        self.s = np.zeros((2,), dtype=float)
        self.d_s = np.zeros((2,), dtype=float)
    def sign(self,num):
        if num == 0:
            return 0
        if num > 0:
            return 1
        else:
            return-1
    
    def calculate_error(self):
        error_q = self.pos_ref - self.pos_real
        yaw_ref = self.pos_ref[2]
        T = np.array([[math.cos(yaw_ref), math.sin(yaw_ref), 0], [-math.sin(yaw_ref), math.cos(yaw_ref), 0], [0, 0, 1]])
        self.error = np.transpose(np.matmul(T,np.transpose(error_q)))

    def calculate_d_error(self):
        xe_dot = self.vel_local_ref - self.vel_local_real*math.cos(self.error[2])+self.error[1]*self.vel_ref[2]
        ye_dot = -(self.vel_local_real*math.sin(self.error[2])-self.error[0]*self.vel_ref[2])
        yaw_e_dot = (self.vel_ref[2]-self.vel_real[2])
        self.d_error = np.array([xe_dot, ye_dot, yaw_e_dot])

    def calculate_control(self, current_time):
        s1 = self.d_error[0] + self.k1*self.error[0]
        s2 = self.d_error[1] + self.k2*self.error[1] + self.error[2]*self.sign(self.error[1])*self.k0
        if(math.cos(self.error[2] != 0.0 )):
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
            omega_c = omega_c_num / omega_c_div
        else:
            omega_c = 0.0
        dt = current_time - self.last_time_control
        self.u_control[0] = self.u_control[0] + d_vc*dt
        self.u_control[1] = omega_c

class StableController(KinematicController):
    def __init__(self):
        self.k_yaw = 0.0
        self.k_x = 0.0
        self.k_y = 0.0 
        