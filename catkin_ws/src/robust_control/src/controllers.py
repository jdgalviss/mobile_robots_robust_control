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
        self.last_time = None


class SlidingModeController(KinematicController):
    def __init__(self):
        self.k0 = 100.0
        self.k1 = 0.3
        self.k2 = 10.0
        self.P = np.array([0.1, 0.05])
        self.Q = np.array([0.1, 7.0])
        self.s = np.zeros((2,), dtype=float)
        self.d_s = np.zeros((2,), dtype=float)
    
    def calculate_error():
        error_q = self.pos_ref - self.pos_real
        yaw_ref = self.pos_ref[2]
        T = np.array([[math.cos(yaw_ref), math.sin(yaw_ref), 0], [-math.sin(yaw_ref), math.cos(yaw_ref), 0], [0, 0, 1]])
        self.error = np.matmul(T,np.transpose(error_q))

    def calculate_d_error():
        xe_dot = vx_ref - vx_local*math.cos(th_e)+ye*om_ref
        ye_dot = -(vx_local*math.sin(th_e)-xe*om_ref)
        th_e_dot = (om_ref-om)
class StableController(KinematicController):
    def __init__(self):
        self.k_yaw = 0.0
        self.k_x = 0.0
        self.k_y = 0.0 
        