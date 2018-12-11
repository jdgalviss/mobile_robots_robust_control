import numpy as np

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
        self.T_local_glob = np.zeros((3,3), dtype=float)
        self.u_control = np.zeros((2,), dtype=float)

class SlidingModeController(KinematicController):
    def __init__(self):
        self.k0 = 0.0
        self.k1 = 0.0
        self.k2 = 0.0
        self.P = np.array([0.0, 0.0])
        self.Q = np.array([0.0, 0.0])
        self.s = np.zeros((2,), dtype=float)
        self.d_s = np.zeros((2,), dtype=float)


class StableController(KinematicController):
    def __init__(self):
        self.k_yaw = 0.0
        self.k_x = 0.0
        self.k_y = 0.0 
        