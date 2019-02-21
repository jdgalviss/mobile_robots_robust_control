import numpy as np
import math
import rospy
import threading
# from keras.models import Sequential
# from keras.layers import Dense, Activation
# from keras import optimizers
# from keras.preprocessing import sequence
# from keras.utils import np_utils
# from keras.models import load_model
# from keras.models import save_model
# from keras.applications import imagenet_utils


class KinematicController(object):
    def __init__(self, initial_time):
        self.error = np.zeros((3,), dtype=float)
        self.d_error = np.zeros((3,), dtype=float)
        self.pos_real = np.zeros((3,), dtype=float)
        self.T = np.zeros((3, 3), dtype=float)
        self.pos_ref = np.zeros((3,), dtype=float)
        self.vel_real = np.zeros((3,), dtype=float)
        self.vel_ref = np.zeros((3,), dtype=float)
        self.accel_real = np.zeros((3,), dtype=float)
        self.accel_ref = np.zeros((3,), dtype=float)
        self.vel_local_real = 0.0
        self.vel_local_ref = 0.0
        self.accel_local_real = 0.0
        self.accel_local_ref = 0.0
        self.T_local_glob = np.zeros((3, 3), dtype=float)
        self.u_control = np.zeros((2,), dtype=float)
        self.last_time_pose = initial_time
        self.last_time_control = initial_time
        self.enable = False
        self.filter_pos = np.empty((0, 3), float)
        self.offset = np.zeros((6,), dtype=float)

    def filter_position(self, actual_position):
        if(abs(actual_position[0][2]-self.pos_real[2]) > math.pi):  # from pi to -pi
            self.filter_pos = np.copy(actual_position)  # reset filter
        else:
            self.filter_pos = np.append(
                self.filter_pos, actual_position, axis=0)
            if(self.filter_pos.shape[0] > 4):
                self.filter_pos = np.delete(self.filter_pos, (0), axis=0)
        self.pos_real = np.average(self.filter_pos, axis=0)

    def calculate_error(self):
        #yaw = self.pos_real[2]
        #yaw_ref = self.pos_ref[2]
        if(self.pos_ref[2] - self.pos_real[2] > 0.75*3.1416):
            print("yaw_real negative, yaw_ref positive" +
                  str(self.pos_real[2]) + "  " + str(self.pos_ref[2]))
            self.pos_real[2] = self.pos_real[2] + 2*3.1416
        else:
            if(self.pos_real[2] - self.pos_ref[2] > 0.75 * 3.1416):
                print("yaw_real positive, yaw_ref negative" +
                      str(self.pos_real[2]) + "  " + str(self.pos_ref[2]))
                self.pos_real[2] = self.pos_real[2] - 2*3.1416
        if(abs(self.pos_real[2] - self.pos_ref[2])<0.2 * 3.1416):
            error_q = self.pos_ref - self.pos_real
            self.T = np.array([[math.cos(self.pos_real[2]), math.sin(self.pos_real[2]), 0], [
                            -math.sin(self.pos_real[2]), math.cos(self.pos_real[2]), 0], [0, 0, 1]])
            self.error = np.transpose(np.matmul(self.T, np.transpose(error_q)))

    def calculate_d_error(self):
        xe_dot = self.vel_local_ref - self.vel_local_real * \
            math.cos(self.error[2])+self.error[1]*self.vel_ref[2]
        ye_dot = -(self.vel_local_real *
                   math.sin(self.error[2])-self.error[0]*self.vel_ref[2])
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

    def sign(self, num):
        if num == 0:
            return 0
        if num > 0:
            return 1
        else:
            return-1

    def calculate_control(self, current_time):
        s1 = self.d_error[0] + self.k1*self.error[0]
        s2 = self.d_error[1] + self.k2*self.error[1] + \
            self.error[2]*self.sign(self.error[1])*self.k0

        if(math.cos(self.error[2]) != 0.0):
            d_vc = ((self.Q[0]*s1 + self.P[0]*self.sign(s1) + self.accel_ref[2]*self.error[1] +
                     self.vel_ref[2]*self.d_error[1] + self.accel_local_ref + self.k1*self.d_error[0] +
                     self.vel_local_real*self.d_error[2]*math.sin(self.error[2])) / (math.cos(self.error[2])))
        else:
            d_vc = ((self.Q[0]*s1 + self.P[0]*self.sign(s1) + self.accel_ref[2]*self.error[1] +
                     self.vel_ref[2]*self.d_error[1] + self.accel_local_ref + self.k1*self.d_error[0] +
                     self.vel_local_real*self.d_error[2]*math.sin(self.error[2])) / (1.0))

        omega_c_num = self.Q[1]*s2 + self.P[1]*self.sign(s2) + self.accel_local_real*math.sin(
            self.error[2]) - self.d_error[0]*self.vel_ref[2] - self.error[0]*self.accel_ref[2] + self.k2*self.d_error[1]
        omega_c_div = self.vel_local_real * \
            math.cos(self.error[2]) + self.k0*self.sign(self.error[1])

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
        self.k_yaw = 1.0
        self.k_x = 1.0
        self.k_y = 2.0

    def calculate_control(self, current_time):
        vc = self.vel_local_ref * \
            math.cos(self.error[2]) + self.k_x * self.error[0]
        wc = self.vel_ref[2] + self.k_y*self.vel_local_ref*self.error[1] + \
            self.k_yaw*self.vel_local_ref*math.sin(self.error[2])
        self.u_control = np.array([vc, wc])


class AdaptiveNNController(KinematicController):
    def __init__(self, initial_time):
        super(AdaptiveNNController, self).__init__(initial_time)
        self.k_yaw = 1.0
        self.k_x = 1.0
        self.k_y = 2.0
        # Network Parameters
        self.num_outputs = 3
        self.num_hidden = 20
        self.num_inputs = 3
        # Backpropagation parameters
        self.gamma = np.matrix([[1.0, 0, 0], [0, 5.0, 0], [0, 0, 1.8]])
        self.betta = 1.0*np.array([0.3, 7.5, 0.9])

        # Network parameters
        self.v_weight = (np.random.rand(
            self.num_inputs, self.num_hidden) - 0.5)*0.1
        self.w_weight = (np.random.rand(
            self.num_hidden, self.num_outputs) - 0.5)*0.1
        self.bias_hidden = (np.random.rand(self.num_hidden) - 0.5)*0.1
        self.bias_out = (np.random.rand(self.num_outputs) - 0.5)*0.1
        self.accuracy = 0.0

        # Network IO
        self.nn_input = np.zeros((self.num_inputs,), dtype=float)
        self.nn_output = np.zeros((self.num_outputs,), dtype=float)
        self.cost_function = 0.0

        # Training thread
        #self.t = threading.Thread(target=self.train_NN)

    def calculate_control(self, current_time):
        vc = self.vel_local_ref * \
            math.cos(self.error[2]) + self.k_x * self.error[0]
        wc = self.vel_ref[2] + self.k_y*self.vel_local_ref*self.error[1] + \
            self.k_yaw*self.vel_local_ref*math.sin(self.error[2])
        self.u_control = np.array([vc, wc])
        self.nn_input = np.array([vc, wc, self.pos_real[2]])

        # Find the derivate of the outputs of the hidden layer (Z) respect to the Inputs (X)
        count_hidden = 0
        # outputs of the hidden layer
        zp = np.zeros((self.num_hidden,), dtype=float)
        dzl_dxj = np.zeros((self.num_hidden, self.num_inputs), dtype=float)
        # Caslculate for every l-th hidden layer neuron
        for v_lj_weight in np.transpose(self.v_weight):
            accumulated = np.sum(v_lj_weight*self.nn_input) + \
                self.bias_hidden[count_hidden]
            zp[count_hidden] = accumulated
            dzl_dxj[count_hidden] = v_lj_weight
            count_hidden = count_hidden + 1

        # Calculate the derivate of the outputs of the network ( Y ) respect to the hidden layer outputs (Z)
        count_out = 0
        d_dqi_dzk = np.zeros((self.num_outputs, self.num_hidden))
        dqi_dzk = np.zeros((self.num_outputs, self.num_hidden))
        dt = rospy.get_time() - self.last_time_control
        # calculate for every i-th output
        for w_il_weight in np.transpose(self.w_weight):
            accumulated = np.sum(w_il_weight*zp) + self.bias_out[count_out]
            d_sigma = (4*math.exp(-accumulated)) / \
                (math.pow((1+math.exp(-accumulated)), 2))
            d_dqi_dzk[count_out] = w_il_weight * d_sigma
            dqi_dzk[count_out] = dqi_dzk[count_out] + d_dqi_dzk[count_out] * dt
            count_out = count_out + 1
        self.last_time_control = rospy.get_time()

        # Calculate the derivate  of the control variables respect to control parameters.
        duc_da = np.matrix([[self.error[0], 0, 0], [0, self.vel_local_ref *
                                                    self.error[1], self.vel_local_ref*math.sin(self.error[2])], [0, 0, 0]])

        # Calculate cost function
        dF_da = -self.error*self.gamma*self.T*dqi_dzk*dzl_dxj*duc_da
        self.cost_function = self.gamma.item((0,0))*self.error[0]**2 + self.gamma.item((1,1))*self.error[1]**2 + self.gamma.item((2,2))*self.error[2]**2
        #print(self.cost_function)
        # Gradiend-descend algorithm to recalculate controller parameters
        
        if(self.accuracy > 0.85):
            self.k_x = self.k_x - self.betta[0] * dF_da[0, 0]
            self.k_y = self.k_y - self.betta[1] * dF_da[0, 1]
            self.k_yaw = self.k_yaw - self.betta[2] * dF_da[0, 2]
            if(self.k_x < 0.0):
                self.k_x = 0.0
            if(self.k_y < 0.0):
                self.k_y = 0.0
            if(self.k_yaw < 0.0):
                self.k_yaw = 0.0



class NeuralNetwork(object):
    def __init__(self):
        self.num_outputs = 3
        self.num_hidden = 20
        self.num_inputs = 3
        self.batch_size = 1500
        self.nn_input_train = np.empty((0, self.num_inputs), float)
        self.nn_output_train = np.empty((0, self.num_outputs), float)
        self.training = False
        # Model
        # Network parameters
        self.v_weight = (np.random.rand(
            self.num_inputs, self.num_hidden) - 0.5)*0.1  # 3x30
        self.w_weight = (np.random.rand(
            self.num_hidden, self.num_outputs) - 0.5)*0.1  # 30x
        self.bias_hidden = (np.random.rand(self.num_hidden) - 0.5)*0.1
        self.bias_out = (np.random.rand(self.num_outputs) - 0.5)*0.1
        self.model = None
        self.firstPublish = False
        self.accuracy = 0.0
        
