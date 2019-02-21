#!/usr/bin/env python
import rospy
import math
import tf
import numpy as np
from controllers import NeuralNetwork
from robust_control.msg import NetworkIO
from robust_control.msg import NetworkWeights
from keras.models import Sequential
from keras.layers import Dense, Activation
from keras import optimizers
from keras.preprocessing import sequence
from keras.utils import np_utils
from keras.models import load_model
from keras.models import save_model
from keras.applications import imagenet_utils


def callback_io(data, neural_network):
    # Capture data for training
    if(not neural_network.training):
        if(neural_network.nn_input_train.shape[0] < neural_network.batch_size):
            neural_network.nn_input_train = np.append(
                neural_network.nn_input_train, np.array([[data.nn_input.x, data.nn_input.y, data.nn_input.z]]), axis=0)
            neural_network.nn_output_train = np.append(
                neural_network.nn_output_train, np.array([[data.nn_output.x, data.nn_output.y, data.nn_output.z]]), axis=0)
        else:
            neural_network.training = True
            


def train_nn():
    rospy.init_node('train_nn', anonymous=True)
    neural_network = NeuralNetwork()
    try:
        neural_network.model = load_model('agv_model.h5')
        neural_network.v_weight = neural_network.model.layers[0].get_weights()[0]  # 3x30
        neural_network.bias_hidden = neural_network.model.layers[0].get_weights()[1]
        neural_network.w_weight = neural_network.model.layers[1].get_weights()[0]  # 30x3
        neural_network.bias_out = neural_network.model.layers[1].get_weights()[1]
            # TO DO: save weights in variable
    except:
        print("no model has been saved yet")

            
    nn_pub = rospy.Publisher('/network_weights', NetworkWeights, queue_size=50)

    rospy.Subscriber('/network_io', NetworkIO,
                     callback_io, neural_network)
    # rospy.Subscriber('slam_out_pose', PoseStamped, callback_pose, train_nn)
    nn_weights = NetworkWeights()
    rate = rospy.Rate(40)  # 50hz
    # log to file to analyse data
    while not rospy.is_shutdown():
        if(neural_network.training):
            # Publish weights
            # Train NN
            if(neural_network.model is None):
                neural_network.model = Sequential()
                neural_network.model.add(Dense(
                    neural_network.num_hidden, activation='sigmoid', input_dim=neural_network.num_inputs, init='normal'))
                neural_network.model.add(
                    Dense(neural_network.num_outputs, activation='linear', init='normal'))
                # ===============Configure training process====================
                # Before training, configure training process (this is called compile) receives: Optimizer,loss (objective) function, metrics
                # For a mean squared error regression problem
                neural_network.model.compile(
                    loss='mean_squared_error', optimizer='rmsprop', metrics=['accuracy'])
            # ====================Do the actual training============
            
            neural_network.model.fit(neural_network.nn_input_train,
                                    neural_network.nn_output_train, epochs=100, verbose=1)
            
            scores = neural_network.model.evaluate(neural_network.nn_input_train, neural_network.nn_output_train, verbose=0)
            if(scores[1]*100.0 > 90):
                neural_network.model.save("agv_model.h5")
            rospy.loginfo("Model Accuracy: %.2f%%" % (scores[1]*100))

            neural_network.nn_input_train = np.empty(
                (0, neural_network.num_inputs), float)
            neural_network.nn_output_train = np.empty(
                (0, neural_network.num_outputs), float)
            neural_network.v_weight = neural_network.model.layers[0].get_weights()[
                0]  # 3x30
            neural_network.bias_hidden = neural_network.model.layers[0].get_weights()[
                1]
            neural_network.w_weight = neural_network.model.layers[1].get_weights()[
                0]  # 30x3
            neural_network.bias_out = neural_network.model.layers[1].get_weights()[
                1]
            neural_network.accuracy = scores[1]*100
            if(neural_network.accuracy > 85):
                neural_network.model.save("agv_model.h5")
            nn_weights.header.stamp = rospy.Time.now()
            nn_weights.weightsV_1 = neural_network.v_weight[0]
            nn_weights.weightsV_2 = neural_network.v_weight[1]
            nn_weights.weightsV_3 = neural_network.v_weight[2]
            nn_weights.weightsW_1 = np.transpose(neural_network.w_weight)[0]
            nn_weights.weightsW_2 = np.transpose(neural_network.w_weight)[1]
            nn_weights.weightsW_3 = np.transpose(neural_network.w_weight)[2]
            nn_weights.bias_hidden = neural_network.bias_hidden
            nn_weights.bias_out = neural_network.bias_out
            nn_weights.accuracy = neural_network.accuracy
            nn_pub.publish(nn_weights)
            neural_network.training = False
            

        if(not neural_network.firstPublish):
            neural_network.firstPublish = True
            nn_weights.header.stamp = rospy.Time.now()
            nn_weights.weightsV_1 = neural_network.v_weight[0]
            nn_weights.weightsV_2 = neural_network.v_weight[1]
            nn_weights.weightsV_3 = neural_network.v_weight[2]
            nn_weights.weightsW_1 = np.transpose(neural_network.w_weight)[0]
            nn_weights.weightsW_2 = np.transpose(neural_network.w_weight)[1]
            nn_weights.weightsW_3 = np.transpose(neural_network.w_weight)[2]
            nn_weights.bias_hidden = neural_network.bias_hidden
            nn_weights.bias_out = neural_network.bias_out
            nn_pub.publish(nn_weights)
        rate.sleep()


if __name__ == '__main__':
    try:
        train_nn()
    except rospy.ROSInterruptException:
        pass
        global om_intvx_int
        global om_intvx_int
        # global om_int
