import numpy as np
import os
import pickle
import cv2
from keras.models import load_model
import time
import tensorflow as tf

from math import ceil, floor

# ROS
import rospy
from styx_msgs.msg import TrafficLight

class TLClassifier(object):

    def __init__(self):
        """ Initialize model and load weights """

        root_lib = os.path.dirname(os.path.realpath(__file__))
        data_path = root_lib + '/models/'
        
        print("Data path: {}".format(data_path))
        self.model = load_model(data_path + 'whole_image_model_gpu0.h5')
        self.graph = tf.get_default_graph()

        #self.model._make_predict_function()

    def get_classification(self,image):  
        """ Predict using pre-trained model """

        image = image[:,:, ::-1]
        image_expanded = np.expand_dims(image, axis=0)
        pred_state = None
        with self.graph.as_default():
            pred_state = self.model.predict(image_expanded)

        return np.argmax(pred_state)


