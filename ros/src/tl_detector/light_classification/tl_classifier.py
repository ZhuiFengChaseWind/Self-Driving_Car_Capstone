import numpy as np
import os
import pickle
import cv2
from keras.models import load_model
from keras.models import model_from_yaml
from keras import backend as K
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
        self.model = load_model(data_path + 'whole_image_model_gpu.h5')
        self.graph = tf.get_default_graph()

        self.model._make_predict_function()

        self.encoder = {
            0: TrafficLight.RED,
            1: TrafficLight.YELLOW,
            2: TrafficLight.GREEN,
            3: TrafficLight.UNKNOWN
        }


    def get_classification(self,image):  
        """ Predict using pre-trained model """

        # scale and center
        # image = (image/255) -.5

        # reshape as array
        # height = 90
        # width = 40
        image = np.expand_dims(image, axis=0)

        with self.graph.as_default():
            pred = self.model.predict(image)
            klass = np.argmax(pred)

        tl_state = self.encoder[klass]
        # print("Got class: {}, {}".format(tl_state, time.time()))

        return tl_state


