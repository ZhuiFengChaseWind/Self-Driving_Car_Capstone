from styx_msgs.msg import TrafficLight
from keras.models import load_model
from keras.models import model_from_yaml
import numpy as np
import cv2
import numpy as py
import tensorflow as tf

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        self.graph = tf.get_default_graph()
        with self.graph.as_default():
            self.model = load_model("light_classification/models/sim_tl_model.h5")

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction

        # img = cv2.imread(image)
        img = image
        img = cv2.resize(img,(150,150))
        img = np.reshape(img,[1,150,150,3])
        with self.graph.as_default():
            classes = self.model.predict_classes(img)
            return classes
        ##return TrafficLight.UNKNOWN
