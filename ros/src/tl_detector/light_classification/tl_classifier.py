from styx_msgs.msg import TrafficLight
from keras.models import load_model
import cv2
import numpy as py

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier        
        self.model = load_model("light_classification/models/sim_tl_model.h5")

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        self.model.compile(loss='binary_crossentropy',
              optimizer='rmsprop',
              metrics=['accuracy'])

        img = cv2.imread(image)
        img = cv2.resize(img,(320,240))
        img = np.reshape(img,[1,320,240,3])

        classes = model.predict_classes(img)
        return classes
        ##return TrafficLight.UNKNOWN
