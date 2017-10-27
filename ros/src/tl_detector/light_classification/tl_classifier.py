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
        # with self.graph.as_default():
        #     self.model = load_model("light_classification/models/sim_tl_model.h5")
        self.sess = None        
        self.modelPath = "light_classification/models/retrain_inception/retrained_graph.pb"

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        # ------------------------------------------------------
        # TrafficLight.msg enum are as below

        # uint8 UNKNOWN=4
        # uint8 GREEN=2
        # uint8 YELLOW=1
        # uint8 RED=0

        # However, tha classifier's label are as below
        # red = 1
        # green = 0
        # yellow = 2
        # ------------------------------------------------------
        image_data = image

        if self.sess == None:
            gd = tf.GraphDef()
            gd.ParseFromString(tf.gfile.GFile(self.modelPath, "rb").read())
            self.sess = tf.Session()
                        
        with self.sess as sess:
            softmax_tensor = sess.graph.get_tensor_by_name('final_result:0')
            predictions = sess.run(softmax_tensor, \
                    {'DecodeJpeg/contents:0': image_data})

            top_k = predictions[0].argsort()[-len(predictions[0]):][::-1]
            prediction = top_k[0]
            print("The prediction label is ", prediction)


            if prediction == TrafficLight.RED:               
            print("Detected is RED")
            prediction = TrafficLight.RED
            elif prediction == TrafficLight.YELLOW:
                print("Detected is YELLOW")
                prediction = TrafficLight.YELLOW
            elif prediction == TrafficLight.GREEN:
                print("Detected is GREEN")
                prediction = TrafficLight.YELLOW
            else:
                print("Path is clear")
                prediction == TrafficLight.UNKNOWN

            return prediction

