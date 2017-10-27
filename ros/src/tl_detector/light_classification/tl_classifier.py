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
        # self.graph = tf.get_default_graph()
        # with self.graph.as_default():
        #     self.model = load_model("light_classification/models/sim_tl_model.h5")
        
        with tf.gfile.FastGFile("light_classification/models/retrain_inception/retrained_graph.pb", 'rb') as f: 
            graph_def = tf.GraphDef()
            graph_def.ParseFromString(f.read())
            _ = tf.import_graph_def(graph_def, name='')

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction

        # img = cv2.imread(image)
        # img = image
        # img = cv2.resize(img,(150,150))
        # img = np.reshape(img,[1,150,150,3])
        # with self.graph.as_default():
        #     classes = self.model.predict_classes(img)
        #     return classes
        ##return TrafficLight.UNKNOWN


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

        with tf.Session() as sess:
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

