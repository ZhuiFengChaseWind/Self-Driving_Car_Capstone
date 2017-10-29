from styx_msgs.msg import TrafficLight
from keras.models import load_model
from keras.models import model_from_yaml
import numpy as np
import io
import scipy as scipy
import cv2
import numpy as py
import tensorflow as tf

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier        
        self.modelPath = "light_classification/models/retrain_inception/retrained_graph.pb"

    def load_graph(self, frozen_graph_filename):
        # We load the protobuf file from the disk and parse it to retrieve the 
        # unserialized graph_def
        with tf.gfile.GFile(frozen_graph_filename, "rb") as f:
            graph_def = tf.GraphDef()
            graph_def.ParseFromString(f.read())

        # Then, we import the graph_def into a new Graph and returns it 
        with tf.Graph().as_default() as graph:
            # The name var will prefix every op/nodes in your graph
            # Since we load everything in a new graph, this is not needed
            tf.import_graph_def(graph_def, name="prefix")
        return graph

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
        # yellow = 2
        # red = 1        
        # green = 0
        # ------------------------------------------------------
        #image_data = image
        
        img = scipy.misc.toimage(image, channel_axis=2)
        jpeg_bytes = io.BytesIO()
        img.save(jpeg_bytes,format="JPEG")
        jpeg_encoded = jpeg_bytes.getvalue()

        graph = self.load_graph(self.modelPath)                              
        with tf.Session(graph=graph) as sess:
            softmax_tensor = sess.graph.get_tensor_by_name('prefix/final_result:0')
            predictions = sess.run(softmax_tensor, \
                    {'prefix/DecodeJpeg/contents:0': jpeg_encoded})

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
                prediction = TrafficLight.GREEN
            else:
                print("Path is clear")
                prediction == TrafficLight.UNKNOWN

            return prediction

