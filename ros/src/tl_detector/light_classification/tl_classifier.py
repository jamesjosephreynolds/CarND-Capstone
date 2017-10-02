from styx_msgs.msg import TrafficLight

import os
import numpy as np
import tensorflow as tf
from utils import label_map_util
from utils import visualization_utils as vis_util
import time
import math

from PIL import Image 
import glob, sys
import rospy


class TLClassifier(object):
    def __init__(self, path_to_ckpt, path_to_label_map, num_classes, score_threshold):
        self.score_threshold = score_threshold

        # Default light state
        self.default_state = TrafficLight.UNKNOWN
        #self.default_state = 4 # Debug

        # Output image
        self.image_np_classified = None

        ##################
        # Load label map  

        label_map = label_map_util.load_labelmap(path_to_label_map)
        categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=num_classes, use_display_name=True)
        self.category_index = label_map_util.create_category_index(categories)
        
        ############################################
        # Load frozen Tensorflow model into memory

        # Create graph
        self.detection_graph = tf.Graph()

        # Avoid crash: "Could not create cuDNN handle"
        # https://github.com/tensorflow/tensorflow/issues/6698
        config = tf.ConfigProto()
        config.gpu_options.allow_growth = True

        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(path_to_ckpt, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

            self.sess = tf.Session(graph=self.detection_graph, config=config)

        ###########################################################
        # Define input and output tensors for the detection_graph

        # Input image tensor
        self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
        # Bounding boxes detected
        self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
        # Scores representing confidence for each detected object
        self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
        # Class labels
        self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
        # Number of objects detected
        self.num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')

        print("Inference graph loaded")


    def dist_box_center_to_point(self, box, point):
        """
        Calculate distance from a box center to a point.
        """
        box_center_y = (box[0] + box[2]) / 2
        box_center_x = (box[1] + box[3]) / 2
        return math.sqrt((point[0] - box_center_x)**2 + (point[1] - box_center_y)**2)


    def vote_on_states(self, states, boxes, point):
        """
        Apply a naive logic: the box whose center point is the closest to 
        the projection point will win the vote.

        TODO: Refine the voting logic.
        """
        result_state = self.default_state
        min_dist = sys.maxsize

        for i in range(len(boxes)):
            # Calculate the distance from this box center to the (light projection) point
            dist = self.dist_box_center_to_point(boxes[i], point)
            # Find the state of the nearest box
            if dist < min_dist:
                min_dist = dist
                result_state = states[i]

        return result_state       


    def get_classification(self, image, projection_point):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # Preprocess image
        image_np_expanded = np.expand_dims(image, axis=0)

        # Run image through the network
        time0 = time.time()

        with self.detection_graph.as_default():
            (boxes, scores, classes, num_detections) = self.sess.run(
                [self.detection_boxes, self.detection_scores, 
                self.detection_classes, self.num_detections],
                feed_dict={self.image_tensor: image_np_expanded})

        time1 = time.time()
        #print("Time in milliseconds: %s" % ((time1 - time0) * 1000))

        boxes = np.squeeze(boxes)
        scores = np.squeeze(scores)
        classes = np.squeeze(classes).astype(np.int32)

        #print("num detections: %s" % num_detections)

        # Analyze light state from the detected boxes
        if num_detections[0] > 0:

            # Detected states and boxes with confidence
            states = []
            boxes_filtered = []

            # Iterate each box
            for i in range(boxes.shape[0]):

                # If detection confidence on this box is above threshold,
                # then we analyze and collect the state and box. 
                if scores[i] > self.score_threshold:
                    class_name = self.category_index[classes[i]]['name']
                    #print("Detected box: %s" % class_name)

                    state = TrafficLight.UNKNOWN
                    if class_name == 'Red':
                        state = TrafficLight.RED
                    elif class_name == 'Yellow':
                        state = TrafficLight.YELLOW
                    elif class_name == 'Green':
                        state = TrafficLight.GREEN

                    #Debug
                    # state = 4
                    # if class_name == 'Red':
                    #     state = 0
                    # elif class_name == 'Yellow':
                    #     state = 1
                    # elif class_name == 'Green':
                    #     state = 2

                    states.append(state)
                    boxes_filtered.append(boxes[i])
            
            if len(states) == 0:            # if no state
                return self.default_state
            elif len(states) == 1:          # if only one state
                return states[0]
            elif states[1:] == states[:-1]: # if all states are identical
                return states[0]
            else:                           # if multiple and different states
                return self.vote_on_states(states, boxes_filtered, projection_point)

        return self.default_state
