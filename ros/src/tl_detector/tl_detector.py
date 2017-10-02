#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import sys
import os
import math
import numpy as np
import glob

from light_msgs.msg import UpcomingLight

from tl_debug import TLDebug

#################################
# Build a dictionary for models

path_to_models = os.path.dirname(os.path.realpath(__file__)) + '/light_classification/models'

# Dictionary for model checkpoints, label maps and numbers of classes
MODEL_DICT = {1: (path_to_models + '/graph_frcnn_resnet_sim_bosch.pb',
                  path_to_models + '/label_map_bosch.pbtxt',
                  14),
              2: (path_to_models + '/graph_frcnn_resnet_real_bosch.pb',
                  path_to_models + '/label_map_bosch.pbtxt',
                  14),
              3: (path_to_models + '/graph_ssd_mobilenet_sim.pb',
                  path_to_models + '/label_map_udacity.pbtxt',
                  4)
              }

##################
# Set parameters

VISIBLE_DISTANCE = 200

# On/Off switch for classifier.
CLF_ON = True

# Use the predicted light state. 
# Otherwise, use true light state from TrafficLightArray message.
# Note: set True only if CLF_ON is True.
USE_PREDICTION = True

# Minimum score (confidence) for a light detection
SCORE_THRESHOLD = 0.5

STATE_COUNT_THRESHOLD = 3

##################


class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        self.lights = []
        self.all_stop_line_wps = None

        #The closest waypoint to car
        self.car_position = None

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        # Find whether simulator config or site config is introduced
        if len(self.config['stop_line_positions']) > 1:
            model_id = 1
        else:
            model_id = 2

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and 
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)       

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()

        # Initialize classifier with specified parameters
        if CLF_ON is True:
            print("Loading inference graph ...")
            ckpt, label_map, n_classes = MODEL_DICT[model_id]

            # Generate the model PD file
            self.prepare_model_file(ckpt)

            self.light_classifier = TLClassifier(ckpt, label_map, n_classes, SCORE_THRESHOLD)
            self.light_classifier_on = True
            print("Light classifier is running")

        self.listener = tf.TransformListener()

        #Publisher for UpcomingLight message
        self.upcoming_light_pub = rospy.Publisher('/upcoming_light', UpcomingLight, queue_size=1)

        # Debug Publisher
        self.debug = TLDebug()

        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        rospy.spin()

    def prepare_model_file(self, model_path):
        """Check if the model is in a single file or splitted in several files.
           If the file is splitted in several files, it creates a single file with
           all the parts in the right order.

        Args:
            model_path (String): model filename

        """
        if not os.path.exists(model_path):          
            wildcard = model_path.replace('.pb','.*')
            files    = sorted([file for file in glob.glob(wildcard)])

            join_command = 'cat {} > {}'.format(" ".join(files), model_path)
            os.system(join_command)

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg

        stop_line_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            stop_line_wp = stop_line_wp if state == TrafficLight.RED else -1
            self.last_wp = stop_line_wp
            self.upcoming_red_light_pub.publish(Int32(stop_line_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1


    def get_unsqrt_distance_between_poses(self, pose_1, pose_2):
        """Calculate the unsquared root distance between two poses in a fast way
           To speed up calculations.
        Args:
            pose_1 (Pose): first pose
            pose_2 (Pose): second pose

        Returns:
            float: unsquared root distance between two poses

        """

        diff_x = pose_1.position.x - pose_2.position.x
        diff_y = pose_1.position.y - pose_2.position.y

        return diff_x*diff_x + diff_y*diff_y


    def get_distance_between_poses(self, pose_1, pose_2):
        """Calculate the distance between two poses
        Args:
            pose_1 (Pose): pose of 1st point
            pose_2 (Pose): pose of 2nd point

        Returns:
            float: distance between two poses

        """

        diff_x = pose_1.position.x - pose_2.position.x
        diff_y = pose_1.position.y - pose_2.position.y
        diff_z = pose_1.position.z - pose_2.position.z

        return math.sqrt(diff_x*diff_x + diff_y*diff_y + diff_z*diff_z)


    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem

        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint (in self.waypoints) to the pose

        """

        # Brute Force Searching (TODO: Try to reduce the number of operations by using divide and conquer)
        # http://rosettacode.org/wiki/Closest-pair_problem#Python
        min_distance           = sys.maxsize
        nearest_waypoint_index = -1
        #print("Type of self.waypoints: %s" % type(self.waypoints))

        if self.waypoints != None:
            for i in range(0, len(self.waypoints.waypoints)):
                waypoint  = self.waypoints.waypoints[i].pose.pose
                posepoint = pose
                
                # It is not needed to use the sqrt distance, since we need only which waypoint is the nearest.
                # We can use no sqrt distance for fast calculation.
                distance = self.get_unsqrt_distance_between_poses(waypoint, posepoint) 
                if distance < min_distance:
                    min_distance = distance
                    nearest_waypoint_index = i

        return nearest_waypoint_index


    def get_all_stop_line_wps(self, stop_line_positions):
        """Find the closest waypoint for each stop line in front of a traffic light

        Args:
            stop_line_positions: list of 2D (x, y) position of all stop lines for traffic lights

        Returns:
            all_stop_line_wps: list of waypoint indices

        """
        all_stop_line_wps = []
        pose = Pose()

        for i in range(len(stop_line_positions)):
            pose.position.x = stop_line_positions[i][0]
            pose.position.y = stop_line_positions[i][1]

            wp = self.get_closest_waypoint(pose)
            all_stop_line_wps.append(wp)

        #print("Waypoint indices of all stop lines in front of lights:\n %s" % all_stop_line_wps)
        return all_stop_line_wps


    def project_to_image_plane(self, point_in_world):
        """Project point from 3D world coordinates to 2D camera image location

        Args:
            point_in_world (Point): 3D location of a point in the world

        Returns:
            x (int): x coordinate of target point in image
            y (int): y coordinate of target point in image

        """

        #fx = self.config['camera_info']['focal_length_x']
        #fy = self.config['camera_info']['focal_length_y']
        #Current focal lengths are probably wrong, which leads incorrect pixel transformation.
        #https://discussions.udacity.com/t/focal-length-wrong/358568

        image_width = self.config['camera_info']['image_width']
        image_height = self.config['camera_info']['image_height']

        x, y = None, None

        # get transform between pose of camera and world frame
        trans, rot = None, None
        try:
            now = rospy.Time.now()
            self.listener.waitForTransform("/base_link", "/world", now, rospy.Duration(1.0))
            (trans, rot) = self.listener.lookupTransform("/base_link", "/world", now)

        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            rospy.logerr("Failed to find camera to map transform")

        # http://docs.ros.org/jade/api/tf/html/c++/classtf_1_1Transformer.html
        # https://w3.cs.jmu.edu/spragunr/CS354_S14/labs/tf_lab/html/tf.listener.TransformerROS-class.html
        # http://wiki.ros.org/tf/TfUsingPython
        # http://www.cse.psu.edu/~rtc12/CSE486/lecture12.pdf
        # http://www.cse.psu.edu/~rtc12/CSE486/lecture13.pdf
        # http://slideplayer.com/slide/4547175/
        # http://slideplayer.com/slide/4852283/
        # http://www.ics.uci.edu/~majumder/VC/classes/cameracalib.pdf
        # https://stackoverflow.com/questions/5288536/how-to-change-3d-point-to-2d-pixel-location?rq=1
        # ex. trans = [-1230.0457257142773, -1080.1731777599543, -0.10696510000000001]
        # ex. rot   = [0.0, 0.0, -0.0436201197059201, 0.9990481896069084]
        # ex. matrix = [[  9.96194570e-01   8.71572032e-02   0.00000000e+00  -1.23004573e+03]
        #               [ -8.71572032e-02   9.96194570e-01   0.00000000e+00  -1.08017318e+03]
        #               [  0.00000000e+00   0.00000000e+00   1.00000000e+00  -1.06965100e-01]
        #               [  0.00000000e+00   0.00000000e+00   0.00000000e+00   1.00000000e+00]]
        
        #### Forward Projection
        #TODO: Debug rot referenced before assignment
        # World to Camera Transformation (Rigid transformation = rotation + translation)
        if trans != None and rot != None:
            transformation_matrix = self.listener.fromTranslationRotation(trans, rot)
            point_in_world_vector = np.array([[point_in_world.x], [point_in_world.y], [point_in_world.z], [1.0]], dtype=float)
            camera_point = np.dot(transformation_matrix, point_in_world_vector)

            #print("Point in camera coords: %s" % camera_point)

            # Perspective Correction
            # Instead of using the focal lengths in simulator config,
            # we use values in site config by hard coding.
            fx, fy = 1345.200806, 1353.838257
            x = int(-fx * camera_point[1] / camera_point[0] + image_width  / 2)
            y = int(-fy * camera_point[2] / camera_point[0] + image_height / 2)

        return (x, y)


    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")

        x, y = self.project_to_image_plane(light.pose.pose.position)
        #print("Projected point: (%s, %s)" %(x, y))
        
        #TODO Prepare the image to be classified
        #crop_image = cv_image[y-20:y+130, x-40:x+40]
        #resized_image = cv2.resize(crop_img, (80, 150)) 

        distance_car_tl = self.get_distance_between_poses(self.pose.pose, light.pose.pose) # Need the real distance

        #Publishing Images and metadata for debug and dataset generator
        if x != None and y != None:
            self.debug.publish_debug_image(cv_image, distance_car_tl, x, y) # Publishing in /debug/image_tl Use 'rqt' to visualize the image
            self.debug.publish_debug_image_metadata(cv_image, 0, y-20, x-40, y+130, x+40) # Publishing in /debug/image_tl_metadata

        #Get classification
        if self.light_classifier_on is True:
            return self.light_classifier.get_classification(cv_image, (x, y))

        return TrafficLight.UNKNOWN


    def get_upcoming_stop_line_wp(self, car_position, all_stop_line_wps):
        """Find the waypoint of the upcoming stop line in front of a light

        Args:
            car_position (Int): the closest waypoint to the car
            all_stop_line_wps ([Int]): list of the closest waypoint for each stop line in front of a light

        Returns: 
            int: the waypoint index of the upcoming stop line in front of a light
            int: the index of the upcoming light in [lights] 

        """
        #Find the interval in which the car is
        interval = 0
        if car_position == 0:
            pass
        else:
            for i in range(len(all_stop_line_wps)):
                if car_position <= all_stop_line_wps[i]:
                    interval = i
                    break
        #print("interval: %s" % interval)

        #Find the upcoming light waypoint and index
        #Note: only go one way along an ascending sequence of waypoints
        stop_line_wp = all_stop_line_wps[interval]
        light_id = interval

        return stop_line_wp, light_id


    def generate_upcominglight_msg(self, waypoint, id, pose, state):
        """Generate upcoming light message

        Args:
            waypoint: index of waypoint closest to the stop line in front of a traffic light
            id      : index of the traffic light in TrafficLightArray
            pose    : light pose obtained from /vehicle/traffic_lights
            state   : true light state obtained from /vehicle/traffic_lights

        Returns:
            msg: message of UpcomingLight type 

        """
        msg = UpcomingLight()
        msg.waypoint = waypoint
        msg.index = id
        msg.pose = pose
        msg.state = state
        return msg


    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closest to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']

        # Find the waypoint closest to car's current position
        if(self.pose):
            position_tmp = self.get_closest_waypoint(self.pose.pose)
            if position_tmp != -1:
                self.car_position = position_tmp

        #Find the closest waypoint for each traffic light
        if self.all_stop_line_wps == None and self.waypoints != None:
            self.all_stop_line_wps = self.get_all_stop_line_wps(stop_line_positions)
        
        #####Find the waypoint and index of the upcoming light
        # 1 - Check that we have traffic lights waypoints and car location
        if self.all_stop_line_wps != None and self.car_position != None:
            # Get the location and index of the upcoming nearest traffic light
            stop_line_wp, light_id = self.get_upcoming_stop_line_wp(self.car_position, self.all_stop_line_wps)
            #print("Upcoming stop line waypoint and index: %s, %s" % (stop_line_wp, light_id))

            # Find the distance between the car and the upcoming light
            if self.pose != None and self.lights != None:
                distance_to_light = self.get_distance_between_poses(self.pose.pose, self.lights[light_id].pose.pose)

                # Publish the message of UpcomingLight (including true light state)
                # As Teaching Assistant noted in Slack, topic /vehicle/traffic_lights containing true light state
                # will also be available and published on Carla in site.
                upcoming_msg = self.generate_upcominglight_msg(stop_line_wp, light_id, self.lights[light_id].pose, self.lights[light_id].state)
                self.upcoming_light_pub.publish(upcoming_msg)

                # Check if the car is in the range of VISIBLE_DISTANCE in order to proceed with the classification
                if distance_to_light < VISIBLE_DISTANCE:
                    light = self.lights[light_id]

        # 2 - Check if light is available, then try to identify the color
        if light:
            # Use predicted light state
            if USE_PREDICTION:
                pred_state = self.get_light_state(light)
                print("Predicted light state: %s" % pred_state)
                return stop_line_wp, pred_state

            # Use true light state
            return stop_line_wp, self.lights[light_id].state

        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
