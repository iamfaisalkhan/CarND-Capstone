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
import math

STATE_COUNT_THRESHOLD = 2

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
        self.last_car_position = []
        self.last_light_pos_wp = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        # sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)

        camera_string = rospy.get_param("/camera_topic")
        sub6 = rospy.Subscriber(camera_string, Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        stop_line_positions = self.config['stop_line_positions']

        print (stop_line_positions)


        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints.waypoints

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
        light_wp, state = self.process_traffic_lights()

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
            light_wp = light_wp if state == TrafficLight.RED else (len(self.waypoints) + 1)
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))

        self.state_count += 1

    def distance(self, pos1, pos2):
        x = pos1.x - pos2.x
        y = pos1.y - pos2.y
        #z = pos1.z - pos2.z

        return math.sqrt(x*x + y*y)   

    def distance_light(self, pos1, pos2):
        x = pos1[0] - pos2.x
        y = pos1[1] - pos2.y
        #z = pos1.z - pos2.z

        return math.sqrt(x*x + y*y)

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
        closest_waypoint_dist = 100000
        closest_waypoint_ind = -1

        #Use loop to find closest one, based on https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        for i in range(0, len(self.waypoints)):
            waypoint_distance = self.distance(self.waypoints[i].pose.pose.position, pose.position)
            if waypoint_distance <= closest_waypoint_dist:
                closest_waypoint_dist = waypoint_distance
                closest_waypoint_ind = i   

        
        return closest_waypoint_ind

    def get_closest_waypoint_light(self, way_point, light_pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
       
        closest_waypoint_dist = 100000
        closest_waypoint_ind = -1

        #Use loop to find closest one, based on https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        for i, point in enumerate(way_point):
            waypoint_distance = self.distance_light(light_pose, point.pose.pose.position)
            
            if waypoint_distance <= closest_waypoint_dist:
                closest_waypoint_dist = waypoint_distance
                closest_waypoint_ind = i   

        
        return closest_waypoint_ind


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

        #x, y = self.project_to_image_plane(light.pose.pose.position)

        #TODO use light location to zoom in on traffic light in image

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']

        light_waypoint_pos = []

        if self.waypoints is None:
            return -1, TrafficLight.UNKNOWN

        for i in range(len(stop_line_positions)):
            light_pos = self.get_closest_waypoint_light(self.waypoints, stop_line_positions[i])
            light_waypoint_pos.append(light_pos)
            print ("light pos", light_pos)


        self.last_light_pos_wp = light_waypoint_pos

        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose)
            if car_position is not None:
                self.last_car_position = car_position
        else:
            return -1, TrafficLight.UNKNOWN

        if self.last_car_position > max(self.last_light_pos_wp):
            light_num_wp = min(self.last_light_pos_wp)
        else:
            light_delta = self.last_light_pos_wp[:]
            light_delta[:] = [x - self.last_car_position for x in light_delta]
            light_num_wp = min(i for i in light_delta if i > 0) + self.last_car_position

        light_ind = self.last_light_pos_wp.index(light_num_wp)
        light = stop_line_positions[light_ind]

        print ("Last car pos ", self.last_car_position)
        light_distance = self.distance_light(light, self.waypoints[self.last_car_position].pose.pose.position)

        print ("Distance to light --- ", light_distance)
        search_for_light_distance = 15
        if light:
            if light_distance >= search_for_light_distance:
                return -1, TrafficLight.UNKNOWN
            else:
                state = self.get_light_state(light)
                return light_num_wp, state    
        
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
