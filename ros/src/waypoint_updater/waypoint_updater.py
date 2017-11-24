#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
import copy

from std_msgs.msg import Int32

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        self.current_pose_sub = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        self.base_waypoints_sub = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        self.traffic_waypoint_sub = rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        #self.obstacle_waypoint_sub = rospy.Subscriber('/obstacle_waypoint', <MessageType>, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.LOOKAHEAD_WPS = 200
        self.MAX_ACCELERATION = 10.0 # m s^-2
        self.MIN_ACCELERATION = -10.0 # m s^-2
        self.MAX_JERK = 10.0 # m s^-3
        self.PREFERRED_STOPPING_DISTANCE = 10.0 # meters

        self.current_pose = 0
        self.base_waypoints_original = []
        self.base_waypoints = []
        self.final_waypoints = []
        self.closest_waypoint_index = 0
        self.traffic_waypoint_index = 2147483647

        self._MAX_DISTANCE_ERROR = 0.0 # meters, maximum distance from the closest waypoint

        rospy.spin()

    def publish(self):
        # Publisher for final_waypoints
        lane = Lane()
        lane.header.frame_id = '/world2'
        lane.header.stamp = rospy.Time(0)
        lane.waypoints = self.final_waypoints
        self.final_waypoints_pub.publish(lane)

    def pose_cb(self, msg):
        # TODO: Implement
        self.current_pose = msg.pose

        # Process waypoints here
        # Step 1: Identify waypoint closest to vehicle, but in front of the vehicle
        self.closest_waypoint_index = self.get_closest_waypoint_front(self.current_pose, self.base_waypoints_original)
        waypoint_position_x = self.base_waypoints_original[self.closest_waypoint_index].pose.pose.position.x
        waypoint_position_y = self.base_waypoints_original[self.closest_waypoint_index].pose.pose.position.y
        current_pose_position_x = self.current_pose.position.x
        current_pose_position_y = self.current_pose.position.y
        dist = math.sqrt((waypoint_position_x - current_pose_position_x)**2 + (waypoint_position_y - current_pose_position_y)**2)
        if dist > self._MAX_DISTANCE_ERROR:
            self._MAX_DISTANCE_ERROR = dist
        rospy.loginfo("Maximum distance error: " + str(self._MAX_DISTANCE_ERROR) + " meters.")

        # Step 2: Process waypoints (TBD when traffic light detection is available)
        # print ("Trarffic waypoint index ", self.traffic_waypoint_index)
        if self.traffic_waypoint_index < len(self.base_waypoints):
            rospy.loginfo("Red light waypoint found")
            #Setting desired velocity value on the index
            self.base_waypoints[self.traffic_waypoint_index].twist.twist.linear.x = 0.0

            #Obtaining waypoint index for waypoint that is PREFERRED_STOPPING_DISTANCE away from the stopline
            waypoint_to_start_deceleration = self.traffic_waypoint_index
            for ctr in range(self.traffic_waypoint_index, 0, -1):
                waypoint_to_start_deceleration = ctr
                thisDistance = self.distance(self.base_waypoints, self.traffic_waypoint_index, ctr)
                if thisDistance >= self.PREFERRED_STOPPING_DISTANCE:
                    break

            #Utilizing a linear function for velocity reduction, with respect to distance
            #This step will update self.base_waypoints, and utilizes numpy.interp
            distance_input = []
            for ctr in range(waypoint_to_start_deceleration, self.traffic_waypoint_index, 1):
                distance_input.append(self.distance(self.base_waypoints, ctr, waypoint_to_start_deceleration))
            distance_points = [0.0, self.distance(self.base_waypoints, waypoint_to_start_deceleration, self.traffic_waypoint_index)]
            velocity_points = [self.base_waypoints[waypoint_to_start_deceleration].twist.twist.linear.x, 0.0]
            velocity_output = np.interp(distance_input, distance_points, velocity_points).tolist()
            for ctr in range(len(velocity_output)):
                self.base_waypoints[waypoint_to_start_deceleration + ctr].twist.twist.linear.x = velocity_output[ctr]

            #Setting a few forward waypoints to zero velocity as well, to ensure that the car stays at a complete stop
            end_idx = min(len(self.base_waypoints), self.traffic_waypoint_index + 50)
            for ctr in range(self.traffic_waypoint_index, end_idx):
                self.base_waypoints[ctr].twist.twist.linear.x = 0.0

            # Step 3: Count LOOKAHEAD_WPS waypoints ahead of the vehicle.
            end_idx = min(self.closest_waypoint_index + self.LOOKAHEAD_WPS - 1, len(self.base_waypoints))
            waypoints_list = self.base_waypoints[self.closest_waypoint_index : end_idx]
        else:
            # Step 3: Count LOOKAHEAD_WPS waypoints ahead of the vehicle.
            end_idx = min(self.closest_waypoint_index + self.LOOKAHEAD_WPS - 1, len(self.base_waypoints_original))
            waypoints_list = self.base_waypoints_original[self.closest_waypoint_index: end_idx]

        # Step 4: Publish waypoints
        self.final_waypoints = waypoints_list
        self.publish()

        pass

    def get_closest_waypoint_front(self, current_pose, waypoints):
        current_pose_position_x = current_pose.position.x
        current_pose_position_y = current_pose.position.y

        #Obtain waypoints that are ahead of the car. This code makes the assumption
        #that the starting position of the car is the same as the position specified
        #in the first base_waypoint
        min_distance_sq = 9999999999999999.0
        min_ctr = 0
        for ctr in range(len(waypoints)):
            waypoint_position_x = waypoints[ctr].pose.pose.position.x
            waypoint_position_y = waypoints[ctr].pose.pose.position.y
            thisDistance_sq = (waypoint_position_x - current_pose_position_x) ** 2 + \
                              (waypoint_position_y - current_pose_position_y) ** 2
            if thisDistance_sq < min_distance_sq:
                min_distance_sq = thisDistance_sq
                min_ctr = ctr

        """
        ctr = self.closest_waypoint_index
        min_ctr = ctr
        prev_distance = 99999999999999999.0
        while ctr < len(waypoints):
            waypoint_position_x = waypoints[ctr].pose.pose.position.x
            waypoint_position_y = waypoints[ctr].pose.pose.position.y
            thisDistance = math.sqrt((waypoint_position_x - current_pose_position_x)**2 + \
                                     (waypoint_position_y - current_pose_position_y)**2)
            if thisDistance > prev_distance:
                #i.e. Inflection point
                min_ctr = max(ctr - 1, 0)
                break
            prev_distance = thisDistance
            ctr += 1
        """
        return min_ctr

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.base_waypoints_original = copy.deepcopy(waypoints.waypoints)
        self.base_waypoints = waypoints.waypoints
        pass

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.traffic_waypoint_index = msg.data
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
