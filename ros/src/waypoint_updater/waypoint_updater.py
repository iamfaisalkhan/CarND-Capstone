#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

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
        self.LOOKAHEAD_WPS = rospy.get_param('~LOOKAHEAD_WPS', 200)
        self.MAX_ACCELERATION = rospy.get_param('~MAX_ACCELERATION', 10) # m s^-2
        self.MIN_ACCELERATION = rospy.get_param('~MIN_ACCELERATION', -10) # m s^-2
        self.MAX_JERK = rospy.get_param('~MAX_JERK', 10) # m s^-3

        self.current_pose = 0
        self.base_waypoints = []
        self.final_waypoints = []
        self.closest_waypoint_index = 0

    def publish(self):
        # Publisher for final_waypoints
        lane = Lane()
        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time(0)
        lane.waypoints = self.final_waypoints
        self.final_waypoints_pub.publish(lane)

    def pose_cb(self, msg):
        # TODO: Implement
        self.current_pose = msg.pose

        # Process waypoints here
        # Step 1: Identify waypoint closest to vehicle, but in front of the vehicle
        self.closest_waypoint_index = self.get_closest_waypoint_front(self.current_pose, self.base_waypoints)

        # Step 2: Count LOOKAHEAD_WPS waypoints ahead of the vehicle.
        end_idx = min(self.closest_waypoint_index + self.LOOKAHEAD_WPS - 1, len(self.base_waypoints))
        waypoints_list = self.base_waypoints[self.closest_waypoint_index : end_idx]

        # Step 3: Process waypoints (TBD when traffic light detection is available)

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

        return min_ctr

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.base_waypoints = waypoints.waypoints
        pass

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
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
