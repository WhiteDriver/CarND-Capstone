#!/usr/bin/env python

import math
import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint, TrafficLightArray
import std_msgs.msg


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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

	# Omar 25.08.2017 Subscribed to Traffic lights waypoints
        rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
	# Omar 25.08.2017 add a variable for the car current position
	self.pose = None
        # Omar 25.08.2017 add a variable lane
	self.lane = Lane()

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.last_waypoints = None
        self.points_in_back = 10

        rospy.spin()

    def pose_cb(self, msg):

        # TODO: Implement
        if self.last_waypoints is not None:
            #rospy.logwarn('GO_555!!!')
            self.pose = msg.pose
            waypoints = self.last_waypoints.waypoints
            lane.header.stamp = rospy.Time.now()
            start_point = self.closest_waypoint(self.pose, waypoints)
            all_waypoints = waypoints + waypoints[:LOOKAHEAD_WPS]
            lane.waypoints = all_waypoints[start_point-self.points_in_back: start_point-self.points_in_back + LOOKAHEAD_WPS]         

            for index in range(len(lane.waypoints)):

                lane.waypoints[index].twist.twist.linear.x = 10 # Meters per second

            self.final_waypoints_pub.publish(lane)

    def waypoints_cb(self, lane):
        # TODO: Implement
        self.last_waypoints = lane

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
	# Omar 25.08.2017 Calculate the closest Waypoint to the current car position
	Closest_TrafficLight = self.closest_waypoint(pose, msg.lights)
	# Omar 25.08.2017 Check if the closest Traffic light is red
        if msg.lights[Closest_TrafficLight].state == 0:
		# Omar 25.08.2017 Find the waypoints between the car and the Traffic Sign and set their velocity to 0
		for index in range(len(lane.waypoints)):
			if self.distance(pose.position, lane.waypoints[index].pose.pose.position) < self.distance(pose.position, msg.lights[Closest_TrafficLight].pose.pose.position):
				self.set_waypoint_velocity(lane.waypoints, index, 0)
                	
     

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    # Omar 25.08.2017 Get the waypoint velocity
    def get_waypoint_velocity(self, waypoint):
	return waypoint.twist.twist.linear.x
        
    # Omar 25.08.2017 Set the waypoint velocity
    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
	waypoints[waypoint].twist.twist.linear.x = velocity
        

    def closest_waypoint(self, pose, waypoints):
        best_waypoint = 0
        min_dist = self.distance(pose.position, waypoints[0].pose.pose.position)
        for i, point in enumerate(waypoints):
            dist = self.distance(pose.position, point.pose.pose.position)
            if dist < min_dist:
                best_waypoint = i
                min_dist = dist
        return best_waypoint

    def distance(self, p1, p2):
        delta_x = p1.x - p2.x
        delta_y = p1.y - p2.y
        return math.sqrt(delta_x*delta_x + delta_y*delta_y)

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
