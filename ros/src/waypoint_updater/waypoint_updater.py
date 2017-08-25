#!/usr/bin/env python

import math
import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
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
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.last_waypoints = None
        self.points_in_back = 10

        rospy.spin()

    def pose_cb(self, msg):

        # TODO: Implement
        if self.last_waypoints is not None:
            #rospy.logwarn('GO_555!!!')
            pose = msg.pose
            waypoints = self.last_waypoints.waypoints
            lane = Lane()
            lane.header.stamp = rospy.Time.now()
            start_point = self.closest_waypoint(pose, waypoints)
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
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        pass

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        pass

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
