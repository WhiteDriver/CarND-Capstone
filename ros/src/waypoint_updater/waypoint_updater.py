#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)


        # TODO: Add other member variables you need below

        # Valtgun 19.08.2017 - added variables for storing global map
        mapX = [] # variable to store waypoint X coordinate from /base_waypoints
        mapY = [] # variable to store waypoint Y coordinate from /base_waypoints
        map_wp_len = 0 #numper of map waypoints in /base_waypoints
        mapWP = [] # array to store map waypoints for full information

        rospy.spin()

    def pose_cb(self, msg):
        # Valtgun 19.08.2017 - locate nearest waypoint ahead
        pose_x = msg.pose.position.x
        pose_y = msg.pose.position.y
        nearest_waypoint = self.closest_waypoint(pose_x, pose_y, self.mapX, self.mapY)

        # TODO: can return closest waypoint even if it is behind, need to add orientation check
        # If need to implement then C++ code can be taken from
        # CarND-Path-Planning-Project function NextWaypoint
        # in that case heading is required

        # Vishnerevsky 21.08.2017:
        quat_x = msg.pose.orientation.x
        quat_y = msg.pose.orientation.y
        quat_z = msg.pose.orientation.z
        quat_w = msg.pose.orientation.w	
        Fi, Theta, Psi = self.Get_Angle(quat_x, quat_y, quat_z, quat_w)
        #rospy.logwarn('Quaternion:')
        #rospy.logwarn(Fi)
        #rospy.logwarn(Theta)
        #rospy.logwarn(Psi) # Angle of the car. +90 deg <-, -90 deg ->
        next_waypoint = self.NextWaypoint(pose_x, pose_y, Psi, self.mapX, self.mapY)
        rospy.logwarn(str(nearest_waypoint) + ' ' + str(next_waypoint))

        # Valtgun 19.08.2017 - get next LOOKAHEAD_WPS waypoints
        pub_list = []
        for i in range(LOOKAHEAD_WPS):
            # Valtgun 20.08.2017 - changed to waypoint to contain full information
            pub_list.append(self.mapWP[nearest_waypoint+i])

        # Valtgun 19.08.2017 - create correct data structure for publishing
        lane = Lane()
        lane.header.frame_id = msg.header.frame_id
        lane.header.stamp = rospy.Time(0)
        lane.waypoints = pub_list

        # Valtgun 19.08.2017 - publish to /final_waypoints
        # TODO: need to check if always need publishing on pose_cb, possibly performance issues
        if (not rospy.is_shutdown()):
            self.final_waypoints_pub.publish(lane)
        pass

    def waypoints_cb(self, waypoints):
        # Valtgun 19.08.2017 - create two lists one with X and other with Y waypoints
        new_map_wp_len = len(waypoints.waypoints)
        new_mapX = []
        new_mapY = []
        new_mapWP = []
        for waypoint in waypoints.waypoints[:]:
            new_mapX.append(waypoint.pose.pose.position.x)
            new_mapY.append(waypoint.pose.pose.position.y)
            new_mapWP.append(waypoint)

        # Valtgun 19.08.2017 - assign to global variables
        self.mapX = new_mapX
        self.mapY = new_mapY
        self.mapWP = new_mapWP
        self.map_wp_len = new_map_wp_len
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

    # Valtgun 19.08.2017 - calculate distance between two points
    def dist_two_points(self, x1, y1, x2, y2):
        return math.sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1))

    # Valtgun 19.08.2017 - finds nearest waypoint
    def closest_waypoint(self, x, y, maps_x, maps_y):
        closest_wp_dist = 999999.9;
        closest_wp = 0;

        for i in range(self.map_wp_len):
            map_x = maps_x[i]
            map_y = maps_y[i]
            dist = self.dist_two_points(x, y, map_x, map_y)
            if (dist < closest_wp_dist):
                closest_wp_dist = dist
                closest_wp = i
        return closest_wp

    # Vishnerevsky 21.08.2017 (Valtgun helper to transform pose quaterion to euler)
    # https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Quaternion_to_Euler_Angles_Conversion
    def Get_Angle(self, x, y, z, w):
        ysqr = y*y
        t0 = +2.0 * (w * x + y*z)
        t1 = +1.0 - 2.0 * (x*x + ysqr)
        X = math.degrees(math.atan2(t0, t1))

        t2 = +2.0 * (w*y - z*x)
        t2 =  1 if t2 > 1 else t2
        t2 = -1 if t2 < -1 else t2
        Y = math.degrees(math.asin(t2))

        t3 = +2.0 * (w * z + x*y)
        t4 = +1.0 - 2.0 * (ysqr + z*z)
        Z = math.degrees(math.atan2(t3, t4))
        return X, Y, Z

    # Vishnerevsky 21.08.2017 NextWaypoint() function from the Path Planning Project
    def NextWaypoint(self, car_x, car_y, theta, mapc_x, mapc_y):
        #pass
        closestWaypoint = self.closest_waypoint(car_x, car_y, mapc_x, mapc_y)
        map_x = mapc_x[closestWaypoint]
        map_y = mapc_y[closestWaypoint]
        heading = math.atan2((map_y - car_y), (map_x - car_x))
        angle = math.fabs(math.radians(theta) - heading)
        if (angle > 3.1415926/4.0):
            closestWaypoint += 1
        return closestWaypoint



if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
