#!/usr/bin/env python

import math
import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint, TrafficLightArray, TrafficLight
import std_msgs.msg
from std_msgs.msg import Bool, Float64, Int32


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
        #rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
	    # Omar 25.08.2017 add a variable for the car current position
        self.pose = None
        # Omar 25.08.2017 add a variable lane
        self.lane = Lane()

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # Vishnerevsky 26.08.2017 Create publisher for velocity reference
        self.velocity_pub = rospy.Publisher('velocity_reference', Float64, queue_size=1)
        self.velocity_reference = 0.0

        # TODO: Add other member variables you need below
        self.last_waypoints = None
        self.points_in_back = 10
        # Vishnerevsky & Omar 26.08.2017
        self.traffic_lights_X = None
        self.traffic_lights_Y = None
        self.traffic_lights_S = None # State of the traffic light

        # Vishnerevsky 29.08.2017: Add subscriber for '/traffic_waypoint' topic
        rospy.Subscriber('/traffic_waypoint', TrafficLight, self.tl_state_cb)
        self.true_tl_state = None # True state of the traffic light
        self.true_tl_distx = 1000 # Distance to the nearest traffic light (X in vehicle coordinate system)

        rospy.spin()

    def pose_cb(self, msg):

        # TODO: Implement
        if ((self.last_waypoints is not None) and (self.traffic_lights_S is not None)):
            #rospy.logwarn('GO_555!!!')
            self.pose = msg.pose
            waypoints = self.last_waypoints.waypoints
            self.lane.header.stamp = rospy.Time.now()
            start_point = self.closest_waypoint(self.pose, waypoints)
            all_waypoints = waypoints + waypoints[:LOOKAHEAD_WPS]
            self.lane.waypoints = all_waypoints[start_point-self.points_in_back: start_point-self.points_in_back + LOOKAHEAD_WPS]

            # Vishnerevsky & Omar 26.08.2017:
            dist_to_the_light = 1000.0
            cur_tl_num = 0 # Current Traffic Light Number
            # Try to convert traffic lignts positions into the vehicle coordinate system:
            # Lest transform our points into the vehicle coordinate system:
            lights_x_car = []
            lights_y_car = []
            # From quaternion to Euler angles:
            x = self.pose.orientation.x
            y = self.pose.orientation.y
            z = self.pose.orientation.z
            w = self.pose.orientation.w
            # Determine car heading:
            t3 = +2.0 * (w * z + x*y)
            t4 = +1.0 - 2.0 * (y*y + z*z)
            theta = math.degrees(math.atan2(t3, t4))
            Xcar = (self.traffic_lights_Y-self.pose.position.y)*math.sin(math.radians(theta))-(self.pose.position.x-self.traffic_lights_X)*math.cos(math.radians(theta))
            Ycar = (self.traffic_lights_Y-self.pose.position.y)*math.cos(math.radians(theta))-(self.traffic_lights_X-self.pose.position.x)*math.sin(math.radians(theta))
            # Condition for the nearest traffic light:
            if ((math.fabs(Ycar) < 15.0) and (Xcar >= 0)):
                dist_to_the_light = Xcar
            else:
                dist_to_the_light = 1000
            #rospy.logwarn(dist_to_the_light)
            # Conditions for stop:
            '''
            if (((dist_to_the_light < 70.0) and (dist_to_the_light >= 30.0)) and ((self.traffic_lights_S[cur_tl_num] == 0) or (self.traffic_lights_S[cur_tl_num] == 1))):
                #rospy.logwarn('STOP!!!!')
                self.velocity_reference = 2.0
            elif (((dist_to_the_light < 30.0) and (dist_to_the_light > 25.0)) and ((self.traffic_lights_S[cur_tl_num] == 0) or (self.traffic_lights_S[cur_tl_num] == 1))):
                self.velocity_reference = 0.0
            else:
                self.velocity_reference = 20.0
            '''
            #rospy.logerr(dist_to_the_light)
            # Vishnerevsky 29.08.2017: Conditions with true_tl_state
            if (((dist_to_the_light < 70.0) and (dist_to_the_light >= 5.0)) and ((self.traffic_lights_S == 0) or (self.traffic_lights_S == 1))):
                #rospy.logwarn('SLOW!!!!')
                self.velocity_reference = 0.7
                #rospy.logerr(self.traffic_lights_S)
            elif (((dist_to_the_light < 5.0) and (dist_to_the_light > 0.0)) and ((self.traffic_lights_S == 0) or (self.traffic_lights_S == 1))):
                self.velocity_reference = 0.0
                #rospy.logwarn('STOP!!!!')
                #rospy.logerr(self.traffic_lights_S)
            else:
                self.velocity_reference = 4.5
                #rospy.logwarn('GO!!!!!!')
                #rospy.logerr(self.traffic_lights_S)
            self.final_waypoints_pub.publish(self.lane)
            self.velocity_pub.publish(self.velocity_reference)

        else:
            rospy.logwarn('Traffic lights list is not defined')
            # Waypoints publishihg:
            #self.final_waypoints_pub.publish(self.lane)
            #self.velocity_pub.publish(self.velocity_reference)



    def waypoints_cb(self, Lane):
        # TODO: Implement
        self.last_waypoints = Lane

    def traffic_cb(self, msg):
        pass


    '''
        # Vishnerevsky 25.08.2017
        if self.pose is not None:
            # TODO: Callback for /traffic_waypoint message. Implement
	    # Omar 25.08.2017 Calculate the closest Waypoint to the current car position
	    Closest_TrafficLight = self.closest_waypoint(self.pose, msg.lights)
	    # Omar 25.08.2017 Check if the closest Traffic light is red
            #rospy.logwarn(msg.lights[Closest_TrafficLight].state)         #LOGWARN!!!!!!
	    # Omar 26.08.2017 Check if the closest Traffic light is red and the distance is less than 3m to the car
            if msg.lights[Closest_TrafficLight].state == 0 and self.distance(self.pose.position, msg.lights[Closest_TrafficLight].pose.pose.position) < 150:
                # Omar 25.08.2017 Find the waypoints between the car and the Traffic Sign and set their velocity to 0
                for index in range(len(self.lane.waypoints)):
                    #if self.distance(self.pose.position, self.lane.waypoints[index].pose.pose.position) < self.distance(self.pose.position, msg.lights[Closest_TrafficLight].pose.pose.position):
                    #self.set_waypoint_velocity(self.lane.waypoints, index, 0)
                    self.lane.waypoints[index].twist.twist.linear.x = 0.0
            else:
                for index in range(len(self.lane.waypoints)):
                    #if self.distance(self.pose.position, self.lane.waypoints[index].pose.pose.position) < self.distance(self.pose.position, msg.lights[Closest_TrafficLight].pose.pose.position):
                    #self.set_waypoint_velocity(self.lane.waypoints, index, 20)
                    self.lane.waypoints[index].twist.twist.linear.x = 20.0
    '''

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    # Omar 25.08.2017 Get the waypoint velocity
    def get_waypoint_velocity(self, waypoint):
	return waypoint.twist.twist.linear.x

    # Omar 25.08.2017 Set the waypoint velocity
    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
	waypoints[waypoint].twist.twist.linear.x = velocity


    def closest_waypoint(self, Pose, waypoints):
        best_waypoint = 0
        min_dist = self.distance(Pose.position, waypoints[0].pose.pose.position)
        for i, point in enumerate(waypoints):
            dist = self.distance(Pose.position, point.pose.pose.position)
            if dist < min_dist:
                best_waypoint = i
                min_dist = dist
        return best_waypoint

    def distance(self, p1, p2):
        delta_x = p1.x - p2.x
        delta_y = p1.y - p2.y
        return math.sqrt(delta_x*delta_x + delta_y*delta_y)

    # Vishnerevsky 29.08.2017: Get the traffic light state message
    def tl_state_cb(self, msg):
        if self.last_waypoints is not None:
            self.true_tl_distn = msg.pose.pose.position.x
            self.traffic_lights_X = msg.pose.pose.position.x
            self.traffic_lights_Y = msg.pose.pose.position.y
            self.traffic_lights_S = msg.state


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
