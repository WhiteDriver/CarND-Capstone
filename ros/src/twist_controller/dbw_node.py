#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped, PoseStamped
from styx_msgs.msg import Lane, Waypoint
import math

from twist_controller import Controller
from yaw_controller import YawController
from pid import PID
import time

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)
        min_speed = 0.1
        kp = 1
        ki = 0.05
        kd = 0

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        # TODO: Create `TwistController` object
        # self.controller = TwistController(<Arguments you wish to provide>)
        self.controller = Controller(max_st_angle = max_steer_angle)
        
        # Vishnerevsky 22.08.2017
        self.yaw_contr = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)
        self.pid_contr = PID(kp, ki, kd)

        # TODO: Subscribe to all the topics you need to
        # Valtgun 20.08.2017 - subscribe to is DBW enabled topic
        self.dbw_enabled = False
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_cb)

        # Valtgun 20.08.2017 - subscribe to next waypoints
        rospy.Subscriber('/final_waypoints', Lane, self.waypoints_cb)
        # target X,Y coordinates for simple waypoint lookup
        self.targetX = 0.0
        self.targetY = 0.0

        # Vishnerevsky 22.08.2017 - target speed and yaw
        self.target_speed = 0.0
        self.target_yaw = 0.0
        self.cur_v = 0.0
        # Vishnerevsky 22.08.2017 - timestamps for PID controller
        self.previous_timestamp = rospy.get_rostime().secs
        self.current_timestamp = 0.0
        self.delta_time = 0.0

        # Valtgun 20.08.2017 - subscribe to current pose
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        self.poseX = 0.0
        self.poseY = 0.0
        self.pose_orient = 0.0 # after conversion to euler
        # values in degrees (-180 to +180)
        # 0 aligned with X axis (in front of car initailly)
        # +90 is to the left of inital car position

        # Vishnerevsky 22.08.2017 - subscribe to /twist_cmd
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.cur_vel_cb)

        self.loop()

    def loop(self):
        rate = rospy.Rate(50) # 50Hz
        while not rospy.is_shutdown():
            # TODO: Get predicted throttle, brake, and steering using `twist_controller`
            # You should only publish the control commands if dbw is enabled
            # throttle, brake, steering = self.controller.control(<proposed linear velocity>,
            #                                                     <proposed angular velocity>,
            #                                                     <current linear velocity>,
            #                                                     <dbw status>,
            #                                                     <any other argument you need>)
            throttle, brake, steer = self.controller.control(dbw = self.dbw_enabled,
                                                            tx = self.targetX,
                                                            ty = self.targetY,
                                                            px = self.poseX,
                                                            py = self.poseY,
                                                            pt = self.pose_orient)
            # Vishnerevsky 22.08.2017:
            steer = self.yaw_contr.get_steering(self.target_speed, self.target_yaw, self.cur_v) 
            current_time = rospy.get_rostime()
            current_secs = current_time.secs
            current_nsecs = current_time.nsecs
            self.current_timestamp = current_secs + current_nsecs/1000000000.0
            self.delta_t = (self.current_timestamp - self.previous_timestamp)
            self.previous_timestamp = self.current_timestamp
            rospy.logwarn(self.delta_t)
            PID_speed = self.pid_contr.step(self.target_speed - self.cur_v, self.delta_t)
            if self.dbw_enabled:
                #self.publish(throttle, brake, steer) # Commented by Vishnerevsky 22.08.2017
                self.publish(PID_speed, brake, steer) 
                #rospy.logwarn("%.2f, %.2f, %.2f" % (self.cur_v, brake, steer))
            rate.sleep()

    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)

    # Valtgun 20.08.2017 - callback for Drive by wire enabled topic subscriber
    def dbw_cb(self, msg):
        if (msg.data == True):
            self.dbw_enabled = True
        else:
            self.dbw_enabled = False
        pass

    # Valtgun 20.08.2017 - callback for receiving next waypoint information
    def waypoints_cb(self, waypoints):
        # TODO: Takes 10 waypoints ahead, need to make more cleaver follower
        self.targetX = waypoints.waypoints[10].pose.pose.position.x
        self.targetY = waypoints.waypoints[10].pose.pose.position.y
        pass

    # Valtgun 20.08.2017 - callback for receiving next waypoint information
    def pose_cb(self, msg):
        self.poseX = msg.pose.position.x
        self.poseY = msg.pose.position.y
        orient = msg.pose.orientation
        X, Y, Z = self.Quaternion_toEulerianAngle(orient.x, orient.y, orient.z, orient.w)
        self.pose_orient = Z
        #rospy.logwarn('Pose theta: ' + str(self.pose_orient))
        # Vishnerevsky 22.08.2017:
        #self.current_timestamp = msg.header.stamp
        #rospy.logwarn(self.current_timestamp)
        pass

    # Vishnerevsky 22.08.2017
    def twist_cb(self, msg):
        #rospy.logwarn('twist_cmd received')
        lin_x = msg.twist.linear.x
        lin_y = msg.twist.linear.y
        lin_z = msg.twist.linear.z
        ang_x = msg.twist.angular.x
        ang_y = msg.twist.angular.y
        ang_z = msg.twist.angular.z
        #rospy.logwarn(lin_x)
        #rospy.logwarn("%.2f, %.2f, %.2f" % (lin_x, lin_y, lin_z))
        #rospy.logwarn("%.2f, %.2f, %.2f" % (ang_x, ang_y, ang_z))
        self.target_speed = lin_x
        self.target_yaw = 100*ang_z
        pass

    # Vishnerevsky 22.08.2017
    def cur_vel_cb(self, msg):
        self.cur_v = msg.twist.linear.x
        pass

    # Valtgun 20.08.2017 - helper to transform pose quaterion to euler
    # https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Quaternion_to_Euler_Angles_Conversion
    def Quaternion_toEulerianAngle(self, x, y, z, w):
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


if __name__ == '__main__':
    DBWNode()
