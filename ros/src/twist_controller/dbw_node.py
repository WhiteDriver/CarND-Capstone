#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool, Float64
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped, PoseStamped
from styx_msgs.msg import Lane, Waypoint
import math
import numpy as np

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

        # Some important variables:
        self.current_pose = None
        self.final_waypoints = None

        # Another important variables:
        self.cur_v = 0.0
        # Valtgun 20.08.2017 - subscribe to is DBW enabled topic
        self.dbw_enabled = False
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_cb)

        # Vishnerevsky 22.08.2017 - timestamps for PID controller
        self.previous_timestamp = rospy.get_rostime().secs
        self.current_timestamp = 0.0
        self.delta_time = 0.0
        # Vishnerevsky 24.08.2017
        #self.V_pid = PID(kp=1.0, ki=0.02, kd=0, mn=decel_limit, mx=accel_limit)
        # Vishnerevsky 25.08.2017
        #self.V_pid = PID(kp=1000.0, ki=5000.0, kd=100.0, mn=decel_limit*10.0, mx=accel_limit*10.0)
        self.V_pid = PID(kp=100.0, ki=0.0, kd=0.0)
        self.S_pid = PID(kp=0.2, ki=0.001, kd=0.5)

        # Vishnerevsky 25.08.2017
        self.previous_V_ref = 0.0

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        # TODO: Create `TwistController` object
        #self.controller = Controller(self.throttle_pid, self.steering_pid)

        # TODO: Subscribe to all the topics you need to
        #rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_commands_cb)
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.cur_vel_cb, queue_size=1)
        rospy.Subscriber('/current_pose', PoseStamped, self.current_pose_cb, queue_size=1)
        rospy.Subscriber('/final_waypoints', Lane, self.final_waypoints_cb)

        # Vishnerevsky 26.08.2017
        # Create subscriber for velocity reference
        rospy.Subscriber('velocity_reference', Float64, self.velocity_cb)
        self.V_reference = 0.0

        self.loop()

    def loop(self):
        rate = rospy.Rate(10) # 50Hz
        while not rospy.is_shutdown():
            # TODO: Get predicted throttle, brake, and steering using `twist_controller`
            # You should only publish the control commands if dbw is enabled
            # throttle, brake, steering = self.controller.control(<proposed linear velocity>,
            #                                                     <proposed angular velocity>,
            #                                                     <current linear velocity>,
            #                                                     <dbw status>,
            #                                                     <any other argument you need>)

            #data = [self.last_twist_command, self.current_velocity, self.current_pose, self.final_waypoints]
            #is_all_data_availabe = all([x is not None for x in data])

            #if self.is_drive_by_wire_enable and is_all_data_availabe:
            if self.dbw_enabled and self.final_waypoints is not None:

                # Vishnerevsky 22.08.2017:
                current_time = rospy.get_rostime()
                current_secs = current_time.secs
                current_nsecs = current_time.nsecs
                self.current_timestamp = current_secs + current_nsecs/1000000000.0
                self.delta_t = (self.current_timestamp - self.previous_timestamp)
                self.previous_timestamp = self.current_timestamp
                #rospy.logwarn('GO7777!!!')

                # Vishnerevsky 25.08.2017:
                # Car Velocity Error:
                #self.V_reference = self.final_waypoints[10].twist.twist.linear.x

                #if ((self.V_reference == 0.0) or (self.V_reference == 20.0)): # To beat 11.1112 appearance
                #    self.previous_V_ref = self.V_reference
                #else:
                #    self.V_reference = self.previous_V_ref

                #rospy.logwarn(V_reference)
                VELE = self.V_reference - self.cur_v
                # Cross Track Error:
                CTE = self.get_CTE(self.final_waypoints, self.current_pose)

                #if (CTE < -2.0):
                #    CTE = -2.0
                #if (CTE > 2.0):
                #    CTE = 2.0
                #throttle, brake, steer = self.controller.control(
                #    VELE, CTE, self.delta_t)

                #rospy.logwarn(CTE)
                throttle = self.V_pid.step(VELE, self.delta_t)
                brake = 0
                if throttle < 0:
                    brake = -10.0 * throttle
                    throttle = 0
                steering = self.S_pid.step(CTE, self.delta_t)
                self.publish(throttle, brake, steering)

            else:
                # Here we must reset PID controllers
                pass
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

    #def twist_commands_cb(self, msg):
    #    self.last_twist_command = msg.twist

    # Valtgun 20.08.2017 - callback for Drive by wire enabled topic subscriber
    def dbw_cb(self, msg):
        if (msg.data == True):
            self.dbw_enabled = True
        else:
            self.dbw_enabled = False
        #if self.dbw_enabled is True:
            #self.V_pid.reset()
            #self.S_pid.reset()

    #def current_velocity_cb(self, msg):
    #    self.current_velocity = msg.twist

    # Vishnerevsky 22.08.2017
    def cur_vel_cb(self, msg):
        self.cur_v = msg.twist.linear.x
        #rospy.logwarn(self.cur_v)

    def current_pose_cb(self, msg):
        self.current_pose = msg.pose

    def final_waypoints_cb(self, msg):
        self.final_waypoints = msg.waypoints

    # Vishnerevsky 26.08.2017
    def velocity_cb(self, msg):
        self.V_reference = msg.data
        #rospy.logwarn(self.V_reference)

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

    # Vishnerevsky 24.08.2017
    def get_CTE(self, waypoints, current_pose):
        # Vishnerevsky 25.05.2017 - Get waypoints coordinates
        points_x = [i.pose.pose.position.x for i in waypoints]
        points_y = [i.pose.pose.position.y for i in waypoints]
        # Decrease length of the lists. We need only 20 - 30 waypoints:
        points_x = points_x[0:30]
        points_y = points_y[0:30]
        # Lest transform our points into the vehicle coordinate system:
        points_x_car = []
        points_y_car = []
        # From quaternion to Euler angles:
        x = current_pose.orientation.x
        y = current_pose.orientation.y
        z = current_pose.orientation.z
        w = current_pose.orientation.w
        # Determine car heading:
        t3 = +2.0 * (w * z + x*y)
        t4 = +1.0 - 2.0 * (y*y + z*z)
        theta = math.degrees(math.atan2(t3, t4))
        # Perform coordinate transformation:
        for i in range(len(points_x)):
            Xcar = (points_y[i]-current_pose.position.y)*math.sin(math.radians(theta))-(current_pose.position.x-points_x[i])*math.cos(math.radians(theta))
            Ycar = (points_y[i]-current_pose.position.y)*math.cos(math.radians(theta))-(points_x[i]-current_pose.position.x)*math.sin(math.radians(theta))
            points_x_car.append(Xcar)
            points_y_car.append(Ycar)
        # Interpolate points in the vehicle coordinate system:
        coeff_xy = list(reversed(np.polyfit(points_x_car, points_y_car, 3)))
        dist_y = 0
        for p, coeff in enumerate(coeff_xy):
            dist_y += coeff * (2.0 ** p)

        return dist_y
        '''
        # We can get points coordinates in this way:
        #points_x = []
        #points_y = []
        #for i in waypoints:
        #    points_x.append(i.pose.pose.position.x)
        #    points_y.append(i.pose.pose.position.y)

        # But in this way it can be done faster:
        points_x = [i.pose.pose.position.x for i in waypoints]
        points_y = [i.pose.pose.position.y for i in waypoints]

        coeff_xy = list(reversed(np.polyfit(points_x, points_y, 2)))
        coeff_yx = list(reversed(np.polyfit(points_y, points_x, 2)))

        dist_y = 0
        for p, coeff in enumerate(coeff_xy):
            dist_y += coeff * (current_pose.position.x ** p)
        b = math.fabs(current_pose.position.y - dist_y)
        dist_x = 0
        for p, coeff in enumerate(coeff_xy):
            dist_x += coeff * (current_pose.position.y ** p)
        a = math.fabs(current_pose.position.x - dist_x)
        # Vishnerevsky 24.08.2017
        # We can add my geometry pictures to write up
        beta = math.atan2(a,b)
        alfa = math.atan2(b,a)

        c1 = b * math.cos(beta)
        c2 = a * math.cos(alfa)

        d1 = math.sqrt(b*b - c1*c1)
        d2 = math.sqrt(a*a - c2*c2)
        d = (d1 + d2)/2.0

        # From quaternion to Euler angles:
        x = current_pose.orientation.x
        y = current_pose.orientation.y
        z = current_pose.orientation.z
        w = current_pose.orientation.w

        # This is not really necessary:
        #ysqr = y*y
        #t0 = +2.0 * (w * x + y*z)
        #t1 = +1.0 - 2.0 * (x*x + ysqr)
        #X = math.degrees(math.atan2(t0, t1))

        #t2 = +2.0 * (w*y - z*x)
        #t2 =  1 if t2 > 1 else t2
        #t2 = -1 if t2 < -1 else t2
        #Y = math.degrees(math.asin(t2))

        t3 = +2.0 * (w * z + x*y)
        t4 = +1.0 - 2.0 * (y*y + z*z)
        Z = math.degrees(math.atan2(t3, t4))

        theta = Z

        # Get one waypoint:
        Xmap = waypoints[10].pose.pose.position.x
        Ymap = waypoints[10].pose.pose.position.y
        # Transform vaypoint coordinate to the car coordinate system:
        #Xcar = (Ymap-current_pose.position.y)*math.sin(math.radians(theta))-(current_pose.position.x-Xmap)*math.cos(math.radians(theta))
        Ycar = (Ymap-current_pose.position.y)*math.cos(math.radians(theta))-(Xmap-current_pose.position.x)*math.sin(math.radians(theta))
        if (Ycar < 0):
            d = d * -1.0
        return d
        '''

if __name__ == '__main__':
    DBWNode()
