#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose, PointStamped
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
from traffic_light_config import config
import math
import numpy as np

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights helps you acquire an accurate ground truth data source for the traffic light
        classifier, providing the location and current color state of all traffic lights in the
        simulator. This state can be used to generate classified images or subbed into your solution to
        help you work on another single component of the node. This topic won't be available when
        testing your solution in real life so don't rely on it in the final submission.
        '''
        ''' Commented to not use
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        '''
        sub6 = rospy.Subscriber('/camera/image_raw', Image, self.image_cb)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.deb_img = rospy.Publisher('/deb_img', Image, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        self.best_waypoint = 0
        self.last_car_position = 0
        self.last_light_pos_wp = []

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

    ''' Commented to not use
    def traffic_cb(self, msg):
        self.lights = msg.lights
    '''
    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light to /traffic_waypoint

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
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        best_waypoint = self.best_waypoint
        if self.waypoints is not None:
            waypoints = self.waypoints.waypoints
            min_dist = self.distance(pose.position, waypoints[0].pose.pose.position)
            for i, point in enumerate(waypoints):
                dist = self.distance(pose.position, point.pose.pose.position)
                if dist < min_dist:
                    best_waypoint = i
                    min_dist = dist
            self.best_waypoint = best_waypoint
            return best_waypoint

    def distance(self, p1, p2):
        delta_x = p1.x - p2.x
        delta_y = p1.y - p2.y
        return math.sqrt(delta_x*delta_x + delta_y*delta_y)

    def project_to_image_plane(self, point_in_world):
        """Project point from 3D world coordinates to 2D camera image location

        Args:
            point_in_world (Point): 3D location of a point in the world

        Returns:
            x (int): x coordinate of target point in image
            y (int): y coordinate of target point in image

        """

        fx = config.camera_info.focal_length_x
        fy = config.camera_info.focal_length_y

        image_width = config.camera_info.image_width
        image_height = config.camera_info.image_height


        # get transform between pose of camera and world frame
        trans = None
        try:
            now = rospy.Time.now()
            self.listener.waitForTransform("/base_link",
                  "/world", now, rospy.Duration(1.0))
            (trans, rot) = self.listener.lookupTransform("/base_link",
                  "/world", now)

        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            rospy.logerr("Failed to find camera to map transform")

        #TODO Use trasnform and rotation to calculate 2D position of light in image
        #http://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
        #cv2.projectPoints(objectPoints, rvec, tvec, cameraMatrix, distCoeffs[, imagePoints[, jacobian[, aspectRatio]]]) -> imagePoints, jacobian
        #Projects 3D points to an image plane
        #https://www.scratchapixel.com/lessons/3d-basic-rendering/computing-pixel-coordinates-of-3d-point/mathematics-computing-2d-coordinates-of-3d-points

        objectPoints = np.array([[point_in_world[0], point_in_world[1], 5.868888]]) # TODO: Check if this is valid in real scenario
        # taken from: rostopic echo /vehicle/traffic_lights

        rvec = tf.transformations.quaternion_matrix(rot)[:3, :3]
        tvec = np.array(trans)
        #rvec = (0,0,0)
        #tvec = (0,0,0)

        cameraMatrix = np.array([[fx,  0, image_width/2],
                                [ 0, fy, image_height/2],
                                [ 0,  0,  1]])
        distCoeffs = None

        rospy.logerr("image_width: " + str(image_width) + "image_height: " + str(image_height))
        #rospy.logerr("objectPoints: " + str(objectPoints))
        #rospy.logerr("rvec: " + str(rvec))
        #rospy.logerr("tvec: " + str(tvec))
        #rospy.logerr("cameraMatrix: " + str(cameraMatrix))

        ret, _ = cv2.projectPoints(objectPoints, rvec, tvec, cameraMatrix, distCoeffs)
        # ret, _ = cv2.projectPoints(objectPoints, rvec, tvec, cameraMatrix, distCoeffs)

        x = int(ret[0,0,0])
        y = int(ret[0,0,1])
        rospy.logerr("x: " + str(x) + " y: " + str(y))

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

        self.camera_image.encoding = "rgb8"
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        x, y = self.project_to_image_plane(light)
        rospy.logwarn("cv_image: " + str(cv_image.shape))
        #TODO use light location to zoom in on traffic light in image
        if ((x is None) or (y is None) or (x < 0) or (y<0) or
            (x>config.camera_info.image_width) or (y>config.camera_info.image_height)):

            self.deb_img.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            return TrafficLight.UNKNOWN
        else:
            left = max(0, x - 50)
            right = min(config.camera_info.image_width, x + 50)
            top = max(0, y - 50)
            bottom = min(config.camera_info.image_height, y + 50)
            #crop = cv_image[top:bottom, left:right]
            crop = cv_image[:, :, :]

            cv2.circle(crop, (x, y), radius=20, color=(255, 0, 0), thickness=4)

            self.deb_img.publish(self.bridge.cv2_to_imgmsg(crop, "bgr8"))
            #rosrun image_view image_view image:=/deb_img
        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None
        light_positions = config.light_positions

        # Valtgun attribute lights to waypoints
        light_pos_wp = []
        if self.waypoints is not None:
            wp = self.waypoints
            for i in range(len(light_positions)):
                l_pos = self.get_closest_waypoint_light(wp, light_positions[i])
                light_pos_wp.append(l_pos)
            self.last_light_pos_wp = light_pos_wp
        else:
            light_pos_wp = self.last_light_pos_wp

        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose)
            if car_position is not None:
                self.last_car_position = car_position

        # valtgun get id of next light
        if (self.last_car_position > max(light_pos_wp)):
             light_num_wp = min(light_pos_wp)
        else:
            light_delta = light_pos_wp[:]
            light_delta[:] = [x - self.last_car_position for x in light_delta]
            light_num_wp = min(i for i in light_delta if i > 0) + self.last_car_position

        light_idx = light_pos_wp.index(light_num_wp)
        light = light_positions[light_idx]

        if light:
            state = self.get_light_state(light)
            return light, state
        self.waypoints = None
        return -1, TrafficLight.UNKNOWN

    def get_closest_waypoint_light(self, wp, l_pos):
        best_waypoint = None
        waypoints = wp.waypoints
        min_dist = self.distance_light(l_pos, waypoints[0].pose.pose.position)
        for i, point in enumerate(waypoints):
            dist = self.distance_light(l_pos, point.pose.pose.position)
            if dist < min_dist:
                best_waypoint = i
                min_dist = dist
        return best_waypoint

    def distance_light(self, l_p, wp):
        delta_x = l_p[0] - wp.x
        delta_y = l_p[1] - wp.y
        return math.sqrt(delta_x*delta_x + delta_y*delta_y)

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
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
