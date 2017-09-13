#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
from light_classification.tl_classifier_vlad import TLClassifierVlad
import tf
import cv2
#from traffic_light_config import config # TODO: Need to check if still valid after merge
import math
import numpy as np
import yaml # From Udacity update
import time

STATE_COUNT_THRESHOLD = 1

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=1)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and 
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        ''' Commented to not use
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        '''
        # Udacity changed from /camera/image_raw
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb, queue_size=1)
        config_string = rospy.get_param("/traffic_light_config") # From Udacity update
        self.config = yaml.load(config_string) # From Udacity update

        # Edited by Vishnerevsky 29.08.2017
        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', TrafficLight, queue_size=1)
		# TODO: Need to check if still valid after merge
		# Udacity changes to int32
		# self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.tl_tx = TrafficLight()
        self.deb_img = rospy.Publisher('/deb_img', Image, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.light_classifier_vlad = TLClassifierVlad()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 11

        self.best_waypoint = 0
        self.last_car_position = 0
        self.last_light_pos_wp = []

        # Vishnerevsky 27.08.2017:
        self.distance_to_light = 1000.0
        self.deviation_of_light = 0.0

        # Vishnerevsky 10.09.2017:
        self.ImageCounter = 0
        self.time = None
        self.TL_GT_State = None
        sub_123 = rospy.Subscriber('traffic_light_GT', Int32, self.tl_gt_cb)

        self.IGNORE_FAR_LIGHT = 100.0

        sub_bagfile = rospy.Subscriber('/image_raw', Image, self.image_cb_bag, queue_size=1)

        rospy.spin()

    # Vishnerevsky 10.09.2017:
    def tl_gt_cb(self, msg):
        self.TL_GT_State = msg.data
        #rospy.logerr(self.TL_GT_State)

    def image_cb_bag(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        res = self.light_classifier.get_classification(cv_image)
        if (res == 0):
            rospy.loginfo('Rosbag: RED')
        elif (res == 1):
            rospy.loginfo('Rosbag: Yellow')
        elif (res == 2):
            rospy.loginfo('Rosbag: Green')
        else:
            rospy.loginfo('Rosbag: Unknown')

        self.deb_img.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        return res

    def pose_cb(self, msg):
        self.pose = msg
        if (self.camera_image is None):
            return
        light_wp, state = self.process_traffic_lights()

        if (state == 0):
            rospy.loginfo('Simul: RED')
        elif (state == 1):
            rospy.loginfo('Simul: Yellow')
        elif (state == 2):
            rospy.loginfo('Simul: Green')
        else:
            rospy.loginfo('Simul: Unknown')

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        #rospy.logerr('Lwp: ' + str(light_wp) + ' state:' + str(state))
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            #self.upcoming_red_light_pub.publish(Int32(state))
            if (light_wp >=0):
                self.tl_tx.pose.pose.position.x = self.waypoints.waypoints[self.last_wp].pose.pose.position.x
                self.tl_tx.pose.pose.position.y = self.waypoints.waypoints[self.last_wp].pose.pose.position.y
                self.tl_tx.pose.pose.position.z = self.waypoints.waypoints[self.last_wp].pose.pose.position.z
            self.tl_tx.state = state
            self.upcoming_red_light_pub.publish(self.tl_tx)
        else:
            self.tl_tx.state = self.state
            self.upcoming_red_light_pub.publish(self.tl_tx)
        self.state_count += 1
        '''
        # Vishnerevsky 30.08.2017: Treshold for 0 state
        if state == 0:
            self.state = state
            self.state_count = 0
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            if (light_wp >=0):
                self.tl_tx.pose.pose.position.x = self.waypoints.waypoints[self.last_wp].pose.pose.position.x
                self.tl_tx.pose.pose.position.y = self.waypoints.waypoints[self.last_wp].pose.pose.position.y
                self.tl_tx.pose.pose.position.z = self.waypoints.waypoints[self.last_wp].pose.pose.position.z
            self.tl_tx.state = state
            self.upcoming_red_light_pub.publish(self.tl_tx)
        else:
            if self.state_count > STATE_COUNT_THRESHOLD:
                self.state = state
                light_wp = light_wp if state == TrafficLight.RED else -1
                self.last_wp = light_wp
                if (light_wp >=0):
                    self.tl_tx.pose.pose.position.x = self.waypoints.waypoints[self.last_wp].pose.pose.position.x
                    self.tl_tx.pose.pose.position.y = self.waypoints.waypoints[self.last_wp].pose.pose.position.y
                    self.tl_tx.pose.pose.position.z = self.waypoints.waypoints[self.last_wp].pose.pose.position.z
                self.tl_tx.state = self.state
                self.upcoming_red_light_pub.publish(self.tl_tx)
                self.state_count += 1
            else:
                self.state = 0
                light_wp = light_wp if state == TrafficLight.RED else -1
                self.last_wp = light_wp
                if (light_wp >=0):
                    self.tl_tx.pose.pose.position.x = self.waypoints.waypoints[self.last_wp].pose.pose.position.x
                    self.tl_tx.pose.pose.position.y = self.waypoints.waypoints[self.last_wp].pose.pose.position.y
                    self.tl_tx.pose.pose.position.z = self.waypoints.waypoints[self.last_wp].pose.pose.position.z
                self.tl_tx.state = self.state
                self.upcoming_red_light_pub.publish(self.tl_tx)
                self.state_count += 1


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

        fx = self.config['camera_info']['focal_length_x']
        fy = self.config['camera_info']['focal_length_y']
        image_width = self.config['camera_info']['image_width']
        image_height = self.config['camera_info']['image_height']

        cord_x = point_in_world[0]
        cord_y = point_in_world[1]


        #rospy.logerr("x: " + str(cord_x) + " y: " + str(cord_y))
        #rospy.logerr("fx: " + str(fx) + " fy: " + str(fy))
        #rospy.logerr("image_width: " + str(image_width) + " image_height: " + str(image_height))

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

        '''
        convert light to car cordinates
        '''
        # From quaternion to Euler angles:
        x = self.pose.pose.orientation.x
        y = self.pose.pose.orientation.y
        z = self.pose.pose.orientation.z
        w = self.pose.pose.orientation.w
        # Determine car heading:
        t3 = +2.0 * (w * z + x*y)
        t4 = +1.0 - 2.0 * (y*y + z*z)
        theta = math.degrees(math.atan2(t3, t4))

        Xcar = (cord_y-self.pose.pose.position.y)*math.sin(math.radians(theta))-(self.pose.pose.position.x-cord_x)*math.cos(math.radians(theta))
        Ycar = (cord_y-self.pose.pose.position.y)*math.cos(math.radians(theta))-(cord_x-self.pose.pose.position.x)*math.sin(math.radians(theta))

        # Vishnerevsky 27.08.2017:
        self.distance_to_light = Xcar
        self.deviation_of_light = Ycar

        #rospy.logerr("trans: " + str(trans) + " rot: " + str(rot))
        #rospy.logerr("Xcar: " + str(Xcar) + " Ycar: " + str(Ycar)) #Commented by Vishnerevsky 27.08.2017

        #https://www.scratchapixel.com/lessons/3d-basic-rendering/computing-pixel-coordinates-of-3d-point/mathematics-computing-2d-coordinates-of-3d-points
        #camX = float(Ycar)
        #camY = 0.0
        #camZ = float(-Xcar)
        objectPoints = np.array([[float(Xcar), float(Ycar), 0.0]], dtype=np.float32)
        #objectPoints = np.array([[float(Ycar), 0, float(Xcar)]]) #-5.868888
        #objectPoints = np.array([[point_in_world[0], point_in_world[1], 5.868888]]) # TODO: Check if this is valid in real scenario
        # taken from: rostopic echo /vehicle/traffic_lights

        #rvec = tf.transformations.quaternion_matrix(rot)[:3, :3]
        #tvec = np.array(trans)
        rvec = (0,0,0)
        tvec = (0,0,0)

        cameraMatrix = np.array([[fx,  0, image_width/2],
                                [ 0, fy, image_height/2],
                                [ 0,  0,  1]])
        distCoeffs = None

        #rospy.logerr("objectPoints: " + str(objectPoints))
        #rospy.logerr("rvec: " + str(rvec))
        #rospy.logerr("tvec: " + str(tvec))
        #rospy.logerr("cameraMatrix: " + str(cameraMatrix))

        ret, _ = cv2.projectPoints(objectPoints, rvec, tvec, cameraMatrix, distCoeffs)

        x = int(ret[0,0,0])
        y = int(ret[0,0,1])
        #rospy.logerr("x: " + str(x) + " y: " + str(y))

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

        #TODO use light location to zoom in on traffic light in image
        if ((x is None) or (y is None) or (x < 0) or (y<0) or
            (x>self.config['camera_info']['image_width']) or (y>self.config['camera_info']['image_height'])):
            return TrafficLight.UNKNOWN
        else:
            #rospy.loginfo('shape' + str(cv_image.shape))
            #left = max(0, self.config['camera_info']['image_width']/2 - int(self.deviation_of_light)*10-200) # Vishnerevsky 27.08.2017
            #right = min(self.config['camera_info']['image_width']/2 - int(self.deviation_of_light)*10+200, self.config['camera_info']['image_width']) # Vishnerevsky 27.08.2017

            #vertical = int(y - (45.0-self.distance_to_light)*150.0/35.0)
            #if vertical<0:
            #    vertical = 0
            #top = min(vertical, self.config['camera_info']['image_height']-1)
            #bottom = min(300 + vertical, self.config['camera_info']['image_height'])
            #crop = cv_image[top:bottom, left:right]
            # Vishnerevsky 27.08.2017
            #crop = cv_image
            #left = 67
            #right = left+137*5
            #top = 0
            #bottom = 65*5
            #crop = cv_image[top:bottom, left:right]
            #crop = cv2.resize(crop,(137, 65), interpolation = cv2.INTER_CUBIC)

            self.time = time.clock() 
            #rospy.logwarn(int(self.time*1000000))
            #if (self.TL_GT_State == 0):
            #    cv2.imwrite('SavedImages/red/' + str(int(self.time*1000000)) + '.jpg', cv_image)
            #    rospy.logerr('NOT SAVED TO RED')
            #if (self.TL_GT_State == 1):
            #    cv2.imwrite('SavedImages/yellow/' + str(int(self.time*1000000)) + '.jpg', cv_image)
            #    rospy.logerr('SAVED TO YELLOW')
            #if (self.TL_GT_State == 2):
            #    cv2.imwrite('SavedImages/green/' + str(int(self.time*1000000)) + '.jpg', cv_image)
            #    rospy.logerr('NOT SAVED TO GREEN')

            # Cropped for Vladimir's trained simulaotr images
            crop = cv2.resize(cv_image,(300, 200), interpolation = cv2.INTER_CUBIC)

            self.deb_img.publish(self.bridge.cv2_to_imgmsg(crop, "bgr8"))
            #rosrun image_view image_view image:=/deb_img                      #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        #Get classification
        return self.light_classifier_vlad.get_classification(crop)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None

        #light_positions = config.light_positions  # TODO: Need to check if still valid after merge
		# The line above is our old config, seems like udacity changed the light publishing
        light_positions = self.config['light_positions'] # This is Udacioty code

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

        # Valtgun 29.08.2017 added distance check to light
        light_distance = self.distance_light(light, self.waypoints.waypoints[self.last_car_position].pose.pose.position)
        #rospy.logerr('light_distance: ' + str(light_distance))

        if light:
            if (light_distance >= self.IGNORE_FAR_LIGHT):
                return -1, TrafficLight.UNKNOWN
            else:
                state = self.get_light_state(light)
                return light_num_wp, state # changed from light to light_num_wp to return waypoint no.
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

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
