import rospy
from styx_msgs.msg import TrafficLight
import numpy as np
import cv2
import tensorflow as tf
from tensorflow.contrib.layers import flatten


class TLClassifier(object):
    def __init__(self):
        '''
        self.pixel_threshold = 8 # Thresholding value for colored pixels

        # Manually set boundaries for colors
        self.boundaries = [
            ([140, 60, 150], [180, 140, 255]), # Red
        ]
        '''
        self.x = tf.placeholder(tf.float32, (None, 65, 137, 3))
        self.y = tf.placeholder(tf.int32, (None))
        #one_hot_y = tf.one_hot(y_train, 4)
        self.logits = self.LeNet(tf.cast(self.x, tf.float32))
        self.save_file = '/home/valtgun/abc/CarND-Capstone/ros/src/tl_detector/light_classification/save/model.ckpt'
        self.saver = tf.train.Saver()
        self.init = tf.global_variables_initializer()
        self.session = tf.Session()
        self.saver.restore(self.session, self.save_file)

    def LeNet(self, x):
        # Hyperparameters
        mu = 0
        sigma = 0.1

        # Layer 1: Convolutional. Input = 32x32x3. Output = 28x28x6.
        conv1_W = tf.Variable(tf.truncated_normal(shape=(5, 5, 3, 3), mean = mu, stddev = sigma))
        conv1_b = tf.Variable(tf.zeros(3))
        conv1   = tf.nn.conv2d(x, conv1_W, strides=[1, 1, 1, 1], padding='VALID') + conv1_b

        # Activation.
        conv1 = tf.nn.relu(conv1)

        # Pooling. Input = 28x28x6. Output = 14x14x6.
        conv1 = tf.nn.max_pool(conv1, ksize=[1, 2, 2, 1], strides=[1, 2, 2, 1], padding='VALID')

        # Layer 2: Convolutional. Output = 10x10x16.
        conv2_W = tf.Variable(tf.truncated_normal(shape=(5, 5, 3, 5), mean = mu, stddev = sigma))
        conv2_b = tf.Variable(tf.zeros(5))
        conv2   = tf.nn.conv2d(conv1, conv2_W, strides=[1, 1, 1, 1], padding='VALID') + conv2_b

        # Activation.
        conv2 = tf.nn.relu(conv2)

        # Pooling. Input = 10x10x16. Output = 5x5x16.
        conv2 = tf.nn.max_pool(conv2, ksize=[1, 2, 2, 1], strides=[1, 2, 2, 1], padding='VALID')

        # Flatten. Input = 5x5x16. Output = 400.
        fc0   = flatten(conv2)

        # Layer 3: Fully Connected. Input = 400. Output = 200.
        fc1_W = tf.Variable(tf.truncated_normal(shape=(2015, 50), mean = mu, stddev = sigma))
        fc1_b = tf.Variable(tf.zeros(50))
        fc1   = tf.matmul(fc0, fc1_W) + fc1_b

        # Activation.
        fc1    = tf.nn.relu(fc1)

        # Layer 4: Fully Connected. Input = 200. Output = 150.
        fc2_W  = tf.Variable(tf.truncated_normal(shape=(50, 25), mean = mu, stddev = sigma))
        fc2_b  = tf.Variable(tf.zeros(25))
        fc2    = tf.matmul(fc1, fc2_W) + fc2_b

        # Activation.
        fc2    = tf.nn.relu(fc2)

        # Layer 5: Fully Connected. Input = 150. Output = 10.
        fc3_W  = tf.Variable(tf.truncated_normal(shape=(25, 4), mean = mu, stddev = sigma))
        fc3_b  = tf.Variable(tf.zeros(4))
        logits = tf.matmul(fc2, fc3_W) + fc3_b
        return logits

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
            0 for red, 4 for green/yellow/none
        """

        # Set default state
        state = TrafficLight.UNKNOWN
        '''
        for (lower, upper) in self.boundaries:
            lower = np.array(lower, dtype = "uint8")
            upper = np.array(upper, dtype = "uint8")

            # Count boundary colors in image
            hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv_image, lower, upper)
            color_detection = cv2.countNonZero(mask)

            # Threshold detected red colors
            if color_detection > self.pixel_threshold:
                state = TrafficLight.RED
        '''
        # Beginning of imported code from Vladimir's neural network
        # resize and prepare image
        res = None
        if (image.shape[0] == 1096 and image.shape[1] == 1368): # rosbag
            image = image[100:image.shape[0]-350, 0:image.shape[1]]
            res = cv2.resize(image,None,fx=0.1, fy=0.1, interpolation = cv2.INTER_CUBIC)
        elif (image.shape[0] == 325 and image.shape[1] == 685): # simulator cropped
            res = cv2.resize(image,None,fx=0.2, fy=0.2, interpolation = cv2.INTER_CUBIC)
        else:
            return state

        inp_img = res.reshape(1, 65, 137, 3)
        out_logits = self.session.run(self.logits, feed_dict={self.x: inp_img})
        out_idx = np.argmax(out_logits)
        # Convert from logits to traffic light colors
        if (out_idx == 0):
            state = TrafficLight.RED
        elif (out_idx == 2):
            state = TrafficLight.YELLOW
        elif (out_idx == 1):
            state = TrafficLight.GREEN

        return state
