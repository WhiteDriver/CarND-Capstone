from styx_msgs.msg import TrafficLight

class TLClassifier(object):
    def __init__(self):
        pass

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
            0 (red / yellow) or 2 (green)
            Not implented returns: 1 (yellow), 4 (unknown)
        """
        
        # Manually set boundaries for colors
        boundaries = [
            ([0, 0, 80], [50, 130, 255]), # Red
            ([200, 200, 40], [255, 255, 140]), # Green
        ]

        results = [0, 0]
        bd_count = 0

        for (lower, upper) in boundaries:
            lower = np.array(lower, dtype = "uint8")
            upper = np.array(upper, dtype = "uint8")

            # Check for boundary colors in images
            mask = cv2.inRange(img, lower, upper)
            output = cv2.bitwise_and(img, img, mask=mask)

            # Sum the masks and save to result array
            # (E.g Red light should have more red than green in image)
            results[bd_count] = np.sum(output)
            bd_count += 1

        state = np.array(results).argmax()   # Returns index of max()
        state += 1 if state == 1 else 0     # Since green is valued 2
        return state
