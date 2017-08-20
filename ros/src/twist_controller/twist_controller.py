from math import atan2, degrees
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        max_st_angle = kwargs.get('max_st_angle', 8.0)
        pass

    def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        dbw = kwargs.get('dbw', False)
        tx = kwargs.get('tx', 0.0) # TODO change to last position
        ty = kwargs.get('ty', 0.0)
        px = kwargs.get('px', 0.0)
        py = kwargs.get('py', 0.0)
        pt = kwargs.get('pt', 0.0)

        # experimental code
        # simple follower, needs improvements
        # just checks angle with next waypoint passed in input
        # and tries to align the car
        #angle = degrees(atan2(-(ty-py), tx-px))
        #delta = -angle + pt
        #rospy.logwarn('Delta: ' + str(delta) + ' Angle: ' + str(angle) + ' Pt: ' + str(pt))

        if (dbw):
            throttle = 1.0
            brake = 0.0
            steer = 0.0
        else:
            throttle = 1.0
            brake = 0.0
            steer = 0.0

        return throttle, brake, steer
