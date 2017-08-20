
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
        if (dbw):
            throttle = 1.0
            brake = 0.0
            steer = 0.0
        else:
            throttle = 1.0
            brake = 0.0
            steer = 0.0

        return throttle, brake, steer
