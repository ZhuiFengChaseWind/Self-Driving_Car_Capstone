
GAS_DENSITY = 2.858
ONE_MPH = 0.44704
from yaw_controller import *

class Controller(object):
    def __init__(self, pid_controller, yaw_controller):
        self.pid_controller = pid_controller
        self.yaw_controller = yaw_controller
        # TODO: Implement
    def control(self, linear_velocity, angular_velocity, current_velocity):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        steer = self.yaw_controller.get_steering(linear_velocity, angular_velocity,
                                                 current_velocity)
        error = linear_velocity - current_velocity
        throttle = self.pid_controller.step(error, 0.002)
        brake = 0.
        if throttle < 0:
            brake = -throttle
            throttle = 0
        if throttle > 1:
            throttle = 1

        return throttle, brake, steer
