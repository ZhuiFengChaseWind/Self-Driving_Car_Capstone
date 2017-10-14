
GAS_DENSITY = 2.858
ONE_MPH = 0.44704
from yaw_controller import *

class Controller(object):
    def __init__(self, pid_controller, yaw_controller, low_pass_filter):
        self.pid_controller = pid_controller
        self.yaw_controller = yaw_controller
	self.low_pass_filter = low_pass_filter
	
	self.BRAKE=1.5
	self.THROTTLE=2.0
	self.THROTTLE_MAX = 0.95
	self.BRAKE_MAX = 0.80

        # TODO: Implement
    def control(self, linear_velocity, angular_velocity, current_velocity):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        steer = self.yaw_controller.get_steering(linear_velocity, angular_velocity,
                                                 current_velocity)
        error = linear_velocity - current_velocity
        throttle = self.pid_controller.step(error, 0.02)
        brake = 0.
        if throttle < 0:
	    brake = -throttle
            #brake = soft_scale(-throttle, self.MAX_BRAKE, self.BRAKE)
            throttle = 0
        if throttle >= 1.0:
            #throttle = soft_scale(throttle, self.MAX_THROTTLE, self.THROTTLE)
            throttle = 1.0
        return throttle, brake, steer

    def soft_scale(self, value, scale, shrinkage):
	""" smooth value using tanh function, also achieve the bounding effects
	"""
	if value<=0.0:
	    return 0.0
	return scale*math.tanh(value*shrinkage)
        
