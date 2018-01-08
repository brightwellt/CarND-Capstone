
import rospy


GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, yaw_controller):
        # TODO: Implement
        self.yaw_controller = yaw_controller
        pass

    def control(self, linear_velocity, angular_velocity, current_velocity, dbw_enabled):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        steer = self.yaw_controller.get_steering(linear_velocity.x, angular_velocity.z, current_velocity.x)

	if current_velocity.x > linear_velocity.x:
            throttle = 0.
        else:
            throttle = 1.

        return throttle, 0., 2 * steer
