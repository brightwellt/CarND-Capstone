import rospy


GAS_DENSITY = 2.858
ONE_MPH = 0.44704



class Controller(object):
    def __init__(self, yaw_controller, pid_controller):
        # TODO: Implement
        self.yaw_controller = yaw_controller
        self.pid_controller = pid_controller
        pass

    def control(self, linear_velocity, angular_velocity, current_velocity, dbw_enabled):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        if (dbw_enabled == False):
            self.pid_controller.reset()
            return 0, 0, 0

        acceleration = self.pid_controller.step(linear_velocity.x - current_velocity.x, 1/50.0) 
        
        if (acceleration > 0):
            throttle = acceleration
            brake = 0
        else:
            throttle = 0
            brake = -acceleration * 2000

        steer = self.yaw_controller.get_steering(linear_velocity.x, angular_velocity.z, current_velocity.x)

        return throttle, brake, 2 * steer
