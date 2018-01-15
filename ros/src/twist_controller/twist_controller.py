import rospy


GAS_DENSITY = 2.858
ONE_MPH = 0.44704



class Controller(object):
    def __init__(self, yaw_controller, pid_controller, low_pass_filter, vehicle_mass, fuel_capacity, wheel_radius):
        # TODO: Implement
        self.yaw_controller = yaw_controller
        self.pid_controller = pid_controller
	self.low_pass_filter = low_pass_filter
	self.previous_time = None
        # Torque = Force * Perpendicular Distance
        # Force  = Mass * Acceleration
        # Torque Factor = Mass * Perpendicular Distance
        self.torque_factor = (vehicle_mass + fuel_capacity * GAS_DENSITY) * wheel_radius
        pass

    def sample_time(self, previous_time):
        current_time = rospy.get_time()
        if (previous_time == None):
	    sample_elapsed = 1 / 50.0
        else:
	    sample_elapsed = current_time - previous_time
	    previous_time = current_time
        return sample_elapsed

    def control(self, linear_velocity, angular_velocity, current_velocity, dbw_enabled):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        if (dbw_enabled == False):
            self.pid_controller.reset()
            return 0, 0, 0
	
        sample_step = self.sample_time(self.previous_time)

        acceleration = self.pid_controller.step(linear_velocity.x - current_velocity.x, sample_step) 
        acceleration = self.low_pass_filter.filt(acceleration)
        
        if (acceleration > 0):
            throttle = acceleration
            brake = 0.0
        else:
            throttle = 0.0
            brake = -acceleration * self.torque_factor

        steer = self.yaw_controller.get_steering(linear_velocity.x, angular_velocity.z, current_velocity.x)

        return throttle, brake, steer
