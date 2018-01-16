#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped, Twist
import math

from twist_controller import Controller
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        # Minimum speed at which we steer the vehicle
        min_speed = 0.0
        
        # Throttle/Brake PID parameters
        k_p = 0.5
        k_i = 0.0
        k_d = 0.1
	
        tau = 0.2
        ts = 0.1

	self.dbw_enabled = False # subscribe from /vehicle/dbw_enabled
	self.target_velocity = Twist() # subscribe from /twist_cmd
	self.current_velocity = Twist() # subsribe from /current_velocity
	self.previous_throttle = 0
	self.previous_steering = 0
	self.previous_brake = 0

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd', SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd', ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd', BrakeCmd, queue_size=1)

        # TODO: Create `TwistController` object
        yaw_controller = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)
        pid_controller = PID(k_p, k_i, k_d, decel_limit, accel_limit)
        low_pass_filter = LowPassFilter(tau, ts)

        self.controller = Controller(yaw_controller, pid_controller, low_pass_filter, vehicle_mass, fuel_capacity, wheel_radius)

        # TODO: Subscribe to all the topics you need to
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cmd_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb)

        self.loop()

    def loop(self):
        rate = rospy.Rate(50) # 50Hz
        while not rospy.is_shutdown():
            # TODO: Get predicted throttle, brake, and steering using `twist_controller`
            # You should only publish the control commands if dbw is enabled

            throttle, brake, steering = self.controller.control(self.target_velocity.linear,
                                                                self.target_velocity.angular,
                                                                self.current_velocity.linear,
                                                                self.dbw_enabled) #,
                                                                # <any other argument you need>)

            if self.dbw_enabled:
               self.publish(throttle, brake, steering)

            rate.sleep()

    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = (abs(throttle - self.previous_throttle) > 0.05)
        if tcmd.enable :
	    self.previous_throttle = throttle
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = (abs(throttle - self.previous_steering) > 0.05)
        if scmd.enable :
            self.previous_steer = steer
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)
            
        bcmd = BrakeCmd()
        bcmd.enable = (abs(brake - self.previous_brake) > 0.05)
        if bcmd.enable :
            self.previous_brake = brake
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)

    def twist_cmd_cb(self, msg):
        # TODO: Implement
	
        self.target_velocity = msg.twist
        pass

    def current_velocity_cb(self, msg):
        # TODO: Implement
        self.current_velocity = msg.twist
        pass

    def dbw_enabled_cb(self, msg):
        self.dbw_enabled = msg.data
        pass

if __name__ == '__main__':
    try:    
	DBWNode()
    except rospy.ROSInterruptException:
	rospy.logerr('Error: Not start dbw_node!')
