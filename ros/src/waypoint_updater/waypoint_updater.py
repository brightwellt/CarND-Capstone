#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
from tf.transformations import euler_from_quaternion

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 20 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.waypoints = []
        self.prev_waypoint_index = -1
        self.prev_final_waypoints = []
        self.traffic_waypoint = -1

        self.velocity = self.kmph2mps(rospy.get_param('/waypoint_loader/velocity'))
        self.wheel_base = rospy.get_param('/dbw_node/wheel_base', 2.8498)

        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
        self.pose = msg.pose

        if len(self.waypoints) > 0:

            # Diagnostic message indicating position of next red traffic light
            #rospy.logwarn("WU: Car at waypoint %s, red traffic light at waypoint %s", self.prev_waypoint_index, self.traffic_waypoint)
            
            # Find closest waypoint
            if self.prev_waypoint_index > -1:
                # If we already know roughly where we are, search locally (much faster)                
                closest_waypoint_index = self.closest_local_waypoint(self.waypoints, self.pose, self.prev_waypoint_index)
            else:
                # We don't know where we are, search the full set of waypoints
                closest_waypoint_index = self.closest_waypoint(self.waypoints, self.pose)
            self.prev_waypoint_index = closest_waypoint_index

            # Create final_waypoints
            final_waypoints = []
            for i in range(LOOKAHEAD_WPS):

                # Add waypoint to list
                final_waypoints.append(self.waypoints[closest_waypoint_index])
                
                # Calculate speed at waypoint based on traffic lights
                if self.traffic_waypoint >= 0:
                    dist = self.distance(self.waypoints, closest_waypoint_index, self.traffic_waypoint)
                    target_vel = min(max(0, (dist - self.wheel_base) / 2), self.velocity)
                    self.set_waypoint_velocity(final_waypoints, i, target_vel)
                else:
                    self.set_waypoint_velocity(final_waypoints, i, self.velocity)

                # Move on to next waypoint
                closest_waypoint_index = closest_waypoint_index + 1
                if (closest_waypoint_index >= len(self.waypoints)):
                    closest_waypoint_index = 0
            self.prev_final_waypoints = final_waypoints

            # Publish final_waypoints
            fwcmd = Lane()
            fwcmd.header = msg.header
            fwcmd.waypoints = final_waypoints
            self.final_waypoints_pub.publish(fwcmd)
        pass

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.waypoints = waypoints.waypoints
        pass

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.traffic_waypoint = msg.data
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def closest_waypoint(self, waypoints, vehicle_pose):
        # Calculate closest waypoint to current vehicle position
        closest_dist = 1000000
        closest_index = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)
        for i in range(1, len(waypoints)):
            dist = dl(waypoints[i].pose.pose.position, vehicle_pose.position)
            if dist < closest_dist:
                closest_index = i
                closest_dist = dist
        return closest_index

    def closest_local_waypoint(self, waypoints, vehicle_pose, last_waypoint):
        # Calculate closest waypoint local to previous closest waypoint
        # NOTE: Assumes waypoints are sequential (but not the direction of vehicle travel)
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)
        closest_dist = dl(waypoints[last_waypoint].pose.pose.position, vehicle_pose.position)
        closest_waypoint = last_waypoint

        # Search forwards
        next_waypoint = last_waypoint + 1 if last_waypoint + 1 < len(waypoints) else 0
        while next_waypoint != closest_waypoint:
            dist = dl(waypoints[next_waypoint].pose.pose.position, vehicle_pose.position)
            if dist < closest_dist:
                closest_dist = dist
                closest_waypoint = next_waypoint
                next_waypoint = next_waypoint + 1 if next_waypoint + 1 < len(waypoints) else 0
            else:
                break

        # Search backwards
        prev_waypoint = last_waypoint - 1 if last_waypoint - 1 >= 0 else len(waypoints) - 1
        while prev_waypoint != closest_waypoint:
            dist = dl(waypoints[prev_waypoint].pose.pose.position, vehicle_pose.position)
            if dist < closest_dist:
                closest_dist = dist
                closest_waypoint = prev_waypoint
                prev_waypoint = prev_waypoint - 1 if prev_waypoint - 1 >= 0 else len(waypoints) - 1
            else:
                break

        return closest_waypoint

    def get_yaw(self, pose):
        # Convert from Quaternion to Euler angles
        orientation_q = pose.orientation 
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        return yaw

    def kmph2mps(self, velocity_kmph):
        return (velocity_kmph * 1000.) / (60. * 60.)

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
