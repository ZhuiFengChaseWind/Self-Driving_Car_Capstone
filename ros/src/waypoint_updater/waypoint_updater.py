#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.base_waypoints = None

        rospy.spin()


    def next_waypoint(self, position):
        waypoints_list = self.base_waypoints.waypoints
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        min_dist = 10000
        nearest_waypoint = -1
        for i in range(len(waypoints_list)):
            waypoint_position = waypoints_list[i].pose.pose.position
            dist = dl(position, waypoint_position)
            if dist < min_dist:
                min_dist = dist
                nearest_waypoint = i
        nearest_waypoint_position = waypoints_list[nearest_waypoint].pose.pose.position
        rospy.loginfo('nearest_waypoint: %s, %s, %s, %s', nearest_waypoint, nearest_waypoint_position.x,
        nearest_waypoint_position.y, nearest_waypoint_position.z)
        next_to_nearest = (nearest_waypoint + 1) % len(waypoints_list)
        next_nearest_wp = waypoints_list[next_to_nearest].pose.pose.position
        rospy.loginfo('next_to_nearest_waypoint: %s, %s, %s, %s', next_to_nearest, next_nearest_wp.x,
        next_nearest_wp.y, next_nearest_wp.z)

        next_x = waypoints_list[next_to_nearest].pose.pose.position.x
        nearest_x = waypoints_list[nearest_waypoint].pose.pose.position.x
        next_y = waypoints_list[next_to_nearest].pose.pose.position.y
        nearest_y = waypoints_list[nearest_waypoint].pose.pose.position.y
        px = position.x
        py = position.y
        if (next_x - nearest_x) * (px - nearest_x) + (next_y - nearest_y) * (py -
            nearest_y) > 0:
            return next_to_nearest
        return nearest_waypoint 

    def pose_cb(self, msg):
        # TODO: Implement
        position = msg.pose.position
        rospy.loginfo("current_car_position: %s, %s, %s", position.x, position.y, position.z)
        next_wp = self.next_waypoint(position)
        next_wp_p = self.base_waypoints.waypoints[next_wp].pose.pose.position
        rospy.loginfo('next_waypoint: %s, %s, %s', next_wp_p.x, next_wp_p.y,
        next_wp_p.z)
        base_waypoints_length = len(self.base_waypoints.waypoints)
        ahead_waypoints = []
        for i in range(LOOKAHEAD_WPS):
            ahead_waypoints.append(self.base_waypoints.waypoints[(next_wp + i) % base_waypoints_length])
        lane = Lane()
        lane.waypoints = ahead_waypoints
        self.final_waypoints_pub.publish(lane)

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        # base waypoints is only published once, so it should be stored here
        self.base_waypoints = waypoints

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
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


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
