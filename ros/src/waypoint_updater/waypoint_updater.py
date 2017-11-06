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
'''
rostopic list
/base_waypoints
/current_pose
/current_velocity
/final_waypoints
/image_color
/rosout
/rosout_agg
/tf
/traffic_waypoint
/twist_cmd
/vehicle/brake_cmd
/vehicle/brake_report
/vehicle/dbw_enabled
/vehicle/lidar
/vehicle/obstacle
/vehicle/obstacle_points
/vehicle/steering_cmd
/vehicle/steering_report
/vehicle/throttle_cmd
/vehicle/throttle_report
/vehicle/traffic_lights

'''
'''
message data type:

std_msgs/Header header
styx_msgs/Waypoint[] waypoints
geometry_msgs/TwistStampd twist
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
max_waypoint_distance = 20.0


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.base_waypoints = []
        self.next_waypoint = None
        self.current_pose = None

        rospy.spin()

    # update next_waypoint depending on base_waypoints and curr pos
    def update_next_waypoint(self):
        if not self.base_waypoints:
            rospy.logwarn("No base_waypoints existed")
            return False

        if not self.current_pose:
            rospy.logwarn("No current pose existed")
            return False

        position = self.current_pose.position


        dist_limit = 10000
        next_wp = None
        nearest_wp = None
        next_to_nearest = None
        base_wp_list = self.base_waypoints.waypoints
        len_base_wp = len(self.base_waypoints.waypoints)


        #TODO:if(next_x - nearest_x) * (px - nearest_x) + (next_y - nearest_y) * (py - nearest_y) > 0:return next_to_nearest
        #TODO return nearest_waypoint
        dist_cal = lambda a,b:math.sqrt((a.x - b.x)**2  + (a.y-b.y)**2 + (a.z - b.z) ** 2)
        for i in range(len_base_wp):
            waypoint_position = self.base_waypoints.waypoints[i].pose.pose.position
            dist = dist_cal(position,waypoint_position)
            if dist < dist_limit:
                dist_limit = dist
                nearest_wp = i

        next_to_nearest = (nearest_wp + 1) % len_base_wp

        if(next_to_nearest is not None and nearest_wp is not None):
            next_x = base_wp_list[next_to_nearest].pose.pose.x
            nearest_x = base_wp_list[nearest_wp].pose.pose.position.x
            next_y = base_wp_list[next_to_nearest].pose.pose.y
            nearest_y = base_wp_list[nearest_wp].pose.pose.y
            px = position.x
            py = position.y
            if (next_x - nearest_x) * (px - nearest_x) + (next_y - nearest_y) * (py - nearest_y) > 0:
                next_wp = next_to_nearest
            else:
                next_wp = nearest_wp

        if next_wp is None:
            return False

        self.next_waypoint = next_wp
        return True


    def publish_waypoints(self):
        # determine if waypoints updated
        if self.update_next_waypoint():
            len_base_wp = len(self.base_waypoints)
            wp_idx = [idx % len_base_wp for idx in range(self.next_waypoint,self.next_waypoint+LOOKAHEAD_WPS)]
            final_waypoints = [self.base_waypoints[wp] for wp in wp_idx]
            self.publish(final_waypoints)

    def publish(self,final_waypoints):
        waypoint_msg = Lane()
        waypoint_msg.waypoints = final_waypoints
        self.final_waypoints_pub.publish(waypoint_msg)

    def pose_cb(self, msg):
        # TODO: Implement
        self.current_pose = msg.pose


    def waypoints_cb(self, msg):
        # TODO: Implement
        # base waypoints is only published once, so it should be stored here
        waypoints = msg.waypoints
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
