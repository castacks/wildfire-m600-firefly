#!/usr/bin/python2

import rospy
from planner_map_interfaces.msg import Plan, Waypoint
from core_trajectory_msgs.msg import TrajectoryXYZVYaw, WaypointXYZVYaw
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Empty, Bool, String
import time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import numpy as np


def get_initial_waypoints(curr_pose, first_x, first_y, first_height, velocity):
    initial_waypoints = []

    wp = WaypointXYZVYaw()
    wp.yaw = np.pi / 2
    wp.velocity = velocity

    if curr_pose.pose.position.z > first_height:
        wp.position.x = first_x
        wp.position.y = first_y
        wp.position.z = curr_pose.pose.position.z
        initial_waypoints.append(wp)
    else:
        wp.position.x = curr_pose.pose.position.x
        wp.position.y = curr_pose.pose.position.y
        wp.position.z = first_height
        initial_waypoints.append(wp)

    wp = WaypointXYZVYaw()
    wp.yaw = np.pi / 2
    wp.velocity = velocity
    wp.position.x = first_x
    wp.position.y = first_y
    wp.position.z = first_height
    initial_waypoints.append(wp)

    return initial_waypoints


class IppPlanToTrajectory:
    def __init__(self):
        self.trajectory_track_pub = rospy.Publisher(
            "trajectory_track", TrajectoryXYZVYaw, queue_size=1
        )
        self.got_initial_plan_pub = rospy.Publisher(
            "got_initial_ipp_plan", Bool, queue_size=1
        )
        self.replan_pub = rospy.Publisher("/planner/replan", String, queue_size=1)
        self.initial_ipp_plan_to_transmit_pub = rospy.Publisher(
            "initial_ipp_plan_to_transmit", Plan, queue_size=1
        )

        rospy.Subscriber("/global_path", Plan, self.ipp_plan_callback)
        rospy.Subscriber("execute_ipp_plan", Bool, self.execute_plan_callback)
        rospy.Subscriber(
            "wait_for_initial_ipp_plan", Empty, self.wait_for_initial_ipp_plan
        )
        odometry_sub = rospy.Subscriber("odometry", Odometry, self.odometry_callback)

        self.waiting_for_initial_ipp_plan = False
        self.executing = False
        self.initial_plan = None

        self.request_replan_timer = rospy.Timer(
            rospy.Duration(1.0), self.request_replan_callback
        )
        self.last_replan_time = None

        self.got_initial_ipp_plan_timer = rospy.Timer(
            rospy.Duration(0.1), self.got_initial_ipp_plan_callback
        )

        self.current_odom = None

    def request_replan_callback(self, event):
        if self.last_replan_time is None:
            return

        if self.executing and time.time() - self.last_replan_time > 15:
            self.replan_pub.publish(String())
            self.last_replan_time = time.time()

    def got_initial_ipp_plan_callback(self, event):
        have_initial_plan = self.initial_plan is not None
        self.got_initial_plan_pub.publish(have_initial_plan)

    def execute_plan_callback(self, msg):
        self.last_replan_time = time.time()
        self.executing = msg.data
        self.waiting_for_initial_ipp_plan = False
        if self.executing:
            if self.initial_plan is not None:
                self.trajectory_track_pub.publish(self.initial_plan)
        else:
            self.initial_plan = None

    def wait_for_initial_ipp_plan(self, empty_msg):
        self.initial_plan = None
        self.waiting_for_initial_ipp_plan = True
        self.executing = False

    def ipp_plan_callback(self, ipp_plan):
        trajectory_msg = TrajectoryXYZVYaw()
        trajectory_msg.header = ipp_plan.header
        trajectory_msg.header.frame_id = "/uav1/map"

        velocity = 3.0

        success, transformed_curr_pose = self.transform_odom("/uav1/map")
        if not success:
            rospy.logerr("failed to transform odom")
            return
        if len(ipp_plan.plan) == 0:
            rospy.logerr("Received IPP plan with 0 waypoints")
            return

        first_x = ipp_plan.plan[0].position.position.x
        first_y = ipp_plan.plan[0].position.position.y
        first_z = ipp_plan.plan[0].position.position.z
        initial_wps = get_initial_waypoints(
            transformed_curr_pose, first_x, first_y, first_z, velocity
        )
        trajectory_msg.waypoints.extend(initial_wps)

        for wp in ipp_plan.plan:
            output_wp = WaypointXYZVYaw()
            output_wp.position.x = wp.position.position.x
            output_wp.position.y = wp.position.position.y
            output_wp.position.z = wp.position.position.z

            orientation = wp.position.orientation
            (_, _, yaw) = euler_from_quaternion(
                [orientation.x, orientation.y, orientation.z, orientation.w]
            )
            output_wp.yaw = yaw

            output_wp.velocity = velocity

            trajectory_msg.waypoints.append(output_wp)

        # wp1 = WaypointXYZVYaw()
        # wp1.position.x = 0
        # wp1.position.y = 0
        # wp1.position.z = 30
        # wp1.yaw = 0
        # wp1.velocity = 3.0
        # trajectory_msg.waypoints.append(wp1)

        if self.executing:
            self.trajectory_track_pub.publish(trajectory_msg)
        elif self.waiting_for_initial_ipp_plan and self.initial_plan is None:
            self.initial_plan = trajectory_msg
            self.initial_ipp_plan_to_transmit_pub.publish(ipp_plan)

    def odometry_callback(self, msg):
        self.current_odom = msg

    def transform_odom(self, target_frame):
        if self.current_odom is None:
            return (False, None)

        curr_pose = PoseStamped()
        curr_pose.header.frame_id = self.current_odom.header.frame_id
        curr_pose.pose = self.current_odom.pose.pose
        return (True, curr_pose)

        # if tf_listener.frameExists(curr_odom.header.frame_id) and tf_listener.frameExists(target_frame):
        #     curr_pose = PoseStamped()
        #     curr_pose.header.frame_id = curr_odom.header.frame_id
        #     curr_pose.pose = curr_odom.pose.pose

        #     try:
        #         transformed_pose = tf_listener.transformPose(target_frame, curr_pose)
        #     except tf.Exception as e:
        #         rospy.logerr("Failed to transform odom pose to target frame.")
        #         return (False, None)

        #     return (True, transformed_pose)
        # else:
        #     rospy.logerr("Odom header frame id or target frame does not exist")
        #     return (False, None)


if __name__ == "__main__":
    rospy.init_node("ipp_plan_to_traj_xyzv_yaw", anonymous=True)
    ipp_plan = IppPlanToTrajectory()
    rospy.spin()
