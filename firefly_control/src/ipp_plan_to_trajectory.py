#!/usr/bin/python2

import rospy
from planner_map_interfaces.msg import Plan, Waypoint
from core_trajectory_msgs.msg import TrajectoryXYZVYaw, WaypointXYZVYaw
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Empty, Bool, String
import time


class IppPlanToTrajectory:
    def __init__(self):
        self.trajectory_track_pub = rospy.Publisher(
            "trajectory_track", TrajectoryXYZVYaw, queue_size=1
        )
        self.got_initial_plan_pub = rospy.Publisher(
            "got_initial_ipp_plan", Bool, queue_size=1
        )
        self.replan_pub = rospy.Publisher("/planner/replan", String, queue_size=1)

        rospy.Subscriber("/global_path", Plan, self.ipp_plan_callback)
        rospy.Subscriber("execute_ipp_plan", Bool, self.execute_plan_callback)
        rospy.Subscriber(
            "wait_for_initial_ipp_plan", Empty, self.wait_for_initial_ipp_plan
        )

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

        # wp1 = WaypointXYZVYaw()
        # wp1.position.x = 0
        # wp1.position.y = 0
        # wp1.position.z = 30
        # wp1.yaw = 0
        # wp1.velocity = 3.0
        # trajectory_msg.waypoints.append(wp1)

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

            output_wp.velocity = 3.0

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


if __name__ == "__main__":
    rospy.init_node("ipp_plan_to_traj_xyzv_yaw", anonymous=True)
    ipp_plan = IppPlanToTrajectory()
    rospy.spin()
