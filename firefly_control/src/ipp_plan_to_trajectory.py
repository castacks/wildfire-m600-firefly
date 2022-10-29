#!/usr/bin/python2

import rospy
from planner_map_interfaces.msg import Plan, Waypoint
from core_trajectory_msgs.msg import TrajectoryXYZVYaw, WaypointXYZVYaw
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Empty


class IppPlanToTrajectory:
    def __init__(self):
        self.trajectory_track_pub = rospy.Publisher(
            "trajectory_track", TrajectoryXYZVYaw, queue_size=1
        )

        rospy.Subscriber("/global_path", Plan, self.ipp_plan_callback)
        rospy.Subscriber("execute_ipp_plan", Empty, self.execute_plan_callback)

        self.trajectory_msg = TrajectoryXYZVYaw()

    def execute_plan_callback(self, empty_msg):
        self.trajectory_track_pub.publish(self.trajectory_msg)

    def ipp_plan_callback(self, ipp_plan):
        self.trajectory_msg.header = ipp_plan.header
        self.trajectory_msg.header.frame_id = "/uav1/map"

        wp1 = WaypointXYZVYaw()
        wp1.position.x = 0
        wp1.position.y = 0
        wp1.position.z = 30
        wp1.yaw = 0
        wp1.velocity = 3.0
        self.trajectory_msg.waypoints.append(wp1)

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

            self.trajectory_msg.waypoints.append(output_wp)

        wp1 = WaypointXYZVYaw()
        wp1.position.x = 0
        wp1.position.y = 0
        wp1.position.z = 30
        wp1.yaw = 0
        wp1.velocity = 3.0
        self.trajectory_msg.waypoints.append(wp1)


if __name__ == "__main__":
    rospy.init_node("ipp_plan_to_traj_xyzv_yaw", anonymous=True)
    ipp_plan = IppPlanToTrajectory()
    rospy.spin()
