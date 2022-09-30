import rospy
from planner_map_interfaces.msg import Plan, Waypoint
from core_trajectory_msgs.msg import TrajectoryXYZVYaw, WaypointXYZVYaw
from tf.transformations import euler_from_quaternion


def ipp_plan_callback(ipp_plan: Plan):
    trajectory_msg = TrajectoryXYZVYaw
    trajectory_msg.header = ipp_plan.header

    for wp in ipp_plan.plan:
        wp = Waypoint()
        output_wp = WaypointXYZVYaw()
        output_wp.position.x = wp.position.x
        output_wp.position.y = wp.position.y
        output_wp.position.z = wp.position.z

        orientation = wp.position.orientation
        (_, _, yaw) = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )
        output_wp.yaw = yaw

        output_wp.velocity = wp.command_speed

        trajectory_msg.waypoints.append(output_wp)

    trajectory_track_pub.publish(trajectory_msg)


if __name__ == "__main__":
    rospy.init_node("ipp_plan_to_traj_xyzv_yaw", anonymous=True)
    trajectory_track_pub = rospy.Publisher(
        "trajectory_track", TrajectoryXYZVYaw, queue_size=1
    )
    rospy.Subscriber("/global_path", Plan, ipp_plan_callback)
    rospy.spin()
