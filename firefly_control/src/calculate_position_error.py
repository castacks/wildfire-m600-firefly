#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
import tf2_ros
import numpy as np


class Node:
    def __init__(self) -> None:
        self.x_error_pub = rospy.Publisher("x_error", Float32, queue_size=1)
        self.y_error_pub = rospy.Publisher("y_error", Float32, queue_size=1)
        self.z_error_pub = rospy.Publisher("z_error", Float32, queue_size=1)
        self.euclidean_error_pub = rospy.Publisher(
            "euclidean_error", Float32, queue_size=1
        )
        self.max_euclidean_error_pub = rospy.Publisher(
            "max_euclidean_error", Float32, queue_size=1
        )
        self.max_error = 0

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.publish_rate = rospy.get_param("~publish_rate", 10)
        timer = rospy.Timer(
            rospy.Duration(1.00 / self.publish_rate), self.timer_callback
        )

    def timer_callback(self, event):
        try:
            odom_transform = self.tfBuffer.lookup_transform(
                "uav1/base_link", "uav1/map", rospy.Time(0.0)
            )
            tracking_point_transform = self.tfBuffer.lookup_transform(
                "uav1/tracking_point", "uav1/map", rospy.Time(0.0)
            )
        except tf2_ros.TransformException as e:
            print(e)
            return

        odom_x = odom_transform.transform.translation.x
        odom_y = odom_transform.transform.translation.y
        odom_z = odom_transform.transform.translation.z
        tracking_point_x = tracking_point_transform.transform.translation.x
        tracking_point_y = tracking_point_transform.transform.translation.y
        tracking_point_z = tracking_point_transform.transform.translation.z
        print(
            "odom x:",
            odom_x,
            "y:",
            odom_y,
            "z:",
            odom_z,
            "track x:",
            tracking_point_x,
            "y:",
            tracking_point_y,
            "z:",
            tracking_point_z,
        )

        euclidean_dist = np.sqrt(
            (odom_x - tracking_point_x) ** 2
            + (odom_y - tracking_point_y) ** 2
            + (odom_z - tracking_point_z) ** 2
        )
        if euclidean_dist > self.max_error:
            self.max_error = euclidean_dist

        self.x_error_pub.publish(odom_x - tracking_point_x)
        self.y_error_pub.publish(odom_y - tracking_point_y)
        self.z_error_pub.publish(odom_z - tracking_point_z)
        self.euclidean_error_pub.publish(euclidean_dist)
        self.max_euclidean_error_pub.publish(self.max_error)


if __name__ == "__main__":
    rospy.init_node("calculate_position_error", anonymous=True)
    node = Node()
    rospy.spin()
