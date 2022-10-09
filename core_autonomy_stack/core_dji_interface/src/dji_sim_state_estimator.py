#!/usr/bin/python
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped, QuaternionStamped, PointStamped
from dji_sdk.msg import VOPosition
from nav_msgs.msg import Odometry

use_vo_position = False
got_velocity = False
got_attitude = False
got_position = False

odom = Odometry()
odom.pose.pose.orientation.w = 1.

def imu_callback(msg):
    global got_attitude, odom
    got_attitude = True
    odom.pose.pose.orientation = msg.orientation

def angular_velocity_callback(msg):
    global got_angular_velocity, odom
    got_angular_velocity = True
    odom.twist.twist.angular.z = msg.vector.z

def velocity_callback(msg):
    global got_velocity, odom
    got_velocity = True
    odom.twist.twist.linear = msg.vector

def attitude_callback(msg):
    global got_attitude, odom
    got_attitude = True
    #odom.pose.pose.orientation = msg.quaternion

def vo_position_callback(msg):
    if use_vo_position:
        global got_position, odom
        got_position = True
        odom.pose.pose.position.x = msg.y
        odom.pose.pose.position.y = msg.x
        odom.pose.pose.position.z = -msg.z
    
def local_position_callback(msg):
    if not use_vo_position:
        global got_position, odom
        got_position = True
        odom.pose.pose.position.x = msg.point.x
        odom.pose.pose.position.y = msg.point.y
        odom.pose.pose.position.z = msg.point.z

def timer_callback(msg):
    global odom
    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = 'uav1/map'
    odom.child_frame_id = 'uav1/map'
    odom_pub.publish(odom)

if __name__ == '__main__':
    rospy.init_node('dji_sim_state_estimator')
    
    use_vo_position = rospy.get_param('~use_vo_position', False)

    odom_pub = rospy.Publisher('dji_odom', Odometry, queue_size=1)
    
    velocity_sub = rospy.Subscriber('dji_sdk/velocity', Vector3Stamped, velocity_callback)
    angular_velocity_sub = rospy.Subscriber('dji_sdk/angular_velocity_fused', Vector3Stamped, angular_velocity_callback)
    attitude_sub = rospy.Subscriber('dji_sdk/attitude', QuaternionStamped, attitude_callback)
    imu_sub = rospy.Subscriber('dji_sdk/imu', Imu, imu_callback)
    vo_position_sub = rospy.Subscriber('dji_sdk/vo_position', VOPosition, vo_position_callback)
    local_position_sub = rospy.Subscriber('dji_sdk/local_position', PointStamped, local_position_callback)
    timer = rospy.Timer(rospy.Duration(1./100.), timer_callback)
    
    rospy.spin()
