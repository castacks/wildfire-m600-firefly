#!/usr/bin/env python3
'''
#########################################################
#########################################################
Project FireFly : 2022 MRSD - Team D
Authors: Arjun Chauhan, Kevin Gmelin, Sabrina Shen, Manuj Trehan and Akshay Venkatesh

GCSTelemetry : Ground Control Station Telemetry Module

Interfaces with Ground Control Station MAVLink, performs parsing and 
handling of messages received from UAV and user commands from GUI 

Created:  03 Apr 2022
#########################################################
#########################################################
'''

from numpy import uint8
import rospy
from std_msgs.msg import Int32MultiArray
from pymavlink import mavutil
import os
import tf
from std_msgs.msg import Empty, Bool, Float32
import time
from sensor_msgs.msg import NavSatFix
import serial
from geometry_msgs.msg import Pose, PoseStamped, PolygonStamped, Point32
from nav_msgs.msg import Path

os.environ['MAVLINK20'] = '1'


class GCSTelemetry:
    def __init__(self):
        self.new_fire_pub = rospy.Publisher("new_fire_bins", Int32MultiArray, queue_size=100)
        self.new_no_fire_pub = rospy.Publisher("new_no_fire_bins", Int32MultiArray, queue_size=100)
        self.init_to_no_fire_with_pose_pub = rospy.Publisher("init_to_no_fire_with_pose_bins", Pose, queue_size=100)
        self.local_pos_ref_pub = rospy.Publisher("local_pos_ref", NavSatFix, queue_size=100)
        self.camera_health_gcs_status = rospy.Publisher("camera_health_telem", Bool, queue_size=10)
        self.battery_status_gcs = rospy.Publisher("battery_status_telem", Float32, queue_size=10)
        self.altitude_status_gcs = rospy.Publisher("altitude_telem", Float32, queue_size=10)
        self.temperature_status_gcs = rospy.Publisher("temperature_status_telem", Float32, queue_size=10)
        self.ipp_plan_pub = rospy.Publisher("ipp_plan_preview", Path, queue_size=10)

        rospy.Subscriber("clear_map", Empty, self.clear_map_callback)
        rospy.Subscriber("set_local_pos_ref", Empty, self.set_local_pos_ref_callback)
        rospy.Subscriber("request_control", Empty, self.request_control_callback)
        rospy.Subscriber("arm", Empty, self.arm_callback)
        rospy.Subscriber("disarm", Empty, self.disarm_callback)
        rospy.Subscriber("takeoff", Empty, self.takeoff_callback)
        rospy.Subscriber("land", Empty, self.land_callback)
        rospy.Subscriber("traj_control", Empty, self.traj_control_callback)
        rospy.Subscriber("coverage_planner", Empty, self.coverage_planner_callback)
        rospy.Subscriber("ipp_planner", Empty, self.ipp_planner_callback)
        rospy.Subscriber("kill_switch", Empty, self.kill_switch_callback)
        rospy.Subscriber("capture_frame", Empty, self.capture_frame_callback)
        rospy.Subscriber("record_rosbag", Empty, self.record_ros_bag_callback)
        rospy.Subscriber("stop_record_rosbag", Empty, self.stop_record_ros_bag_callback)
        rospy.Subscriber("execute_ipp_plan", Empty, self.execute_ipp_plan_callback)
        rospy.Subscriber("idle", Empty, self.idle_callback)
        rospy.Subscriber(
            "reset_behavior_tree", Empty, self.reset_behavior_tree_callback
        )
        rospy.Subscriber('coverage_poly', PolygonStamped, self.load_polygon_callback)
        rospy.Subscriber('send_coverage_poly', Empty, self.send_coverage_poly_callback)
        rospy.Subscriber('view_coverage_poly', Empty, self.reset_polygon)


        # See https://en.wikipedia.org/wiki/Sliding_window_protocol
        self.map_received_buf = {}
        self.nr = 0
        self.wr = 20

        self.ipp_plan_received_buf = {}
        self.nr_ipp = 0

        self.ipp_plan = Path()
        self.ipp_plan.header.frame_id = "world"

        self.br = tf.TransformBroadcaster()

        self.clear_map_send_flag = False
        self.set_local_pos_ref_send_flag = False
        self.request_control_send_flag = False
        self.arm_send_flag = False
        self.disarm_send_flag = False
        self.takeoff_send_flag = False
        self.land_send_flag = False
        self.traj_control_send_flag = False
        self.coverage_planner_send_flag = False
        self.ipp_planner_send_flag = False
        self.kill_switch_flag = False
        self.capture_frame_send_flag = False
        self.heartbeat_send_flag = False
        self.record_ros_bag_send_flag = False
        self.stop_record_ros_bag_send_flag = False
        self.execute_ipp_plan_flag = False
        self.idle_send_flag = False
        self.reset_behavior_tree_send_flag = False

        self.bytes_per_sec_send_rate = 2000.0
        self.mavlink_packet_overhead_bytes = 12

        rospy.Timer(rospy.Duration(1), self.heartbeat_send_callback)

        self.last_heartbeat_time = None
        self.connectedToOnboard = False
        self.watchdog_timeout = 2.0

        self.send_coverage_poly_flag = False
        self.polygon_list = {}
        self.polygon_pt_idx = 0

        try:
            self.connection = mavutil.mavlink_connection(
                "/dev/mavlink", baud=115200, dialect="firefly"
            )
            self.connectedToGCSRadio = True
            rospy.loginfo("Opened connection to GCS radio")
        except serial.serialutil.SerialException:
            self.connectedToGCSRadio = False
            rospy.logerr("Failed to open connection to GCS radio")

        self.last_serial_attempt_time = time.time()
        self.serial_reconnect_wait_time = 1.0

    def run(self):

        if (self.last_heartbeat_time is None) or (time.time() - self.last_heartbeat_time > self.watchdog_timeout):
            if self.connectedToOnboard:
                self.connectedToOnboard = False
                rospy.logerr("Disconnected from onboard radio")
        else:
            if not self.connectedToOnboard:
                self.connectedToOnboard = True
                rospy.loginfo("Connected to onboard radio")

        if self.connectedToGCSRadio:
            try:
                self.read_incoming()
                self.send_outgoing()
            except serial.serialutil.SerialException as e:
                self.connectedToGCSRadio = False
                rospy.logerr(e)
        elif time.time() - self.last_serial_attempt_time >= self.serial_reconnect_wait_time:
            try:
                self.connection = mavutil.mavlink_connection(
                    "/dev/mavlink", baud=115200, dialect="firefly"
                )
                rospy.loginfo("Opened connection to GCS radio")
                self.connectedToGCSRadio = True
            except serial.serialutil.SerialException as e:
                rospy.logerr(e)
            self.last_serial_attempt_time = time.time()

    def process_new_map_packet(self, msg, map_packet_type):

        if (msg['seq_num'] - self.nr) % 128 > self.wr:
            # Reject packet
            pass
        else:
            if map_packet_type == 2:
                updated_bins_msg = Pose()
                updated_bins_msg.position.x = msg['x']
                updated_bins_msg.position.y = msg['y']
                updated_bins_msg.position.z = msg['z']

                updated_bins_msg.orientation.x = msg['q'][0]
                updated_bins_msg.orientation.y = msg['q'][1]
                updated_bins_msg.orientation.z = msg['q'][2]
                updated_bins_msg.orientation.w = msg['q'][3]
            else:
                updated_bins_msg = Int32MultiArray()
                payload = msg['payload']
                for i in range(int(msg['payload_length'] / 3)):
                    bin_bytes = payload[3 * i:3 * i + 3]
                    bin = int.from_bytes(bin_bytes, byteorder='big')
                    updated_bins_msg.data.append(bin)

            if msg['seq_num'] == self.nr:
                if map_packet_type == 0:
                    self.new_fire_pub.publish(updated_bins_msg)
                elif map_packet_type == 1:
                    self.new_no_fire_pub.publish(updated_bins_msg)
                elif map_packet_type == 2:
                    self.init_to_no_fire_with_pose_pub.publish(updated_bins_msg)
                self.nr = (self.nr + 1) % 128
                while True:
                    if self.nr in self.map_received_buf:
                        map_packet_type, updated_bins_msg = self.map_received_buf.pop(self.nr)
                        if map_packet_type == 0:
                            self.new_fire_pub.publish(updated_bins_msg)
                        elif map_packet_type == 1:
                            self.new_no_fire_pub.publish(updated_bins_msg)
                        elif map_packet_type == 2:
                            self.init_to_no_fire_with_pose_pub.publish(updated_bins_msg)
                        self.nr = (self.nr + 1) % 128
                    else:
                        break
            else:
                self.map_received_buf[msg['seq_num']] = (map_packet_type, updated_bins_msg)

        self.connection.mav.firefly_map_ack_send(msg['seq_num'])
        rospy.sleep((self.mavlink_packet_overhead_bytes + 1) / self.bytes_per_sec_send_rate)

    @staticmethod
    def extract_pose_from_ipp_waypoint(msg):
        poseStamped = PoseStamped()
        poseStamped.header.frame_id = "world"  # TODO: check frame?
        poseStamped.pose.position.x = msg["x"]
        poseStamped.pose.position.y = msg["y"]
        poseStamped.pose.position.z = msg["z"]

        poseStamped.pose.orientation.x = msg["q"][0]
        poseStamped.pose.orientation.y = msg["q"][1]
        poseStamped.pose.orientation.z = msg["q"][2]
        poseStamped.pose.orientation.w = msg["q"][3]
        return poseStamped

    def process_new_ipp_packet(self, msg, ipp_packet_type):
        assert ipp_packet_type in ["start", "end", "waypoint"]

        if (msg["seq_num"] - self.nr_ipp) % 128 > self.wr:
            # Reject packet, outside window size
            pass
        else:
            if ipp_packet_type == "waypoint":
                poseStamped = self.extract_pose_from_ipp_waypoint(msg)
            else:
                poseStamped = None

            if (
                msg["seq_num"] == self.nr_ipp
            ):  # Correct seq num received as per order. Add to path array and publish
                if ipp_packet_type == "start":
                    self.ipp_plan = Path()
                    self.ipp_plan.header.frame_id = "world"
                elif ipp_packet_type == "end":
                    pass
                else:
                    self.ipp_plan.poses.append(poseStamped)

                self.nr_ipp = (self.nr_ipp + 1) % 128
                while (
                    True
                ):  # Check if further seq nums already received and stored in buf. If yes, add to path array in order and publish
                    if self.nr_ipp in self.ipp_plan_received_buf:
                        packet = self.ipp_plan_received_buf.pop(self.nr_ipp)
                        if packet["ipp_packet_type"] == "start":
                            self.ipp_plan = Path()
                            self.ipp_plan.header.frame_id = "world"
                        elif packet["ipp_packet_type"] == "end":
                            pass
                        elif packet["ipp_packet_type"] == "waypoint":
                            self.ipp_plan.poses.append(packet["poseStamped"])
                        self.nr_ipp = (self.nr_ipp + 1) % 128
                    else:
                        break
                self.ipp_plan_pub.publish(self.ipp_plan)
            else:  # Future seq num received. Store in buf for future use
                self.ipp_plan_received_buf[msg["seq_num"]] = {
                    "ipp_packet_type": ipp_packet_type,
                    "poseStamped": poseStamped,
                }

        self.connection.mav.firefly_ipp_plan_preview_ack_send(msg['seq_num']) # send an ack of the seq num received
        rospy.sleep((self.mavlink_packet_overhead_bytes + 1) / self.bytes_per_sec_send_rate)

    def read_incoming(self):
        msg = self.connection.recv_match()
        if msg is not None:
            msg = msg.to_dict()
            rospy.logdebug(msg)

            if msg['mavpackettype'] == 'FIREFLY_NEW_FIRE_BINS':
                self.process_new_map_packet(msg, 0)
            elif msg['mavpackettype'] == 'FIREFLY_NEW_NO_FIRE_BINS':
                self.process_new_map_packet(msg, 1)
            elif msg['mavpackettype'] == 'FIREFLY_INIT_TO_NO_FIRE_POSE':
                self.process_new_map_packet(msg, 2)
            elif msg['mavpackettype'] == 'FIREFLY_POSE':
                self.br.sendTransform((msg['x'], msg['y'], msg['z']),
                                      msg['q'],
                    rospy.Time.now(),
                    "world",
                                      "base_link")
            elif msg['mavpackettype'] == 'FIREFLY_LOCAL_POS_REF':
                nav_msg = NavSatFix()
                nav_msg.header.frame_id = 'world'
                nav_msg.latitude = msg['latitude']
                nav_msg.longitude = msg['longitude']
                nav_msg.altitude = msg['altitude']
                self.local_pos_ref_pub.publish(nav_msg)
                rospy.set_param("local_pos_ref", [msg['latitude'], msg['longitude'], msg['altitude']])
            elif msg['mavpackettype'] == 'FIREFLY_HEARTBEAT':
                #get camera status and publish to node
                telemetry_status = msg['empty']
                if telemetry_status == 1:
                    self.camera_health_gcs_status.publish(False)
                elif telemetry_status == 2:
                    self.camera_health_gcs_status.publish(True)
                self.last_heartbeat_time = time.time()
            elif msg['mavpackettype'] == 'FIREFLY_BATTERY_STATUS':
                battery_status = msg['battery']
                self.battery_status_gcs.publish(battery_status)
            elif msg['mavpackettype'] == 'FIREFLY_ONBOARD_TEMP':
                temperature_status = msg['temp']
                self.temperature_status_gcs.publish(temperature_status)
            elif msg['mavpackettype'] == 'FIREFLY_ALTITUDE':
                #mobile app is relative altitude : https://developer.dji.com/onboard-sdk/documentation/guides/component-guide-altitude.html
                altitude = msg['alt'] 
                self.altitude_status_gcs.publish(altitude)
            elif msg["mavpackettype"] == "FIREFLY_IPP_PLAN_PREVIEW":
                self.process_new_ipp_packet(msg, "waypoint")
            elif msg["mavpackettype"] == "FIREFLY_IPP_TRANSMIT_COMPLETE":
                self.process_new_ipp_packet(msg, "end")
            elif msg["mavpackettype"] == "FIREFLY_IPP_TRANSMIT_START":
                self.process_new_ipp_packet(msg, "start")
            elif msg["mavpackettype"] == "FIREFLY_COVERAGE_POLYGON_POINT_ACK":
                if msg["seq_num"] in self.polygon_list:
                    self.polygon_list.pop(msg["seq_num"])
                if len(self.polygon_list.keys()) == 0:
                    self.send_coverage_poly_flag = False


    def send_outgoing(self):
        if self.clear_map_send_flag:
            self.connection.mav.firefly_clear_map_send(0)
            rospy.loginfo("Clearing Map")
            self.clear_map_send_flag = False
            rospy.sleep((self.mavlink_packet_overhead_bytes + 1) / self.bytes_per_sec_send_rate)

        if self.set_local_pos_ref_send_flag:
            self.connection.mav.firefly_set_local_pos_ref_send(0)
            rospy.loginfo("Setting Local Position Reference")
            self.set_local_pos_ref_send_flag = False
            rospy.sleep((self.mavlink_packet_overhead_bytes + 1) / self.bytes_per_sec_send_rate)

        if self.request_control_send_flag:
            self.connection.mav.firefly_request_control_send(0)
            rospy.loginfo("Requesting Autonomous Control")
            self.request_control_send_flag = False
            rospy.sleep((self.mavlink_packet_overhead_bytes + 1) / self.bytes_per_sec_send_rate)

        if self.arm_send_flag:
            self.connection.mav.firefly_arm_send(0)
            rospy.loginfo("Arming Drone")
            self.arm_send_flag = False
            rospy.sleep((self.mavlink_packet_overhead_bytes + 1) / self.bytes_per_sec_send_rate)

        if self.disarm_send_flag:
            self.connection.mav.firefly_disarm_send(0)
            rospy.loginfo("Disarming Drone")
            self.disarm_send_flag = False
            rospy.sleep((self.mavlink_packet_overhead_bytes + 1) / self.bytes_per_sec_send_rate)

        if self.takeoff_send_flag:
            self.connection.mav.firefly_takeoff_send(0)
            rospy.loginfo("Taking Off")
            self.takeoff_send_flag = False
            rospy.sleep((self.mavlink_packet_overhead_bytes + 1) / self.bytes_per_sec_send_rate)

        if self.land_send_flag:
            self.connection.mav.firefly_land_send(0)
            rospy.loginfo("Landing")
            self.land_send_flag = False
            rospy.sleep((self.mavlink_packet_overhead_bytes + 1) / self.bytes_per_sec_send_rate)

        if self.traj_control_send_flag:
            self.connection.mav.firefly_traj_control_send(0)
            rospy.loginfo("Enabling Trajectory Control")
            self.traj_control_send_flag = False
            rospy.sleep((self.mavlink_packet_overhead_bytes + 1) / self.bytes_per_sec_send_rate)

        if self.coverage_planner_send_flag:
            self.connection.mav.firefly_coverage_planner_send(0)
            rospy.loginfo("Enabling Coverage Planner")
            self.coverage_planner_send_flag = False
            rospy.sleep((self.mavlink_packet_overhead_bytes + 1) / self.bytes_per_sec_send_rate)

        if self.ipp_planner_send_flag:
            self.connection.mav.firefly_ipp_planner_send(0)
            rospy.loginfo("Enabling IPP Planner")
            self.ipp_planner_send_flag = False
            rospy.sleep((self.mavlink_packet_overhead_bytes + 1) / self.bytes_per_sec_send_rate)

        if self.kill_switch_flag:
            self.connection.mav.firefly_kill_send(0)
            rospy.loginfo("KILLSWITCH TRIGGERED")
            self.kill_switch_flag = False
            rospy.sleep((self.mavlink_packet_overhead_bytes + 1) / self.bytes_per_sec_send_rate)

        if self.capture_frame_send_flag:
            self.connection.mav.firefly_get_frame_send(1)
            rospy.loginfo("Capturing Frame")
            self.capture_frame_send_flag = False
            rospy.sleep((self.mavlink_packet_overhead_bytes + 1) / self.bytes_per_sec_send_rate)

        if self.heartbeat_send_flag:
            self.connection.mav.firefly_heartbeat_send(1)
            rospy.logdebug("Sending Heartbeat")
            self.heartbeat_send_flag = False
            rospy.sleep((self.mavlink_packet_overhead_bytes + 1) / self.bytes_per_sec_send_rate)

        if self.record_ros_bag_send_flag:
            rospy.loginfo("Recording ROS Bags")
            self.connection.mav.firefly_record_bag_send(1)
            self.record_ros_bag_send_flag = False
            rospy.sleep((self.mavlink_packet_overhead_bytes + 1) / self.bytes_per_sec_send_rate)

        if self.stop_record_ros_bag_send_flag:
            rospy.loginfo("Stopping ROS Bag recording")
            self.connection.mav.firefly_record_bag_send(0)
            self.stop_record_ros_bag_send_flag = False
            rospy.sleep((self.mavlink_packet_overhead_bytes + 1) / self.bytes_per_sec_send_rate)

        if self.execute_ipp_plan_flag:
            rospy.loginfo("Executing IPP Plan")
            self.connection.mav.firefly_execute_ipp_plan_send(1)
            self.execute_ipp_plan_flag = False
            rospy.sleep(
                (self.mavlink_packet_overhead_bytes + 1) / self.bytes_per_sec_send_rate
            )

        if self.idle_send_flag:
            rospy.loginfo("Entering Idle Mode")
            self.connection.mav.firefly_idle_send(1)
            self.idle_send_flag = False
            rospy.sleep(
                (self.mavlink_packet_overhead_bytes + 1) / self.bytes_per_sec_send_rate
            )

        if self.reset_behavior_tree_send_flag:
            rospy.loginfo("Resetting behavior tree")
            self.connection.mav.firefly_reset_behavior_tree_send(1)
            self.reset_behavior_tree_send_flag = False
            rospy.sleep(
                (self.mavlink_packet_overhead_bytes + 1) / self.bytes_per_sec_send_rate
            )

        if self.send_coverage_poly_flag:
            key_list = list(self.polygon_list.keys())
            if len(key_list) == 0:
                return
            pt_idx = key_list[0]

            pt = self.polygon_list.pop(pt_idx)
            self.polygon_list[pt_idx] = pt # for round robin transmission
            if len(self.polygon_list.keys()) > 1:
                self.connection.mav.firefly_coverage_polygon_point_send(
                    pt_idx,
                    uint8(pt.z),
                    pt.x,
                    pt.y
                )
            else:
                self.connection.mav.firefly_coverage_polygon_point_end_send(
                    pt_idx,
                    uint8(pt.z),
                    pt.x,
                    pt.y
                )
            rospy.sleep(
                (self.mavlink_packet_overhead_bytes + 10) / self.bytes_per_sec_send_rate
            )

    def clear_map_callback(self, empty_msg):
        self.clear_map_send_flag = True

    def set_local_pos_ref_callback(self, empty_msg):
        self.set_local_pos_ref_send_flag = True

    def request_control_callback(self, empty_msg):
        self.request_control_send_flag = True

    def arm_callback(self, empty_msg):
        self.arm_send_flag = True

    def disarm_callback(self, empty_msg):
        self.disarm_send_flag = True

    def takeoff_callback(self, empty_msg):
        self.takeoff_send_flag = True

    def land_callback(self, empty_msg):
        self.land_send_flag = True

    def traj_control_callback(self, empty_msg):
        self.traj_control_send_flag = True

    def coverage_planner_callback(self, empty_msg):
        self.coverage_planner_send_flag = True

    def ipp_planner_callback(self, empty_msg):
        self.ipp_planner_send_flag = True

    def kill_switch_callback(self, empty_msg):
        self.kill_switch_flag = True

    def capture_frame_callback(self, empty_msg):
        self.capture_frame_send_flag = True

    def heartbeat_send_callback(self, event):
        self.heartbeat_send_flag = True

    def record_ros_bag_callback(self, empty_msg):
        self.record_ros_bag_send_flag = True

    def stop_record_ros_bag_callback(self, empty_msg):
        self.stop_record_ros_bag_send_flag = True

    def execute_ipp_plan_callback(self, empty_msg):
        self.execute_ipp_plan_flag = True

    def idle_callback(self, empty_msg):
        self.idle_send_flag = True

    def reset_behavior_tree_callback(self, empty_msg):
        self.reset_behavior_tree_send_flag = True

    def load_polygon_callback(self, polygon):
        for pt in polygon.polygon.points:
            point  = Point32()
            point.x = pt.x
            point.y = pt.y
            '''
            * using Z as a proxy for polygon ID with 0 being the outer polygon and very subsequent ID corresponding to holes
            '''
            point.z = pt.z
            self.polygon_list[self.polygon_pt_idx] = point
            self.polygon_pt_idx += 1

    def send_coverage_poly_callback(self, empty_msg):
        if(len(self.polygon_list) != 0):
            self.send_coverage_poly_flag = True

    def reset_polygon(self, empty_msg):
        self.polygon_list = {}
        self.polygon_pt_idx = 0


if __name__ == "__main__":
    rospy.init_node("gcs_telemetry", anonymous=True)
    onboard_telemetry = GCSTelemetry()

    while not rospy.is_shutdown():
        onboard_telemetry.run()
