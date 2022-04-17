#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32MultiArray
from pymavlink import mavutil
import os
import tf
from std_msgs.msg import Empty, Bool, Float32
import time
from sensor_msgs.msg import NavSatFix
import serial
from geometry_msgs.msg import Pose

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

        rospy.Subscriber("clear_map", Empty, self.clear_map_callback)
        rospy.Subscriber("set_local_pos_ref", Empty, self.set_local_pos_ref_callback)
        rospy.Subscriber("capture_frame", Empty, self.capture_frame_callback)
        rospy.Subscriber("record_rosbag", Empty, self.record_ros_bag_callback)
        rospy.Subscriber("stop_record_rosbag", Empty, self.stop_record_ros_bag_callback)

        # See https://en.wikipedia.org/wiki/Sliding_window_protocol
        self.map_received_buf = {}
        self.nr = 0
        self.wr = 20

        self.br = tf.TransformBroadcaster()

        self.clear_map_send_flag = False
        self.set_local_pos_ref_send_flag = False
        self.capture_frame_send_flag = False
        self.heartbeat_send_flag = False
        self.record_ros_bag_send_flag = False
        self.stop_record_ros_bag_send_flag = False

        self.bytes_per_sec_send_rate = 1000.0
        self.mavlink_packet_overhead_bytes = 12

        rospy.Timer(rospy.Duration(1), self.heartbeat_send_callback)

        self.last_heartbeat_time = None
        self.connectedToOnboard = False
        self.watchdog_timeout = 2.0

        try:
            self.connection = mavutil.mavlink_connection('/dev/mavlink', baud=57600, dialect='firefly')
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

            except serial.serialutil.SerialException as e:
                self.connectedToGCSRadio = False
                rospy.logerr(e)
        elif time.time() - self.last_serial_attempt_time >= self.serial_reconnect_wait_time:
            try:
                self.connection = mavutil.mavlink_connection('/dev/mavlink', baud=57600, dialect='firefly')
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

    def clear_map_callback(self, empty_msg):
        self.clear_map_send_flag = True

    def set_local_pos_ref_callback(self, empty_msg):
        self.set_local_pos_ref_send_flag = True

    def capture_frame_callback(self, empty_msg):
        self.capture_frame_send_flag = True

    def heartbeat_send_callback(self, event):
        self.heartbeat_send_flag = True

    def record_ros_bag_callback(self, empty_msg):
        self.record_ros_bag_send_flag = True

    def stop_record_ros_bag_callback(self, empty_msg):
        self.stop_record_ros_bag_send_flag = True


if __name__ == "__main__":
    rospy.init_node("gcs_telemetry", anonymous=True)
    onboard_telemetry = GCSTelemetry()

    while not rospy.is_shutdown():
        onboard_telemetry.run()

