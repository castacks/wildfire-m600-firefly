#!/usr/bin/env python2

import math

import rospy
from std_msgs.msg import Int32MultiArray, Empty
from pymavlink import mavutil
import os
from threading import Lock
import tf2_ros
import struct
from firefly_telemetry.srv import SetLocalPosRef
import time
import serial
import datetime
from geometry_msgs.msg import Pose

os.environ['MAVLINK20'] = '1'


class OnboardTelemetry:
    def __init__(self):
        self.connection = mavutil.mavlink_connection('/dev/mavlink', baud=57600, dialect='firefly')

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.new_fire_bins = set()
        self.new_no_fire_bins = set()
        self.init_to_no_fire_poses = []
        self.new_bins_mutex = Lock()

        # See https://en.wikipedia.org/wiki/Sliding_window_protocol
        self.wt = 20
        self.map_transmitted_buf = []
        self.nt = 0
        self.na = 0  # Set to -1 in uint8 format
        self.retransmit_timeout = 2.0
        self.map_payload_size = 60  # Bytes

        self.pose_send_flag = False

        rospy.Subscriber("new_fire_bins", Int32MultiArray, self.new_fire_bins_callback)
        rospy.Subscriber("new_no_fire_bins", Int32MultiArray, self.new_no_fire_bins_callback)
        rospy.Subscriber("init_to_no_fire_with_pose_bins", Pose, self.init_to_no_fire_with_pose_callback)
        self.set_local_pos_ref_pub = rospy.Publisher("set_local_pos_ref", Empty, queue_size=100)
        self.clear_map_pub = rospy.Publisher("clear_map", Empty, queue_size=100)

        rospy.Timer(rospy.Duration(0.2), self.pose_send_callback)
        self.extract_frame_pub = rospy.Publisher("extract_frame", Empty, queue_size=1)

        self.bytes_per_sec_send_rate = 1000.0
        self.mavlink_packet_overhead_bytes = 12

        self.last_heartbeat_time = None
        rospy.Timer(rospy.Duration(1), self.heartbeat_send_callback)

        self.last_heartbeat_time = None
        self.connectedToGCS = False
        self.watchdog_timeout = 2.0
        self.heartbeat_send_flag = False

        self.recording_ros_bag = False
        try:
            self.connection = mavutil.mavlink_connection('/dev/mavlink', baud=57600, dialect='firefly')
            self.connectedToOnboardRadio = True
            rospy.loginfo("Opened connection to onboard radio")
        except serial.serialutil.SerialException:
            self.connectedToOnboardRadio = False
            rospy.logerr("Failed to open connection to onboard radio")

        self.last_serial_attempt_time = time.time()
        self.serial_reconnect_wait_time = 1.0

    def new_fire_bins_callback(self, data):
        with self.new_bins_mutex:
            for bin in data.data:
                self.new_no_fire_bins.discard(bin)
                self.new_fire_bins.add(bin)

    def new_no_fire_bins_callback(self, data):
        with self.new_bins_mutex:
            for bin in data.data:
                self.new_fire_bins.discard(bin)
                self.new_no_fire_bins.add(bin)

    def init_to_no_fire_with_pose_callback(self, data):
        with self.new_bins_mutex:
            self.init_to_no_fire_poses.extend(data)

    def pose_send_callback(self, event):
        self.pose_send_flag = True

    def send_map_update(self):
        if (len(self.map_transmitted_buf) != 0) and (
                time.time() - self.map_transmitted_buf[0][0] > self.retransmit_timeout):
            # Resend packet if timeout

            _, map_packet_type, seq_num, payload_length, payload = self.map_transmitted_buf.pop(0)
            if map_packet_type == 0:
                self.connection.mav.firefly_new_fire_bins_send(seq_num, payload_length, payload)
                self.map_transmitted_buf.append((time.time(), 0, seq_num, payload_length, payload))
                rospy.sleep((self.mavlink_packet_overhead_bytes + self.map_payload_size) / self.bytes_per_sec_send_rate)
            elif map_packet_type == 1:
                self.connection.mav.firefly_new_no_fire_bins_send(seq_num, payload_length, payload)
                self.map_transmitted_buf.append((time.time(), 1, seq_num, payload_length, payload))
                rospy.sleep((self.mavlink_packet_overhead_bytes + self.map_payload_size) / self.bytes_per_sec_send_rate)
            elif map_packet_type == 2:
                x = payload.translation.x
                y = payload.translation.y
                z = payload.translation.z
                q = [payload.rotation.x,
                     payload.rotation.y,
                     payload.rotation.z,
                     payload.rotation.w]
                self.connection.mav.firefly_init_to_no_fire_pose_send(x, y, z, q)
                self.map_transmitted_buf.append((time.time(), 2, self.nt, -1, payload))
                rospy.sleep((self.mavlink_packet_overhead_bytes + 29) / self.bytes_per_sec_send_rate)

            rospy.logwarn("Warning: Had to resend packet with sequence id: %d" % seq_num)

            return
        elif (self.nt - self.na) % 128 >= self.wt:
            # Waiting for acks
            return
        else:
            # Send the next packet
            max_bins_to_send = int(
                math.floor(self.map_payload_size / 3))  # Since payload is 126 bytes and each bin represented by 3 bytes
            with self.new_bins_mutex:
                if len(self.new_fire_bins) > 0:  # Prioritize fire bins over no fire bins
                    updates_to_send = [self.new_fire_bins.pop()
                                       for _ in range(max_bins_to_send) if len(self.new_fire_bins) > 0]
                    map_packet_type = 0
                elif len(self.init_to_no_fire_poses) > 0:
                    updates_to_send = self.init_to_no_fire_poses.pop(0)
                    map_packet_type = 2
                elif len(self.new_no_fire_bins) > 0:
                    updates_to_send = [self.new_no_fire_bins.pop()
                                       for _ in range(max_bins_to_send) if len(self.new_no_fire_bins) > 0]
                    map_packet_type = 1
                else:
                    return

            if map_packet_type == 0 or map_packet_type == 1:
                payload = bytearray()
                for update in updates_to_send:
                    payload.extend(struct.pack(">i", update)[-3:])

                payload_length = len(payload)
                if len(payload) < self.map_payload_size:
                    payload.extend(bytearray(self.map_payload_size - len(payload)))  # Pad payload so it has 128 bytes

                if map_packet_type == 0:
                    self.connection.mav.firefly_new_fire_bins_send(self.nt, payload_length, payload)
                    self.map_transmitted_buf.append((time.time(), 0, self.nt, payload_length, payload))
                elif map_packet_type == 1:
                    self.connection.mav.firefly_new_no_fire_bins_send(self.nt, payload_length, payload)
                    self.map_transmitted_buf.append((time.time(), 1, self.nt, payload_length, payload))
                rospy.sleep((self.mavlink_packet_overhead_bytes + self.map_payload_size) / self.bytes_per_sec_send_rate)
            elif map_packet_type == 2:
                x = updates_to_send.translation.x
                y = updates_to_send.translation.y
                z = updates_to_send.translation.z
                q = [updates_to_send.rotation.x,
                     updates_to_send.rotation.y,
                     updates_to_send.rotation.z,
                     updates_to_send.rotation.w]
                self.connection.mav.firefly_init_to_no_fire_pose_send(x, y, z, q)
                self.map_transmitted_buf.append((time.time(), 2, self.nt, -1, updates_to_send))
                rospy.sleep((self.mavlink_packet_overhead_bytes + 29) / self.bytes_per_sec_send_rate)

            self.nt = (self.nt + 1) % 128

    def send_pose_update(self):
        if not self.pose_send_flag:
            return

        try:
            transform = self.tfBuffer.lookup_transform('base_link', 'world', rospy.Time(0))
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z
            q = [transform.transform.rotation.x,
                 transform.transform.rotation.y,
                 transform.transform.rotation.z,
                 transform.transform.rotation.w]
            self.connection.mav.firefly_pose_send(x, y, z, q)
            rospy.sleep((self.mavlink_packet_overhead_bytes + 28) / self.bytes_per_sec_send_rate)
        except tf2_ros.TransformException as e:
            pass
        self.pose_send_flag = False

    def run(self):
        if (self.last_heartbeat_time is None) or (time.time() - self.last_heartbeat_time > self.watchdog_timeout):
            if self.connectedToGCS:
                self.connectedToGCS = False
                rospy.logerr("Disconnected from GCS")
        else:
            if not self.connectedToGCS:
                self.connectedToGCS = True
                rospy.loginfo("Connected to GCS")

        if self.connectedToOnboardRadio:
            try:
                self.read_incoming()
                if self.connectedToGCS:
                    self.send_map_update()
                    self.send_pose_update()

                if self.heartbeat_send_flag:
                    self.connection.mav.firefly_heartbeat_send(1)
                    rospy.logdebug("Sending Heartbeat")
                    self.heartbeat_send_flag = False
            except serial.serialutil.SerialException as e:
                self.connectedToOnboardRadio = False
                rospy.logerr(e)
        elif time.time() - self.last_serial_attempt_time >= self.serial_reconnect_wait_time:
            try:
                self.connection = mavutil.mavlink_connection('/dev/mavlink', baud=57600, dialect='firefly')
                rospy.loginfo("Opened connection to Onboard radio")
                self.connectedToOnboardRadio = True
            except serial.serialutil.SerialException as e:
                rospy.logerr(e)
            self.last_serial_attempt_time = time.time()

    def read_incoming(self):
        msg = self.connection.recv_match()
        if msg is None:
            return
        msg = msg.to_dict()
        rospy.logdebug(msg)

        if msg['mavpackettype'] == 'FIREFLY_CLEAR_MAP':
            self.clear_map_pub.publish(Empty())
        elif msg['mavpackettype'] == 'FIREFLY_SET_LOCAL_POS_REF':
            self.clear_map_pub.publish(Empty())
            rospy.wait_for_service('set_local_pos_ref', timeout=0.1)
            try:
                set_local_pos_ref = rospy.ServiceProxy('set_local_pos_ref', SetLocalPosRef)
                response = set_local_pos_ref()
                self.connection.mav.firefly_local_pos_ref_send(response.latitude, response.longitude, response.altitude)
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s" % e)
        elif msg['mavpackettype'] == 'FIREFLY_GET_FRAME':
            if msg['get_frame'] == 1:
                # tell perception handler to extract frame
                e = Empty()
                self.extract_frame_pub.publish(e)
        elif msg['mavpackettype'] == 'FIREFLY_HEARTBEAT':
            self.last_heartbeat_time = time.time()
        elif msg['mavpackettype'] == 'FIREFLY_RECORD_BAG':
            if msg['get_frame'] == 1 and not self.recording_ros_bag:
                rospy.loginfo("Atempting to record ros bag")
                DEFAULT_ROOT = "/mnt/nvme0n1/data"
                time_rosbag = datetime.datetime.now()
                time_rosbag = time_rosbag.strftime("%d-%m-%Y-%H:%M:%S")
                dir = DEFAULT_ROOT + "/" + time_rosbag
                rospy.loginfo("Starting ros bag recording to file : " + dir + time_rosbag + "_dji_sdk_and_thermal.bag")
                os.system(
                    "rosbag record -a -O " + dir + "_dji_sdk_and_thermal.bag __name:='data_collect' -x '(.*)/compressed(.*)|(.*)/theora(.*)' &")
                self.recording_ros_bag = True
            elif msg['get_frame'] == 0 and self.recording_ros_bag:
                rospy.loginfo("Stopping ros bag recording")
                os.system("rosnode kill data_collect &")
                self.recording_ros_bag = False
        elif msg['mavpackettype'] == 'FIREFLY_MAP_ACK':
            # time.time(), False, self.nt, payload_length, payload`
            for i in range(len(self.map_transmitted_buf) - 1, -1, -1):
                if self.map_transmitted_buf[i][2] == msg['seq_num']:
                    self.map_transmitted_buf.pop(i)
            if self.na == msg['seq_num']:
                self.na = self.nt
                for i in range(len(self.map_transmitted_buf) - 1, -1, -1):
                    if self.map_transmitted_buf[i][2] < self.na:
                        self.na = self.map_transmitted_buf[i][2]

    def heartbeat_send_callback(self, event):
        self.heartbeat_send_flag = True


if __name__ == "__main__":
    rospy.init_node("onboard_telemetry", anonymous=True)
    onboard_telemetry = OnboardTelemetry()

    while not rospy.is_shutdown():
        onboard_telemetry.run()
