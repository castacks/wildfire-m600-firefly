#!/usr/bin/env python2

import math

import rospy
from std_msgs.msg import Int32MultiArray, Empty
from pymavlink import mavutil
import os
from threading import Lock
from enum import Enum
import tf2_ros
import struct
from firefly_telemetry.srv import SetLocalPosRef
import time
import serial

os.environ['MAVLINK20'] = '1'


class PayloadTunnelType(Enum):
    FireBins = 32768
    NonFireBins = 32769


class OnboardTelemetry:
    def __init__(self):
        self.connection = mavutil.mavlink_connection('/dev/mavlink', baud=57600, dialect='firefly')

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.new_fire_bins = []
        self.new_no_fire_bins = []
        self.new_bins_mutex = Lock()

        self.pose_send_flag = False

        rospy.Subscriber("new_fire_bins", Int32MultiArray, self.new_fire_bins_callback)
        rospy.Subscriber("new_no_fire_bins", Int32MultiArray, self.new_no_fire_bins_callback)
        self.set_local_pos_ref_pub = rospy.Publisher("set_local_pos_ref", Empty, queue_size=100)
        self.clear_map_pub = rospy.Publisher("clear_map", Empty, queue_size=100)

        rospy.Timer(rospy.Duration(1), self.pose_send_callback)
        self.extract_frame_pub = rospy.Publisher("extract_frame", Empty, queue_size=1)

        self.bytes_per_sec_send_rate = 1152.0
        self.mavlink_packet_overhead_bytes = 12

        rospy.Timer(rospy.Duration(1), self.heartbeat_send_callback)

        self.last_heartbeat_time = None
        self.connectedToGCS = False
        self.watchdog_timeout = 2.0
        self.heartbeat_send_flag = False

        try:
            self.connection = mavutil.mavlink_connection('/dev/mavlink', baud=57600, dialect='firefly')
            self.connectedToOnboardRadio = True
            print("Opened connection to onboard radio")
        except serial.serialutil.SerialException:
            self.connectedToOnboardRadio = False
        self.last_serial_attempt_time = time.time()
        self.serial_reconnect_wait_time = 1.0

    def new_fire_bins_callback(self, data):
        with self.new_bins_mutex:
            self.new_fire_bins.extend(data.data)

    def new_no_fire_bins_callback(self, data):
        with self.new_bins_mutex:
            self.new_no_fire_bins.extend(data.data)

    def pose_send_callback(self, event):
        self.pose_send_flag = True

    def send_map_update(self):
        updates_to_send = None
        sending_fire_bins = None
        max_bins_to_send = int(math.floor(128/3)) # Since max payload is 128 bytes and each bin represented by 3 bytes
        with self.new_bins_mutex:
            if len(self.new_fire_bins) > 0:
                updates_to_send = self.new_fire_bins[:max_bins_to_send]
                self.new_fire_bins = self.new_fire_bins[max_bins_to_send:]
                sending_fire_bins = True
            elif len(self.new_no_fire_bins) > 0:
                updates_to_send = self.new_no_fire_bins[:max_bins_to_send]
                self.new_no_fire_bins = self.new_no_fire_bins[max_bins_to_send:]
                sending_fire_bins = False
            else:
                return

        payload = bytearray()
        for update in updates_to_send:
            payload.extend(struct.pack(">i", update)[-3:])

        payload_length = len(payload)
        if len(payload) < 128:
            payload.extend(bytearray(128-len(payload)))  # Pad payload so it has 128 bytes

        if sending_fire_bins:
            self.connection.mav.tunnel_send(0, 0, PayloadTunnelType.FireBins.value, payload_length, payload)
        else:
            self.connection.mav.tunnel_send(0, 0, PayloadTunnelType.NonFireBins.value, payload_length, payload)

        # Tunnel message is 145 bytes. Sleep by this much to not overwhelm the serial baud rate
        rospy.sleep((self.mavlink_packet_overhead_bytes + 128 + 5)/self.bytes_per_sec_send_rate)

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
            self.pose_send_flag = True
            self.connection.mav.firefly_pose_send(x, y, z, q)
            rospy.sleep((self.mavlink_packet_overhead_bytes + 28) / self.bytes_per_sec_send_rate)
        except tf2_ros.TransformException as e:
            print(e)
            self.pose_send_flag = False

    def run(self):
        if (self.last_heartbeat_time is None) or (time.time() - self.last_heartbeat_time > self.watchdog_timeout):
            if self.connectedToGCS:
                self.connectedToGCS = False
                print("Disconnected from GCS")
        else:
            if not self.connectedToGCS:
                self.connectedToGCS = True
                print("Connected to GCS")

        if self.connectedToOnboardRadio:
            try:
                self.read_incoming()
                self.send_map_update()
                self.send_pose_update()

                if self.heartbeat_send_flag:
                    self.connection.mav.firefly_heartbeat_send(1)
                    print("Sending Heartbeat")
                    self.heartbeat_send_flag = False
            except serial.serialutil.SerialException as e:
                self.connectedToOnboardRadio = False
                print(e)
        elif time.time() - self.last_serial_attempt_time >= self.serial_reconnect_wait_time:
            try:
                self.connection = mavutil.mavlink_connection('/dev/mavlink', baud=57600, dialect='firefly')
                print("Opened connection to Onboard radio")
                self.connectedToOnboardRadio = True
            except serial.serialutil.SerialException as e:
                print(e)
            self.last_serial_attempt_time = time.time()

    def read_incoming(self):
        msg = self.connection.recv_match()
        if msg is None:
            return
        msg = msg.to_dict()
        print(msg)

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
                print("Service call failed: %s" % e)
        elif msg['mavpackettype'] == 'FIREFLY_GET_FRAME':
            if msg['get_frame'] == 1:
                # tell perception handler to extract frame
                e = Empty()
                self.extract_frame_pub.publish(e)
        elif msg['mavpackettype'] == 'FIREFLY_HEARTBEAT':
            self.heartbeat_last_time = time.Time()

    def heartbeat_send_callback(self, event):
        self.heartbeat_send_flag = True


if __name__ == "__main__":
    rospy.init_node("onboard_telemetry", anonymous=True)
    onboard_telemetry = OnboardTelemetry()

    while not rospy.is_shutdown():
        onboard_telemetry.run()
