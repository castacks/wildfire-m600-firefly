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

        self.pose_mutex = Lock()
        self.x = None
        self.y = None
        self.z = None
        self.q = None
        self.send_pose_flag = False

        rospy.Subscriber("new_fire_bins", Int32MultiArray, self.new_fire_bins_callback)
        rospy.Subscriber("new_no_fire_bins", Int32MultiArray, self.new_no_fire_bins_callback)
        self.set_local_pos_ref_pub = rospy.Publisher("set_local_pos_ref", Empty, queue_size=100)
        self.clear_map_pub = rospy.Publisher("clear_map", Empty, queue_size=100)

        rospy.Timer(rospy.Duration(1), self.one_sec_timer_callback)
        self.extract_frame_pub = rospy.Publisher("extract_frame", Empty, queue_size=1)

        self.bytes_per_sec_send_rate = 1152.0
        self.mavlink_packet_overhead_bytes = 12

    def new_fire_bins_callback(self, data):
        with self.new_bins_mutex:
            self.new_fire_bins.extend(data.data)

    def new_no_fire_bins_callback(self, data):
        with self.new_bins_mutex:
            self.new_no_fire_bins.extend(data.data)

    def one_sec_timer_callback(self, event):
        try:
            transform = self.tfBuffer.lookup_transform('base_link', 'world', rospy.Time(0))
            with self.pose_mutex:
                self.x = transform.transform.translation.x
                self.y = transform.transform.translation.y
                self.z = transform.transform.translation.z
                self.q = [transform.transform.rotation.x,
                          transform.transform.rotation.y,
                          transform.transform.rotation.z,
                          transform.transform.rotation.w]
                self.send_pose_flag = True

        except Exception as e:
            print(e)
            with self.pose_mutex:
                self.x = None
                self.y = None
                self.z = None
                self.q = None

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
           # payload.extend(update.to_bytes(3, byteorder='big'))
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
        with self.pose_mutex:
            if self.send_pose_flag:
                self.send_pose_flag = False
                local_x = self.x
                local_y = self.y
                local_z = self.z
                local_q = self.q
            else:
                return
        self.connection.mav.firefly_pose_send(local_x, local_y, local_z, local_q)
        # Tunnel message is 145 bytes. Sleep by this much to not overwhelm the serial baud rate
        rospy.sleep((self.mavlink_packet_overhead_bytes + 28) / self.bytes_per_sec_send_rate)

    def run(self):
        self.send_map_update()
        self.send_pose_update()

        msg = self.connection.recv_match()
        if msg is None:
            return
        msg = msg.to_dict()

        if msg['mavpackettype'] == 'FIREFLY_CLEAR_MAP':
            self.clear_map_pub.publish(Empty())
        elif msg['mavpackettype'] == 'FIREFLY_SET_LOCAL_POS_REF':
            self.clear_map_pub.publish(Empty())
            self.set_local_pos_ref_pub.publish(Empty())
            pass
        print(msg)

        if msg['mavpackettype'] == 'FIREFLY_GET_FRAME':
            if msg['get_frame'] == 1:
                # tell perception handler to extract frame
                e = Empty()
                self.extract_frame_pub.publish(e)


if __name__ == "__main__":
    rospy.init_node("onboard_telemetry", anonymous=True)
    onboard_telemetry = OnboardTelemetry()

    while not rospy.is_shutdown():
        onboard_telemetry.run()
