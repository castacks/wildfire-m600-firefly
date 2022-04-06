#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32MultiArray
from pymavlink import mavutil
import os
import tf
from std_msgs.msg import Empty
import time
from sensor_msgs.msg import NavSatFix
import serial

os.environ['MAVLINK20'] = '1'


class GCSTelemetry:
    def __init__(self):
        self.new_fire_pub = rospy.Publisher("new_fire_bins", Int32MultiArray, queue_size=100)
        self.new_no_fire_pub = rospy.Publisher("new_no_fire_bins", Int32MultiArray, queue_size=100)
        self.local_pos_ref_pub = rospy.Publisher("local_pos_ref", NavSatFix, queue_size=100)

        rospy.Subscriber("clear_map", Empty, self.clear_map_callback)
        rospy.Subscriber("set_local_pos_ref", Empty, self.set_local_pos_ref_callback)
        rospy.Subscriber("capture_frame", Empty, self.capture_frame_callback)

        self.br = tf.TransformBroadcaster()

        self.clear_map_send_flag = False
        self.set_local_pos_ref_send_flag = False
        self.capture_frame_send_flag = False
        self.heartbeat_send_flag = False

        rospy.Timer(rospy.Duration(1), self.heartbeat_send_callback)

        self.last_heartbeat_time = None
        self.connectedToOnboard = False
        self.watchdog_timeout = 2.0

        try:
            self.connection = mavutil.mavlink_connection('/dev/mavlink', baud=57600, dialect='firefly')
            self.connectedToGCSRadio = True
            print("Opened connection to GCS radio")
        except serial.serialutil.SerialException:
            self.connectedToGCSRadio = False
        self.last_serial_atempt_time = time.time()
        self.serial_reconnect_wait_time = 1.0

    def run(self):

        if (self.last_heartbeat_time is None) or (time.time() - self.last_heartbeat_time > self.watchdog_timeout):
            if self.connectedToOnboard:
                self.connectedToOnboard = False
                print("Disconnected from onboard radio")
        else:
            if not self.connectedToOnboard:
                self.connectedToOnboard = True
                print("Connected to onboard radio")

        if self.connectedToGCSRadio:
            try:
                self.read_incoming()

                if self.clear_map_send_flag:
                    self.connection.mav.firefly_clear_map_send(0)
                    print("Clearing Map")
                    self.clear_map_send_flag = False

                if self.set_local_pos_ref_send_flag:
                    self.connection.mav.firefly_set_local_pos_ref_send(0)
                    print("Setting Local Position Reference")
                    self.set_local_pos_ref_send_flag = False

                if self.capture_frame_send_flag:
                    self.connection.mav.firefly_get_frame_send(1)
                    print("Capturing Frame")
                    self.capture_frame_send_flag = False

                if self.heartbeat_send_flag:
                    self.connection.mav.firefly_get_frame_send(1)
                    print("Sending Heartbeat")
                    self.heartbeat_send_flag = False
            except serial.serialutil.SerialException as e:
                self.connectedToGCSRadio = False
                print(e)
                # print("GCS radio was disconnected")
        elif time.time() - self.last_serial_atempt_time >= self.serial_reconnect_wait_time:
            try:
                self.connection = mavutil.mavlink_connection('/dev/mavlink', baud=57600, dialect='firefly')
                print("Opened connection to GCS radio")
                self.connectedToGCSRadio = True
            except serial.serialutil.SerialException as e:
                print(e)
            self.last_serial_atempt_time = time.time()

    def read_incoming(self):
        msg = self.connection.recv_match()
        if msg is not None:
            msg = msg.to_dict()
            print(msg)

            if msg['mavpackettype'] == 'TUNNEL':
                payload = msg['payload']
                updated_bins_msg = Int32MultiArray()
                for i in range(int(msg['payload_length']/3)):
                    bin_bytes = payload[3*i:3*i+3]
                    bin = int.from_bytes(bin_bytes, byteorder='big')
                    updated_bins_msg.data.append(bin)

                if msg['payload_type'] == 32768:
                    self.new_fire_pub.publish(updated_bins_msg)
                elif msg['payload_type'] == 32769:
                    self.new_no_fire_pub.publish(updated_bins_msg)
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
            elif msg['mavpackettype'] == 'FIREFLY_HEARTBEAT':
                self.last_heartbeat_time = time.time()

    def clear_map_callback(self, empty_msg):
        self.clear_map_send_flag = True

    def set_local_pos_ref_callback(self, empty_msg):
        self.set_local_pos_ref_send_flag = True

    def capture_frame_callback(self, empty_msg):
        self.capture_frame_send_flag = True

    def heartbeat_send_callback(self, event):
        self.heartbeat_send_flag = True


if __name__ == "__main__":
    rospy.init_node("gcs_telemetry", anonymous=True)
    onboard_telemetry = GCSTelemetry()

    while not rospy.is_shutdown():
        onboard_telemetry.run()
