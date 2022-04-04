#!/usr/bin/env python3

import math

import rospy
from std_msgs.msg import Int32MultiArray
from pymavlink import mavutil
import os
import tf
from std_msgs.msg import Empty
import time

os.environ['MAVLINK20'] = '1'

class OnboardTelemetry:
    def __init__(self):
        self.now = time.time()
        self.connection = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600, dialect='firefly')
        self.new_fire_pub = rospy.Publisher("new_fire_bins", Int32MultiArray, queue_size=100)
        self.new_no_fire_pub = rospy.Publisher("new_no_fire_bins", Int32MultiArray, queue_size=100)
        self.clear_map_sub = rospy.Subscriber("clear_map", Empty, self.clear_map_callback)
        self.clear_map_sub = rospy.Subscriber("set_local_pos_ref", Empty, self.set_local_pos_ref_callback)

        self.br = tf.TransformBroadcaster()

        self.clear_map_flag = False
        self.set_local_pos_ref_flag = False


    def run(self):
        if(time.time() - self.now > 10):
            self.connection.mav.firefly_get_frame_send(1)
            self.now = time.time()

        msg = self.connection.recv_match()
        if msg is None:
            return
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
                                  "base_link1",
                                  "world")

        if self.clear_map_flag:
            self.connection.mav.firefly_clear_map_send(0)
            self.clear_map_flag = False

        if self.set_local_pos_ref_flag:
            self.connection.mav.firefly_set_local_pos_ref_send(0)
            self.clear_map_flag = False

    def clear_map_callback(self, empty_msg):
        self.clear_map_flag = True

    def set_local_pos_ref_callback(self, empty_msg):
        self.set_local_pos_ref_flag = True


if __name__ == "__main__":
    rospy.init_node("gcs_telemetry", anonymous=True)
    onboard_telemetry = OnboardTelemetry()

    while not rospy.is_shutdown():
        onboard_telemetry.run()
