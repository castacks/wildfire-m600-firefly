#!/usr/bin/env python2

import serial
import rospy
from std_msgs.msg import Float32
import os


class PcbInterface:
    def __init__(self):
        try:
            self.ser = serial.Serial('/dev/temp_pcb', baudrate=9600)
            self.connected = True
            rospy.loginfo("Connected to temperature pcb")
        except serial.SerialException:
            self.connected = False
            rospy.logwarn("Failed to connect to temperature pcb")

        self.serialBuffer = ""
        self.temp = 0.0
        self.pub = rospy.Publisher('onboard_temperature', Float32, queue_size = 10)
        rospy.Timer(rospy.Duration(0.2), self.read_serial)
        rospy.Timer(rospy.Duration(1.0), self.attempt_reconnect)

    def read_serial(self, event):
        if self.connected:
            try:
                self.serialBuffer += self.ser.read_all()
            except IOError:
                self.connected = False
                rospy.logwarn("Disconnected from temperature pcb")
                return

            packetEndIdx = self.serialBuffer.find("\r\n")
            if packetEndIdx == -1:
                return

            while packetEndIdx != -1:
                packet = self.serialBuffer[:packetEndIdx]
                try:
                    self.temp = float(packet)
                except ValueError:
                    pass
                self.serialBuffer = self.serialBuffer[packetEndIdx+2:]
                packetEndIdx = self.serialBuffer.find("\r\n")

            self.pub.publish(temperature.temp)

    def attempt_reconnect(self, event):
        if self.connected:
            return
        try:
            if os.path.exists('/dev/temp_pcb'):
                rospy.sleep(5) # Wait for 5 seconds after port shows up (had bugs when connecting right away)
                self.ser = serial.Serial('/dev/temp_pcb', baudrate=9600)
                self.connected = True
                self.serialBuffer = ""
                rospy.loginfo("Reconnected to temperature pcb")
        except serial.SerialException:
            self.connected = False


if __name__ == "__main__":

    rospy.init_node("temperature", anonymous=True)
    rospy.sleep(1)

    temperature = PcbInterface()

    while not rospy.is_shutdown():
        continue
