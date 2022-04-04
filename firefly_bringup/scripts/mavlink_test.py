#!/usr/bin/env python

from __future__ import print_function
import time


print("Python script sleeping ")
time.sleep(20)
print("Opening mavlink port")
from pymavlink import mavutil

master = mavutil.mavlink_connection('/dev/mavlink', baud=57600)

t1 = time.time()

counts = {}

bytes_sent = 0
bytes_recv = 0
count = 0

while True:
	string = "M600 Pro: Hi, " + str(count)
	master.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_NOTICE, string.encode())

	msg = master.recv_msg()
	if msg is not None:
		print("Outer: ", msg.to_dict())

	count += 1
	time.sleep(1)
