#!/usr/bin/env python
# -*- coding: iso-8859-15 -*-

import sys
import os
import re
import rospy
import tf
import tf2_ros
from time import sleep

rospy.init_node('wifi_strength')

tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)
regex_lq = re.compile("Link Quality=(\d*)/(\d*)")
while not rospy.is_shutdown():
	f = os.popen("export LANG=C; /sbin/iwconfig wlan0")
	for line in f:
		line = line.strip()
		match_lq = regex_lq.match(line)
		if match_lq is not None:
			lq = float(match_lq.group(1)) / float(match_lq.group(2))
			pos = tfBuffer.lookup_transform("map", 'base_link', rospy.Time(0), rospy.Duration(1.0))
			print '{"x":%.2f, "y":%.2f, "link":%.2f},' % (pos.transform.translation.x, pos.transform.translation.y, lq)
			sys.stdout.flush()
			sleep(0.5)
