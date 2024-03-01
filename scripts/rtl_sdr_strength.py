#!/usr/bin/env python
# -*- coding: iso-8859-15 -*-

import sys
import subprocess
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
	popen = subprocess.Popen("rtl_power -f 868.2M:868.5M:10k -g 42 -i 1".split(), stdout=subprocess.PIPE)
	while True:
		line = popen.stdout.readline()
		if line == "":
			break
		power = line.replace("\n", "").split(",")[6:]
		power = [float(s) for s in power]
		pos = tfBuffer.lookup_transform("map", 'base_link', rospy.Time(0), rospy.Duration(1.0))
		print '{"x":%.2f, "y":%.2f, "power": %s},' % (pos.transform.translation.x, pos.transform.translation.y, power)
		sys.stdout.flush()
