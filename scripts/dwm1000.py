#!/usr/bin/env python
# -*- coding: iso-8859-15 -*-

VISUALIZE = False

import threading
import struct
import rospy
import tf
import numpy as np
from math import *
from datetime import datetime
from i2c import i2c
from time import sleep
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from wild_thumper.srv import DWM1000Center, DWM1000CenterResponse
from pyshared.simple_kalman import simple_kalman
if VISUALIZE:
	import matplotlib.pyplot as plt


class DW1000(threading.Thread):
	def __init__(self, name, addr, offset):
		threading.Thread.__init__(self)
		self.setDaemon(1)
		self.dist = 0
		self.offset = offset
		self.addr = addr
		self.name = name
		self.last_update = datetime.min

		self.pub = rospy.Publisher(name, Float32, queue_size=16)

		self.start()

	def get_value(self):
		dev = i2c(self.addr)
		ret = struct.unpack("f", dev.read(4))
		dev.close()
		return ret[0]

	def distance(self):
		return self.dist

	# Returns each distance only if current
	def distance_valid(self):
		if (datetime.now() - self.last_update).seconds < 1:
			return self.dist
		return None

	def run(self):
		last_val = 10
		while not rospy.is_shutdown():
			val = self.get_value()
			if abs(val - last_val)  > 50:
				rospy.logwarn("Ignoring values too far apart %s: %.2f - %.2f", self.name, val, last_val)
			elif not isnan(val):
				self.dist = val + self.offset
				self.last_update = datetime.now()
				self.pub.publish(self.distance())
				last_val = val
			sleep(0.1)

class Position:
	def __init__(self):
		R = np.matrix([[0.5, 0.0], [0.0, 0.2]])		# Varianz des Messfehlers
		P_est = 1e50 * np.eye(2)			# Fehlerkovarianz
		Q = np.matrix([[0.001, 0.0], [0.0, 0.0001]])	# Systemrauschen
		self.filter_xy = simple_kalman(np.array([[1.0, 0.0]]).T, P_est, Q, R)
		self.speed_x = 0
		self.speed_y = 0
		self.speed_z = 0
		self.last_time = rospy.Time.now()
		rospy.Subscriber("/odom_combined", Odometry, self.odomReceived)

	def odomReceived(self, msg):
		self.speed_x = msg.twist.twist.linear.x
		self.speed_y = msg.twist.twist.linear.y
		self.speed_z = msg.twist.twist.angular.z

	def filter(self, x, y):
		# Correct estimation with speed
		current_time = rospy.Time.now()
		dt = (current_time - self.last_time).to_sec()
		# Subtract vehicle speed
		pos = self.filter_xy.x_est.T
		# translation
		pos -= np.array([self.speed_x*dt, self.speed_y*dt])
		# rotation
		rot = np.array([[np.cos(self.speed_z*dt), -np.sin(self.speed_z*dt)],
				[np.sin(self.speed_z*dt),  np.cos(self.speed_z*dt)]])
		pos = np.dot(pos, rot)
		# copy back
		self.filter_xy.x_est = pos.T

		# run kalman if new measurements are valid
		pos = None
		if x != None and y != None:
			pos = self.filter_xy.run(np.array([[x, y]]).T)

			# Update variance
			dist = np.linalg.norm([x, y])
			cov_00 = np.polyval([0.018, 0.0, 0.0], dist)
			cov_11 = np.polyval([0.006, 0.0, 0.0], dist)
			self.filter_xy.set_measure_cov(np.matrix([[cov_00, 0.0], [0.0, cov_11]]))
		else:
			pos = self.filter_xy.x_est
			self.filter_xy.P_est = np.matrix(1e50 * np.eye(2)) # reset estimate when lost


		x = pos.item((0, 0))
		y = pos.item((1, 0))

		self.last_time = current_time
		return x,y


def handle_center_call(req):
	diff = dwleft.distance_valid() - dwright.distance_valid()
	dwleft.offset -= diff/2
	dwright.offset += diff/2
	rospy.loginfo("Centering to %.2f %.2f", dwleft.offset, dwright.offset)
	return DWM1000CenterResponse()

if __name__ == "__main__":
	rospy.init_node('DWM1000', log_level=rospy.DEBUG)
	dwleft  = DW1000("uwb_dist_left",  0xc2, -0.04)
	dwright = DW1000("uwb_dist_right", 0xc0, +0.04)
	dist_l_r = 0.285 # Distance between both DWM1000
	rate = rospy.Rate(10)
	pos = Position()
	tf_broadcaster = tf.broadcaster.TransformBroadcaster()
	rospy.Service('/DWM1000/center', DWM1000Center, handle_center_call)

	while not rospy.is_shutdown() and dwleft.is_alive() and dwright.is_alive():
		dist_left = dwleft.distance_valid()
		dist_right = dwright.distance_valid()
		x = None
		y = None
		if dist_left == None or dist_right == None:
			rospy.logerr_throttle(10, "no valid sensor update %r %r" % (dist_left, dist_right))
			# run kalman prediction only
			x,y = pos.filter(None, None)
		else:
			dir = "left" if (dist_left < dist_right) else "right"

			diff = abs(dist_left - dist_right)
			if diff >= dist_l_r:
				# difference too high, correct to maximum
				off = diff - dist_l_r + 0.01
				if dist_left > dist_right:
					dist_left -= off/2
					dist_right += off/2
				else:
					dist_left += off/2
					dist_right -= off/2
			rospy.logdebug("%.2f %.2f %.2f %.2f %s", dwleft.distance(), dwright.distance(), dist_left, dist_right, dir)

			a_r = (-dist_right**2 + dist_left**2 - dist_l_r**2) / (-2*dist_l_r)
			x = dist_l_r/2 - a_r
			t = dist_right**2 - a_r**2
			if t >= 0:
				y = sqrt(t)
				rospy.logdebug("x=%.2f, y=%.2f", x, y)
				# Rotate 90 deg
				x, y = (y, -x)

				x, y = pos.filter(x, y)

				if VISUALIZE:
					circle_left = plt.Circle((-dist_l_r/2, 0), dwleft.distance, color='red', fill=False)
					circle_right = plt.Circle((dist_l_r/2, 0), dwright.distance, color='green', fill=False)
					plt.gca().add_patch(circle_left)
					plt.gca().add_patch(circle_right)
					plt.grid(True)
					plt.axis('scaled')
					plt.show()
			else:
				# No current position, still need up update kalman prediction
				x, y = pos.filter(None, None)
		tf_broadcaster.sendTransform((x, y, 0.0), (0, 0, 0, 1), rospy.Time.now(), "uwb_beacon", "base_footprint")

		rate.sleep()
