#!/usr/bin/env python
# -*- coding: iso-8859-15 -*-

import subprocess
import re
import rospy
import tf
from time import sleep
from math import *
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, Range, BatteryState
from wild_thumper.msg import LedStripe, Led

class SelfTest:
	def __init__(self):
		rospy.init_node('wild_thumper_self_test')
		self.voltage = None
		self.current = None
		self.yaw = None
		self.pub_led = rospy.Publisher("led_stripe", LedStripe, queue_size=16)
		self.pub_vel = rospy.Publisher("cmd_vel_out", Twist, queue_size=16)
		self.sub_bat = rospy.Subscriber("battery", BatteryState, self.batteryReceived)
		rospy.Subscriber("imu", Imu, self.imuReceived)
		rospy.Subscriber("range_forward_left", Range, self.rangeReceived)
		rospy.Subscriber("range_forward_right", Range, self.rangeReceived)
		rospy.Subscriber("range_backward", Range, self.rangeReceived)
		rospy.Subscriber("range_left", Range, self.rangeReceived)
		rospy.Subscriber("range_right", Range, self.rangeReceived)
		self.run()

	def run(self):
		self.test_led()
		self.test_power()
		self.test_wlan()
		self.test_engine()
		self.test_range()

	def test_range(self):
		assert self.sonar_forward_left > 0.04 and self.sonar_forward_left < 6
		assert self.sonar_forward_right > 0.04 and self.sonar_forward_right < 6
		assert self.sonar_backward > 0.04 and self.sonar_backward < 6

	def rangeReceived(self, msg):
		if msg.header.frame_id == "sonar_forward_left":
			self.sonar_forward_left = msg.range
		elif msg.header.frame_id == "sonar_forward_right":
			self.sonar_forward_right = msg.range
		elif msg.header.frame_id == "sonar_backward":
			self.sonar_backward = msg.range
		else:
			raise Exception("Unknown frame %s" % msg.header.frame_id)

	def test_engine(self):
		msg = Twist()

		# forward
		msg.linear.x = 0.20
		msg.angular.z = 0
		self.pub_vel.publish(msg)
		sleep(1)
		msg.linear.x = 0
		msg.angular.z = 0
		self.pub_vel.publish(msg)

		# turn
		yaw1 = self.yaw
		msg.linear.x = 0
		msg.angular.z = pi/2
		self.pub_vel.publish(msg)
		sleep(1)
		msg.linear.x = 0
		msg.angular.z = 0
		self.pub_vel.publish(msg)
		yaw2 = self.yaw
		diff = abs(pi/2 - (yaw2 - yaw1))
		if diff > 2*pi:
			diff-=2*pi
		assert diff < pi/4

	def imuReceived(self, msg):
		(roll, pitch, yaw) = tf.transformations.euler_from_quaternion(msg.orientation.__getstate__())
		self.yaw = yaw

	def test_wlan(self):
		cmd = subprocess.Popen('/sbin/iwconfig wlan0', shell=True, stdout=subprocess.PIPE)
		for line in cmd.stdout:
			if "Bit Rate" in line:
				bit_rate = float(re.findall(r"[\w]+", line.strip())[2])
				assert bit_rate > 50

	def test_power(self):
		assert self.voltage > 6 and self.voltage <= 12
		assert self.current > 0.2 and self.current < 1

	def batteryReceived(self, msg):
		self.voltage = msg.voltage
		self.current = msg.current
		self.sub_bat.unregister()

	def test_led(self):
		led_msg = LedStripe()
		led_msg = [Led()]
		for i in range(16):
			led_msg[0].num = i
			led_msg[0].red = 10
			led_msg[0].green = 0
			led_msg[0].blue = 0
			self.pub_led.publish(led_msg)
			sleep(0.1)
		for i in range(16):
			led_msg[0].num = i
			led_msg[0].red = 0
			led_msg[0].green = 10
			led_msg[0].blue = 0
			self.pub_led.publish(led_msg)
			sleep(0.1)
		for i in range(16):
			led_msg[0].num = i
			led_msg[0].red = 0
			led_msg[0].green = 0
			led_msg[0].blue = 10
			self.pub_led.publish(led_msg)
			sleep(0.1)
		for i in range(16):
			led_msg[0].num = i
			led_msg[0].red = 0
			led_msg[0].green = 0
			led_msg[0].blue = 0
			self.pub_led.publish(led_msg)
			sleep(0.1)

if __name__ == "__main__":
	SelfTest()
