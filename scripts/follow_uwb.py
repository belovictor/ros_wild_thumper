#!/usr/bin/env python
# -*- coding: iso-8859-15 -*-

import rospy
import tf
import actionlib
import tf2_ros
import dynamic_reconfigure.client
import numpy as np
import thread
import threading
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from math import *


class FollowUWB:
	def __init__(self):
		rospy.init_node('follow_uwb')
		rospy.on_shutdown(self.on_shutdown)
		self.dist_stop = 1.0

		self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
		self.tfBuffer = tf2_ros.Buffer()
		self.listener = tf2_ros.TransformListener(self.tfBuffer)

		rospy.loginfo("Setting paramters")
		self.dynreconf = dynamic_reconfigure.client.Client("/move_base/TrajectoryPlannerROS")
		self.speed = None
		self.__lock_set_dynreconf = threading.Lock()
		self.set_speed("slow")

		rospy.loginfo("Waiting for the move_base action server to come up")
		self.move_base.wait_for_server()
		rospy.loginfo("Got move_base action server")

	def set_speed(self, mode):
		with self.__lock_set_dynreconf:
			if mode == "fast" and self.speed != "fast":
				print "speed", mode
				self.dynreconf.update_configuration({'max_vel_x': 1.0, 'max_vel_theta': 1.2, 'min_in_place_vel_theta': 1.0})
			elif mode == "slow" and self.speed != "slow":
				print "speed", mode
				self.dynreconf.update_configuration({'max_vel_x': 0.5, 'max_vel_theta': 1.0, 'min_in_place_vel_theta': 0.4})
			self.speed = mode

	def next_pos(self, x, y):
		rospy.loginfo("Moving to (%f, %f)" % (x, y))

		# calculate angle between current position and goal
		angle_to_goal = atan2(y, x)
		odom_quat = tf.transformations.quaternion_from_euler(0, 0, angle_to_goal)

		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = "base_link"
		goal.target_pose.header.stamp = rospy.Time.now()
		goal.target_pose.pose.position.x = x
		goal.target_pose.pose.position.y = y
		goal.target_pose.pose.orientation.x = odom_quat[0]
		goal.target_pose.pose.orientation.y = odom_quat[1]
		goal.target_pose.pose.orientation.z = odom_quat[2]
		goal.target_pose.pose.orientation.w = odom_quat[3]
		self.move_base.send_goal(goal)

		while not self.move_base.wait_for_result(rospy.Duration(1.0)) and not rospy.is_shutdown():
			# Get the current position
			pos = self.tfBuffer.lookup_transform("uwb_beacon", 'base_link', rospy.Time(0), rospy.Duration(2.0))
			# Cancel if we are close enough to goal
			diff = np.linalg.norm([pos.transform.translation.y, pos.transform.translation.x])
			if diff < self.dist_stop:
				rospy.loginfo("Goal within %0fm, canceling", self.dist_stop)
				self.move_base.cancel_goal()
				return

			if diff > 5:
				self.set_speed("fast")
			elif diff < 3:
				self.set_speed("slow")

		if self.move_base.get_state() == GoalStatus.SUCCEEDED:
			rospy.loginfo("The base moved to (%f, %f)" % (x, y))
		else:
			rospy.logerr("The base failed to (%f, %f). Error: %d" % (x, y, self.move_base.get_state()))
			exit(1)

	def on_shutdown(self):
		rospy.loginfo("Canceling all goals")
		self.move_base.cancel_all_goals()

	def run(self):
		rate = rospy.Rate(2.0)
		coords_last = (0, 0)
		angle_last = 0
		while not rospy.is_shutdown():
			# Get the current position
			beacon_pos = self.tfBuffer.lookup_transform('base_link', "uwb_beacon", rospy.Time(0), rospy.Duration(2.0))
			# Create a position 1m away by creating an inverse vector
			coords_cur = (beacon_pos.transform.translation.x, beacon_pos.transform.translation.y)
			angle_cur = atan2(coords_cur[1], coords_cur[0])

			# Check difference to last goal
			if ((np.linalg.norm(np.array(coords_cur) - np.array(coords_last)) > 0.5
			    and np.linalg.norm(np.array(coords_cur) > self.dist_stop))
			    or abs(angle_cur - angle_last) > 1.0):
				print "Setting new goal"
				# Set a new goal
				thread.start_new_thread(self.next_pos, (coords_cur[0], coords_cur[1]))
				coords_last = coords_cur
			rate.sleep()


if __name__ == "__main__":
	p = FollowUWB()
	p.run()
