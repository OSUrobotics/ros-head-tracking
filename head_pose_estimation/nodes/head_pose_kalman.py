#!/usr/bin/env python
import roslib; roslib.load_manifest('head_pose_estimation')
import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Vector3
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from collections import deque
from state_filters import KalmanFilter
import numpy as np
import cv
from threading import RLock
pose_pub = None

cov_m = [
	[0.09, 0.00],
	[0.00, 0.09]
]

cov_p = [
	[0.28409617, 0.00000000],
	[0.00000000, 6.24341614]
]

class Filter(object):
	control_lock = RLock()
	kf = KalmanFilter(np.float32(cov_m), 2, 2, 2)
	control = np.float32([0,0])
	def __init__(self):
		self.kf.set_control_matrix(cv.fromarray(np.matrix([[1.0,0.0],[0.0,1.0]])))
	def pose_sub(self, pose_msg):
		r,p,y = euler_from_quaternion([
				pose_msg.pose.orientation.x,
				pose_msg.pose.orientation.y,
				pose_msg.pose.orientation.z,
				pose_msg.pose.orientation.w
		])					
					
		self.kf.update([p,y])
		with self.control_lock:
			filtered = self.kf.predict(self.control)
			self.control = np.float32([0,0])
		
		filtered_pose = PoseStamped(
			header	= pose_msg.header,
			pose	= Pose(
						pose_msg.pose.position,
						Quaternion(*quaternion_from_euler(r,filtered[0], filtered[1]))
					  )
		)
		pose_pub.publish(filtered_pose)
		

	def flow_cb(self, msg):
		with self.control_lock:
			self.control += [msg.x, msg.y]

if __name__ == '__main__':
	rospy.init_node('head_pose_filter')
	f = Filter()
	rospy.Subscriber('head_pose', PoseStamped, f.pose_sub)
	rospy.Subscriber('flow', Vector3, f.flow_cb)
	pose_pub = rospy.Publisher('head_pose_filtered', PoseStamped)
	rospy.spin()
