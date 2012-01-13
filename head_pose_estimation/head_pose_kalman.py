#!/usr/bin/env python
import roslib; roslib.load_manifest('head_pose_estimation')
import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion 
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from collections import deque
import numpy as np
import cv

pose_pub = None

#cov =	[[  5.55998108e-07,  1.51052839e-06,  2.70912318e-07,  1.41404627e-06, -1.30121986e-06, -2.80605779e-07,  4.75892308e-06],
#		 [  1.51052839e-06,  7.17208438e-06,  1.31837655e-06,  8.73036832e-06, -6.50006751e-06, -1.01956219e-06,  2.17534623e-05],
#		 [  2.70912318e-07,  1.31837655e-06,  1.06275060e-06,  5.18499261e-06, -9.35650695e-07,  3.21164986e-07,  3.03461396e-06],
#		 [  1.41404627e-06,  8.73036832e-06,  5.18499261e-06,  3.91587133e-05, -5.01898298e-06,  2.94139073e-06,  1.77374094e-05],
#		 [ -1.30121986e-06, -6.50006751e-06, -9.35650695e-07, -5.01898298e-06,  8.66211501e-06,  1.67388675e-06, -2.39346681e-05],
#		 [ -2.80605779e-07, -1.01956219e-06,  3.21164986e-07,  2.94139073e-06,  1.67388675e-06,  9.75778750e-07, -6.77562064e-06],
#		 [  4.75892308e-06,  2.17534623e-05,  3.03461396e-06,  1.77374094e-05, -2.39346681e-05, -6.77562064e-06,  9.58577820e-05]]

cov_m = [[ 6.65921452e-07, -9.26203669e-07,  7.80620204e-07,  1.25446877e-06, -5.00137301e-06, -6.32387447e-04],
		 [-9.26203669e-07,  3.29044761e-06, -2.37407896e-06, -3.99733843e-06,  1.76915711e-05,  1.07968331e-03],
		 [ 7.80620204e-07, -2.37407896e-06,  2.54476706e-06,  2.77106225e-06, -1.76503533e-05, -1.19405279e-03],
		 [ 1.25446877e-06, -3.99733843e-06,  2.77106225e-06,  1.26514645e-05, -2.23517956e-05, -1.72435948e-03],
		 [-5.00137301e-06,  1.76915711e-05, -1.76503533e-05, -2.23517956e-05,  1.67134668e-04,  7.30493336e-03],
		 [-6.32387447e-04,  1.07968331e-03, -1.19405279e-03, -1.72435948e-03,  7.30493336e-03,  2.22511800e+00]]

#cov = np.zeros([6,6])


cov_m = [[
		 [0.25, 0.00, 0.00, 0.00, 0.00, 0.00],
		 [0.00, 0.25, 0.00, 0.00, 0.00, 0.00],
		 [0.00, 0.00, 0.25, 0.00, 0.00, 0.00],
		 [0.00, 0.00, 0.00, 0.09, 0.00, 0.00],
		 [0.00, 0.00, 0.00, 0.00, 0.09, 0.00],
		 [0.00, 0.00, 0.00, 0.00, 0.00, 0.09]]]

cov_p =	[[
		 [0.09843394, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000],
		 [0.00000000, 0.11289739, 0.00000000, 0.00000000, 0.00000000, 0.00000000],
		 [0.00000000, 0.00000000, 0.02674426, 0.00000000, 0.00000000, 0.00000000],
		 [0.00000000, 0.00000000, 0.00000000, 0.12437708, 0.00000000, 0.00000000],
		 [0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.28409617, 0.00000000],
		 [0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 6.24341614]]]

kf = cv.CreateKalman(6, 6)
cv.SetIdentity(kf.measurement_matrix, cv.RealScalar(1))
cv.Copy(cv.fromarray(np.array(cov_m, dtype=np.float32).T.copy()), kf.measurement_noise_cov)
cv.Copy(cv.fromarray(np.array(cov_p, dtype=np.float32).T.copy()), kf.process_noise_cov)

def pose_sub(pose_msg):
	# meas = np.array([[
	# 				pose_msg.pose.position.x,
	# 				pose_msg.pose.position.y,
	# 				pose_msg.pose.position.z,
	# 				pose_msg.pose.orientation.x,
	# 				pose_msg.pose.orientation.y,
	# 				pose_msg.pose.orientation.z,
	# 				pose_msg.pose.orientation.w
	# 			    ]], dtype=np.float32)
								
	meas = np.concatenate([
		[
			pose_msg.pose.position.x,
			pose_msg.pose.position.y,
			pose_msg.pose.position.z
		],
		euler_from_quaternion([
			pose_msg.pose.orientation.x,
			pose_msg.pose.orientation.y,
			pose_msg.pose.orientation.z,
			pose_msg.pose.orientation.w
	    ])])						
	meas = np.array([meas], dtype=np.float32)
	meas_cv = cv.fromarray(meas.T.copy())
			
#	if cv.CountNonZero(kf.state_pre) == 0:
#		cv.Copy(meas_cv, kf.state_pre)
#		cv.Copy(meas_cv, kf.state_post)
	
	filtered = np.asarray(cv.KalmanCorrect(kf, meas_cv))
	
	print filtered
	
	filtered_pose = PoseStamped(
		header	= pose_msg.header,
		pose	= Pose(
					Point(*filtered[:3]),
					Quaternion(*(quaternion_from_euler(*filtered[3:])))
				  )
	)
	pose_pub.publish(filtered_pose)
	cv.KalmanPredict(kf)

if __name__ == '__main__':
#	import sys; sys.exit(1)
	rospy.init_node('head_pose_filter')
	rospy.Subscriber('head_pose', PoseStamped, pose_sub)
	pose_pub = rospy.Publisher('head_pose_filtered', PoseStamped)
	rospy.spin()
