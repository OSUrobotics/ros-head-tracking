#!/usr/bin/env python
import roslib; roslib.load_manifest('head_pose_estimation')
import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion 
from collections import deque
import numpy as np
import cv

pose_pub = None

cov =	[[  5.55998108e-07,  1.51052839e-06,  2.70912318e-07,  1.41404627e-06, -1.30121986e-06, -2.80605779e-07,  4.75892308e-06],
		 [  1.51052839e-06,  7.17208438e-06,  1.31837655e-06,  8.73036832e-06, -6.50006751e-06, -1.01956219e-06,  2.17534623e-05],
		 [  2.70912318e-07,  1.31837655e-06,  1.06275060e-06,  5.18499261e-06, -9.35650695e-07,  3.21164986e-07,  3.03461396e-06],
		 [  1.41404627e-06,  8.73036832e-06,  5.18499261e-06,  3.91587133e-05, -5.01898298e-06,  2.94139073e-06,  1.77374094e-05],
		 [ -1.30121986e-06, -6.50006751e-06, -9.35650695e-07, -5.01898298e-06,  8.66211501e-06,  1.67388675e-06, -2.39346681e-05],
		 [ -2.80605779e-07, -1.01956219e-06,  3.21164986e-07,  2.94139073e-06,  1.67388675e-06,  9.75778750e-07, -6.77562064e-06],
		 [  4.75892308e-06,  2.17534623e-05,  3.03461396e-06,  1.77374094e-05, -2.39346681e-05, -6.77562064e-06,  9.58577820e-05]]

kf = cv.CreateKalman(7, 7)
cv.SetIdentity(kf.measurement_matrix, cv.RealScalar(1))
cv.Copy(cv.fromarray(np.array(cov, dtype=np.float32)), kf.measurement_noise_cov)

def pose_sub(pose_msg):
	meas = np.array([[
					pose_msg.pose.position.x,
					pose_msg.pose.position.y,
					pose_msg.pose.position.z,
					pose_msg.pose.orientation.x,
					pose_msg.pose.orientation.y,
					pose_msg.pose.orientation.z,
					pose_msg.pose.orientation.w
				    ]], dtype=np.float32)
				
	meas_cv = cv.fromarray(meas.T.copy())
			
	if cv.CountNonZero(kf.state_pre) == 0:
		cv.Copy(meas_cv, kf.state_pre)
		cv.Copy(meas_cv, kf.state_post)
	
	filtered = np.asarray(cv.KalmanCorrect(kf, meas_cv))
	
	filtered_pose = PoseStamped(
		header	= pose_msg.header,
		pose	= Pose(Point(*filtered[:3]), Quaternion(*filtered[3:]))
	)
	pose_pub.publish(filtered_pose)
	cv.KalmanPredict(kf)

if __name__ == '__main__':
	rospy.init_node('head_pose_filter')
	rospy.Subscriber('head_pose', PoseStamped, pose_sub)
	pose_pub = rospy.Publisher('head_pose_filtered', PoseStamped)
	rospy.spin()
