#!/usr/bin/env python
import roslib; roslib.load_manifest('head_pose_estimation')
import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion 
from headmath.conversions import pose_quat_to_euler, pose_euler_to_quat
from collections import deque
import numpy as np



poses = None
pose_pub = None

def pose_sub(pose_msg):
	poses.append(pose_quat_to_euler(pose_msg))
				 
	if len(poses) == poses.maxlen:
		filtered = np.median(np.array(poses), 0)
		pose_pub.publish(
			PoseStamped(
				header = pose_msg.header,
				pose   = pose_euler_to_quat(filtered)
			)
		)


if __name__ == '__main__':
	rospy.init_node('head_pose_filter')
	
	window_size = rospy.get_param("~window_size", 3)
	rospy.loginfo("window size is: %s" % window_size)
	poses = deque([], window_size)
	rospy.Subscriber('head_pose', PoseStamped, pose_sub)
	pose_pub = rospy.Publisher('head_pose_filtered', PoseStamped)
	rospy.spin()
