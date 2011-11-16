#!/usr/bin/env python
import roslib; roslib.load_manifest('head_pose_estimation')
import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion 
from collections import deque
import numpy as np

poses = None
pose_pub = None

def pose_sub(pose_msg):
	poses.append([
					pose_msg.pose.position.x,
					pose_msg.pose.position.y,
					pose_msg.pose.position.z,
					pose_msg.pose.orientation.x,
					pose_msg.pose.orientation.y,
					pose_msg.pose.orientation.z,
					pose_msg.pose.orientation.w
				 ])
				 
	if len(poses) == poses.maxlen:
		filtered = np.median(np.array(poses), 0)
		filtered_pose = PoseStamped(
			header	= pose_msg.header,
			pose	= Pose(Point(*filtered[:3]), Quaternion(*filtered[3:]))
		)
		pose_pub.publish(filtered_pose)


if __name__ == '__main__':
	rospy.init_node('head_pose_filter')
	
	window_size = rospy.get_param("~window_size", 3)
	rospy.loginfo("window size is: %s" % window_size)
	poses = deque([], window_size)
	rospy.Subscriber('head_pose', PoseStamped, pose_sub)
	pose_pub = rospy.Publisher('head_pose_filtered', PoseStamped)
	rospy.spin()
