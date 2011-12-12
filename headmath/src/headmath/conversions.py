import roslib; roslib.load_manifest('head_pose_estimation')
import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion 
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np


def pose_quat_to_euler(pose_msg):
	return np.concatenate([
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
	
def pose_euler_to_quat(pose):
	return Pose(
		Point(*pose[:3]),
		Quaternion(*(quaternion_from_euler(*pose[3:])))
	  )
