#!/usr/bin/env python

import roslib; roslib.load_manifest('head_pose_estimation')
import rospy
from sensor_msgs.msg import RegionOfInterest, Image, CameraInfo
import message_filters
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PolygonStamped, Point32, PointStamped
import numpy as np
import dynamic_reconfigure.client
from threading import RLock
import contextlib # get rid of this once PR2 moves to python 2.7
from rospy.service import ServiceException

class Extract(object):
	bridge = CvBridge()

	xfilter = dynamic_reconfigure.client.Client('xfilter')
	yfilter = dynamic_reconfigure.client.Client('yfilter')
	zfilter = dynamic_reconfigure.client.Client('zfilter')
	
	depth_im = None
	roi      = None

	image_lock = RLock()
	roi_lock   = RLock()
	
	K = []

	def __init__(self):
		roi_sub   = rospy.Subscriber('roi', RegionOfInterest, self.roi_cb)
		depth_sub = message_filters.Subscriber('depth_image', Image)
		info_sub  = message_filters.Subscriber('camera_info', CameraInfo)
		
		ts = message_filters.TimeSynchronizer([depth_sub, info_sub], 10)
		ts.registerCallback(self.depth_cb);
		
		r = rospy.Rate(10)
		while not rospy.is_shutdown() and not self.roi and not self.depth_im:
			r.sleep()
		while not rospy.is_shutdown():
			self.update_params()
			r.sleep()

	def roi_cb(self, msg):
		with self.roi_lock:
			self.roi = msg

	def depth_cb(self, depth_msg, info_msg):
		with self.image_lock:
			self.depth_im = self.bridge.imgmsg_to_cv(depth_msg, '32FC1')
			self.K = info_msg.K

	def update_params(self):
		with contextlib.nested(self.roi_lock, self.image_lock):
			if self.depth_im and self.roi:
				x = self.roi.x_offset
				y = self.roi.y_offset
				w = self.roi.width
				h = self.roi.height
				
				depth = self.depth_im[y+h/2,x+w/2]

				face_center_img = np.array((x+w/2.0, y+h/2))
				face_center_pt  = (face_center_img - (self.K[2], self.K[5])) / np.array([self.K[0], self.K[4]])
				face_center_depth = self.depth_im[face_center_img]
				
				try:
					self.xfilter.update_configuration({
						'filter_field_name': 'x',
						'filter_limit_min': face_center_pt[0]-0.2,
						'filter_limit_max': face_center_pt[0]+0.2
					})

					self.yfilter.update_configuration({
						'filter_field_name': 'y',
						'filter_limit_min': face_center_pt[1]-0.2,
						'filter_limit_max': face_center_pt[1]+0.2
					})

					self.zfilter.update_configuration({
						'filter_field_name': 'z',
						'filter_limit_min': depth-0.2,
						'filter_limit_max': depth+0.2
					})
				except ServiceException:
					rospy.logwarn("Service exception with dynamic reconfigure")
			

if __name__ == '__main__':
	rospy.init_node('extract_head_cloud')
	Extract()
	rospy.spin()
