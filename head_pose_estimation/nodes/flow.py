#!/usr/bin/env python
import roslib; roslib.load_manifest('head_pose_estimation')
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3

lk_params = dict( winSize  = (15, 15), 
                  maxLevel = 2, 
                  criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03),
                  derivLambda = 0.0 )
                  
# feat_params = dict( maxCorners   = 500, 
#                     qualityLevel = 0.3,
#                     minDistance  = 7,
#                     blockSize    = 7 )

feat_params = dict( 
                    maxCorners   = 100,
                    qualityLevel = 0.01,
                    minDistance  = 5)


class OpticFlow(object):
    last_frame = None
    prev_pts = None
    bridge = CvBridge()
    vect_pub = rospy.Publisher('flow', Vector3)

    def image_cb(msg):
        im = self.bridge.imgmsg_to_cv(msg, 'mono8')
        if self.last_frame is not None:
            next_pts, status, err = cv2.calcOpticalFlowPyrLK(self.last_frame, frame, self.prev_pts, None, **lk_params)
            diffs = (next_pts - self.prev_pts).squeeze()
            mean = diffs.mean(0)
            sigma = diffs.std(0)
            # mask = (np.abs(diffs - [mean]*diffs.shape[0]) > 5*sigma).all(1)
            # mask = np.array([mask,mask]).T
            # shape = diffs.shape
            # next_pts_m = np.ma.masked_array(next_pts.squeeze(),mask=mask)
            # prev_pts_m = np.ma.masked_array(prev_pts.squeeze(),mask=mask)
            
            # center = np.array([im.shape[1], im.shape[0]])/2
            self.vect_pub.publish(mean[0], mean[1], 0)
            self.prev_pts = next_pts
            
        else:
            self.prev_pts = cv2.goodFeaturesToTrack(frame, **feat_params)
        self.last_frame = im

if __name__ == '__main__':
    rospy.init_node('optic_flow')
    flow = OpticFlow()
    rospy.Subscriber('image', Image, flow.image_cb)