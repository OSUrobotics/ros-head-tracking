#!/usr/bin/env python
# Copyright (c) 2013, Oregon State University
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Oregon State University nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL OREGON STATE UNIVERSITY BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Author Dan Lazewatsky/lazewatd@engr.orst.edu

import roslib; roslib.load_manifest('rosmouse')
from geometry_msgs.msg import PoseStamped, PointStamped
from ctypes import cdll
import Xlib.display
import rospy
from headmath.conversions import pose_quat_to_euler, pose_euler_to_quat
from threading import RLock, Thread
from functools import wraps
from math import tan
from rosmouse.filters import MeanFilter, KalmanFilter
from dynamic_reconfigure.server import Server
from rosmouse.cfg import MouseConfig
import numpy as np

def move_mouse(x,y):
    dll = cdll.LoadLibrary('libX11.so')
    d = dll.XOpenDisplay(None)
    root = dll.XDefaultRootWindow(d)
    dll.XWarpPointer(d,None,root,0,0,0,0,x,y)
    dll.XCloseDisplay(d)

def thread(func):
	@wraps(func)
	def wrap(*args, **kwargs):
		t = Thread(target=func, args=args, kwargs=kwargs)
		t.start()
		return t        
	return wrap

def InterpolationFactory(x1, x2, y1, y2):
	def fun(X):
		return int((y2-y1)/(x2-x1)*(X-x1)+y1)
	return fun

class Mouse(object):
	yaw_left   = None
	yaw_right  = None
	pitch_up   = None
	pitch_dn   = None
	screen_res = None
	fn_ready = False
	ready = False
	run = False
	
	xFun = None
	yFun = None
	
	last_pose = None
	pose_lock = RLock()
	
	do_move_mouse = True
    
    # cov = np.matrix([[ 4.1208e3, 2.3380e3], [-5.3681,   1.9369e3]])
	cov = np.matrix([[6.71485028,  18.75061549], [ 14.25320564,  82.71265061]])
	filter = KalmanFilter(cov, 2, 2)
	
	def __init__(self):
		self.do_move_mouse = rospy.get_param('~move_mouse', False)
		self.subscribe()
		self.calibrate()
		self.mouse_pub = rospy.Publisher('mouse', PointStamped)
		self.listen_for_keyboard()

	@thread
	def listen_for_keyboard(self):
		help_msg  = 'Keboard commands:\n'
		help_msg += '	Recalibrate:   c\n'
		help_msg += '	Disable Mouse: d\n'
		help_msg += '	Enable Mouse:  e\n'
		help_msg += '	Print Help:    h\n'
		help_msg += '	Exit:          q\n'
		
		print help_msg
		
		while not rospy.is_shutdown():
			cmd = raw_input('-->')
			if cmd == 'c':
				self.calibrate()
			elif cmd == 'd':
				self.ready = False
			elif cmd == 'e':
				self.ready = True
			elif cmd == 'q':
				rospy.signal_shutdown(0)
			elif cmd == 'h':
				print help_msg

	@thread
	def calibrate_threaded(self):
		self.fn_ready = False
		raw_input('Look at the left edge of the screen')
		with self.pose_lock:
			self.yaw_left = self.last_pose[5]
		raw_input('Look at the right edge of the screen')
		with self.pose_lock:
			self.yaw_right = self.last_pose[5]
		raw_input('Look at the top edge of the screen')
		with self.pose_lock:
			self.pitch_up = self.last_pose[4]
		raw_input('Look at the bottom edge of the screen')
		with self.pose_lock:
			self.pitch_dn = self.last_pose[4]
		self.fn_ready = True
	
	def calibrate(self):
		self.ready = False
		screen = Xlib.display.Display().screen()
		res = (screen['width_in_pixels'], screen['height_in_pixels'])
		while not self.last_pose is not None and not rospy.is_shutdown():
			rospy.sleep(0.1)
		self.calibrate_threaded()
		while not self.fn_ready and not rospy.is_shutdown():
			rospy.sleep(0.1)
			
		self.xFun = InterpolationFactory(tan(self.yaw_left), tan(self.yaw_right), 0, res[0])
		self.yFun = InterpolationFactory(tan(self.pitch_up), tan(self.pitch_dn),  0, res[1])
			
		self.ready = True
		
	def subscribe(self):
		rospy.Subscriber('head_pose', PoseStamped, self.pose_sub)
		self.cfg_srv = Server(MouseConfig, self.config_cb)
		
	def pose_sub(self, pose_msg):
		pose = pose_quat_to_euler(pose_msg)
		if self.ready:
			obs = [self.xFun(tan(pose[5])), self.yFun(tan(pose[4]))]
			x, y = [int(n) for n in self.filter.observation(obs)]
			if self.do_move_mouse:
				move_mouse(x, y)
			mouse_msg = PointStamped()
			mouse_msg.header.stamp = rospy.Time.now()
			mouse_msg.point.x = x
			mouse_msg.point.y = y
			mouse_msg.point.z = 0
			self.mouse_pub.publish(mouse_msg)
		else:
			with self.pose_lock:
				self.last_pose = pose
				
	def config_cb(self, config, level):
		cov = np.matrix([[ 3844.37658853,  1483.79897381], [ 1483.79897381,  2648.80916764]])
		if config['filter_type'] == 1:
			rospy.loginfo('Setting window size to %s' % config['window_size'])
			self.filter = MeanFilter(window_size=config['window_size'])
		elif config['filter_type'] == 2:
			if type(self.filter) == KalmanFilter:
				rospy.loginfo('Setting covariance multiplier to %s' % config['cov_mul'])
				self.filter.set_cov(cov*config['cov_mul'])
			else:
				rospy.loginfo('Switching to Kalman Filter with covariance multiplier to %s' % config['cov_mul'])
				self.filter = KalmanFilter(cov*config['cov_mul'], 2, 2)
				
		return config

if __name__ == '__main__':
	rospy.init_node('mouse')
	m = Mouse()
	rospy.spin()
