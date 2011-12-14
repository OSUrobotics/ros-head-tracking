#!/usr/bin/env python
import roslib; roslib.load_manifest('rosmouse')
from geometry_msgs.msg import PoseStamped
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
	
	cov = np.matrix([[ 4.1208e3, 2.3380e3], [-5.3681,   1.9369e3]])
	filter = KalmanFilter(cov, 2, 2)
	
	def __init__(self):
		self.subscribe()
		self.calibrate()
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
				print help_msg3	

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
			move_mouse(x, y)
		else:
			with self.pose_lock:
				self.last_pose = pose
				
	def config_cb(self, config, level):
		#self.filter = MeanFilter(window_size=config['window_size'])
		cov = np.matrix([[ 4.1208e3, 2.3380e3], [-5.3681,   1.9369e3]])
		self.filter = KalmanFilter(cov, 2, 2)
		return config

if __name__ == '__main__':
	rospy.init_node('mouse')
	m = Mouse()
	rospy.spin()
