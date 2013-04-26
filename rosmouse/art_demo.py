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
import rospy
from geometry_msgs.msg import PointStamped
import pygame
from pygame.locals import * 
from math import sqrt, pi, exp, sin
import time

def gaussian(x, sigma):
	# sigma = 15.0
	mu = 1
	return exp(-(x-mu)**2/(2*sigma*sigma)) / (sigma*sqrt(2*pi))

class Line(object):
	brightness = 255
	def __init__(self, p1, p2):
		self.p1 = p1
		self.p2 = p2
		self.stamp = time.clock()
		
	def draw(self, surface):
		c = (0, int(self.brightness*(self.stamp/time.clock())**2), 0)
		pygame.draw.aaline(surface, c, (self.p1[0], self.p1[1]-1), (self.p2[0], self.p2[1]-1))
		pygame.draw.aaline(surface, c, self.p1, self.p2)
		pygame.draw.aaline(surface, c, (self.p1[0], self.p1[1]+1), (self.p2[0], self.p2[1]+1))
		

class Gaussian(object):
	brightness = 255
	def __init__(self, center, sigma=15.0, decay=False):
		self.sigma = sigma
		self.radius = self.find_radius()
		self.center = center
		self.stamp = time.clock()
		
	def find_radius(self):
		c_max = self.find_max()
		c = int(self.brightness*gaussian(1, self.sigma)/c_max)
		r = 2
		while c > 0:
			c = int(self.brightness*gaussian(r, self.sigma)/c_max)
			r += 1
		return r
		
	def find_max(self):
		return gaussian(1, self.sigma)
		
	# def draw(self, screen):
	#	c_max = self.find_max()
	#	TRANSPARENT = (255,0,255)
	#	for r in range(self.radius, 0, -1):
	#		s = pygame.Surface((r,r))
	#		# s.fill(TRANSPARENT)
	#		# s.set_colorkey(TRANSPARENT)
	#		red = int(self.brightness*gaussian(r, self.sigma)/c_max)
	#		c = (self.brightness, 0, 0)
	#		# pygame.draw.circle(screen, pygame.Color(self.brightness, 0, 0, 255-red), self.center, r)
	#		# s.fill((255, 255, 255, 255-red))
	#		
	#		# pygame.draw.circle(s, (0, self.brightness, 0, 255-red), self.center, r)
	#		pygame.draw.circle(s, (red,0,0), (r/2,r/2), r)
	#		s.set_alpha(red)
	#		screen.blit(s, (self.center[0]-r/2, self.center[1]-r/2))
	def draw(self, screen):
		pygame.draw.circle(screen, (int(self.brightness*(self.stamp/time.clock())**2), 0, 0), self.center, int(self.sigma))
			
		

class ArtDemo(object): 
	width = 500
	height = 500
	pos = None
	running = False
	gaussians = []

	def __init__(self, fullscreen=False, use_mouse=True):
		self.use_mouse = use_mouse
		self.screen = pygame.display.set_mode((self.width, self.height))
		if fullscreen:
			modes = pygame.display.list_modes()
			pygame.display.set_mode(modes[0], FULLSCREEN)
		
		pygame.mouse.set_visible(False)

	def process_events(self, events): 
		for event in events: 
			if event.type == QUIT: 
				self.running = False
			elif event.type == KEYUP:
				if event.key == 27:
					self.running = False
			elif (event.type == MOUSEMOTION) and self.use_mouse:
				self.pos = event.pos
			else: 
				# print event
				pass
			
	def run(self):
		self.running = True
		last_pos = self.pos
		while self.running:
			self.process_events(pygame.event.get())
			if not self.pos: continue
			if not last_pos:
				last_pos = self.pos
				continue
			self.screen.fill((0,0,0))
			# pygame.draw.circle(self.screen, (255,0,0), (50,50), 5)
			# self.draw_gaussian(50, self.pos)
			# self.gaussians.append(Gaussian(self.pos))
			# for g in self.gaussians:
			#	g.draw(self.screen)
			self.gaussians.append(Line(self.pos, last_pos))
			for g in self.gaussians:
				g.draw(self.screen)
			pygame.draw.circle(self.screen, (0, 255*abs(sin(time.clock()*2)), 0), self.pos, int(10*abs(sin(time.clock()*2))))

			#if len(self.gaussians) > 100:
			#	Line(self.pos, self.gaussians[-100].p1).draw(self.screen)


			pygame.display.flip()
			if self.gaussians:
				last_pos = self.gaussians[-1].p1
			else:
				last_pos = self.pos


def mouse_cb(point, d):
	d.pos = int(point.point.x), int(point.point.y)

if __name__ == '__main__':
	d = ArtDemo(fullscreen=True, use_mouse=True)
	rospy.init_node('art_demo')
	rospy.Subscriber('mouse', PointStamped, mouse_cb, d)
	d.run()
