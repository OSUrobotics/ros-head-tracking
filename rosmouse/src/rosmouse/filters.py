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

from collections import deque
import numpy as np
import cv

class MeanFilter(object):
	def __init__(self, window_size=3):
		self.window_size = window_size
		self.observations = deque([], window_size)
	def observation(self, obs):
		self.observations.append(obs)
		weights = range(len(self.observations), 0, -1)
		return np.int32(np.average(self.observations, axis=0, weights=weights)).tolist()

class KalmanFilter(object):
	initialized = False
	def __init__(self, cov, dynam_params, measure_params, control_params=0):
		self.kf = cv.CreateKalman(dynam_params, measure_params, control_params)
		cv.SetIdentity(self.kf.measurement_matrix, cv.RealScalar(1))
		self.set_cov(cv.fromarray(cov))

	def set_cov(self, cov):
		cov = np.float32(cov)
		cv.Copy(cv.fromarray(cov), self.kf.measurement_noise_cov)
		assert np.all(cov == np.asarray(self.kf.measurement_noise_cov)), "Covariance matrix didn't get set"
		
	def set_control(self, cont):
		cont = np.float32(cont)
		cv.Copy(cv.fromarray(cont), self.kf.control_matrix)
		assert np.all(cont == np.asarray(self.kf.control_matrix)), "Control matrix didn't get set"
		
	def observation(self, meas):
		meas = np.float32([meas])
		if not self.initialized:
			cv.Copy(cv.fromarray(meas.T.copy()), self.kf.state_post)
			cv.Copy(cv.fromarray(meas.T.copy()), self.kf.state_pre)
			self.initialized = True
		if self.kf.CP == 0:
			cv.KalmanPredict(self.kf)
		corrected = np.asarray(cv.KalmanCorrect(self.kf, cv.fromarray(meas.T.copy())))
		return corrected.T[0].copy()
		
	def control(self, signal):
		signal = np.float32([signal])
		if not self.initialized:
			return np.zeros(1,self.kf.DP)
		predicted = np.asarray(cv.KalmanPredict(self.kf, cv.fromarray(signal.T.copy())))
		return predicted.T[0].copy()


if __name__ == '__main__':
	def make_obs(x):
		#gt = np.array([np.sin(x/75.0), np.cos(x/75.0)])*100
		gt = [x,x]
		# gt = [ 2361.90541702 + x,	352.84006879 + x ]
		# noise = np.random.multivariate_normal((0,0), cov)
		# return gt, gt+noise
		return gt, np.random.multivariate_normal(gt, cov)
	
	import matplotlib.pyplot as plt
	# cov = np.matrix([[ 4.1208e3, 2.3380e3], [-5.3681,	1.9369e3]])
	cov = np.ones((2,2))*100
	f = KalmanFilter(cov, 2, 2, 2)
	f.set_control(cv.fromarray(np.matrix([[1.0,0.0],[0.0,1.0]])))
	print np.asarray(f.kf.control_matrix)

	observations = []
	filtered = []
	gts = []
	
	for gt, obs in (make_obs(x) for x in xrange(0,100,1)):
		gts.append(gt)
		# print obs
		observations.append(obs)
		
		corrected = f.observation(obs)
		# filtered.append(corrected)
		predicted = f.control([1,0])
		filtered.append(corrected)
	
	observations = np.array(observations)
	filtered = np.array(filtered)
	gts = np.array(gts)
	# print observations.std(0), filtered.std(0)
	plt.clf()
	plt.plot(observations[:,0])
	plt.hold(True)
	plt.plot(filtered[:,0])
	plt.legend(['obs', 'filtered'])
	plt.show()