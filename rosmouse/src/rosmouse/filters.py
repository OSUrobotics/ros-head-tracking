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
	def __init__(self, cov, dynam_params, measure_params):
		self.kf = cv.CreateKalman(dynam_params, measure_params)
		cv.SetIdentity(self.kf.measurement_matrix, cv.RealScalar(1))
		self.set_cov(cov)

	def set_cov(self, cov):
		cov = np.float32(cov)
		cv.Copy(cv.fromarray(cov), self.kf.measurement_noise_cov)
		assert np.all(cov == np.asarray(self.kf.measurement_noise_cov)), "Covariance matrix didn't get set"
		
	def observation(self, meas):
		meas = np.array([meas], dtype=np.float32)
		cv.KalmanPredict(self.kf)
		corrected = np.asarray(cv.KalmanCorrect(self.kf, cv.fromarray(meas.T.copy())))
		return corrected.T[0].copy()

if __name__ == '__main__':
	def make_obs(x):
		#gt = np.array([np.sin(x/75.0), np.cos(x/75.0)])*100
        # gt = [0,0]
		gt = [ 2361.90541702,   352.84006879]
		# noise = np.random.multivariate_normal((0,0), cov)
		# return gt, gt+noise
		return gt, np.random.multivariate_normal(gt, cov)
	
	import matplotlib.pyplot as plt
	cov = np.matrix([[ 4.1208e3, 2.3380e3], [-5.3681,   1.9369e3]])
	f = KalmanFilter(cov, 2, 2)

	observations = []
	filtered = []
	gts = []
	
	for gt, obs in (make_obs(x) for x in xrange(1000)):
		gts.append(gt)
		observations.append(obs)
		corrected = f.observation(obs)
		filtered.append(corrected)
	
	observations = np.array(observations)
	filtered = np.array(filtered)
	gts = np.array(gts)
	print observations[5:,0].std(), filtered[5:,0].std()
	# plt.clf()
	# plt.plot(observations[:,0])
	# plt.plot(filtered[:,0])
	# plt.show()
