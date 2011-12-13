from collections import deque
import numpy as np

class MeanFilter(object):
	def __init__(self, window_size=3):
		self.window_size = window_size
		self.observations = deque([], window_size)
	def observation(self, obs):
		self.observations.append(obs)
		weights = range(len(self.observations), 0, -1)
		return np.int32(np.average(self.observations, axis=0, weights=weights)).tolist()
