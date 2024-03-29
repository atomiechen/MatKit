"""
Utility function tools.

"""

import numpy as np

def gen(data):
	"""generator function to wrap data into a generator
	
	Args:
		data (Any type): input data
	
	Yields:
		data (Any type): the same as input data
	"""
	while True:
		try:
			yield data
		except GeneratorExit:
			return

def gen_reshape(data, N):
	while True:
		try:
			yield np.reshape(data, (N, N))
		except GeneratorExit:
			return
