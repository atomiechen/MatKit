import numpy as np

from .blob_parser import BlobParser
from .interpolator import Interpolator


class Processor:
	def __init__(self, interp=16, **kwargs):
		self.interp = interp
		self.noblob = False  # enable blob detection by default
		self.threshold = 0.1
		self.total = 3
		self.order = 3
		self.normalize = True  # normalize point coordinates to [0, 1]
		self.alpha = 1  # no smoothing
		self.config(**kwargs)

		self.interpolator = Interpolator(self.interp)
		self.blobparser = BlobParser(self.interp)
	
	def config(self, *, noblob=None, threshold=None, total=None, 
				order=None, normalize=None):
		if noblob is not None:
			self.noblob = noblob
		if threshold is not None:
			self.threshold = threshold
		if total is not None:
			self.total = total
		if order is not None:
			self.order = order
		if normalize is not None:
			self.normalize = normalize

	def gen_wrapper(self, generator, **kwargs):
		self.config(**kwargs)
		generator = self.interpolator.gen_wrapper(generator, order=self.order)
		if not self.noblob:
			generator = self.blobparser.gen_wrapper(generator, 
							threshold=self.threshold, total=self.total)
		return generator

	def gen_points(self, generator, **kwargs):
		self.config(**kwargs)
		if self.noblob:
			raise Exception("Must set noblob=True")
		generator = self.interpolator.gen_wrapper(generator, order=self.order)
		generator = self.blobparser.gen_points(generator, 
						threshold=self.threshold, total=self.total, 
						normalize=self.normalize)
		return generator

	def transform(self, data, reshape=False, **kwargs):
		self.config(**kwargs)
		if reshape:
			size = np.shape(data)[0]
			side = int(size**0.5)
			data = np.reshape(data, (side, side))
		data_out = self.interpolator.interpolate(data, order=self.order)
		if not self.noblob:
			data_out = self.blobparser.transform(data_out,
						threshold=self.threshold, total=self.total)
		if reshape:
			data_out = np.reshape(data_out, self.interp * self.interp)
		return data_out

	def parse(self, data, reshape=False, **kwargs):
		self.config(**kwargs)
		if self.noblob:
			raise Exception("Must set noblob=True")
		if reshape:
			size = np.shape(data)[0]
			side = int(size**0.5)
			data = np.reshape(data, (side, side))
		data_out = self.interpolator.interpolate(data, order=self.order)
		return self.blobparser.parse(data_out, threshold=self.threshold, 
				total=self.total, normalize=self.normalize)

	def print_info(self):
		print("Processor details:")
		msg_interp = "{0} × {0}".format(self.interp)
		print(f"  Interpolation:          {msg_interp}")
		if not self.noblob:
			print(f"  Blob filtered out:      threshold = {self.threshold}")
			print(f"  Normalized coordinates: {self.normalize}")
		else:
			print(f"  No blob filtered out")
