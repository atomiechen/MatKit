import argparse

from collections import deque
from enum import Enum, IntEnum
import numpy as np
import scipy.interpolate

from toolkit.uclient import Uclient
from toolkit.process import Processor, CursorController
from toolkit.tools import load_config, blank_config


CONFIG_PATH = "config/tmp_config_mouth_16_16.yaml"
LEFT = 0.1
RIGHT = 0.9
UP = 0.1
DOWN = 0.9


## swipe gestures
class SwipeGesture(IntEnum):
	NONE = 0
	LEFT = 1
	RIGHT = 2
	UP = 3
	DOWN = 4

class SwipeGestureClassifier:

	INTERP_NUM = 40
	THRESHOLD = 5

	def __init__(self):
		# self.gesture = SwipeGesture.NONE
		self.in_gesture = False
		self.point_x = []
		self.point_y = []

		self.template_hori = np.zeros((2, self.INTERP_NUM))
		self.template_hori[0] = np.linspace(0, 1, self.INTERP_NUM)
		self.template_vert = np.zeros((2, self.INTERP_NUM))
		self.template_vert[1] = np.linspace(0, 1, self.INTERP_NUM)

	def check(self):
		ret = SwipeGesture.NONE

		print(len(self.point_x), len(self.point_y))

		## !!! Note: splprep cannot handle < successive identical points >
		## ref: https://stackoverflow.com/a/61859042/11854304
		try:
			tck, u = scipy.interpolate.splprep([self.point_x, self.point_y], s=0)
		except Exception as e:
			print(e)
			return ret
		new_points = scipy.interpolate.splev(np.linspace(0,1,self.INTERP_NUM), tck)
		new_points = np.array(new_points)

		## reverse the swipe direction or not
		reverse = False

		## normalize the long axis to be within [0, 1]
		x_range = new_points[0].max() - new_points[0].min()
		y_range = new_points[1].max() - new_points[1].min()
		if x_range > y_range:
			## horizontal
			new_points[0] -= new_points[0].min()
			new_points *= 1 / x_range
			if new_points[0][-1] - new_points[0][0] < 0:
				new_points = np.flip(new_points, axis=1)
				reverse = True
			## compare with template
			diff = new_points - self.template_hori
			sim = np.linalg.norm(diff, axis=0).mean()
			print(f"horizontal sim: {sim}  reverse: {reverse}")
			if sim <= self.THRESHOLD:
				if reverse:
					ret = SwipeGesture.LEFT
				else:
					ret = SwipeGesture.RIGHT

		else:
			## vertical
			new_points[1] -= new_points[1].min()
			new_points *= 1 / y_range
			if new_points[1][-1] - new_points[1][0] < 0:
				new_points = np.flip(new_points, axis=1)
				reverse = True
			## compare with template
			diff = new_points - self.template_vert
			sim = np.linalg.norm(diff, axis=0).mean()
			print(f"vertical sim: {sim}  reverse: {reverse}")
			if sim <= self.THRESHOLD:
				if reverse:
					ret = SwipeGesture.UP
				else:
					ret = SwipeGesture.DOWN

		return ret

		# new_points[0] -= new_points[0].min()
		# new_points[0] *= 1 / (new_points[0].max() - new_points[0].min())
		# new_points[1] -= new_points[1].min()
		# new_points[1] *= 1 / (new_points[1].max() - new_points[1].min())

		# center = new_points.mean(axis=1).reshape(2,-1)
		# new_points -= center0

		# new_points[0].std()
		# new_points[1].std()


	def classify(self, x, y, moving):
		gesture = SwipeGesture.NONE
		if moving:
			if len(self.point_x) == 0 or self.point_x[-1] != x or self.point_y[-1] != y:
				self.point_x.append(x)
				self.point_y.append(y)
			self.in_gesture = True
		else:
			if self.in_gesture:
				gesture = self.check()
				self.point_x = []
				self.point_y = []
			self.in_gesture = False
		return gesture


def mapping(x, y, left=0, right=1, up=0, down=1):
	x0 = 1-y
	y0 = 1-x
	x1 = min(max(x0 - left, 0) / (right - left), 0.999)
	y1 = min(max(y0 - up, 0) / (down - up), 0.999)
	return x1, y1

def main():
	## load config and combine commandline arguments
	if args.config:
		config = load_config(args.config)
	else:
		config = blank_config()

	my_processor = Processor(
		config['process']['interp'], 
		blob=config['process']['blob'], 
		threshold=config['process']['threshold'],
		order=config['process']['interp_order'],
		total=config['process']['blob_num'],
		special=config['process']['special_check'],
	)
	my_processor.print_info()
	my_cursor = CursorController(
		mapcoor=config['pointing']['direct_map'],
		alpha=config['pointing']['alpha'], 
		trackpoint=config['pointing']['trackpoint']
	)
	my_cursor.print_info()

	with Uclient(
		config['connection']['client_address'], 
		config['connection']['server_address'], 
		udp=config['connection']['udp'], 
		n=config['sensor']['shape']
	) as my_client:
		my_classifier = SwipeGestureClassifier()
		frame_idx = 0
		while True:
			## get a new frame
			frame, frame_idx = my_client.fetch_frame_and_index(new=True)

			row, col, val = my_processor.parse(frame)
			moving, x, y, val = my_cursor.update(row, col, val)
			x, y = mapping(x, y, left=LEFT, right=RIGHT, up=UP, down=DOWN)

			gesture = my_classifier.classify(x, y, moving)
			if gesture != SwipeGesture.NONE:
				print(f"gesture: {gesture.name}")


if __name__ == '__main__':
	parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
	parser.add_argument('--config', dest='config', action='store', default=CONFIG_PATH, help="specify configuration file")
	args = parser.parse_args()

	main()
