from enum import Enum, IntEnum
import numpy as np
import scipy.interpolate


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

	def __init__(self, debug=False):
		# self.gesture = SwipeGesture.NONE
		self.in_gesture = False
		self.point_x = []
		self.point_y = []

		self.template_hori = np.zeros((2, self.INTERP_NUM))
		self.template_hori[0] = np.linspace(0, 1, self.INTERP_NUM)
		self.template_vert = np.zeros((2, self.INTERP_NUM))
		self.template_vert[1] = np.linspace(0, 1, self.INTERP_NUM)

		self.debug = debug

	def check(self):
		ret = SwipeGesture.NONE

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
			if self.debug:
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
			if self.debug:
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


if __name__ == '__main__':
	my_classifier = SwipeGestureClassifier()

	gesture = my_classifier.classify(0, 0, False)
	gesture = my_classifier.classify(0, 0.1, True)
	gesture = my_classifier.classify(0, 0.2, True)
	gesture = my_classifier.classify(0, 0.3, True)
	gesture = my_classifier.classify(0, 0.4, True)
	gesture = my_classifier.classify(0, 0.5, False)
	print(gesture)

	gesture = my_classifier.classify(0, 0, False)
	gesture = my_classifier.classify(0.1, 0, True)
	gesture = my_classifier.classify(0.2, 0, True)
	gesture = my_classifier.classify(0.3, 0, True)
	gesture = my_classifier.classify(0.4, 0, True)
	gesture = my_classifier.classify(0.5, 0, False)
	print(gesture)

	gesture = my_classifier.classify(0, -0, False)
	gesture = my_classifier.classify(0, -0.1, True)
	gesture = my_classifier.classify(0, -0.2, True)
	gesture = my_classifier.classify(0, -0.3, True)
	gesture = my_classifier.classify(0, -0.4, True)
	gesture = my_classifier.classify(0, -0.5, False)
	print(gesture)

	gesture = my_classifier.classify(-0, 0, False)
	gesture = my_classifier.classify(-0.1, 0, True)
	gesture = my_classifier.classify(-0.2, 0, True)
	gesture = my_classifier.classify(-0.3, 0, True)
	gesture = my_classifier.classify(-0.4, 0, True)
	gesture = my_classifier.classify(-0.5, 0, False)
	print(gesture)
