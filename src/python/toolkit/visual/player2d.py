from collections import deque
import time
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib.offsetbox import AnchoredText


class Player2D:
	## generator: data generator
	## timespan: visible time span, in seconds
	## ytop & ybottom: y-axis range, auto calculated by Matplotlib if not set
	## fps: frame rate, in frames per second
	## show_value: show value on top or not
	def __init__(self, generator, timespan=5, ytop=None, ybottom=None, fps=100, show_value=True):
		self.timespan = timespan
		self.generator = generator
		self.fps = fps
		self.ytop = ytop
		self.ybottom = ybottom
		self.show_value = show_value

		## data
		self.x = deque([])
		self.y = deque([])
		self.cur_x = 0

		## visualization
		self.fig = plt.figure()
		self.ax = self.fig.add_subplot(1,1,1)

		## timestamps
		self.start_time = None
		self.cur_time = None
		self.pause_time = None

	def _draw(self, data):
		if self.start_time is None:
			self.start_time = time.time()
			self.cur_time = self.start_time
		else:
			self.cur_time = time.time()

		self.cur_x = self.cur_time - self.start_time
		self.x.append(self.cur_x)
		self.y.append(data)

		if self.cur_x - self.x[0] > self.timespan:
			self.x.popleft()
			self.y.popleft()

		self.ax.clear()
		self.ax.plot(self.x, self.y)
		self.ax.set_xlim(left=max(-0.1, self.cur_x-self.timespan), right=self.cur_x+self.timespan*0.3)
		if self.ytop is not None:
			self.ax.set_ylim(top=self.ytop)
		if self.ybottom is not None:
			self.ax.set_ylim(bottom=self.ybottom)

		if self.show_value:
			value_str = f"Value: {data}"

			plt.text(0.01, 1.02, value_str, transform=plt.gca().transAxes)
			# self.ax.add_artist(
			# 	AnchoredText(
			# 		value_str, loc='lower left', pad=0.4, borderpad=0,
			# 		bbox_to_anchor=(0., 1.), 
			# 		bbox_transform=plt.gca().transAxes, 
			# 		prop=dict(size=10), frameon=False))


	def _start(self):
		plt.show()

	def _close(self):
		plt.close()

	def toggle_pause(self, *args, **kwargs):
		self.pause ^= True
		## make sure the curve is continuous
		if self.pause:
			self.pause_time = time.time()
		else:
			self.start_time += time.time() - self.pause_time

	def update_stream(self, *args, **kwargs):
		if not self.pause:
			try:
				data_raw = next(self.generator)
				self._draw(data_raw)	
			except StopIteration:
				self._close()

	def _prepare_stream(self):
		self.fig.canvas.mpl_connect('key_press_event', self.on_key_stream)
		timeout = 1000 / self.fps
		self.ani = animation.FuncAnimation(self.fig, self.update_stream, interval=timeout)

	def on_key_stream(self, event):
		if event.key == ' ':
			self.toggle_pause()
		elif event.key == 'q':
			self._close()

	def run_stream(self):
		if not self.generator:
			raise Exception("generator not set")
		self.pause = False
		self._prepare_stream()
		self._start()


if __name__ == '__main__':
	import random
	def gen_random():
		while True:
			a = random.random()
			yield a

	my_generator = gen_random()
	my_player = Player2D(generator=my_generator, timespan=10, ybottom=-0.1, 
						ytop=1.1, show_value=True)
	my_player.run_stream()

