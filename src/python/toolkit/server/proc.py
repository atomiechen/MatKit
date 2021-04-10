from math import exp, hypot, pi, sin
from enum import Enum
import numpy as np
import time

from serial import Serial
from multiprocessing import Array  # 共享内存
from queue import Empty

from .flag import FLAG, FLAG_REC
from .exception import SerialTimeout
from ..tools import check_shape


class FILTER_SPATIAL(Enum):
	NONE = None  # no spatial filter
	IDEAL = "ideal"  # ideal kernel
	BUTTERWORTH = "butterworth"  # butterworth kernel
	GAUSSIAN = "gaussian"  # gaussian kernel


class FILTER_TEMPORAL(Enum):
	NONE = None  # no temporal filter
	MA = "moving average"  # moving average
	RW = "rectangular window"  # rectangular window filter (sinc)


class DataSetterSerial:
	def __init__(self, total, baudrate, port, timeout=None):
		self.my_serial = self.connect_serial(baudrate, port, timeout)
		self.total = total
		self.DELIM = 0xFF
		self.start_time = time.time()

	@staticmethod
	def connect_serial(baudrate, port, timeout=None):
		# 超时设置,None：永远等待操作，0为立即返回请求结果，其他值为等待超时时间(单位为秒）
		ser = Serial(port, baudrate, timeout=timeout)
		print("串口详情参数：", ser)
		return ser

	def __call__(self, data_tmp, **kwargs):
		while True:
			recv = self.my_serial.read()
			if len(recv) != 1:
				raise SerialTimeout
			if recv[0] == self.DELIM:
				data = self.my_serial.read(self.total)
				if len(data) != self.total:
					raise SerialTimeout
				data_tmp[:self.total] = list(data)
				break
		if "filename" in kwargs.keys():
			filename = kwargs['filename'].decode().split('.')
			filename[0] += ('_' + str(kwargs['filename_id']))
			filename = '.'.join(filename)
			with open(filename, 'a', encoding='utf-8') as f:
				for d in data_tmp:
					f.write(str(int(d)) + ',')
				f.write(str(kwargs['idx_out'].value) + ',' + str(int(time.time() * (10**6))) + ',\n')


class DataSetterFile:
	## TODO add file as source
	def __init__(self):
		self.start_time = time.time()

	def __call__(self, data_tmp, **kwargs):
		if "filename" in kwargs.keys():
			with open(kwargs['filename'], 'a', encoding='utf-8') as f:
				f.write(str(data_tmp) + ',' + str(kwargs['idx_out']) + ',' + str(time.time() - self.start_time) + '\n')


class Proc:
	## data mode
	my_raw = False

	## default filters
	my_filter_spatial = FILTER_SPATIAL.GAUSSIAN
	my_filter_temporal = FILTER_TEMPORAL.RW

	## voltage-resistance conversion
	my_convert = True
	V0 = 255
	R0_RECI = 1  ## a constant to multiply the value

	## process parameters
	SAFE_FILTER_SIZE = 100
	my_SF_D0 = 3.5
	my_BUTTER_ORDER = 2
	my_LP_SIZE = 15
	my_LP_W = 0.04
	my_INIT_CALI_FRAMES = 200
	my_WIN_SIZE = 10000

	## fps checking
	FPS_CHECK_TIME = 1  # seconds of interval to check fps

	## for warm up, make CPU schedule more time for serial reading
	WARM_UP = 1  # seconds

	## run loop or wait for waking up from server
	RUN_LOOP = True

	## filename received from server
	filename = None

	filename_id = 0

	def __init__(self, n, data_setter, data_out, data_raw, idx_out, **kwargs):
		## sensor info
		self.n = check_shape(n)
		self.mask = None

		## default data source
		self.data_setter = data_setter

		self.total = self.n[0] * self.n[1]
		self.cols = self.n[1]//2 + 1

		## intermediate data
		self.data_tmp = np.zeros(self.total, dtype=float)
		self.data_reshape = self.data_tmp.reshape(self.n[0], self.n[1])
		## output data
		# self.data_out = Array('d', self.total)  # d for double
		## raw data
		# self.data_raw = np.zeros(self.total, dtype=float)
		self.data_out = data_out
		self.data_raw = data_raw
		self.idx_out = idx_out

		## for multiprocessing communication
		self.queue_from_server = None

		self.config(**kwargs)

	def config(self, *, raw=None, 
		filter_spatial=None, filter_spatial_cutoff=None, butterworth_order=None,
		filter_temporal=None, filter_temporal_size=None, rw_cutoff=None,
		V0=None, R0_RECI=None, convert=None, mask=None, 
		RUN_LOOP=None, queue_from_server=None):
		if raw is not None:
			self.my_raw = raw
		if V0:
			self.V0 = V0
		if R0_RECI:
			self.R0_RECI = R0_RECI
		if convert is not None:
			self.my_convert = convert
		if filter_spatial in FILTER_SPATIAL.__members__.values():
			self.my_filter_spatial = filter_spatial
		if filter_spatial_cutoff is not None:
			self.my_SF_D0 = filter_spatial_cutoff
		if butterworth_order is not None:
			self.my_BUTTER_ORDER = butterworth_order
		if filter_temporal in FILTER_TEMPORAL.__members__.values():
			self.my_filter_temporal = filter_temporal
		if filter_temporal_size is not None:
			self.my_LP_SIZE = filter_temporal_size
		if rw_cutoff is not None:
			self.my_LP_W = rw_cutoff
		if mask is not None:
			self.mask = mask
		if RUN_LOOP:
			self.RUN_LOOP = RUN_LOOP
		if queue_from_server:
			self.queue_from_server = queue_from_server

	def reset(self):
		## for output
		self.idx_out.value = 0
		## for fps checking
		self.last_frame_idx = 0
		self.last_time = self.start_time

	@staticmethod
	def calReciprocalResistance(voltage, v0, r0_reci):
		if v0 - voltage <= 0:
			return 0
		return r0_reci * voltage / (v0 - voltage)

	@staticmethod
	def calReci_numpy_array(np_array, v0, r0_reci):
		np_array[np_array >= v0] = 0
		np_array /= (v0 - np_array)
		np_array *= r0_reci

	def get_raw_frame(self):
		if self.filename:
			self.data_setter(data_tmp=self.data_tmp, filename=self.filename,
							idx_out=self.idx_out, filename_id=self.filename_id)
		else:
			self.data_setter(data_tmp=self.data_tmp)
		if self.mask is not None:
			self.data_reshape *= self.mask
		if self.my_convert:
			# for i in range(self.total):
			# 	self.data_tmp[i] = self.calReciprocalResistance(self.data_tmp[i], self.V0, self.R0_RECI)
			self.calReci_numpy_array(self.data_tmp, self.V0, self.R0_RECI)
		self.idx_out.value += 1

	def post_action(self):
		if self.cur_time - self.last_time >= self.FPS_CHECK_TIME:
			duration = self.cur_time - self.last_time
			run_duration = self.cur_time - self.start_time
			frames = self.idx_out.value - self.last_frame_idx
			print(f"  frame rate: {frames/duration:.3f} fps  runing time: {run_duration:.3f} s")
			self.last_frame_idx = self.idx_out.value
			self.last_time = self.cur_time

	def prepare_spatial(self):
		def gaussianLP(distance):
			return exp(-distance**2/(2*(self.my_SF_D0)**2))

		def butterworthLP(distance):
			return 1 / (1 + (distance / self.my_SF_D0)**(2 * self.my_BUTTER_ORDER))

		def idealFilterLP(distance):
			if distance <= self.my_SF_D0:
				return 1
			else:
				return 0

		if self.my_filter_spatial == FILTER_SPATIAL.NONE:
			return

		if self.my_filter_spatial == FILTER_SPATIAL.IDEAL:
			freq_window = idealFilterLP
		elif self.my_filter_spatial == FILTER_SPATIAL.BUTTERWORTH:
			freq_window = butterworthLP
		elif self.my_filter_spatial == FILTER_SPATIAL.GAUSSIAN:
			freq_window = gaussianLP
		else:
			raise Exception("Unknown spatial filter!")

		row_divide = self.n[0] // 2
		self.kernel_sf = np.zeros((self.n[0], self.cols), dtype=float)
		for i in range(row_divide + 1):
			for j in range(self.cols):
				distance = hypot(i, j)
				self.kernel_sf[i][j] = freq_window(distance)
		for i in range(row_divide + 1, self.n[0]):
			for j in range(self.cols):
				distance = hypot(self.n[0]-i, j)
				self.kernel_sf[i][j] = freq_window(distance)

	def spatial_filter(self):
		if self.my_filter_spatial == FILTER_SPATIAL.NONE:
			return
		# self.data_tmp = self.data_tmp.reshape(self.n[0], self.n[1])
		freq = np.fft.rfft2(self.data_reshape)
		freq *= self.kernel_sf
		## must specify shape when the final axis number is odd
		self.data_reshape[:] = np.fft.irfft2(freq, self.data_reshape.shape)

	@staticmethod
	def getNextIndex(idx, size):
		return (idx+1) if idx != (size-1) else 0

	def prepare_temporal(self):
		if self.my_filter_temporal == FILTER_TEMPORAL.NONE:
			return

		print("Initiating temporal filter...");
		self.data_filter = np.zeros((self.my_LP_SIZE-1, self.total), dtype=float)
		self.kernel_lp = np.zeros(self.my_LP_SIZE, dtype=float)
		self.filter_frame_idx = 0
		need_cache = self.my_LP_SIZE - 1

		if self.my_filter_temporal == FILTER_TEMPORAL.MA:
			## moving average
			self.kernel_lp[:] = 1 / self.my_LP_SIZE
		elif self.my_filter_temporal == FILTER_TEMPORAL.RW:
			## FIR Rectangular window filter (sinc low pass)
			sum_all = 0
			for t in range(self.my_LP_SIZE):
				shifted = t - (self.my_LP_SIZE-1) / 2
				if shifted == 0:
					## limit: t -> 0, sin(t)/t -> 1
					self.kernel_lp[t] = 2 * pi * self.my_LP_W
				else:
					self.kernel_lp[t] = sin(2 * pi * self.my_LP_W * shifted) / shifted
				sum_all += self.kernel_lp[t]
			self.kernel_lp /= sum_all
		else:
			raise Exception("Unknown temporal filter!")

		if need_cache > 0:
			print(f"Cache {need_cache} frames for filter.")
			while need_cache > 0:
				self.get_raw_frame()
				self.filter()
				need_cache -= 1

	def temporal_filter(self):
		if self.my_filter_temporal == FILTER_TEMPORAL.NONE:
			return

		stored = self.data_tmp.copy()
		## convolve
		self.data_tmp *= self.kernel_lp[0]
		## oldest point in data_filter is firstly visited
		for t in range(1, self.my_LP_SIZE):
			self.data_tmp += self.data_filter[self.filter_frame_idx] * self.kernel_lp[t]
			self.filter_frame_idx = self.getNextIndex(self.filter_frame_idx, self.my_LP_SIZE-1)
		self.data_filter[self.filter_frame_idx] = stored
		## update to next index
		self.filter_frame_idx = self.getNextIndex(self.filter_frame_idx, self.my_LP_SIZE-1)

	def filter(self):
		self.spatial_filter()
		self.temporal_filter()

	def prepare_cali(self):
		if self.my_INIT_CALI_FRAMES <= 0:
			return

		print("Initiating calibration...");
		self.data_zero = np.zeros(self.total, dtype=float)
		self.data_win = np.zeros((self.my_WIN_SIZE, self.total), dtype=float)
		self.win_frame_idx = 0
		frame_cnt = 0
		## accumulate data
		while frame_cnt < self.my_INIT_CALI_FRAMES:
			self.get_raw_frame()
			self.filter()
			self.data_zero += self.data_tmp
			frame_cnt += 1
		## get average
		self.data_zero /= frame_cnt
		## calculate data_win
		self.data_win[:] = self.data_zero

	def calibrate(self):
		if self.my_INIT_CALI_FRAMES <= 0:
			return
		stored = self.data_tmp.copy()
		## calibrate
		self.data_tmp -= self.data_zero
		## the value should be positive
		self.data_tmp[self.data_tmp < 0] = 0
		## adjust window if using dynamic window
		if self.my_WIN_SIZE > 0:
			## update data_zero (zero position) and data_win (history data)
			self.data_zero += (stored - self.data_win[self.win_frame_idx]) / self.my_WIN_SIZE
			self.data_win[self.win_frame_idx] = stored
			## update frame index
			self.win_frame_idx = self.getNextIndex(self.win_frame_idx, self.my_WIN_SIZE)

	def print_proc(self):
		print(f"Voltage-resistance conversion: {self.my_convert}")
		print(f"Data mode: {'raw' if self.my_raw else 'processed'}")
		if not self.my_raw:
			## collect spatial filter info
			arg_list_s = {}
			if self.my_filter_spatial == FILTER_SPATIAL.BUTTERWORTH:
				arg_list_s["order"] = self.my_BUTTER_ORDER
			if self.my_filter_spatial != FILTER_SPATIAL.NONE:
				arg_list_s["cut-off freqency"] = self.my_SF_D0

			## collect temporal filter info
			arg_list_t = {}
			if self.my_filter_temporal == FILTER_TEMPORAL.RW:
				arg_list_t["cut-off normalized freqency"] = self.my_LP_W
			if self.my_filter_temporal != FILTER_TEMPORAL.NONE:
				arg_list_t["kernel size"] = self.my_LP_SIZE

			## output to screen
			print(f"  - Spatial filter: {self.my_filter_spatial.value}")
			for value, key in arg_list_s.items():
				print(f"    {value}: {key}")
			print(f"  - Temporal filter: {self.my_filter_temporal.value}")
			for value, key in arg_list_t.items():
				print(f"    {value}: {key}")

			print(f"  - Calibration: {'No' if self.my_INIT_CALI_FRAMES == 0 else ''}")
			if self.my_INIT_CALI_FRAMES != 0:
				print(f"    Initializing frames:     {self.my_INIT_CALI_FRAMES}")
				if self.my_WIN_SIZE == 0:
					print("    Static calibration")
				else:
					print("    Dynamic calibration")
					print(f"    Calibration window size: {self.my_WIN_SIZE}")

	def loop_proc(self):
		print("Running processing...")
		while True:
			## check signals from the other process
			if self.queue_from_server is not None:
				try:
					flag = self.queue_from_server.get_nowait()
					if flag == FLAG.FLAG_STOP:
						break
					if flag == FLAG_REC.FLAG_REC_DATA:
						self.RUN_LOOP = True
						self.my_raw = False
						while True:
							filename = self.queue_from_server.get_nowait()
							if filename is not None:
								self.filename = filename
								break
					elif flag == FLAG_REC.FLAG_REC_RAW:
						self.RUN_LOOP = True
						self.my_raw = True
						while True:
							filename = self.queue_from_server.get_nowait()
							if filename is not None:
								self.filename = filename
								break
					elif flag == FLAG_REC.FLAG_REC_STOP:
						self.RUN_LOOP = False
						self.filename = None
						self.filename_id = 0
					elif flag == FLAG_REC.FLAG_REC_BREAK:
						self.filename_id += 1
				except Empty:
					pass

			# if self.RUN_LOOP:
			try:
				self.get_raw_frame()
			except SerialTimeout:
				continue
			self.cur_time = time.time()
			self.data_raw[:] = self.data_tmp
			if not self.my_raw:
				self.filter()
				self.calibrate()
			self.data_out[:] = self.data_tmp
			self.post_action()

	def warm_up(self):
		print("Warming up processing...")
		begin = time.time()
		while time.time() - begin < self.WARM_UP:
			try:
				self.get_raw_frame()
			except SerialTimeout:
				pass

	def run(self):
		self.warm_up()

		self.start_time = time.time()
		self.reset()
		self.print_proc()

		if not self.my_raw:
			self.prepare_spatial()
			self.prepare_temporal()
			self.prepare_cali()
		self.loop_proc()


if __name__ == '__main__':
	my_setter = DataSetterSerial(16*16, 500000, 'COM4')
	my_proc = Proc(16, my_setter)
	# my_proc.run()
