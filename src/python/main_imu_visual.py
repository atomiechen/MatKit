from serial import Serial
from serial.tools.list_ports import comports
import argparse
import numpy as np
from struct import calcsize, pack, unpack, unpack_from
import time
from datetime import datetime
from socket import socket, AF_INET, SOCK_STREAM, timeout

from multiprocessing import Process  # 进程
from multiprocessing import Array  # 共享内存
from multiprocessing import Value  # 共享内存

import ahrs
from scipy.spatial.transform import Rotation as R
import quaternion

from toolkit.visual.player1d import Player1D
from toolkit import filemanager

from pca import pca_h, pca_v

BAUDRATE = 1000000
TIMEOUT = 1  # in seconds
devices_found = comports()
PORT = None
try:
	## default use the last port on the list
	PORT = devices_found[-1].device
except:
	pass

IP = "localhost"
# IP = "183.173.190.54"
IP_PORT = 8081


def enumerate_ports():
	# 查看可用端口
	print("All serial ports:")
	for item in devices_found:
		print(item)


class CursorClient:
	def __init__(self, server_addr, port, timeout=1):
		self.my_socket = socket(AF_INET, SOCK_STREAM)
		self.my_socket.settimeout(timeout)
		self.connect(server_addr, port)

	def __enter__(self):
		return self

	def __exit__(self, type, value, traceback):
		self.close()

	def connect(self, address, port):
		self.my_socket.connect((address, port))
		print(f"client connecting to server: {address}")

	def close(self):
		self.my_socket.close()
		print("remote client socket closed")

	def send(self, touch_state, x, y):
		## touch state: 1按下，2按下时移动，3松开，4松开时移动
		paras = [touch_state, x, y]
		self.my_socket.send((" ".join([str(item) for item in paras])+"\n").encode())
		print(f"send: {paras}")


class ProcIMU:

	RESOLUTION = 2**15
	MAX_ACC = 2 * 9.8  ## m/s^2
	MAX_GYR = 2000  ## degree/s (dps)
	UNIT_ACC = MAX_ACC / RESOLUTION
	UNIT_GYR = MAX_GYR / RESOLUTION
	UNIT_GYR_RAD = UNIT_GYR * np.pi / 180

	FILENAME_TEMPLATE = "imu_%Y%m%d%H%M%S.csv"

	def __init__(self, measure, flag, port, baudrate, timeout=None, acc=False, gyr=False, my_cursor_client=None):
		self.measure = measure
		self.flag = flag
		self.port = port
		self.baudrate = baudrate
		self.timeout = timeout
		self.acc = acc
		self.gyr = gyr
		self.my_cursor_client = my_cursor_client

		## AHRS
		self.madgwick = ahrs.filters.Madgwick(gain=1)
		# self.Q = np.array([1., 0., 0., 0.]) # Allocate for quaternions
		self.quat = np.quaternion(1., 0., 0., 0.)
		self.Q = quaternion.as_float_array(self.quat)
		self.Q_output = np.array([1., 0., 0., 0.]) # Allocate for quaternions
		self.Q_forward = np.array([1., 0., 0., 0.]) # Allocate for quaternions

		## store data
		self.data_pressure = np.zeros(256, dtype=float)
		self.data_imu = np.zeros(6, dtype=float)
		self.frame_size = 256+12

		## check fps
		self.frame_idx = 0
		self.last_frame_idx = 0
		self.cur_time = time.time()
		self.last_time = self.cur_time
		self.last_frame_time = self.cur_time

		## calibration
		self.hover = 2

		self.filename = datetime.now().strftime(self.FILENAME_TEMPLATE)

		self.touching = False

	## ref: https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
	## ref2: https://stackoverflow.com/a/56207565/11854304
	@staticmethod
	def euler_from_quaternion(w, x, y, z):
			"""
			Convert a quaternion into euler angles (roll, pitch, yaw)
			roll is rotation around x in radians (counterclockwise)
			pitch is rotation around y in radians (counterclockwise)
			yaw is rotation around z in radians (counterclockwise)
			"""
			t0 = +2.0 * (w * x + y * z)
			t1 = +1.0 - 2.0 * (x * x + y * y)
			roll_x = np.degrees(np.arctan2(t0, t1))

			t2 = +2.0 * (w * y - z * x)
			t2 = +1.0 if t2 > +1.0 else t2
			t2 = -1.0 if t2 < -1.0 else t2
			pitch_y = np.degrees(np.arcsin(t2))

			t3 = +2.0 * (w * z + x * y)
			t4 = +1.0 - 2.0 * (y * y + z * z)
			yaw_z = np.degrees(np.arctan2(t3, t4))

			return roll_x, pitch_y, yaw_z # in degrees

	## ref: https://automaticaddison.com/how-to-multiply-two-quaternions-together-using-python/
	@staticmethod
	def quaternion_multiply(Q0, Q1):
	    """
	    Multiplies two quaternions.
	 
	    Input
	    :param Q0: A 4 element array containing the first quaternion (q01,q11,q21,q31) 
	    :param Q1: A 4 element array containing the second quaternion (q02,q12,q22,q32) 
	 
	    Output
	    :return: A 4 element array containing the final quaternion (q03,q13,q23,q33) 
	 
	    """
	    # Extract the values from Q0
	    w0, x0, y0, z0 = Q0
	     
	    # Extract the values from Q1
	    w1, x1, y1, z1 = Q1
	     
	    # Computer the product of the two quaternions, term by term
	    Q0Q1_w = w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1
	    Q0Q1_x = w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1
	    Q0Q1_y = w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1
	    Q0Q1_z = w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1
	     
	    # Create a 4 element array containing the final quaternion
	    final_quaternion = np.array([Q0Q1_w, Q0Q1_x, Q0Q1_y, Q0Q1_z])
	     
	    # Return a 4 element array containing the final quaternion (q02,q12,q22,q32) 
	    return final_quaternion

	def run(self):
		## serial
		self.my_serial = Serial(self.port, self.baudrate, timeout=self.timeout)
		print(self.my_serial)

		# self.cali()

		print(f"recording to {self.filename}")
		while self.flag.value != 0:
			self.put_frame()
			self.update_quaternion()

			# quat_forward = quaternion.from_float_array(self.Q_forward)
			# quat_org = quaternion.from_float_array(self.Q)
			# quat_new = quat_forward * quat_org / quat_forward
			# self.Q_output[:] = quaternion.as_float_array(quat_new)[:]


			# self.visualize()

			x = pca_h.transform([self.Q])[0][0] * -5
			y = pca_v.transform([self.Q])[0][0] * 5 + 0.5
			self.measure[0:3] = self.data_imu[3:] # self.Q[1:4]
			# self.measure[1] = y

			value = np.mean(self.data_pressure)
			print(value)
			self.send(x, y, value)

			# filemanager.write_line(self.filename, self.Q, tags=int(self.cur_time*1000000))

			if self.cur_time - self.last_time >= 1:
				duration = self.cur_time - self.last_time
				frames = self.frame_idx - self.last_frame_idx
				print(f"  frame rate: {frames/duration:.3f} fps")
				self.last_time = self.cur_time
				self.last_frame_idx = self.frame_idx

		self.my_serial.close()

	def send(self, x, y, value):
		threshold = 135
		if value > threshold:
			if self.touching:
				self.my_cursor_client.send(2, x, y)
			else:
				self.my_cursor_client.send(1, x, y)
				self.touching = True
		else:
			if self.touching:
				self.my_cursor_client.send(3, x, y)
				self.touching = False
			else:
				self.my_cursor_client.send(4, x, y)


	def read_byte(self):
		recv = self.my_serial.read()
		if len(recv) != 1:
			raise Exception("Serial timeout!")
		return recv[0]

	def put_frame(self):
		## protocol ref: https://blog.csdn.net/weixin_43277501/article/details/104805286
		frame = bytearray()
		begin = False
		while True:
			recv = self.read_byte()
			if begin:
				if recv == 0x5C:
					## escape bytes
					recv = self.read_byte()
					if recv == 0x00:
						frame.append(0x5C)
					elif recv == 0x01:
						frame.append(0x5B)
					elif recv == 0x02:
						frame.append(0x5D)
					# else:
						# print(f"Wrong ESCAPE byte: {recv}")
				elif recv == 0x5D:
					## end a frame
					if len(frame) != self.frame_size:
						## wrong length, re-fetch a frame
						# print(f"Wrong frame size: {len(frame)}")
						frame = bytearray()
						begin = False
					else:
						self.data_pressure[:256] = frame[:256]
						if self.data_imu is not None:
							pos = 256
							for i in range(6):
								self.data_imu[i] = unpack_from(f"=h", frame, pos)[0]
								pos += calcsize(f"=h")
						self.data_imu[:3] *= self.UNIT_ACC
						self.data_imu[3:] *= self.UNIT_GYR_RAD
						self.cur_time = time.time()
						self.frame_idx += 1
						break
				else:
					frame.append(recv)
			elif recv == 0x5B:
				## begin a frame
				begin = True

	def visualize(self):
		## visualize data
		if self.acc:
			## accelerometer
			self.measure[:3] = self.data_imu[:3]
		elif self.gyr:
			## gyroscope
			self.measure[:3] = self.data_imu[3:]
		else:
			## AHRS
			# v3 = Q[1:]
			# v3 = Q[1:] / Q[0]

			# r = R.from_quat(Q)
			# v3 = r.as_rotvec()
			# v3 = r.as_euler('zyx', degrees=True)

			v3 = self.euler_from_quaternion(*self.Q)

			# self.measure[:3] = v3
			self.measure[1] = v3[1]

	def update_quaternion(self):
		self.madgwick.Dt = self.cur_time - self.last_frame_time
		# self.madgwick.frequency = 1 / self.madgwick.Dt
		self.last_frame_time = self.cur_time
		self.Q = self.madgwick.updateIMU(self.Q, gyr=self.data_imu[3:], acc=self.data_imu[:3])

	def record_quaternion(self, hover):
		start_time = time.time()
		while time.time() - start_time < hover and self.flag.value != 0:
			self.put_frame()
			self.update_quaternion()
			self.visualize()
		return self.Q.copy()

	def cali(self):
		# print(f"Please head left for {self.hover} seconds...")
		# Q_left = self.record_quaternion(self.hover)
		# print(f"Please head right for {self.hover} seconds...")
		# Q_right = self.record_quaternion(self.hover)
		# print(f"Please head up for {self.hover} seconds...")
		# Q_up = self.record_quaternion(self.hover)
		# print(f"Please head down for {self.hover} seconds...")
		# Q_down = self.record_quaternion(self.hover)
		# print(Q_left)
		# print(Q_right)
		# print(Q_up)
		# print(Q_down)

		print(f"Please head forward for {self.hover} seconds...")
		Q_forward = self.record_quaternion(self.hover)


		print(Q_forward)
		self.Q_forward = Q_forward


def task_serial(measure, flag, port, baudrate, timeout=None, acc=False, gyr=False, ip=IP, ip_port=IP_PORT):
	with CursorClient(ip, ip_port) as my_cursor_client:
		my_proc = ProcIMU(measure, flag, port, baudrate, timeout, acc, gyr, my_cursor_client)
		my_proc.run()


def task_debug(measure, flag):
	import random
	while flag.value != 0:
		measure[0] = random.random()
		measure[1] = random.random()
		measure[2] = random.random()

def main(args):
	if args.enumerate:
		enumerate_ports()
		return

	# measure = Value('d')  # d for double
	measure = Array('d', 3)  # d for double
	flag = Value('i')  # i for int
	flag.value = 1

	def gen_wrapper():
		while True:
			yield measure

	if not args.debug:
		p = Process(target=task_serial, args=(measure, flag, args.port, 
									args.baudrate, args.timeout, args.acc, args.gyr))
	else:
		p = Process(target=task_debug, args=(measure,flag,))
	p.start()

	ytop = None
	ybottom = None
	if args.acc:
		ytop = MAX_ACC
		ybottom = -MAX_ACC
	elif args.gyr:
		ytop = MAX_GYR
		ybottom = -MAX_GYR
	my_player = Player1D(generator=gen_wrapper(), channels=3, timespan=5, 
						ytop=ytop, ybottom=ybottom)
	my_player.run_stream()

	flag.value = 0
	p.join()


if __name__ == '__main__':
	parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
	parser.add_argument('-e', dest='enumerate', action=('store_true'), default=False, help="enumerate all serial ports")
	parser.add_argument('-p', dest='port', action=('store'), default=PORT, help="specify serial port")
	parser.add_argument('-b', dest='baudrate', action=('store'), default=BAUDRATE, type=int, help="specify baudrate")
	parser.add_argument('-t', dest='timeout', action=('store'), default=TIMEOUT, type=float, help="specify timeout in seconds")
	parser.add_argument('-d', '--debug', dest='debug', action=('store_true'), default=False, help="debug mode")
	parser.add_argument('-g', '--gyr', dest='gyr', action=('store_true'), default=False, help="show gyroscope data")
	parser.add_argument('-a', '--acc', dest='acc', action=('store_true'), default=False, help="show accelerometer data")

	args = parser.parse_args()

	main(args)
