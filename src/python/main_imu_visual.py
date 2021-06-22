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
from ahrs.filters import EKF
from ahrs.common.orientation import acc2q

from scipy.spatial.transform import Rotation as R
import quaternion
from pyquaternion import Quaternion

from toolkit.visual.player1d import Player1D
from toolkit import filemanager

from pca import pca_h, pca_v
from scipy.optimize import leastsq


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
IP_PORT = 8081  ## browser
# IP_PORT = 23456  ## Nine Keys


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
		print(f"send: {paras}")
		self.my_socket.send((" ".join([str(item) for item in paras])+"\n").encode())

	def sendButton(self, cmd):
		self.my_socket.send((cmd+"\n").encode())


MAX_ACC = 2 * 9.8  ## m/s^2
MAX_GYR = 2000  ## degree/s (dps)
def quat_to_mat(q):
	w, x, y, z = q
	return np.matrix([[1-2*y*y-2*z*z, 2*x*y+2*w*z, 2*x*z-2*w*y], [2*x*y-2*w*z, 1-2*x*x-2*z*z, 2*y*z+2*w*x], [2*x*z+2*w*y, 2*y*z-2*w*x, 1-2*x*x-2*y*y]])	

class Smooth:
	def __init__(self):
		self.N = 10
		self.data = [0 for i in range(self.N)]
	def update(self, d):
		self.data.append(d)
		if len(self.data) > self.N:
			self.data = self.data[-self.N:]
	def getData(self):
		return np.sum(self.data)
class ProcIMU:

	RESOLUTION = 2**15
	MAX_ACC = 2 * 9.8  ## m/s^2
	MAX_GYR = 2000  ## degree/s (dps)
	UNIT_ACC = MAX_ACC / RESOLUTION
	UNIT_GYR = MAX_GYR / RESOLUTION
	UNIT_GYR_RAD = UNIT_GYR * np.pi / 180

	FILENAME_TEMPLATE = "imu_%Y%m%d%H%M%S.csv"

	V0 = 255
	R0_RECI = 1  ## a constant to multiply the value

	def calibrate(self):
		self.h_vec = np.zeros([9], dtype=np.float32)
		self.v_vec = np.zeros([9], dtype=np.float32)
		def fit_func(v1, A):
			return np.array(np.dot(A, np.array(v1)))[0]

		def residual_func(v1, A, y):
			return (fit_func(np.array(v1), A) - y)**2 + 0.1 * np.mean(np.array(v1)**2)


		M = 5			
		datah = [[quat_to_mat((0.3986767 ,-0.5995983,0.69121914,-0.06127698)), quat_to_mat(( 0.49719739,-0.0639633 ,0.86514903,-0.01485314))],  [quat_to_mat((0.41771886,-0.50197834,0.73663552,0.17577491)), quat_to_mat((0.42981814,-0.13975846,0.88027534,-0.14435813))],[quat_to_mat((0.3986767 ,-0.5995983,0.69121914,-0.06127698)), quat_to_mat(( 0.49719739,-0.0639633 ,0.86514903,-0.01485314))],[quat_to_mat((0.3986767 ,-0.5995983,0.69121914,-0.06127698)), quat_to_mat(( 0.49719739,-0.0639633 ,0.86514903,-0.01485314))],[quat_to_mat((0.3986767 ,-0.5995983,0.69121914,-0.06127698)), quat_to_mat(( 0.49719739,-0.0639633 ,0.86514903,-0.01485314))]]
		datah = np.reshape(np.array(datah), (M, 2, -1))
		datav = [[quat_to_mat((0.61033867,-0.30779079,0.72248902,-0.10373595)),quat_to_mat(( 0.31355373,-0.35337103,0.85580681,-0.2107313))],   [quat_to_mat((0.59525167,-0.22015925,0.77228712,0.02789193)), quat_to_mat((0.32557408,-0.29069872,0.89971519,0.00288946))],[quat_to_mat((0.61033867,-0.30779079,0.72248902,-0.10373595)),quat_to_mat(( 0.31355373,-0.35337103,0.85580681,-0.2107313))],[quat_to_mat((0.61033867,-0.30779079,0.72248902,-0.10373595)),quat_to_mat(( 0.31355373,-0.35337103,0.85580681,-0.2107313))],[quat_to_mat((0.61033867,-0.30779079,0.72248902,-0.10373595)),quat_to_mat(( 0.31355373,-0.35337103,0.85580681,-0.2107313))]]# [[0,0,0], [1,1,1]]
		datav = np.reshape(np.array(datav), (M,2,-1))
		print(datah.shape)
		
		A = []
		ys = []
		for i in range(M):
			A.append(datah[i][1] - datah[i][0])
			ys.append(1)
		for i in range(M):
			A.append(datav[i][1] - datav[i][0])
			ys.append(0)
		A = np.matrix(np.stack(A, axis=0))
		print('A', A.shape)
		self.h_vec = np.array((A.I * A.T.I * A.T * np.matrix(ys).T).T)[0]
		self.h_vec = leastsq(residual_func, self.v_vec, args=(A,ys))[0]

		A = []
		ys = []
		for i in range(M):
			A.append(datah[i][1] - datah[i][0])
			ys.append(0)
		for i in range(M):
			A.append(datav[i][1] - datav[i][0])
			ys.append(1)
		A = np.matrix(np.stack(A, axis=0))
		self.v_vec = np.array((A.I * A.T.I * A.T * np.matrix(ys).T).T)[0]
		self.v_vec = leastsq(residual_func, self.v_vec, args=(A,ys))[0]



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
		self.madgwick = ahrs.filters.Madgwick(gain=1)# 0.03)
		self.ekf = EKF()
		self.Q = np.array([1., 0., 0., 0.]) # Allocate for quaternions

		self.Q_output = np.array([1., 0., 0., 0.]) # Allocate for quaternions

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

		#swnnnn
		self.last_x = 0.5
		self.last_y = 0.5
		self.calibrate()
		print(self.h_vec, self.v_vec)
		self.smoothX = Smooth()
		self.smoothY = Smooth()
		self.inputting = False


		# self.my_remote_handle = CursorClient("localhost", 8081)

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
		# print(*self.rotation.inverse)
		self.rotation = Quaternion(0.7506447624364577,-0.0,-0.12936740371211275,-0.6479170591082638)

		print(f"recording to {self.filename}")
		while self.flag.value != 0:
			self.put_frame()
			self.update_quaternion()

			self.visualize()



			Quat_current = Quaternion(self.Q)
			# rot_current = self.rotation.inverse*Quat_current
			rot_current = self.rotation*Quat_current
			x, y, z = (rot_current).rotate([0, 0, 1])
			# u = (x+y)/2
			# v = (x-y)/2

			euler = self.euler_from_quaternion(*self.Q)
			# x = (- pca_h.transform([euler])[0][0] - 20) / 70
			# y = ((-pca_v.transform([euler])[0][0] - 20) / 20 + 2 ) / 3

			# x = -pca_h.transform([self.Q])[0][0] 
			# y = pca_v.transform([self.Q])[0][0]
			print(f':{x}, y:{y}')
			print('quad:', self.Q)
			H = np.sum(np.dot(self.h_vec, np.reshape(quat_to_mat(self.Q), (9, -1))))
			V = np.sum(np.dot(self.v_vec, np.reshape(quat_to_mat(self.Q), (9, -1))))
			self.smoothX.update(H)
			H = self.smoothX.getData()
			self.smoothY.update(V)
			V = self.smoothY.getData()
			print(H.shape)
			print('HHHH', H)
			print('VVVV', V)


			# self.measure[0:3] = self.data_imu[3:] # self.Q[1:4]
			# self.measure[1] = y

			# v = (v)
			# x /= 2
			# self.measure[0] = y
			# self.measure[1] = x

			value = np.sum(self.data_pressure.reshape(16,-1)[:,2:4])
			# value = np.mean(self.data_pressure)
			print(value)

			# self.send(H, V, value)

####
			maxv = np.argmax(np.abs(self.measure))
			# print(self.measure[:])
			# if np.abs(self.measure[1]) > 5:
			self.measure[0:3] = self.data_imu[3:]
			dy = -self.measure[1] * (self.cur_time - self.last_time)
			# else:
			# 	dy = 0
			# if np.abs(self.measure[0]) + np.abs(self.measure[2]) > 10:
			dx = (self.measure[0] + self.measure[2]) * (self.cur_time - self.last_time) / 2
			# else:
			# 	dx = 0
####
			# print(Quat_current)
			
			self.measure[0:3] = [euler[0], euler[1], euler[2]]
			# self.measure[0:3] = self.Q[1:]
			# print(value)
			self.send(x, y, value, (dx/4 , dy /6))
			# self.send_nine(dx, -dy, value)
			# self.send_nine_swn(x, y, value, (dx / 350, dy / 550))

			# filemanager.write_line(self.filename, self.Q, tags=int(self.cur_time*1000000))
			# filemanager.write_line(self.filename, euler, tags=int(self.cur_time*1000000))

			if self.cur_time - self.last_time >= 1:
				duration = self.cur_time - self.last_time
				frames = self.frame_idx - self.last_frame_idx
				print(f"  frame rate: {frames/duration:.3f} fps")
				self.last_time = self.cur_time
				self.last_frame_idx = self.frame_idx

		self.my_serial.close()
	def send_nine_swn(self, x, y, value, delta):
		threshold = 1.2
		dx, dy = delta
		print(f'delata:{delta}')
		self.last_x += dx
		self.last_y += dy

		if value > threshold:
			if self.touching:
				# self.my_cursor_client.send(2, dx, dy)
				pass
			else:
				self.my_cursor_client.send(1, dx, dy)
				self.last_x = self.last_y = 0.5
				self.touching = True
		else:
			if self.touching:
				# self.my_cursor_client.send(0, self.last_x, self.last_y)
				self.touching = False
			else:
				self.my_cursor_client.send(0, self.last_x, self.last_y)




	def send_nine(self, dx, dy, value):
		threshold = 1.5
		# print(f'dx{dx}, dy{dy}')
		
		if value > threshold:
			if self.touching:
				# self.my_cursor_client.send(2, dx, dy)
				pass
			else:
				self.my_cursor_client.send(1, dx, dy)
				self.touching = True
		else:
			if abs(dx) < 5 and abs(dy) < 5:
				return
			if self.touching:
				self.my_cursor_client.send(0, dx, dy)
				self.touching = False
			else:
				self.my_cursor_client.send(0, dx, dy)


	def send(self, x, y, value):
		print('in send', x, y)
		threshold = 1.5
		if x < 0 or y < 0:
			return
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

	def send(self, x, y, value, delta):
		threshold = 40
		dx, dy = delta
		print(f'delata:{delta}')
		# self.last_x += dx
		# self.last_y += dy
		# x = self.last_x - 0.5
		# y = self.last_y - 0.5
		# if np.sqrt(x*x + y*y) < 0.1:
		# 	pos = 4
		# else:
		# 	angle = np.arctan2(y, x)
		# 	pos = int('52103678'[int((angle + np.pi / 8) *4 // np.pi)])
		# cmds = ['up left', 'up', 'up right', 'left', '', 'right', 'down left', 'down', 'down right']
		# for cmd in cmds[pos].split(' '):
		# 	self.my_remote_handle.sendButton(cmd)
		# # elif pos in []
		# if value > threshold:
		# 	if not self.touching:
		# 		self.my_cursor_client.send(pos, self.last_x, self.last_y)
		# 		self.touching = True
		# 		self.last_x = self.last_y = 0.5
		# else:
		# 	if self.touching:
		# 		self.touching = False
		if value > threshold + 30:
			self.last_x = 0.5
			self.last_y = 0.5
		if value > threshold:
			if self.touching:
				self.last_x += dx
				self.last_y += dy
				# self.my_cursor_client.send(2, self.last_x, self.last_y)
			else:
				# self.last_x = 0.5
				# self.last_y = 0.5
				self.last_x += dx
				self.last_y += dy
				# self.my_cursor_client.send(1,self.last_x, self.last_y)
				self.touching = True
		else:
			if self.touching:
				self.last_x += dx
				self.last_y += dy
				# self.my_cursor_client.send(3, self.last_x, self.last_y)
				self.touching = False
				self.inputting = not self.inputting
				if not self.inputting:
					self.my_cursor_client.send(3, self.last_x, self.last_y)
				else:
					self.my_cursor_client.send(1,self.last_x, self.last_y)
			else:
				self.last_x += dx
				self.last_y += dy
				self.my_cursor_client.send(2 if self.inputting else 4, self.last_x, self.last_y)


	def read_byte(self):
		recv = self.my_serial.read()
		if len(recv) != 1:
			raise Exception("Serial timeout!")
		return recv[0]

	@staticmethod
	def calReci_numpy_array(np_array, v0, r0_reci):
		np_array[np_array >= v0] = 0
		np_array /= (v0 - np_array)
		np_array *= r0_reci

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
						self.calReci_numpy_array(self.data_pressure, self.V0, self.R0_RECI)
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
		# self.

	def record_quaternion(self, hover):
		start_time = time.time()
		while time.time() - start_time < hover and self.flag.value != 0:
			self.put_frame()
			self.update_quaternion()
			self.visualize()
		return self.Q.copy()

	def cali(self):
		print(f"Please head forward for {self.hover} seconds...")
		Q_forward = self.record_quaternion(self.hover)
		print(f"Please head up for {self.hover} seconds...")
		Q_up = self.record_quaternion(self.hover)

		quat = Quaternion(Q_forward).inverse * Quaternion(Q_up)
		norm_org = quat.axis
		norm_new = np.array([-1, 0, 0])
		axis = np.cross(norm_org, norm_new)
		w = np.linalg.norm(norm_org)*np.linalg.norm(norm_new)+np.dot(norm_org, norm_new)
		self.rotation = Quaternion(w, *axis).normalised



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
	# if args.acc:
	# 	ytop = MAX_ACC
	# 	ybottom = -MAX_ACC
	# elif args.gyr:
	# 	ytop = MAX_GYR
	# 	ybottom = -MAX_GYR
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
