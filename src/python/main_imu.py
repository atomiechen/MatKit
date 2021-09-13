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
from pyquaternion import Quaternion

from toolkit.uclient import Uclient
from toolkit.visual.player1d import Player1D
from toolkit import filemanager
from toolkit.process import Processor, CursorController, PressureSelector
from toolkit.tools import blank_config, check_config, load_config, DEST_SUFFIX

from demokit.swipe_gesture import SwipeGesture, SwipeGestureClassifier
from demokit.cursor_client import CursorClient

from scipy.optimize import leastsq
from scipy.signal import savgol_filter
from scipy import linalg
import json
import pathlib


N = 16
UDP = False

CONFIG_PATH = "config/tmp_config_mouth_16_16.yaml"


IP = "localhost"
IP_PORT = 8081  ## browser
# IP_PORT = 23456  ## Nine Keys


# class CursorClient:
# 	def __init__(self, server_addr, port, timeout=1):
# 		self.my_socket = socket(AF_INET, SOCK_STREAM)
# 		self.my_socket.settimeout(timeout)
# 		self.connect(server_addr, port)

# 	def __enter__(self):
# 		return self

# 	def __exit__(self, type, value, traceback):
# 		self.close()

# 	def connect(self, address, port):
# 		self.my_socket.connect((address, port))
# 		print(f"client connecting to server: {address}")

# 	def close(self):
# 		self.my_socket.close()
# 		print("remote client socket closed")

# 	def send(self, touch_state, x, y):
# 		## touch state: 1按下，2按下时移动，3松开，4松开时移动
# 		paras = [touch_state, x, y]
# 		# print(f"[send]: {paras}")
# 		self.my_socket.send((" ".join([str(item) for item in paras])+"\n").encode())

# 	def sendButton(self, cmd):
# 		self.my_socket.send((cmd+"\n").encode())


RESOLUTION = 2**15
MAX_ACC = 2 * 9.801  ## m/s^2
MAX_GYR = 2000  ## degree/s (dps)
UNIT_ACC = MAX_ACC / RESOLUTION
UNIT_GYR = MAX_GYR / RESOLUTION
UNIT_GYR_RAD = UNIT_GYR * np.pi / 180


# MAX_ACC = 2 * 9.8  ## m/s^2
# MAX_GYR = 2000  ## degree/s (dps)


LEFT = 0.1
RIGHT = 0.9
UP = 0.1
DOWN = 0.9

def mapping(x, y, left=0, right=1, up=0, down=1):
	x0 = 1-y
	y0 = 1-x
	x1 = min(max(x0 - left, 0) / (right - left), 0.999)
	y1 = min(max(y0 - up, 0) / (down - up), 0.999)
	return x1, y1


def quat_to_mat(q):
	w, x, y, z = q
	return np.array([[1-2*y*y-2*z*z, 2*x*y+2*w*z, 2*x*z-2*w*y], [2*x*y-2*w*z, 1-2*x*x-2*z*z, 2*y*z+2*w*x], [2*x*z+2*w*y, 2*y*z-2*w*x, 1-2*x*x-2*y*y]]).T

class Smooth:
	def __init__(self):
		self.data = 0
	def update(self, d):
		self.data = d
	def getData(self):
		return self.data
class SavgolSmooth(Smooth):
	def __init__(self, N = 200, window_length = 25):
		self.N = N
		self.window_length = window_length
		self.data = [0 for i in range(self.N)]
	def update(self, d):
		self.data.append(d)
		if len(self.data) > self.N:
			self.data = self.data[-self.N:]
	def getData(self):
		y = savgol_filter(self.data, self.window_length, 5)
		return y[-1]
class NearbySmooth(Smooth):
	def __init__(self, threshold = 0.1):
		self.threshold = threshold
		self.lastData = -100000
	def update(self, d):
		if self.lastData == -100000 or abs(self.lastData - d) < self.threshold:
			self.lastData = d
	def getData(self):
		return self.lastData
class AverageSmooth(Smooth):
	def __init__(self, N = 10):
		self.data = []
		self.N = N
	def update(self, d):
		self.data.append(d)
		if len(self.data) > self.N:
			self.data = self.data[-self.N:]
	def getData(self):
		return np.mean(self.data)



class ProcIMU:

	# RESOLUTION = 2**15
	# MAX_ACC = 2 * 9.801  ## m/s^2
	# MAX_GYR = 2000  ## degree/s (dps)
	# UNIT_ACC = MAX_ACC / RESOLUTION
	# UNIT_GYR = MAX_GYR / RESOLUTION
	# UNIT_GYR_RAD = UNIT_GYR * np.pi / 180

	FILENAME_TEMPLATE = "imu_%Y%m%d%H%M%S.csv"

	# def __init__(self, measure, flag, port, baudrate, timeout=None, acc=False, gyr=False, needCalibrate=False, my_cursor_client=None):
	def __init__(self, kwargs):
		print(kwargs)
		self.measure = kwargs['measure']
		self.flag = kwargs['flag']
		self.acc = kwargs['acc']
		self.gyr = kwargs['gyr']
		self.my_cursor_client = kwargs['client']
		self.my_client = kwargs['uclient']
		self.kwargs = kwargs

		## AHRS
		self.madgwick = ahrs.filters.Madgwick(gain=1)# 0.03)
		self.ekf = EKF()
		self.ekf = EKF()
		self.Q = np.array([1., 0., 0., 0.]) # Allocate for quaternions

		self.Q_output = np.array([1., 0., 0., 0.]) # Allocate for quaternions

		## store data
		# self.data_pressure = np.zeros(256, dtype=float)
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
		# print(self.h_vec, self.v_vec)
		self.smoothX = AverageSmooth()
		self.smoothY = AverageSmooth()
		self.inputting = False
		# self.std_Q = (0.48688449,-0.28526537,0.82494558,0.03212407)
		# self.rotQ = (quat_to_mat(self.std_Q)).I

		self.needCalibrate = kwargs['calibrate']
		self.M = np.array([[0.18883633390998655, -0.9758716640520846, 0.10961448031921497], [0.9179313065437382, 0.21507203171789888, 0.3333858689861354],[-0.34891683172690546, 0.03766319785879299, 0.9363965655985224]]).T
		self.firstPass = True
		self.gravityCheck = False
		self.gravityData = []
		self.running = False
		self.G_ref = 9.801
		self.G_threshold = 0.2
		self.x_bot = 20
		self.x_top = 0
		self.y_bot = -120
		self.y_top = -130
		self.pressure_threshold = 1.2
		self.click_threshold = 5
		if not self.needCalibrate:
			self.loadM()
		if self.kwargs['imu_calibrate']:
			self.calibrateCenter = [0, 0, 0]
			self.calibrateScale = [9.801, 9.801, 9.801]
		else:
			#CWH data
			self.calibrateCenter = [-0.085487926459334, -0.038224993082962,0.556024530070166]
			self.calibrateScale = [9.870533168553060, 9.832376613350656, 9.743367778916230]
			#SWN data
			self.calibrateCenter = [ 0.00987332, -0.01452841, 0.19158983] 
			self.calibrateScale = [9.81646455236567, 9.871776503810343, 9.719935747473379]
			#SWN data2
			# self.calibrateCenter = [0.04218729, 0.08655178, -0.24946698]
			# self.calibrateScale = [9.89370337798936, 9.852362167496885, 9.738741250318913]
			# #SWN data3
			# self.calibrateCenter = [-0.07968226, 0.01155016, 0.07111075]
			# self.calibrateScale = [9.791473022742816, 9.755175219164151, 9.765819348983804]

		config = kwargs['config']
		self.my_processor = Processor(
			config['process']['interp'], 
			blob=config['process']['blob'], 
			threshold=config['process']['threshold'],
			order=config['process']['interp_order'],
			total=config['process']['blob_num'],
			special=config['process']['special_check'],
		)
		self.my_processor.print_info()
		self.my_cursor = CursorController(
			mapcoor=config['pointing']['direct_map'],
			alpha=config['pointing']['alpha'], 
			trackpoint=config['pointing']['trackpoint']
		)
		self.my_cursor.print_info()
		self.my_classifier = SwipeGestureClassifier()



	def loadM(self):
		p = pathlib.Path('.') / 'M_value.txt'
		if not p.exists():
			return
		with open(p) as f:
			data = json.loads(f.read().strip())
			self.M = np.array(data)
			print(f'[calibrate] load M matrix complete!')
	
	def saveM(self):
		with open('M_value.txt', 'w') as f:
			f.write(json.dumps(self.M.tolist()))
			print(f'[calibrate] M matrix saved!')



	def checkGravity(self):
		normAcc = np.linalg.norm(self.data_imu[:3])
		print(f'\rnorm:{normAcc}, data: {self.data_imu[0]}, {self.data_imu[1]}, {self.data_imu[2]}', end='')
		if abs(normAcc - self.G_ref) < self.G_threshold:
			self.gravityData.append(self.data_imu[:3])
			if len(self.gravityData) >= 100:
				self.gravityCheck = True
		else:
			self.gravityData = []
	
	def checkStatic(self, data):
		varvalue = np.var(data, axis=0)
		normvar = np.linalg.norm(varvalue)
		# if normvar < 


	def getQ0(self, gReal, gCur):
		print('in get q0')
		print(gReal, gCur)
		angle = np.arccos(np.dot(gReal, gCur) / np.linalg.norm(gReal) / np.linalg.norm(gCur))
		axis = np.cross(gReal, gCur)
		axis = axis / np.linalg.norm(axis)
		return (np.cos(angle / 2), axis[0]*np.sin(angle / 2), axis[1] * np.sin(angle / 2), axis[2] * np.sin(angle / 2))

	def run(self):
		## serial
		# self.my_serial = Serial(self.port, self.baudrate, timeout=self.timeout)
		# print(self.my_serial)

		if self.kwargs['imu_calibrate']:
			with open('imu_calibration_data.txt', 'w') as f:
				while self.flag.value != 0:
					self.put_frame()
					print(self.data_imu[:3])
					f.write(json.dumps(self.data_imu[:3].tolist())+'\n')
			return

		

		if self.needCalibrate:
			print(f'START CALIBRATE')
			while self.flag.value != 0:
				self.put_frame()
				self.update_quaternion()
				if not self.gravityCheck:
					self.checkGravity()
					# print(f'\r[calibrate] Checking Gravity...\n', end='')
				else:
					if self.firstPass:
						print(f'\n[calibrate] Gravity pass!')
						self.firstPass = False
						self.gravityVec = np.mean(self.gravityData, axis=0)
						self.gravityData = []
						print(f'[calibrate] Gravity vector: {self.gravityVec}')
					else:
						print(f'\rmove up', end='')
						self.checkGravity()
						if len(self.gravityData) >= 100:
							self.upVec = np.mean(self.gravityData, axis=0)
							print(f'\n[calibrate] Up Grivity vector:{self.upVec}')
							print(f'[calibrate] Crossed Gravity: {self.gravityVec}')
							angle = np.degrees(np.arccos(np.dot(self.gravityVec, self.upVec) / np.linalg.norm(self.gravityVec) / np.linalg.norm(self.upVec)))
							print(f'[calibrate] angle: {angle}°')
							if abs(angle) < 10:
								print(f'[calibrate] angle too small!!!!!!')
							else:
								self.xVec = np.cross(self.gravityVec, self.upVec)
								self.yVec = -self.gravityVec
								self.xVec = self.xVec / np.linalg.norm(self.xVec)
								self.yVec = self.yVec / np.linalg.norm(self.yVec)
								self.zVec = np.cross(self.xVec, self.yVec)
								assert(abs(np.linalg.norm(self.zVec) - 1.0) < 0.01)
								print(f'[calibrate] M matrix calculated!')
								print(f'[[{self.xVec[0]}, {self.xVec[1]}, {self.xVec[2]}], [{self.yVec[0]}, {self.yVec[1]}, {self.yVec[2]}],[{self.zVec[0]}, {self.zVec[1]}, {self.zVec[2]}]]')
								self.M = np.array([self.xVec, self.yVec, self.zVec]).T
								self.saveM()
							self.gravityData = []
		else: # self.needCalibrate == False
			print(f'[running] M: {self.M}')
			while self.flag.value != 0:
				self.put_frame()
				normAcc = np.linalg.norm(self.data_imu[:3])
				if not self.running and abs(normAcc - self.G_ref) < self.G_threshold:
					print(f'[running] getting q0...')
					self.Q0 = self.getQ0(self.M.dot(np.array([0, -1, 0])), self.data_imu[:3])
					# self.Q0 = ahrs.common.orientation.acc2q(self.data_imu[:3])
					self.madgwick = ahrs.filters.Madgwick(gain = 1)
					self.ekf = EKF()
					# self.madgwick.Dt = self.cur_time - self.last_frame_time
					# self.madgwick.frequency = 1 / self.madgwick.Dt
					self.last_frame_time = self.cur_time
					self.Q = self.madgwick.updateIMU(self.Q0, gyr=self.data_imu[3:], acc=self.data_imu[:3])
					# self.Q = self.ekf.update(self.Q0, gyr=self.data_imu[3:], acc=self.data_imu[:3])
					self.running = True
				elif self.running:
					self.Q = self.madgwick.updateIMU(self.Q, gyr=self.data_imu[3:], acc=self.data_imu[:3])
					# self.Q = self.ekf.update(self.Q, gyr=self.data_imu[3:], acc=self.data_imu[:3])
					R = quat_to_mat(self.Q)
					# headRot = self.M.I * R * self.M
					headRot = linalg.inv(self.M).dot(R).dot(self.M)
					headPos = headRot.dot(np.array([0, 0, -1]))
					pos = headPos
					assert(abs(np.linalg.norm(pos) - 1.0) < 0.01)
					# x = np.degrees(np.arctan(pos[2] / pos[0]))
					# y = np.degrees(np.arccos(pos[1]))
					x = np.degrees(np.arctan2(-pos[2], pos[0]))
					y = np.degrees(np.arcsin(pos[1]))

					value = np.sum(self.data_pressure[:,2:4])
					r32 = headRot[2][1]
					r33 = headRot[2][2]
					r31 = headRot[2][0]
					r21 = headRot[1][0]
					r11 = headRot[0][0]
					ax = np.degrees(np.arctan2(r32, r33))
					ay = np.degrees(np.arctan2(-r31, np.sqrt(r32*r32+r33*r33)))
					az = np.degrees(np.arctan2(r21, r11))
					# print(f'[running] x: {ax}, y: {ay}, z:{az}')
					###print(f'[running] pos: {pos[0]}, {pos[1]}, {pos[2]}')
					###print(f'[running] euler: {ax}, {ay}, {az}')
					#0:blue 1:orange 2: green
					# self.measure[:3] = [ax, ay, 0]
					
					self.measure[:3] = [az, ax, value]
					# self.measure[:3] = self.Q[1:]
					# self.measure[:3] = self.data_imu[:3]
					###print(f'[running] x: {x}, y: {y}, v: {value}')
					# print(f'[running] x: {ay}, y: {ax}, v:{value}')
					self.smoothX.update(az)
					self.smoothY.update(ax)
					x = self.smoothX.getData()
					y = self.smoothY.getData()
					if self.kwargs['inputCommand'].value == 1:
						self.y_top = y
						print('y top set!!!')
						self.kwargs['inputCommand'].value = 0
					elif self.kwargs['inputCommand'].value == 2:
						self.y_bot = y
						print('y bot set!!!')
						self.kwargs['inputCommand'].value = 0
					elif self.kwargs['inputCommand'].value == 3:
						self.x_bot = x
						print('x bot set!!!')
						self.kwargs['inputCommand'].value = 0
					elif self.kwargs['inputCommand'].value == 4:
						self.x_top = x
						print('x top set!!!')
						self.kwargs['inputCommand'].value = 0
					elif self.kwargs['inputCommand'].value == 5:
						self.pressure_threshold = value + 0.3
						print('value set!!!!')
						self.kwargs['inputCommand'].value = 0
					elif self.kwargs['inputCommand'].value == 6:
						self.click_threshold = value
						print('click value set!!!')
						self.kwargs['inputCommand'].value = 0
					self.send_swn(x, self.x_bot, self.x_top, y, self.y_bot, self.y_top, value)
					# self.send_swn(az, y, value)
	def send_swn(self, anglex, x_bot, x_top, angley, y_bot, y_top, value):
		threshold = self.pressure_threshold
		# x_top = 0.3
		# x_bot = 0.1
		# y_top = -2.3	
		# y_bot = -2.13
		# if value > self.click_threshold:
		# 	self.my_cursor_client.sendButton('click')
		# 	return

		x = (anglex - x_bot) / (x_top - x_bot)
		y = (y_top - angley) / (y_top - y_bot)
		x = 0 if x < 0 else x
		y = 0 if y < 0 else y
		x = 1 if x > 1 else x
		y = 1 if y > 1 else y
		# x = anglex
		# y = angley
		# if not self.calibrating:
		# 	print(f'calculated x:{x}, y:{y}')
		# toSend = True
		# if abs(x - self.last_x) > 0.1 or abs(y - self.last_y) > 0.1:
		# 	toSend = False
		# if toSend and abs(x - self.last_x) < 0.05 and abs(y - self.last_y)<0.05:
		# 	toSend = False
		# print(f'send? {toSend}')
		self.last_x = x
		self.last_y = y
		# if not toSend:
		# 	return
		# self.measure[0:3] = [x, y, 0]
		if value > threshold:
			if self.touching:
				pass
				# self.my_cursor_client.send(2, self.last_x, self.last_y)
			else:
				# self.last_x = 0.5
				# self.last_y = 0.5
				# self.my_cursor_client.send(1,self.last_x, self.last_y)
				self.touching = True
		else:
			if self.touching:
				# self.my_cursor_client.send(3, self.last_x, self.last_y)
				self.touching = False
				self.inputting = not self.inputting
				if not self.inputting:
					self.my_cursor_client.send(3, x, y)
				else:
					self.my_cursor_client.send(1, x, y)
			else:
				self.my_cursor_client.send(2 if self.inputting else 4, x, y)

		row, col, val = self.my_processor.parse(self.data_pressure)
		moving, x, y, val = self.my_cursor.update(row, col, val)
		x, y = mapping(x, y, left=LEFT, right=RIGHT, up=UP, down=DOWN)
		gesture = self.my_classifier.classify(x, y, moving)

		if gesture == SwipeGesture.UP:
			self.my_cursor_client.sendButton('up')
		elif gesture == SwipeGesture.DOWN:
			self.my_cursor_client.sendButton('down')
		elif gesture == SwipeGesture.LEFT:
			self.my_cursor_client.sendButton('left')
		elif gesture == SwipeGesture.RIGHT:
			self.my_cursor_client.sendButton('right')
		elif gesture == SwipeGesture.CLICK:
			self.my_cursor_client.sendButton('click')


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

	def send(self, x, y, value):
		# print('in send', x, y)
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


	def put_frame(self):
		self.data_imu, self.frame_idx = self.my_client.fetch_imu_and_index(new=True)  ## get a new IMU frame
		self.data_pressure = self.my_client.fetch_frame()

		self.data_imu[:3] *= UNIT_ACC
		self.data_imu[3:] *= UNIT_GYR_RAD


		self.data_imu[0] -= self.calibrateCenter[0]
		self.data_imu[1] -= self.calibrateCenter[1]
		self.data_imu[2] -= self.calibrateCenter[2]

		self.data_imu[0] /= (self.calibrateScale[0] / 9.801)
		self.data_imu[1] /= (self.calibrateScale[1] / 9.801)
		self.data_imu[2] /= (self.calibrateScale[2] / 9.801)

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
			# self.measure[1] = v3[1]

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


def task_data(measure, flag, args, config):
	kwargs = args
	kwargs['ip'] = IP
	kwargs['ip_port'] = IP_PORT
	kwargs['measure'] = measure
	kwargs['flag'] = flag

	with Uclient(
		config['connection']['client_address'], 
		config['connection']['server_address'], 
		udp=config['connection']['udp'], 
		n=config['sensor']['shape']
	) as my_client:
		with CursorClient(kwargs['ip'], kwargs['ip_port']) as my_cursor_client:
			kwargs['client'] = my_cursor_client
			kwargs['uclient'] = my_client
			kwargs['config'] = config
			my_proc = ProcIMU(kwargs)
			my_proc.run()

def task_debug(measure, flag):
	import random
	while flag.value != 0:
		measure[0] = random.random()
		measure[1] = random.random()
		measure[2] = random.random()

def prepare_config(args):
	## load config and combine commandline arguments
	if args.config:
		config = load_config(args.config)
	else:
		config = blank_config()
	## priority: commandline arguments > config file > program defaults
	if config['sensor']['shape'] is None or hasattr(args, 'n'+DEST_SUFFIX):
		config['sensor']['shape'] = args.n
	if config['connection']['udp'] is None or hasattr(args, 'udp'+DEST_SUFFIX):
		config['connection']['udp'] = args.udp
	if config['connection']['server_address'] is None or hasattr(args, 'server_address'+DEST_SUFFIX):
		config['connection']['server_address'] = args.server_address
	if config['connection']['client_address'] is None or hasattr(args, 'client_address'+DEST_SUFFIX):
		config['connection']['client_address'] = args.client_address
	check_config(config)

	return config

def task_visual(flag, args):
	def gen_wrapper():
		while True:
			yield args['measure']
	ytop = None
	ybottom = None
	my_player = Player1D(generator=gen_wrapper(), channels=3, timespan=5, 
							ytop=ytop, ybottom=ybottom, exit_flag=flag)
	my_player.run_stream()
	flag.value = 0
	

def main(args):
	config = prepare_config(args)

	# measure = Value('d')  # d for double
	measure = Array('d', 3)  # d for double
	flag = Value('i')  # i for int
	flag.value = 1
	inputCommand = Value('i')
	inputCommand.value = 0
	kwargs = vars(args)
	kwargs['inputCommand'] = inputCommand
	kwargs['measure'] = measure

	

	if not args.debug:
		# p = Process(target=task_data, args=(measure, flag, args.port, 
		# 							args.baudrate, args.timeout, args.acc, args.gyr, args.calibrate))
		p = Process(target=task_data, args=(measure, flag, kwargs, config))
	else:
		p = Process(target=task_debug, args=(measure,flag,))
	p.start()

	
	if not (args.calibrate or args.imu_calibrate):
		pVisual = Process(target = task_visual, args=(flag, kwargs))
		pVisual.start()
	
	# if args.acc:
	# 	ytop = MAX_ACC
	# 	ybottom = -MAX_ACC
	# elif args.gyr:
	# 	ytop = MAX_GYR
	# 	ybottom = -MAX_GYR
	while flag.value != 0:
		key = input('Waiting for command__')
		if key == 'w':
			inputCommand.value = 1
		elif key == 's':
			inputCommand.value = 2
		elif key == 'a':
			inputCommand.value = 3
		elif key == 'd':
			inputCommand.value = 4
		elif key == 'v':
			inputCommand.value = 5
		elif key == 'x':
			inputCommand.value = 6
		elif key == 'q':
			flag.value = 0
	
	
	# else:
	# 	q = input('wating for exit....')
	# 	flag.value = 0
	p.join()
	if not (args.calibrate or args.imu_calibrate):
		pVisual.join()


if __name__ == '__main__':
	parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
	parser.add_argument('--server_address', dest='server_address', action=('store'), help="specify server socket address")
	parser.add_argument('--client_address', dest='client_address', action=('store'), help="specify client socket address")
	parser.add_argument('-u', '--udp', dest='udp', action=('store_true'), default=UDP, help="use UDP protocol")
	parser.add_argument('-n', dest='n', action=('store'), default=[N], type=int, nargs='+', help="specify sensor shape")

	parser.add_argument('--config', dest='config', action=('store'), default=CONFIG_PATH, help="specify configuration file")

	parser.add_argument('-d', '--debug', dest='debug', action=('store_true'), default=False, help="debug mode")
	parser.add_argument('-g', '--gyr', dest='gyr', action=('store_true'), default=False, help="show gyroscope data")
	parser.add_argument('-a', '--acc', dest='acc', action=('store_true'), default=False, help="show accelerometer data")
	parser.add_argument('-c', '--calibrate', dest='calibrate', action=('store_true'), default=False, help="Need calibration or not")
	parser.add_argument('-i', '--imu_calibrate', dest='imu_calibrate', action=('store_true'), default=False, help="IMU offset calibration or not")

	args = parser.parse_args()

	main(args)
