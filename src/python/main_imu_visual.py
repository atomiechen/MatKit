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
from pyquaternion import Quaternion

from toolkit.visual.player1d import Player1D
from toolkit import filemanager

from pca import pca_h, pca_v
from scipy.optimize import leastsq
from scipy.signal import savgol_filter
from scipy import linalg
import json
import pathlib

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
		print("remote?client socket closed")

	def send(self, touch_state, x, y):
		## touch state: 1按下，2按下时移动，3松开，4松开时移动
		paras = [touch_state, x, y]
		print(f"[send]: {paras}")
		self.my_socket.send((" ".join([str(item) for item in paras])+"\n").encode())

	def sendButton(self, cmd):
		self.my_socket.send((cmd+"\n").encode())


MAX_ACC = 2 * 9.8  ## m/s^2
MAX_GYR = 2000  ## degree/s (dps)
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
	def __init__(self, N = 100):
		self.data = []
		self.N = N
	def update(self, d):
		self.data.append(d)
		if len(self.data) > self.N:
			self.data = self.data[-self.N:]
	def getData(self):
		return np.mean(self.data)



class ProcIMU:

	RESOLUTION = 2**15
	MAX_ACC = 2 * 9.801  ## m/s^2
	MAX_GYR = 2000  ## degree/s (dps)
	UNIT_ACC = MAX_ACC / RESOLUTION
	UNIT_GYR = MAX_GYR / RESOLUTION
	UNIT_GYR_RAD = UNIT_GYR * np.pi / 180

	FILENAME_TEMPLATE = "imu_%Y%m%d%H%M%S.csv"

	V0 = 255
	R0_RECI = 1  ## a constant to multiply the value

	def __init__(self, measure, flag, port, baudrate, timeout=None, acc=False, gyr=False, needCalibrate=False, my_cursor_client=None):
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
		# print(self.h_vec, self.v_vec)
		self.smoothX = Smooth()
		self.smoothY = Smooth()
		self.inputting = False
		# self.std_Q = (0.48688449,-0.28526537,0.82494558,0.03212407)
		# self.rotQ = (quat_to_mat(self.std_Q)).I

		self.needCalibrate = needCalibrate
		self.M = np.array([[0.18883633390998655, -0.9758716640520846, 0.10961448031921497], [0.9179313065437382, 0.21507203171789888, 0.3333858689861354],[-0.34891683172690546, 0.03766319785879299, 0.9363965655985224]]).T
		self.firstPass = True
		self.gravityCheck = False
		self.gravityData = []
		self.running = False
		self.G_ref = 10.0
		self.G_threshold = 0.2
		if not self.needCalibrate:
			self.loadM()
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

	def getQ0(self, gReal, gCur):
		print('in get q0')
		print(gReal, gCur)
		angle = np.arccos(np.dot(gReal, gCur) / np.linalg.norm(gReal) / np.linalg.norm(gCur))
		axis = np.cross(gReal, gCur)
		axis = axis / np.linalg.norm(axis)
		return (np.cos(angle / 2), axis[0]*np.sin(angle / 2), axis[1] * np.sin(angle / 2), axis[2] * np.sin(angle / 2))
	def run(self):
		## serial
		self.my_serial = Serial(self.port, self.baudrate, timeout=self.timeout)
		print(self.my_serial)

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
					self.madgwick = ahrs.filters.Madgwick(gain = 1)
					# self.madgwick.Dt = self.cur_time - self.last_frame_time
					# self.madgwick.frequency = 1 / self.madgwick.Dt
					self.last_frame_time = self.cur_time
					self.Q = self.madgwick.updateIMU(self.Q0, gyr=self.data_imu[3:], acc=self.data_imu[:3])
					self.running = True
				elif self.running:
					self.Q = self.madgwick.updateIMU(self.Q, gyr=self.data_imu[3:], acc=self.data_imu[:3])
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

					value = np.sum(self.data_pressure.reshape(16,-1)[:,2:4])
					r32 = headRot[2][1]
					r33 = headRot[2][2]
					r31 = headRot[2][0]
					r21 = headRot[1][0]
					r11 = headRot[0][0]
					ax = np.degrees(np.arctan2(r32, r33))
					ay = np.degrees(np.arctan2(-r31, np.sqrt(r32*r32+r33*r33)))
					az = np.degrees(np.arctan2(r21, r11))
					# print(f'[running] x: {ax}, y: {ay}, z:{az}')
					print(f'[running] pos: {pos[0]}, {pos[1]}, {pos[2]}')
					print(f'[running] euler: {ax}, {ay}, {az}')
					#0:blue 1:orange 2: green
					# self.measure[:3] = [ax, ay, 0]
					
					self.measure[:3] = [ax, ay, az]
					# self.measure[:3] = self.data_imu[:3]
					print(f'[running] x: {x}, y: {y}, v: {value}')
					# print(f'[running] x: {ay}, y: {ax}, v:{value}')
					self.smoothX.update(x)
					self.smoothY.update(y)
					x = self.smoothX.getData()
					y = self.smoothY.getData()
					self.send_swn(az, -1, -30, ax, -110, -130, value)
					# self.send_swn(az, y, value)
	def send_swn(self, anglex, x_bot, x_top, angley, y_bot, y_top, value):
		threshold = 43
		# x_top = 0.3
		# x_bot = 0.1
		# y_top = -2.3	
		# y_bot = -2.13

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
	


	




	def runn(self):
		## serial
		self.my_serial = Serial(self.port, self.baudrate, timeout=self.timeout)
		print(self.my_serial)

		# self.cali()
		# print(*self.rotation.inverse)
		self.rotation = Quaternion(0.7506447624364577,-0.0,-0.12936740371211275,-0.6479170591082638)
		self.gravityCheck = False
		self.gLen = 0
		self.firstPass = False
		print(f'START CALIBRATE')
		if self.calibrate:
			while self.flag.value != 0:
				self.put_frame()
				accData = self.data_imu[:3]
				gyrData = self.data_imu[3:]
				if not self.gravityCheck:
					self.checkGravity()
					print(f'\r[calibrate] Checking Gravity...', end='')
				else:
					print(f'[calibrate] Gravity pass!')
		else:
		

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
				# if self.calibrating:
				# 	print('calibrating Q:', self.Q)

				euler = self.euler_from_quaternion(*self.Q)
				# x = (- pca_h.transform([euler])[0][0] - 20) / 70
				# y = ((-pca_v.transform([euler])[0][0] - 20) / 20 + 2 ) / 3

				# x = -pca_h.transform([self.Q])[0][0] 
				# y = pca_v.transform([self.Q])[0][0]
				##############new try
				self.curPos = self.rotQ * quat_to_mat(self.Q)
				self.curVec = self.curPos * np.matrix([1, 0, 0]).T
				# print(self.curVec)
				Xangle = np.degrees(np.arctan(self.curVec[1][0] / self.curVec[0][0]))
				Yangle = np.degrees(np.arccos(self.curVec[2][0]))
				if not self.calibrating:
					print(f'angles: {Xangle}, {Yangle}')

				self.smoothX.update(Xangle.A[0][0])
				Xangle = self.smoothX.getData()
				self.smoothY.update(Yangle.A[0][0])
				Yangle = self.smoothY.getData()
				if not self.calibrating:
					print(f'smooth angles: {Xangle}, {Yangle}')

				#calculate alpha and phi
				##############new try end
				# print(f':{x}, y:{y}')
				# print('quad:', self.Q)
				H = np.sum(np.dot(self.h_vec, np.reshape(quat_to_mat(self.Q), (9, -1))))
				V = np.sum(np.dot(self.v_vec, np.reshape(quat_to_mat(self.Q), (9, -1))))
				
				# print(H.shape)
				# print('HHHH', H)
				# print('VVVV', V)


				# self.measure[0:3] = self.data_imu[3:] # self.Q[1:4]
				# self.measure[1] = y

				# v = (v)
				# x /= 2
				# self.measure[0] = y
				# self.measure[1] = x

				value = np.sum(self.data_pressure.reshape(16,-1)[:,2:4])
				# value = np.mean(self.data_pressure)
				if not self.calibrating:
					print(f'value:{value}')
				print('self.measure:', self.measure[:])
				self.measure[:] = [self.last_x, self.last_y, 0]
				self.send_swn(Xangle, Yangle, value)
				# self.send(H, V, value)

	####
				maxv = np.argmax(np.abs(self.measure))
				# print(self.measure[:])
				# if np.abs(self.measure[1]) > 5:
				# self.measure[0:3] = self.data_imu[3:]
				dy = -self.measure[1] * (self.cur_time - self.last_time)
				# else:
				# 	dy = 0
				# if np.abs(self.measure[0]) + np.abs(self.measure[2]) > 10:
				dx = (self.measure[0] + self.measure[2]) * (self.cur_time - self.last_time) / 2
				# else:
				# 	dx = 0
	####
				# print(Quat_current)
				
				

				# self.measure[0:3] = self.Q[1:]
				# print(value)
				# self.send(x, y, value, (dx/4 , dy /6))
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
		# print(f'delata:{delta}')
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

	def send(self, x, y, value, delta):
		threshold = 40
		dx, dy = delta
		# print(f'delata:{delta}')
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



def task_serial(measure, flag, port, baudrate, timeout=None, acc=False, gyr=False, needCalibrate=False, ip=IP, ip_port=IP_PORT):
	with CursorClient(ip, ip_port) as my_cursor_client:
		my_proc = ProcIMU(measure, flag, port, baudrate, timeout, acc, gyr, needCalibrate, my_cursor_client)
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
									args.baudrate, args.timeout, args.acc, args.gyr, args.calibrate))
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
	if not args.calibrate:
		my_player = Player1D(generator=gen_wrapper(), channels=3, timespan=5, 
							ytop=ytop, ybottom=ybottom)
		my_player.run_stream()

		flag.value = 0
	else:
		p = input()
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
	parser.add_argument('-c', '--calibrate', dest='calibrate', action=('store_true'), default=False, help="Need calibration or not")

	args = parser.parse_args()

	main(args)
