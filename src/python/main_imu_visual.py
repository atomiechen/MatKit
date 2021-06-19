from serial import Serial
from serial.tools.list_ports import comports
import argparse
import numpy as np
from struct import calcsize, pack, unpack, unpack_from
import time

from multiprocessing import Process  # 进程
from multiprocessing import Array  # 共享内存
from multiprocessing import Value  # 共享内存

import ahrs

from toolkit.visual.player1d import Player1D


BAUDRATE = 1000000
TIMEOUT = 1  # in seconds
devices_found = comports()
PORT = None
try:
	## default use the last port on the list
	PORT = devices_found[-1].device
except:
	pass

RESOLUTION = 2**15
MAX_ACC = 2 * 9.8  ## m/s^2
MAX_GYR = 2000  ## degree/s (dps)
UNIT_ACC = MAX_ACC / RESOLUTION
UNIT_GYR = MAX_GYR / RESOLUTION


def enumerate_ports():
	# 查看可用端口
	print("All serial ports:")
	for item in devices_found:
		print(item)

def task_serial(measure, flag, port, baudrate, timeout=None, acc=False, gyr=False):
	def read_byte():
		recv = my_serial.read()
		if len(recv) != 1:
			raise Exception("Serial timeout!")
		return recv[0]

	## serial
	my_serial = Serial(port, baudrate, timeout=timeout)
	print(my_serial)

	## AHRS
	madgwick = ahrs.filters.Madgwick()
	Q = np.array([1., 0., 0., 0.]) # Allocate for quaternions

	## store IMU data
	data_imu = np.zeros(6, dtype=float)
	frame_size = 256+12

	## check fps
	frame_idx = 0
	last_frame_idx = 0
	cur_time = time.time()
	last_time = cur_time
	last_frame_time = cur_time

	## protocol ref: https://blog.csdn.net/weixin_43277501/article/details/104805286
	frame = bytearray()
	begin = False
	while flag.value != 0:
		recv = read_byte()
		if begin:
			if recv == 0x5C:
				## escape bytes
				recv = read_byte()
				if recv == 0x00:
					frame.append(0x5C)
				elif recv == 0x01:
					frame.append(0x5B)
				elif recv == 0x02:
					frame.append(0x5D)
				else:
					print(f"Wrong ESCAPE byte: {recv}")
			elif recv == 0x5D:
				## end a frame
				if len(frame) != frame_size:
					## wrong length, re-fetch a frame
					print(f"Wrong frame size: {len(frame)}")
				else:
					frame_idx += 1
					last_frame_time = cur_time
					cur_time = time.time()
					if cur_time - last_time >= 1:
						duration = cur_time - last_time
						frames = frame_idx - last_frame_idx
						print(f"  frame rate: {frames/duration:.3f} fps")
						last_time = cur_time
						last_frame_idx = frame_idx
					# data_array[:256] = frame[:256]
					if data_imu is not None:
						pos = 256
						for i in range(6):
							data_imu[i] = unpack_from(f"=h", frame, pos)[0]
							pos += calcsize(f"=h")
						data_imu[:3] *= UNIT_ACC
						data_imu[3:] *= UNIT_GYR
						if acc:
							## accelerometer
							measure[:3] = data_imu[:3]
						elif gyr:
							## gyroscope
							measure[:3] = data_imu[3:]
						else:
							## AHRS
							madgwick.Dt = cur_time - last_frame_time
							madgwick.frequency = 1 / madgwick.Dt
							Q = madgwick.updateIMU(Q, gyr=data_imu[3:]*np.pi/180, acc=data_imu[:3])
							v3 = Q[1:] / Q[0]

							measure[:3] = v3
				frame = bytearray()
				begin = False
			else:
				frame.append(recv)
		elif recv == 0x5B:
			## begin a frame
			begin = True
	my_serial.close()

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
