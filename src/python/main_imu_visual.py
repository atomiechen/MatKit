from serial import Serial
from serial.tools.list_ports import comports
import argparse
import numpy as np
from struct import calcsize, pack, unpack, unpack_from

from multiprocessing import Process  # 进程
from multiprocessing import Array  # 共享内存
from multiprocessing import Value  # 共享内存

from toolkit.visual.player2d import Player2D


BAUDRATE = 1000000
devices_found = comports()
PORT = None
try:
	## default use the last port on the list
	PORT = devices_found[-1].device
except:
	pass


def enumerate_ports():
	# 查看可用端口
	print("All serial ports:")
	for item in devices_found:
		print(item)

def task_serial(measure, flag, port, baudrate):
	my_serial = Serial(port, baudrate)
	print(my_serial)

	data_imu = np.zeros(6, dtype=float)
	frame_size = 256+12

	## ref: https://blog.csdn.net/weixin_43277501/article/details/104805286
	frame = bytearray()
	begin = False
	while flag.value != 0:
		recv = my_serial.read()[0]
		if begin:
			if recv == 0x5C:
				## escape bytes
				recv = my_serial.read()[0]
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
					# data_array[:256] = frame[:256]
					if data_imu is not None:
						pos = 256
						for i in range(6):
							data_imu[i] = unpack_from(f"=h", frame, pos)[0]
							pos += calcsize(f"=h")
						measure[:3] = data_imu[3:]
				frame = bytearray()
				begin = False
			else:
				frame.append(recv)
		elif recv == 0x5B:
			## begin a frame
			begin = True

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
		p = Process(target=task_serial, args=(measure,flag,args.port,args.baudrate))
	else:
		p = Process(target=task_debug, args=(measure,flag,))
	p.start()

	my_player = Player2D(generator=gen_wrapper(), channels=3, timespan=10)
	my_player.run_stream()

	flag.value = 0
	p.join()


if __name__ == '__main__':
	parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
	parser.add_argument('-e', dest='enumerate', action=('store_true'), default=False, help="enumerate all serial ports")
	parser.add_argument('-p', dest='port', action=('store'), default=PORT, help="specify serial port")
	parser.add_argument('-b', dest='baudrate', action=('store'), default=BAUDRATE, type=int, help="specify baudrate")
	parser.add_argument('-d', '--debug', dest='debug', action=('store_true'), default=False, help="debug mode")

	args = parser.parse_args()

	main(args)
