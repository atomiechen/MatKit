import argparse

from collections import deque
from enum import Enum, IntEnum
import numpy as np

from socket import socket, AF_INET, SOCK_STREAM, timeout
import time

from toolkit.uclient import Uclient
from toolkit.process import Processor, CursorController
from toolkit.tools import load_config, blank_config


CONFIG_PATH = "config/tmp_config_mouth_16_16.yaml"

IP = "localhost"
PORT = 8081


def auto_reconnect(wait_time=1):
	def decorator(func):
		def wrapper(self, *args, **kwargs):
			try:
				return func(self, *args, **kwargs)
			except Exception as e:
				print(f"Remote server no respond. Reconnecting...")

			while True:
				try:
					self.connect()
					return
				except Exception as e:
					print(f"Sleep {wait_time}s to retry...")
					time.sleep(wait_time)
		return wrapper
	return decorator

class CursorClient:

	@auto_reconnect()
	def __init__(self, server_addr, port, timeout=1):

		self.server_addr = server_addr
		self.port = port
		self.timeout = timeout

		self.connect()

	def __enter__(self):
		return self

	def __exit__(self, type, value, traceback):
		self.close()

	def connect(self):
		self.my_socket = socket(AF_INET, SOCK_STREAM)
		self.my_socket.settimeout(self.timeout)
		self.my_socket.connect((self.server_addr, self.port))
		print(f"client connecting to server: {self.server_addr}:{self.port}")

	def close(self):
		self.my_socket.close()
		print("remote client socket closed")

	@auto_reconnect()
	def send(self, touch_state, x, y):
		paras = [touch_state, x, y]
		self.my_socket.send(str(" ".join([str(item) for item in paras]) + "\n").encode())
	
	@auto_reconnect()
	def sendButton(self, cmd):
		self.my_socket.send((cmd+"\n").encode())
	
	@auto_reconnect()
	def sendPressure(self, pressure):
		self.my_socket.send((str(pressure)+"\n").encode())


def task(config):
	my_remote_handle = CursorClient(IP, PORT)

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
		while True:
			## get a new frame
			frame, frame_idx = my_client.fetch_frame_and_index(new=True)
			row, col, val = my_processor.parse(frame)
			moving, x, y, val = my_cursor.update(row, col, val)
			my_remote_handle.sendPressure(val)


def main():
	## load config and combine commandline arguments
	if args.config:
		config = load_config(args.config)
	else:
		config = blank_config()

	task(config)


if __name__ == '__main__':
	parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
	parser.add_argument('--config', dest='config', action='store', default=CONFIG_PATH, help="specify configuration file")
	args = parser.parse_args()

	main()
