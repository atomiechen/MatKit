import argparse

from collections import deque
from enum import Enum, IntEnum
import numpy as np

from socket import socket, AF_INET, SOCK_STREAM, timeout

from toolkit.uclient import Uclient
from toolkit.process import Processor, CursorController
from toolkit.tools import load_config, blank_config


CONFIG_PATH = "config/tmp_config_mouth_16_16.yaml"

IP = "localhost"
PORT = 8081


class CursorClient:
    def __init__(self, server_addr, port, timeout=1):
        self.my_socket = socket(AF_INET, SOCK_STREAM)
        self.my_socket.settimeout(timeout)
        self.connect(server_addr, port)

    def __exit__(self, type, value, traceback):
        self.close()

    def connect(self, address, port):
        self.my_socket.connect((address, port))
        print(f"client connecting to server: {address}")

    def close(self):
        self.my_socket.close()
        print("remote client socket closed")

    def send(self, touch_state, x, y):
        paras = [touch_state, x, y]
        self.my_socket.send(str(" ".join([str(item) for item in paras]) + "\n").encode())
    
    def sendButton(self, cmd):
        self.my_socket.send((cmd+"\n").encode())
    
    def sendPressure(self, pressure):
        self.my_socket.send((str(pressure)+"\n").encode())


def main():
	## load config and combine commandline arguments
	if args.config:
		config = load_config(args.config)
	else:
		config = blank_config()

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

	my_remote_handle = CursorClient(IP, PORT)

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


if __name__ == '__main__':
	parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
	parser.add_argument('--config', dest='config', action='store', default=CONFIG_PATH, help="specify configuration file")
	args = parser.parse_args()

	main()
