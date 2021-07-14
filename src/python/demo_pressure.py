import argparse

from collections import deque
from enum import Enum, IntEnum
import numpy as np

from socket import socket, AF_INET, SOCK_STREAM, timeout
import time

from toolkit.uclient import Uclient
from toolkit.process import Processor, CursorController
from toolkit.tools import load_config, blank_config

from demokit.cursor_client import CursorClient


CONFIG_PATH = "config/tmp_config_mouth_16_16.yaml"

IP = "localhost"
PORT = 8081

def task(config):
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

	with CursorClient(IP, PORT) as my_remote_handle:
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
