import argparse
from os import times
try:
	import readline
except ImportError:
	pass
import copy
import argparse
import time, sys

from toolkit.uclient import CMD, Uclient
from toolkit.process import Processor
from toolkit.tools import (
	parse_ip_port, load_config, blank_config, check_shape, check_config,
	make_action, DEST_SUFFIX
)

IP = "localhost"
PORT = 8081

from socket import socket, AF_INET, SOCK_STREAM, timeout
N = 16
ZLIM = 3
FPS = 194
TH = 0.15
UDP = False
RIGHT_THRESHOLD = 1.2
LEFT_THRESHOLD = 1.8
UP_THRESHOLD = 2
DOWN_THRESHOLD = 2

def run_client_interactive(my_client):
	while True:
		try:
			data = input('>> ').strip()
			if data and "quit".startswith(data) or data == "exit":
				return
		except (EOFError, KeyboardInterrupt):
			return

		try:
			data = data.strip().split()
			if not data:
				raise Exception
			my_cmd = int(data[0])
		except:
			continue

		my_client.interactive_cmd(my_cmd)

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

def run_client_output(my_client):
	my_remote_handle = CursorClient(IP, PORT)
	last_press = False
	last_direction = "right"
	last_timer = 0
	dtime = 0
	peak = 1.5
	last_send_time = 0
	SEND_INTERVAL = 0.2
	while True:
		time.sleep(0.01)
		my_client.send_cmd(1)
		max_row = 0
		for i in range(int(len(my_client.recv_frame()[0]) / 6)):
			sum_row = 0
			for j in range(6):
				sum_row += my_client.recv_frame()[0][i * 6 + j]
			if sum_row > max_row:
				max_row = sum_row
				max_id = i
		# print(max_row)
		# print(max_id)
		if max_id < 5 or max_id > 22:
			if max_row > peak / 3 and max_row > DOWN_THRESHOLD:
				if max_row > peak:
					peak = max_row
				# print(max_row)
				# print(max_id)
				last_direction = "down"
				last_press = True
				dtime = int((time.time() - last_timer) * 1000)
				if dtime > 500:
					print((dtime - 500) / 1000)
				continue
		elif max_id <= 9:
			if max_row > peak / 3 and max_row > RIGHT_THRESHOLD:
				# print(peak)
				if max_row > peak:
					peak = max_row
				# print(max_row)
				# print(max_id)
				last_direction = "right"
				last_press = True
				dtime = int((time.time() - last_timer) * 1000)
				# if dtime > 500:
				# 	print((dtime - 500) / 1000)
				continue
		elif max_id <= 16:
			if max_row > peak / 3 and max_row > UP_THRESHOLD:
				if max_row > peak:
					peak = max_row
				# print(max_row)
				# print(max_id)
				last_direction = "up"
				last_press = True
				dtime = int((time.time() - last_timer) * 1000)
				# if dtime > 500:
				# 	print((dtime - 500) / 1000)
				continue
		else:
			if max_row > peak / 3 and max_row > LEFT_THRESHOLD:
				if max_row > peak:
					peak = max_row
				# print(max_row)
				# print(max_id)
				last_direction = "left"
				last_press = True
				dtime = int((time.time() - last_timer) * 1000)
				# if dtime > 500:
				# 	print((dtime - 500) / 1000)
				continue
		last_timer = time.time()
		if last_press:
			last_press = False
			if dtime < 500 and 100 < dtime:
				if time.time() - last_send_time > SEND_INTERVAL:
					print(last_direction)
					# print(max_row)
					my_remote_handle.sendButton(last_direction)
					last_send_time = time.time()
			elif dtime > 1000:
				if time.time() - last_send_time > SEND_INTERVAL:
					my_remote_handle.sendButton('click')
					print("click")
					last_send_time = time.time()
		dtime = 0
		if (max_row < 2):
			peak = 1.5

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
	if config['process']['interp'] is None or hasattr(args, 'interp'+DEST_SUFFIX):
		config['process']['interp'] = args.interp
	if config['process']['blob'] is None or hasattr(args, 'noblob'+DEST_SUFFIX):
		config['process']['blob'] = not args.noblob
	if config['process']['threshold'] is None or hasattr(args, 'threshold'+DEST_SUFFIX):
		config['process']['threshold'] = args.threshold
	if config['visual']['zlim'] is None or hasattr(args, 'zlim'+DEST_SUFFIX):
		config['visual']['zlim'] = args.zlim
	if config['visual']['fps'] is None or hasattr(args, 'fps'+DEST_SUFFIX):
		config['visual']['fps'] = args.fps
	if config['visual']['pyqtgraph'] is None or hasattr(args, 'matplot'+DEST_SUFFIX):
		config['visual']['pyqtgraph'] = not args.matplot
	if config['client_mode']['raw'] is None or hasattr(args, 'raw'+DEST_SUFFIX):
		config['client_mode']['raw'] = args.raw
	if config['client_mode']['interactive'] is None or hasattr(args, 'interactive'+DEST_SUFFIX):
		config['client_mode']['interactive'] = args.interactive
	if config['client_mode']['output'] is None or hasattr(args, 'output'+DEST_SUFFIX):
		config['client_mode']['output'] = args.output
	check_config(config)

	## some modifications
	if config['process']['interp'] is None:
		config['process']['interp'] = copy.deepcopy(config['sensor']['shape'])

	return config


def main(args):
	config = prepare_config(args)

	with Uclient(
		config['connection']['client_address'], 
		config['connection']['server_address'], 
		udp=config['connection']['udp'], 
		n=config['sensor']['shape']
	) as my_client:
		if config['client_mode']['interactive']:
			print("Interactive mode")
			run_client_interactive(my_client)
		elif config['client_mode']['output']:
			print("Output mode")
			run_client_output(my_client)
		else:
			print("Plot mode")
			if config['visual']['pyqtgraph']:
				from toolkit.visual.player_pyqtgraph import Player3DPyqtgraph as Player
			else:
				from toolkit.visual.player_matplot import Player3DMatplot as Player
			if config['client_mode']['raw']:
				print("  raw data")
				input_arg = CMD.RAW
				config['process']['blob'] = False
			else:
				print("  processed data")
				input_arg = CMD.DATA

			my_processor = Processor(
				config['process']['interp'], 
				blob=config['process']['blob'], 
				threshold=config['process']['threshold'],
				order=config['process']['interp_order'],
				total=config['process']['blob_num'],
				special=config['process']['special_check'],
			)
			my_processor.print_info()
			my_player = Player(
				zlim=config['visual']['zlim'], 
				N=config['process']['interp']
			)

			my_generator = my_client.gen(input_arg)
			my_generator = my_processor.gen_wrapper(my_generator)
			my_player.run_stream(
				generator=my_generator, 
				fps=config['visual']['fps']
			)


if __name__ == '__main__':
	parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
	parser.add_argument('--server_address', dest='server_address', action=make_action('store'), help="specify server socket address")
	parser.add_argument('--client_address', dest='client_address', action=make_action('store'), help="specify client socket address")
	parser.add_argument('-u', '--udp', dest='udp', action=make_action('store_true'), default=UDP, help="use UDP protocol")
	parser.add_argument('-r', '--raw', dest='raw', action=make_action('store_true'), default=False, help="plot raw data")
	parser.add_argument('-n', dest='n', action=make_action('store'), default=[N], type=int, nargs='+', help="specify sensor shape")
	parser.add_argument('--interp', dest='interp', action=make_action('store'), default=None, type=int, nargs='+', help="interpolated shape")
	parser.add_argument('--noblob', dest='noblob', action=make_action('store_true'), default=False, help="do not filter out blob")
	parser.add_argument('--th', dest='threshold', action=make_action('store'), default=TH, type=float, help="blob filter threshold")
	parser.add_argument('-i', '--interactive', dest='interactive', action=make_action('store_true'), default=False, help="interactive mode")
	parser.add_argument('-z', '--zlim', dest='zlim', action=make_action('store'), default=ZLIM, type=float, help="z-axis limit")
	parser.add_argument('-f', dest='fps', action=make_action('store'), default=FPS, type=int, help="frames per second")
	parser.add_argument('-m', '--matplot', dest='matplot', action=make_action('store_true'), default=False, help="use mathplotlib to plot")
	parser.add_argument('--config', dest='config', action=make_action('store'), default=None, help="specify configuration file")
	args = parser.parse_args()

	main(args)
