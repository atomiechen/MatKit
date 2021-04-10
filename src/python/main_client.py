import argparse
try:
	import readline
except ImportError:
	pass
import copy

from toolkit.uclient import CMD, Uclient
from toolkit.process import Processor
from toolkit.tools import parse_ip_port, load_config, blank_config, check_shape

N = 16
ZLIM = 3
FPS = 194
TH = 0.15
UDP = False

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

def main(args):
	## load config and combine commandline arguments
	if args.config:
		config = load_config(args.config)
	else:
		config = blank_config()
	if config['sensor']['shape'] is None:
		config['sensor']['shape'] = args.n
	if config['connection']['udp'] is None:
		config['connection']['udp'] = args.udp
	if config['connection']['server_address'] is None:
		config['connection']['server_address'] = args.server_addr
	if config['connection']['client_address'] is None:
		config['connection']['client_address'] = args.client_addr
	if config['process']['interp'] is None:
		config['process']['interp'] = args.interp
	if config['process']['blob'] is None:
		config['process']['blob'] = not args.noblob
	if config['process']['threshold'] is None:
		config['process']['threshold'] = args.threshold
	if config['visual']['zlim'] is None:
		config['visual']['zlim'] = args.zlim
	if config['visual']['fps'] is None:
		config['visual']['fps'] = args.fps
	if config['visual']['pyqtgraph'] is None:
		config['visual']['pyqtgraph'] = not args.matplot
	if config['client_mode']['raw'] is None:
		config['client_mode']['raw'] = args.raw
	if config['client_mode']['interactive'] is None:
		config['client_mode']['interactive'] = args.interactive

	## some modifications
	config['sensor']['shape'] = check_shape(config['sensor']['shape'])
	if config['process']['interp'] is None:
		config['process']['interp'] = copy.deepcopy(config['sensor']['shape'])
	else:
		config['process']['interp'] = check_shape(config['process']['interp'])
	print(config)

	with Uclient(
		config['connection']['client_address'], 
		config['connection']['server_address'], 
		udp=config['connection']['udp'], 
		n=config['sensor']['shape']
	) as my_client:
		if config['client_mode']['interactive']:
			print("Interactive mode")
			run_client_interactive(my_client)
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
	parser.add_argument('-s', dest='server_addr', action='store', help="specify server socket address")
	parser.add_argument('-c', dest='client_addr', action='store', help="specify client socket address")
	parser.add_argument('-u', '--udp', dest='udp', action='store_true', default=UDP, help="use UDP protocol")
	parser.add_argument('-r', '--raw', dest='raw', action='store_true', default=False, help="plot raw data")
	parser.add_argument('-n', dest='n', action='store', default=[N], type=int, nargs='+', help="specify sensor shape")
	parser.add_argument('--interp', dest='interp', action='store', default=None, type=int, nargs='+', help="interpolated shape")
	parser.add_argument('--noblob', dest='noblob', action='store_true', default=False, help="do not filter out blob")
	parser.add_argument('--th', dest='threshold', action='store', default=TH, type=float, help="blob filter threshold")
	parser.add_argument('-i', '--interactive', dest='interactive', action='store_true', default=False, help="interactive mode")
	parser.add_argument('-z', '--zlim', dest='zlim', action='store', default=ZLIM, type=float, help="z-axis limit")
	parser.add_argument('-f', dest='fps', action='store', default=FPS, type=int, help="frames per second")
	parser.add_argument('-m', '--matplot', dest='matplot', action='store_true', default=False, help="use mathplotlib to plot")
	parser.add_argument('--config', dest='config', action='store', default=None, help="specify configuration file")
	args = parser.parse_args()

	main(args)
