import argparse
try:
	import readline
except ImportError:
	pass
from toolkit.uclient import CMD, Uclient
from toolkit.process import Processor
from toolkit.tools import parse_ip_port

N = 16
INTERP = None
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
	if args.udp:
		if args.client_addr is not None:
			args.client_addr = parse_ip_port(args.client_addr)
		if args.server_addr is not None:
			args.server_addr = parse_ip_port(args.server_addr)
	with Uclient(args.client_addr, args.server_addr, udp=args.udp, n=args.n) as my_client:
		if args.interactive:
			print("Interactive mode")
			run_client_interactive(my_client)
		else:
			print("Plot mode")
			if args.matplot:
				from toolkit.visual.player_matplot import Player3DMatplot as Player
			else:
				from toolkit.visual.player_pyqtgraph import Player3DPyqtgraph as Player
			if args.raw:
				print("  raw data")
				input_arg = CMD.RAW
				args.noblob = True
			else:
				print("  processed data")
				input_arg = CMD.DATA

			if not args.interp:
				args.interp = args.n
			my_processor = Processor(args.interp, noblob=args.noblob, 
							threshold=args.threshold)
			my_processor.print_info()
			my_player = Player(zlim=3, N=args.interp)

			my_generator = my_client.gen(input_arg)
			my_generator = my_processor.gen_wrapper(my_generator)
			my_player.run_stream(generator=my_generator, fps=args.fps)


if __name__ == '__main__':
	parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
	parser.add_argument('-s', dest='server_addr', action='store', help="specify server socket address")
	parser.add_argument('-c', dest='client_addr', action='store', help="specify client socket address")
	parser.add_argument('-u', '--udp', dest='udp', action='store_true', default=UDP, help="use UDP protocol")
	parser.add_argument('-r', '--raw', dest='raw', action='store_true', default=False, help="plot raw data")
	parser.add_argument('-n', dest='n', action='store', default=N, type=int, help="sensor side size")
	parser.add_argument('--interp', dest='interp', action='store', default=INTERP, type=int, help="interpolated side size")
	parser.add_argument('--noblob', dest='noblob', action='store_true', default=False, help="do not filter out blob")
	parser.add_argument('--th', dest='threshold', action='store', default=TH, type=float, help="blob filter threshold")
	parser.add_argument('-i', '--interactive', dest='interactive', action='store_true', default=False, help="interactive mode")
	parser.add_argument('-f', dest='fps', action='store', default=FPS, type=int, help="frames per second")
	parser.add_argument('-m', '--matplot', dest='matplot', action='store_true', default=False, help="use mathplotlib to plot")
	args = parser.parse_args()

	main(args)
