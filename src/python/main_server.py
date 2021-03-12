from serial import Serial
from serial.tools.list_ports import comports
import argparse

from multiprocessing import Process
from multiprocessing import Array  # 共享内存
from multiprocessing import Value  # 共享内存
from multiprocessing import Queue

import traceback

from toolkit.server import Proc, Userver, DataSetterSerial, DataSetterFile
from toolkit.server import FLAG, CustomException
from toolkit.tools import parse_ip_port

N = 16  # sensor side length
BAUDRATE = 500000
TIMEOUT = 1  # in seconds
devices_found = comports()
PORT = None
try:
	## default use the last port on the list
	PORT = devices_found[-1].device
except:
	pass

UDP = False
NO_CONVERT = False

ZLIM = 3
FPS = 100


def enumerate_ports():
	# 查看可用端口
	print("All serial ports:")
	for item in devices_found:
		print(item)

def task_serial(paras):
	try:
		my_setter = DataSetterSerial(paras['n']**2, paras['baudrate'], 
									paras['port'], paras['timeout'])
		my_proc = Proc(paras['n'], my_setter, paras['data_out'], 
					paras['data_raw'], paras['idx_out'], queue=paras['queue_to_serial'],
					raw=paras['raw'], convert=paras['convert'], queue_from_server=paras['queue_to_file'])
		my_proc.run()
	except KeyboardInterrupt:
		pass
	except CustomException as e:
		print(e)
	except BaseException as e:
		traceback.print_exc()
		# print(e)
	finally:
		paras['queue'].put(FLAG.FLAG_STOP)
	print("Processing stopped.")

def task_server(paras):
	try:
		with Userver(paras['data_out'], paras['data_raw'], paras['idx_out'], 
					paras['server_addr'], n=paras['n'],
					queue_to_serial=paras['queue_to_serial'],
					queue_to_file=paras['queue_to_file'],
					queue=paras['queue'],
					udp=paras['udp']) as my_server:
			my_server.run_service()
	except KeyboardInterrupt:
		pass
	except CustomException as e:
		print(e)
	except BaseException as e:
		traceback.print_exc()
		# print(e)
	finally:
		paras['queue'].put(FLAG.FLAG_STOP)

def task_file(paras):
	## TODO
	pass

def main(args):
	if args.enum:
		enumerate_ports()
		return

	data_out = Array('d', args.n**2)  # d for double
	data_raw = Array('d', args.n**2)  # d for double
	idx_out = Value('i')  # i for int
	idx_out_file = Value('i')
	queue = Queue()
	queue_to_serial = Queue()
	queue_to_file = Queue()

	if args.udp and args.address is not None:
		args.address = parse_ip_port(args.address)

	paras = {
		"n": args.n,
		"baudrate": args.baudrate,
		"port": args.port,
		"timeout": args.timeout,
		"raw": args.raw,
		"data_out": data_out,
		"data_raw": data_raw,
		"idx_out": idx_out,
		"idx_out_file": idx_out_file,
		"queue": queue,
		"queue_to_serial": queue_to_serial,
		"queue_to_file": queue_to_file,
		"convert": not args.no_convert,
		"udp": args.udp,
		"server_addr": args.address,
	}

	if args.visualize:
		p = Process(target=task_serial, args=(paras,))
		p.start()

		if not args.pyqtgraph:
			from toolkit.visual.player_matplot import Player3DMatplot as Player
			print("Activate visualization using matplotlib")
		else:
			from toolkit.visual.player_pyqtgraph import Player3DPyqtgraph as Player
			print("Activate visualization using pyqtgraph")
		## visualization must be in main process
		from toolkit.visual import gen_reshape
		my_player = Player(zlim=args.zlim, N=args.n)
		my_player.run_stream(generator=gen_reshape(data_out, args.n), fps=args.fps)

		p.join()
	else:
		if args.service:
			p_server = Process(target=task_server, args=(paras,))
			p_server.start()

		task_serial(paras)

		if args.service:
			p_server.join()


if __name__ == '__main__':
	parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
	parser.add_argument('-e', dest='enum', action='store_true', default=False, help="enumerate all serial ports")
	parser.add_argument('-p', dest='port', action='store', default=PORT, help="specify serial port")
	parser.add_argument('-b', dest='baudrate', action='store', default=BAUDRATE, type=int, help="specify baudrate")
	parser.add_argument('-t', dest='timeout', action='store', default=TIMEOUT, type=float, help="specify timeout in seconds")
	parser.add_argument('-n', dest='n', action='store', default=N, type=int, help="specify sensor size")
	parser.add_argument('-s', '--service', dest='service', action='store_true', default=False, help="run service")
	parser.add_argument('-a', '--address', dest='address', action='store', help="specify server socket address")
	parser.add_argument('-u', '--udp', dest='udp', action='store_true', default=UDP, help="use UDP protocol")
	parser.add_argument('-r', '--raw', dest='raw', action='store_true', default=False, help="raw data mode")
	parser.add_argument('-nc', '--no_convert', dest='no_convert', action='store_true', default=NO_CONVERT, help="do not apply voltage-resistance conversion")
	parser.add_argument('-v', '--visualize', dest='visualize', action='store_true', default=False, help="enable visualization")
	parser.add_argument('-z', '--zlim', dest='zlim', action='store', default=ZLIM, type=float, help="z-axis limit")
	parser.add_argument('-f', dest='fps', action='store', default=FPS, type=int, help="frames per second")
	parser.add_argument('--pyqtgraph', dest='pyqtgraph', action='store_true', default=False, help="use pyqtgraph to plot")
	# parser.add_argument('-m', '--matplot', dest='matplot', action='store_true', default=False, help="use matplotlib to plot")
	args = parser.parse_args()

	main(args)
