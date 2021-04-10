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
from toolkit.tools import parse_ip_port, load_config, blank_config, check_shape, parse_mask


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
		my_setter = DataSetterSerial(
			paras['config']['sensor']['total'], 
			paras['config']['serial']['baudrate'], 
			paras['config']['serial']['port'], 
			paras['config']['serial']['timeout'],
		)
		my_proc = Proc(
			paras['config']['sensor']['shape'], 
			my_setter, 
			paras['data_out'], 
			paras['data_raw'], 
			paras['idx_out'],
			raw=paras['config']['server_mode']['raw'], 
			V0=paras['config']['process']['V0'],
			R0_RECI=paras['config']['process']['R0_RECI'],
			convert=paras['config']['process']['convert'],
			filter_spatial=paras['config']['process']['filter_spatial'],
			filter_spatial_cutoff=paras['config']['process']['filter_spatial_cutoff'],
			butterworth_order=paras['config']['process']['butterworth_order'],
			filter_temporal=paras['config']['process']['filter_temporal'],
			filter_temporal_size=paras['config']['process']['filter_temporal_size'],
			rw_cutoff=paras['config']['process']['rw_cutoff'],
			mask=paras['config']['sensor']['mask'],
			queue_from_server=paras['queue_to_main_client'],
		)
		my_proc.run()
	except KeyboardInterrupt:
		pass
	except CustomException as e:
		print(e)
	except BaseException as e:
		traceback.print_exc()
		# print(e)
	finally:
		paras['queue_to_main_client'].put(FLAG.FLAG_STOP)
	print("Processing stopped.")

def task_server(paras):
	try:
		with Userver(
			paras['data_out'], 
			paras['data_raw'], 
			paras['idx_out'], 
			paras['config']['connection']['server_address'], 
			total=paras['config']['sensor']['total'],
			queue_to_main_client=paras['queue_to_main_client'],
			udp=paras['config']['connection']['udp']
		) as my_server:
			my_server.run_service()
	except KeyboardInterrupt:
		pass
	except CustomException as e:
		print(e)
	except BaseException as e:
		traceback.print_exc()
		# print(e)
	finally:
		paras['queue_to_main_client'].put(FLAG.FLAG_STOP)

def task_file(paras):
	## TODO
	pass

def print_sensor_info(config):
	# print(config)
	print(f"Sensor shape: {config['sensor']['shape']}")
	print(f"Sensor size:  {config['sensor']['total']}")
	print(f"Sensor mask:  {'None' if config['sensor']['mask'] is None else ''} ")
	if config['sensor']['mask'] is not None:
		print(config['sensor']['mask'])

def main(args):
	## load config and combine commandline arguments
	if args.config:
		config = load_config(args.config)
	else:
		config = blank_config()
	if config['sensor']['shape'] is None:
		config['sensor']['shape'] = args.n
	if config['serial']['baudrate'] is None:
		config['serial']['baudrate'] = args.baudrate
	if config['serial']['timeout'] is None:
		config['serial']['timeout'] = args.timeout
	if config['serial']['port'] is None:
		config['serial']['port'] = args.port
	if config['connection']['udp'] is None:
		config['connection']['udp'] = args.udp
	if config['connection']['server_address'] is None:
		config['connection']['server_address'] = args.address
	if config['process']['convert'] is None:
		config['process']['convert'] = not args.no_convert
	if config['visual']['zlim'] is None:
		config['visual']['zlim'] = args.zlim
	if config['visual']['fps'] is None:
		config['visual']['fps'] = args.fps
	if config['visual']['pyqtgraph'] is None:
		config['visual']['pyqtgraph'] = args.pyqtgraph
	if config['server_mode']['service'] is None:
		config['server_mode']['service'] = args.service
	if config['server_mode']['raw'] is None:
		config['server_mode']['raw'] = args.raw
	if config['server_mode']['visualize'] is None:
		config['server_mode']['visualize'] = args.visualize
	if config['server_mode']['enumerate'] is None:
		config['server_mode']['enumerate'] = args.enumerate

	## some modifications
	config['sensor']['shape'] = check_shape(config['sensor']['shape'])
	config['sensor']['total'] = config['sensor']['shape'][0] * config['sensor']['shape'][1]
	if config['sensor']['mask'] is not None:
		config['sensor']['mask'] = parse_mask(config['sensor']['mask'])

	## enumerate serial ports
	if config['server_mode']['enumerate']:
		enumerate_ports()
		return

	print_sensor_info(config)

	## shared variables
	data_out = Array('d', config['sensor']['total'])  # d for double
	data_raw = Array('d', config['sensor']['total'])  # d for double
	idx_out = Value('i')  # i for int
	idx_out_file = Value('i')
	queue_to_main_client = Queue()

	paras = {
		"config": config,
		"data_out": data_out,
		"data_raw": data_raw,
		"idx_out": idx_out,
		"idx_out_file": idx_out_file,
		"queue_to_main_client": queue_to_main_client,
	}

	if config['server_mode']['visualize']:
		p = Process(target=task_serial, args=(paras,))
		p.start()

		if not config['visual']['pyqtgraph']:
			from toolkit.visual.player_matplot import Player3DMatplot as Player
			print("Activate visualization using matplotlib")
		else:
			from toolkit.visual.player_pyqtgraph import Player3DPyqtgraph as Player
			print("Activate visualization using pyqtgraph")
		## visualization must be in main process
		from toolkit.visual import gen_reshape
		my_player = Player(
			zlim=config['visual']['zlim'], 
			N=config['sensor']['shape']
		)
		my_player.run_stream(
			generator=gen_reshape(data_out, config['sensor']['shape']), 
			fps=config['visual']['fps']
		)

		p.join()
	else:
		if config['server_mode']['service']:
			p_server = Process(target=task_server, args=(paras,))
			p_server.start()

		task_serial(paras)

		if config['server_mode']['service']:
			p_server.join()


if __name__ == '__main__':
	parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
	parser.add_argument('-e', dest='enumerate', action='store_true', default=False, help="enumerate all serial ports")
	parser.add_argument('-p', dest='port', action='store', default=PORT, help="specify serial port")
	parser.add_argument('-b', dest='baudrate', action='store', default=BAUDRATE, type=int, help="specify baudrate")
	parser.add_argument('-t', dest='timeout', action='store', default=TIMEOUT, type=float, help="specify timeout in seconds")
	parser.add_argument('-n', dest='n', action='store', default=[N], type=int, nargs='+', help="specify sensor shape")
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
	parser.add_argument('--config', dest='config', action='store', default=None, help="specify configuration file")
	args = parser.parse_args()

	main(args)
