import argparse

from toolkit.uclient import Uclient
from toolkit.process import Processor, CursorController
from toolkit.tools import load_config, blank_config

from demokit.swipe_gesture import SwipeGesture, SwipeGestureClassifier
from demokit.cursor_client import CursorClient


CONFIG_PATH = "config/tmp_config_mouth_16_16.yaml"
LEFT = 0.1
RIGHT = 0.9
UP = 0.1
DOWN = 0.9

IP = "localhost"
PORT = 8081


def mapping(x, y, left=0, right=1, up=0, down=1):
	x0 = 1-y
	y0 = 1-x
	x1 = min(max(x0 - left, 0) / (right - left), 0.999)
	y1 = min(max(y0 - up, 0) / (down - up), 0.999)
	return x1, y1

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

	## 0: 舌势交互
	## 1: 文本输入
	mode = 1

	with CursorClient(IP, PORT) as my_remote_handle:
		with Uclient(
			config['connection']['client_address'], 
			config['connection']['server_address'], 
			udp=config['connection']['udp'], 
			n=config['sensor']['shape']
		) as my_client:
			my_classifier = SwipeGestureClassifier(debug=args.debug)
			frame_idx = 0
			while True:
				## get a new frame
				frame, frame_idx = my_client.fetch_frame_and_index(new=True)

				row, col, val = my_processor.parse(frame)
				moving, x, y, val = my_cursor.update(row, col, val)
				x, y = mapping(x, y, left=LEFT, right=RIGHT, up=UP, down=DOWN)

				gesture = my_classifier.classify(x, y, moving)
				if gesture != SwipeGesture.NONE:
					print(f"gesture: {gesture.name}")

				if args.morse:
					if gesture == SwipeGesture.UP:
						my_remote_handle.sendButton('di')
					elif gesture == SwipeGesture.DOWN:
						my_remote_handle.sendButton('da')
					elif gesture == SwipeGesture.LEFT:
						my_remote_handle.sendButton('reset')
					elif gesture == SwipeGesture.RIGHT:
						my_remote_handle.sendButton('reset')
					elif gesture == SwipeGesture.CLICK:
						my_remote_handle.sendButton('space')
				else:
					if gesture == SwipeGesture.UP:
						print("识别到后滑舌势")
						my_remote_handle.sendButton('up')
					elif gesture == SwipeGesture.DOWN:
						print("识别到前滑舌势")
						my_remote_handle.sendButton('down')
					elif gesture == SwipeGesture.LEFT:
						print("识别到左滑舌势")
						my_remote_handle.sendButton('left')
					elif gesture == SwipeGesture.RIGHT:
						print("识别到右滑舌势")
						my_remote_handle.sendButton('right')
					elif gesture == SwipeGesture.CLICK:
						my_remote_handle.sendButton('click')
					elif gesture == SwipeGesture.PRESS:
						if mode == 0:
							print("进入文本打字模式")
							mode = 1
						else:
							print("进入舌势交互模式")
							mode = 0


if __name__ == '__main__':
	parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
	parser.add_argument('--config', dest='config', action='store', default=CONFIG_PATH, help="specify configuration file")
	parser.add_argument('-m', '--morse', dest='morse', action='store_true', default=False, help="Morse input")

	parser.add_argument('--debug', dest='debug', action='store_true', default=False, help="debug mode")
	args = parser.parse_args()

	main()