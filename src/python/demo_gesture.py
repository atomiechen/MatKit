import argparse

from toolkit.uclient import Uclient
from toolkit.process import Processor, CursorController
from toolkit.tools import load_config, blank_config

from demokit.swipe_gesture import SwipeGesture, SwipeGestureClassifier


CONFIG_PATH = "config/tmp_config_mouth_16_16.yaml"
LEFT = 0.1
RIGHT = 0.9
UP = 0.1
DOWN = 0.9


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

	if args.no_safe_check:
		config['process']['threshold'] = 0.1
		config['process']['special'] = False
		config['pointing']['alpha'] = 1

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

	cnt = 0

	with Uclient(
		config['connection']['client_address'], 
		config['connection']['server_address'], 
		udp=config['connection']['udp'], 
		n=config['sensor']['shape']
	) as my_client:
		my_classifier = SwipeGestureClassifier(debug=args.debug, safe_check=not args.no_safe_check)
		frame_idx = 0
		while True:
			## get a new frame
			frame, frame_idx = my_client.fetch_frame_and_index(new=True)

			row, col, val = my_processor.parse(frame)
			moving, x, y, val = my_cursor.update(row, col, val)
			x, y = mapping(x, y, left=LEFT, right=RIGHT, up=UP, down=DOWN)

			gesture = my_classifier.classify(x, y, moving)
			if gesture != SwipeGesture.NONE:
				cnt += 1
				print(f"gesture: {gesture.name}")
				print(f"触发次数：{cnt}")


if __name__ == '__main__':
	parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
	parser.add_argument('--config', dest='config', action='store', default=CONFIG_PATH, help="specify configuration file")
	parser.add_argument('--debug', dest='debug', action='store_true', default=False, help="debug mode")
	parser.add_argument('--no_safe_check', dest='no_safe_check', action='store_true', default=False, help="disable safe check")
	args = parser.parse_args()

	main()
