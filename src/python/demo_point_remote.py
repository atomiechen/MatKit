import argparse
import time
from datetime import datetime

from toolkit.uclient import Uclient
from toolkit.process import Processor, CursorController, PressureSelector
from toolkit import filemanager

from demokit.cursor_client import CursorClient


N = 16
INTERP = 64
TH = 0.15
# TH = 20
ALPHA = 0.25
# ALPHA = 0.2
## 点击阈值的设置因人而异，设为用户压力最大值的三分之一
CLICK = 0.25
# CLICK = 50
TOTAL = 10

UDP = False

## cursor state
STATE_HOVER = 0
STATE_MOVE = 1

IP = "localhost"
# IP = "183.173.190.54"
PORT = 8081

FORMAT_TIMESTAMP = "%Y%m%d%H%M%S%f"
FORMAT_FILE_TIME = "%Y%m%d%H%M%S"
PATH_OUTFILE = "../../data/tmp/click_{}.csv"

timestamp = datetime.now().strftime(FORMAT_FILE_TIME)
filename = PATH_OUTFILE.format(timestamp)

def get_timestamp():
	return datetime.now().strftime(FORMAT_TIMESTAMP)

# def mapping(x, y, left=0.5, right=0.75, up=0.2, down=0.7):
def mapping(x, y, left=0.35, right=0.75, up=0.2, down=0.6):
	x0 = 1-y
	y0 = 1-x
	x1 = min(max(x0 - left, 0) / (right - left), 0.999)
	y1 = min(max(y0 - up, 0) / (down - up), 0.999)
	return x1, y1

def constrain(value, vmin=0, vmax=0.999):
	return max(min(value, vmax), vmin)

def main(args):
	if args.mapcoor or args.trackpoint:
		ratioX = 1
		ratioY = 1
	else:
		ratioX = 1.5
		ratioY = 1.5

	state = STATE_HOVER
	touching = False

	my_processor = Processor(args.interp, threshold=args.threshold, total=TOTAL)
	my_cursor = CursorController(ratioX=ratioX, ratioY=ratioY, 
					mapcoor=args.mapcoor, alpha=args.alpha, 
					trackpoint=args.trackpoint)

	abs_x = 0
	abs_y = 0

	with CursorClient(args.ip, args.port) as my_cursor_client:
		with Uclient(args.client_addr, args.server_addr, udp=args.udp, n=args.n) as my_client:
			my_processor.print_info()
			my_cursor.print_info()
			my_generator = my_processor.gen_points(my_client.gen())
			my_generator = my_cursor.gen_coors(my_generator)
			last_time = time.time()
			cnt = 0
			check_time = 1
			for ret, x, y, val in my_generator:
				if ret:
					if not touching:
						print("touch")
						# content = [5, get_timestamp(), x, y]
						# filemanager.write_line(filename, content)	
					touching = True

					if args.mapcoor or args.trackpoint:
						x, y = mapping(x, y)
					else:
						x, y = -y, -x
						abs_x = constrain(abs_x+x)
						abs_y = constrain(abs_y+y)
						x = abs_x
						y = abs_y
					print(x, y)

					if val >= args.click:
						if state == STATE_HOVER:
							my_cursor_client.send(1, x, y)
							print("press down")
							# content = [1, get_timestamp(), x, y]
							# filemanager.write_line(filename, content)
						else:
							my_cursor_client.send(2, x, y)
							# content = [2, get_timestamp(), x, y]
						state = STATE_MOVE
					else:
						if state == STATE_MOVE:
							my_cursor_client.send(3, x, y)
							# content = [3, get_timestamp(), x, y]
							# filemanager.write_line(filename, content)
							print("press up")
						else:
							my_cursor_client.send(4, x, y)
							# content = [4, get_timestamp(), x, y]
						state = STATE_HOVER
				else:
					if touching:
						print("untouch")
						# content = [6, get_timestamp(), x, y]
						# filemanager.write_line(filename, content)
					touching = False	
					time.sleep(0.004)  # about > 194 fps

				cnt += 1
				cur_time = time.time()
				duration = cur_time - last_time
				if duration >= check_time:
					print(f"{cnt / duration} fps")
					last_time = cur_time
					cnt = 0


if __name__ == '__main__':
	parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
	parser.add_argument('-s', dest='server_addr', action='store', help="specify server socket address")
	parser.add_argument('-c', dest='client_addr', action='store', help="specify client socket address")
	parser.add_argument('-u', '--udp', dest='udp', action='store_true', default=UDP, help="use UDP protocol")
	parser.add_argument('-n', dest='n', action='store', default=N, type=int, help="sensor side size")
	parser.add_argument('--ip', dest='ip', action='store', default=IP, help="specify remote server ip address")
	parser.add_argument('--port', dest='port', action='store', default=PORT, type=int, help="specify remote server port")
	parser.add_argument('--mapcoor', dest='mapcoor', action='store_true', default=False, help="map coordinates directly")
	parser.add_argument('--trackpoint', dest='trackpoint', action='store_true', default=False, help="TrackPoint style")
	parser.add_argument('--interp', dest='interp', action='store', default=INTERP, type=int, help="interpolated side size")
	parser.add_argument('--th', dest='threshold', action='store', default=TH, type=float, help="blob filter threshold")
	parser.add_argument('--alpha', dest='alpha', action='store', default=ALPHA, type=float, help="point smoothing value")
	parser.add_argument('--click', dest='click', action='store', default=CLICK, type=float, help="click value")
	args = parser.parse_args()

	main(args)
