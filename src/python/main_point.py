import argparse
import time
import pyautogui

from toolkit.uclient import Uclient
from toolkit.process import Processor, CursorController, PressureSelector

N = 16
INTERP = 64
TH = 0.1
ALPHA = 0.5
## 点击阈值的设置因人而异，设为用户压力最大值的三分之一
CLICK = 0.46

## cursor state
STATE_REST = 0
STATE_HOVER = 1

UDP = False

def main(args):
	## control cursor
	pyautogui.FAILSAFE = False
	screenWidth, screenHeight = pyautogui.size()
	## 两个缩放系数ratioX和ratioY与两个方向活动范围的倒数成比例时，缩放后的活动范围为正方形

	## 演示绝对映射，将归一化坐标直接线性映射到屏幕上
	# ratioX = screenWidth - 1
	# ratioY = screenHeight - 1

	## 演示相对滑动，即实际触摸板的情况：手指滑动-抬起，然后再滑动-抬起，光标一段一段移动
	ratioX = 400
	ratioY = 600

	## trackpoint方式：类似ThinkPad小红点，通过偏移向量来间接控制光标
	## 陈伟浩的trackpoint参数
	# ratioX = 1/0.5334880856434193
	# ratioY = 1/0.4088509411708816

	state = STATE_REST

	my_processor = Processor(args.interp, threshold=args.threshold)
	my_cursor = CursorController(ratioX=ratioX, ratioY=ratioY, 
					mapcoor=args.mapcoor, alpha=args.alpha, 
					trackpoint=args.trackpoint)
	my_selector_qr = PressureSelector(levels=[args.click, 2*args.click], 
					mode="quickrelease")
	my_selector_dw = PressureSelector(levels=[args.click], 
					mode="dwell", timeout=0.3)

	with Uclient(args.client_addr, args.server_addr, udp=args.udp, n=args.n) as my_client:
		my_processor.print_info()
		my_cursor.print_info()
		my_generator = my_processor.gen_points(my_client.gen())
		my_generator = my_cursor.gen_coors(my_generator)
		last_time = time.time()
		cnt = 0
		check_time = 1
		for ret, x, y, val in my_generator:
			x = 1-x
			# ret, x, y, val = my_cursor.update(row, col, val)
			if ret:
				# print(x, y)
				if args.mapcoor:
					pyautogui.moveTo(x, y, _pause=False)
				else:
					pyautogui.move(x, y, _pause=False)
			else:
				time.sleep(0.004)  # about > 194 fps

			selection_qr, _ = my_selector_qr.get_selection(val)
			selection_dw, region = my_selector_dw.get_selection(val)

			if selection_dw == 1:
				state = STATE_HOVER
				pyautogui.mouseDown(_pause=False)
			elif state == STATE_HOVER:
				if region == 0:
					state = STATE_REST
					pyautogui.mouseUp(_pause=False)
			elif selection_qr == 1:
				## left click
				# pyautogui.click(_pause=False)
				pyautogui.mouseDown(_pause=False)
				pyautogui.mouseUp(_pause=False)
			elif selection_qr == 2:
				## right click
				# pyautogui.rightClick(_pause=False)
				pyautogui.mouseDown(button="right", _pause=False)
				pyautogui.mouseUp(button="right", _pause=False)

			# if val > 3 * args.click:
			# 	if state == 1:
			# 		pyautogui.mouseUp(_pause=False)  # release left button
			# 		pyautogui.rightClick(_pause=False)
			# 	state = 2
			# elif val > args.click:
			# 	if state == 0:
			# 		pyautogui.mouseDown(_pause=False)
			# 	state = 1
			# else:
			# 	if state != 0:
			# 		pyautogui.mouseUp(_pause=False)
			# 	state = 0

			# pyautogui.click(button="right", _pause=False)
			# pyautogui.mouseDown(button='right', _pause=False)
			# time.sleep(2)
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
	parser.add_argument('--mapcoor', dest='mapcoor', action='store_true', default=False, help="map coordinates directly")
	parser.add_argument('--trackpoint', dest='trackpoint', action='store_true', default=False, help="TrackPoint style")
	parser.add_argument('--interp', dest='interp', action='store', default=INTERP, type=int, help="interpolated side size")
	parser.add_argument('--th', dest='threshold', action='store', default=TH, type=float, help="blob filter threshold")
	parser.add_argument('--alpha', dest='alpha', action='store', default=ALPHA, type=float, help="point smoothing value")
	parser.add_argument('--click', dest='click', action='store', default=CLICK, type=float, help="click value")
	args = parser.parse_args()

	main(args)
