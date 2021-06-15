import argparse
import copy
import socketserver
import time
from socket import gethostname, gethostbyname

from multiprocessing import Process
from multiprocessing import Queue

import numpy as np

from toolkit.uclient import CMD, Uclient
from toolkit.tools import (
	parse_ip_port, load_config, blank_config, check_shape, parse_mask, 
	check_config, print_sensor
)


CONFIG_PATH = "config/config_finger_8_8.yaml"

TH = 0.25

## 0: blank
## 1: surface
## 2: edge

state = 0


def classifier(frame):
	global state
	shape = frame.shape
	max_value = np.max(frame)

	# print(f"max={max_value}")
	# count = np.count_nonzero(frame>TH)
	if max_value < TH:
		## nothing
		state = 0
		# print("none")
		return "0,0,0\n"
	else: 
		mat = frame[frame >= TH]
		count = len(mat)
		data = frame - TH
		data[data < 0] = 0

		if count > 20 or state == 1:
			## surface
			state = 1

			r_sum = 0
			c_sum = 0
			w_sum = 0
			for i in range(shape[0]):
				for j in range(shape[1]):
					r_sum += data[i, j] * i
					c_sum += data[i, j] * j
					w_sum += data[i, j]
			r_center = r_sum / w_sum - (shape[0]-1)/2
			c_center = c_sum / w_sum - (shape[1]-1)/2
			r_center *= 0.005
			c_center *= 0.005
			r_center -= 0.002
			c_center += 0.006
			print(1, r_center, c_center)
			return f"1,{r_center},{c_center}\n"
		else:
			## edge
			state = 2

			row_mean = np.mean(data, axis=1)
			row_edge = np.argmax(row_mean)
			row_sim = row_mean[row_edge]

			col_mean = np.mean(data, axis=0)
			col_edge = np.argmax(col_mean)
			col_sim = col_mean[col_edge]


			if row_sim > col_sim:
				# print("ROW!!!")
				# print(row_sim)
				row_sim *= -1000
				return f"3,{row_sim},0\n"
			else:
				# print("COL!!!")
				# print(coor)
				tmp = data[:, col_edge]
				coor = 0
				w_sum = 0
				for i in range(shape[0]):
					coor += i * tmp[i]
					w_sum += tmp[i]
				coor /= w_sum
				coor -= (shape[0]-1)/2
				coor *= 0.001
				return f"2,{coor},0\n"


class MyHandler(socketserver.BaseRequestHandler):
	def handle(self):
		# global connect_request
		print('connected!')
		self.data = self.request.recv(1024).strip()
		print("{} wrote:".format(self.client_address[0]))
		print(self.data)

		while True:
			frame = self.server.my_client.fetch_frame()
			self.data = classifier(frame)
			print(self.data[:-1]+"\r", end="")
			self.request.sendall(bytes(self.data, 'utf-8'))
			time.sleep(0.01)

			# # 0, 1, 1
			# # 1, 0.2, 0.1
			# # 2, 1.2
			# # 3, 10
			# # null, null
			# # move, dirx, diry
			# # resize, scale
			# # rotate, speed        
			# time.sleep(2)
			# self.data = "1,0.03,0.05\n"
			# self.request.sendall(bytes(self.data, 'utf-8'))

			# time.sleep(2)
			# self.data = "1,-0.03,-0.05\n"
			# self.request.sendall(bytes(self.data, 'utf-8'))

			# time.sleep(2)
			# self.data = "2,0.004,0\n"
			# self.request.sendall(bytes(self.data, 'utf-8'))
			# time.sleep(2)
			# self.data = "2,-0.004,0\n"
			# self.request.sendall(bytes(self.data, 'utf-8'))
			# time.sleep(2)
			# self.data = "3,-60,0\n"
			# self.request.sendall(bytes(self.data, 'utf-8'))

			# time.sleep(2)
			# self.data = "0,0,0\n"
			# self.request.sendall(bytes(self.data, 'utf-8'))
			# # just send back the same data, but upper-cased
			# # self.request.sendall(bytes(self.data.upper(), 'utf-8'))


def class_vedge(frame, n, mask=None):
	if mask is None:
		mask = np.ones(n)
	kernel = np.array([1,1,1,1,1,1,1,1])
	sim = np.zeros(n[0])
	for i in range(n[1]):
		# print(sum(mask[:, i]))
		sim[i] = kernel.dot(frame[:, i] * mask[:, i]) / sum(mask[:, i])

	th_max = 5
	th_var = 1
	print(sim, sim.max(), sim.var())
	if sim.max() > th_max and sim.var() < th_var:
		return True
	else:
		return False

def task_hololens_server(my_client):
	HOSTNAME = gethostname()
	try:
		MY_IP_ADDR = gethostbyname(HOSTNAME)
	except:
		MY_IP_ADDR = gethostbyname("localhost")
	print(f'My IP address: {MY_IP_ADDR}')

	HOST, PORT = "0.0.0.0", 23456
	with socketserver.TCPServer((HOST, PORT), MyHandler) as server:
		server.my_client = my_client
		print("Start to serve forever...")
		server.serve_forever()


def main(args):
	## load config and combine commandline arguments
	if args.config:
		config = load_config(args.config)
	else:
		config = blank_config()

	## some modifications
	if config['process']['interp'] is None:
		config['process']['interp'] = copy.deepcopy(config['sensor']['shape'])

	print_sensor(config)

	with Uclient(
		config['connection']['client_address'], 
		config['connection']['server_address'], 
		udp=config['connection']['udp'], 
		n=config['sensor']['shape']
	) as my_client:
		task_hololens_server(my_client)

		# # test
		# while True:
		# 	frame = my_client.fetch_frame()
		# 	data = classifier(frame)
		# 	print(data[:-1]+"\r", end="")
		# 	time.sleep(0.01)


if __name__ == '__main__':
	parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
	parser.add_argument('--config', dest='config', action='store', default=CONFIG_PATH, help="specify configuration file")
	args = parser.parse_args()

	main(args)
