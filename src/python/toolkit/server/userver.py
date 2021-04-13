from socket import (
	socket, AF_INET, SOCK_DGRAM, timeout, 
	SOL_SOCKET, SO_REUSEADDR, SO_SNDBUF,
	gethostname, gethostbyname
)
try:
	from socket import AF_UNIX
	support_unix_socket = True
except ImportError:
	support_unix_socket = False
from enum import IntEnum
from struct import calcsize, pack, unpack, unpack_from
from os import unlink

from queue import Empty

from .flag import FLAG, FLAG_REC


class CMD(IntEnum):

	"""Pre-defined commands for communication with server.
	
	Attributes:
		CLOSE (int): Close the server
		DATA (int): get processed data frame and frame index
		RAW (int): get raw data frame and frame index
		REC_DATA (int): record processed data to file
		REC_RAW (int): record raw data to file
		REC_STOP (int): stop recording
		RESTART (int): restart the server with processing parameters
		PARAS (int): get current processing parameters of the server
		REC_BREAK (int): stop current recording and start a new one
	"""
	
	CLOSE = 0
	DATA = 1
	RAW = 2
	REC_DATA = 3
	REC_RAW = 4
	REC_STOP = 5
	RESTART = 6
	PARAS = 7
	REC_BREAK = 8

class Userver:

	UDP = False
	## UNIX domain socket address
	SERVER_FILE = "/var/tmp/unix.socket.server"
	## UDP socket address
	hostname = gethostname()

	try:
		ip_address = gethostbyname(hostname)
	except:
		ip_address = gethostbyname("localhost")

	# SERVER_HOST = "localhost"
	SERVER_HOST = ip_address
	SERVER_PORT = 25530
	SERVER_IPADDR = (SERVER_HOST, SERVER_PORT)

	TOTAL = 16 * 16
	TIMEOUT = 0.1
	BUF_SIZE = 2048
	REC_ID = 0

	def __init__(self, data_out, data_raw, idx_out, server_addr=None, **kwargs):
		## for multiprocessing communication
		self.queue_to_main_client = None

		self.config(**kwargs)
		self.data_out = data_out
		self.data_raw = data_raw
		self.idx_out = idx_out
		self.server_addr = server_addr

		self.binded = False
		self.frame_format = f"={self.TOTAL}di"
		self.frame_size = calcsize(self.frame_format)

		self.init_socket()

	def config(self, *, total=None, udp=None, timeout=None, queue_to_main_client=None):
		if total:
			self.TOTAL = total
		if udp is not None:
			self.UDP = udp
		if timeout:
			self.TIMEOUT = timeout
		if queue_to_main_client:
			self.queue_to_main_client = queue_to_main_client

	def init_socket(self):
		if not support_unix_socket:
			## this machine does not support UNIX domain socket
			self.UDP = True
		if self.UDP:
			## UDP socket
			self.my_socket = socket(AF_INET, SOCK_DGRAM)
		else:
			## UNIX domain socket
			self.my_socket = socket(AF_UNIX, SOCK_DGRAM)

		self.my_socket.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)
		self.my_socket.setsockopt(SOL_SOCKET, SO_SNDBUF, self.frame_size*2)
		self.my_socket.settimeout(self.TIMEOUT)

		if not self.server_addr:
			if self.UDP:
				self.server_addr = self.SERVER_IPADDR
			else:
				self.server_addr = self.SERVER_FILE
		else:
			## check if ip-port address needs to be filled
			if self.UDP:
				tmp_addr = list(self.server_addr)
				if tmp_addr[0] is None:
					tmp_addr[0] = self.SERVER_HOST
				if tmp_addr[1] is None:
					tmp_addr[1] = self.SERVER_PORT
				self.server_addr = tuple(tmp_addr)
		self.my_socket.bind(self.server_addr)
		self.binded = True

	def exit(self):
		self.my_socket.close()
		if self.binded and not self.UDP:
			unlink(self.server_addr)
		print("Service stopped.")

	def __enter__(self):
		return self

	def __exit__(self, type, value, traceback):
		self.exit()

	def proc_cmd(self):
		if self.data[0] == CMD.CLOSE:
			reply = pack("=B", 0)
			self.my_socket.sendto(reply, self.client_addr)
			self.queue_to_main_client.put(FLAG_REC.FLAG_REC_STOP)
			return CMD.CLOSE
		elif self.data[0] == CMD.DATA:
			reply = pack(self.frame_format, *(self.data_out), self.idx_out.value)
			self.my_socket.sendto(reply, self.client_addr)
		elif self.data[0] == CMD.RAW:
			reply = pack(self.frame_format, *(self.data_raw), self.idx_out.value)
			self.my_socket.sendto(reply, self.client_addr)
		## TODO
		elif self.data[0] == CMD.REC_DATA:
			reply = pack("=B", 0) + self.data[1:]
			self.my_socket.sendto(reply, self.client_addr)
			self.queue_to_main_client.put(FLAG_REC.FLAG_REC_DATA)
			self.queue_to_main_client.put(str(self.data[1:], encoding = "utf-8"))
		elif self.data[0] == CMD.REC_RAW:
			reply = pack("=B", 0) + self.data[1:]
			self.my_socket.sendto(reply, self.client_addr)
			self.queue_to_main_client.put(FLAG_REC.FLAG_REC_RAW)
			self.queue_to_main_client.put(str(self.data[1:], encoding = "utf-8"))
		elif self.data[0] == CMD.REC_STOP:
			reply = pack("=B", 0)
			self.my_socket.sendto(reply, self.client_addr)
			self.queue_to_main_client.put(FLAG_REC.FLAG_REC_STOP)
		elif self.data[0] == CMD.RESTART:
			pass
		elif self.data[0] == CMD.PARAS:
			pass
		elif self.data[0] == CMD.REC_BREAK:
			reply = pack("=B", 0)
			self.my_socket.sendto(reply, self.client_addr)
			self.queue_to_main_client.put_nowait(FLAG_REC.FLAG_REC_BREAK)

	def print_service(self):
		if self.UDP:
			protocol_str = 'UDP'
		else:
			protocol_str = 'UNIX domain datagram'
		print(f"Service protocol: {protocol_str}")
		print(f"  - Server address: {self.server_addr}")

	def run_service(self):
		self.print_service()
		print(f"Running service...")
		while True:
			## check signals from the other process
			if self.queue_to_main_client is not None:
				try:
					flag = self.queue_to_main_client.get_nowait()
					if flag == FLAG.FLAG_STOP:
						break
				except Empty:
					pass

			## try to receive requests from client(s)
			try:
				self.data, self.client_addr = self.my_socket.recvfrom(self.BUF_SIZE)
				ret = self.proc_cmd()
				if ret == CMD.CLOSE:
					self.queue_to_main_client.put(FLAG.FLAG_STOP)
					break
			except timeout:
				pass
			except (FileNotFoundError, ConnectionResetError):
				print("client off-line")

