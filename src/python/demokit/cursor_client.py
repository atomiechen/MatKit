from socket import socket, AF_INET, SOCK_STREAM, timeout
import time


def auto_reconnect(wait_time=1):
	def decorator(func):
		def wrapper(self, *args, **kwargs):
			try:
				return func(self, *args, **kwargs)
			except Exception as e:
				print(f"Remote server no respond. Reconnecting...")

			while True:
				try:
					self.connect()
					return
				except Exception as e:
					print(f"Sleep {wait_time}s to retry...")
					time.sleep(wait_time)
		return wrapper
	return decorator


class CursorClient:

	@auto_reconnect()
	def __init__(self, server_addr, port, timeout=1):

		self.server_addr = server_addr
		self.port = port
		self.timeout = timeout

		self.connect()

	def __enter__(self):
		return self

	def __exit__(self, type, value, traceback):
		self.close()

	def connect(self):
		self.my_socket = socket(AF_INET, SOCK_STREAM)
		self.my_socket.settimeout(self.timeout)
		self.my_socket.connect((self.server_addr, self.port))
		print(f"client connecting to server: {self.server_addr}:{self.port}")

	def close(self):
		self.my_socket.close()
		print("remote client socket closed")

	@auto_reconnect()
	def send(self, touch_state, x, y):
		paras = [touch_state, x, y]
		self.my_socket.send(str(" ".join([str(item) for item in paras]) + "\n").encode())
	
	@auto_reconnect()
	def sendButton(self, cmd):
		self.my_socket.send((cmd+"\n").encode())
	
	@auto_reconnect()
	def sendPressure(self, pressure):
		self.my_socket.send((str(pressure)+"\n").encode())


if __name__ == '__main__':
	IP = "localhost"
	PORT = 8081

	with CursorClient(IP, PORT) as my_remote_handle:
		print("sending...")
		my_remote_handle.send(1, 0.6, 0.75)
		time.sleep(1)
		my_remote_handle.send(2, 0.25, 0.58)
		time.sleep(1)
		my_remote_handle.send(2, 0.9, 0.75)
		time.sleep(1)
		my_remote_handle.send(3, 0.85, 0.58)

		print("sending buttons...")
		my_remote_handle.sendButton('up')
		time.sleep(1)
		my_remote_handle.sendButton('right')
		time.sleep(1)
		my_remote_handle.sendButton('click')
		time.sleep(1)
		my_remote_handle.sendButton('down')
		time.sleep(1)
		my_remote_handle.sendButton('left')
		time.sleep(1)
