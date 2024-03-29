class CustomException(Exception):
	pass


class SerialTimeout(CustomException):
	def __init__(self):
		msg = "Serial connection timeout!"
		super().__init__(msg)


if __name__ == '__main__':
	raise SerialTimeout
