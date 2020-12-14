def parse_ip_port(content):
	paras = content.split(":")
	ip = paras[0]
	port = None
	if len(paras) >= 2:
		port = int(paras[1])
	return (ip, port)


if __name__ == '__main__':
	a = parse_ip_port("192.168.1.1:255")
	b = parse_ip_port("192.168.1.1")
	c = parse_ip_port("")
	print(a, b, c)
