import yaml
import copy
import pkgutil
import numpy as np


TEMPLATE_PATH = "blank_template.yaml"
data = pkgutil.get_data(__name__, TEMPLATE_PATH)
blank = yaml.safe_load(data)


def parse_ip_port(content):
	paras = content.split(":")
	ip = paras[0]
	port = None
	if len(paras) >= 2:
		port = int(paras[1])
	return (ip, port)


def check_shape(n):
	try:
		n[1]
		n = (n[0], n[1])
	except:
		try: 
			n[0]
			n = (n[0], n[0])
		except:
			n = (n, n)
	return n


def parse_mask(string_in):
	rows = string_in.splitlines()
	mask = [row.split() for row in rows]
	mask = np.array(mask, dtype=int)
	return mask


## recursion
def check_config(config):
	def recurse(dict_default, dict_target):
		for key in dict_default:
			if key in dict_target:
				if isinstance(dict_default[key], dict):
					recurse(dict_default[key], dict_target[key])
			else:
				dict_target[key] = copy.deepcopy(dict_default[key])
	## recurse to fill empty fields
	recurse(blank, config)
	## some transformation for certain fields
	if config['sensor']['shape'] is not None:
		config['sensor']['shape'] = check_shape(config['sensor']['shape'])
		config['sensor']['total'] = config['sensor']['shape'][0] * config['sensor']['shape'][1]
	if config['process']['interp'] is not None:
		config['process']['interp'] = check_shape(config['process']['interp'])
	if isinstance(config['sensor']['mask'], str):
		config['sensor']['mask'] = parse_mask(config['sensor']['mask'])
	if isinstance(config['connection']['server_address'], str):
		config['connection']['server_address'] = parse_ip_port(config['connection']['server_address'])
	if isinstance(config['connection']['client_address'], str):
		config['connection']['client_address'] = parse_ip_port(config['connection']['client_address'])


def load_config(filename):
	with open(filename) as fin:
		config = yaml.safe_load(fin)
	check_config(config)
	return config


def blank_config():
	return copy.deepcopy(blank)


if __name__ == '__main__':
	a = parse_ip_port("192.168.1.1:255")
	b = parse_ip_port("192.168.1.1")
	c = check_shape([3,4])
	print(a, b, c)
