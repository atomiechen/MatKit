import os
import glob
import numpy as np
from datetime import datetime
from typing import Iterable


def check_root(filename):
	root_dir, bare_filename = os.path.split(filename)
	if root_dir and not os.path.exists(root_dir):
		# os.mkdir(root_dir)
		os.makedirs(root_dir)

def write_file(filename, content, override=False):
	check_root(filename)
	mode = 'w' if override else 'a'
	with open(filename, mode) as fout:
		fout.write(content)

def writelines_file(filename, lines, override=False):
	check_root(filename)
	mode = 'w' if override else 'a'
	with open(filename, mode) as fout:
		fout.writelines(lines)

def readlines_file(filename):
	with open(filename, 'r') as fin:
		ret = fin.readlines()
	return ret

def findall(pattern):
	return glob.glob(pattern)

def parse_line(line, N=16, delim=','):
	paras = line.strip().split(delim)
	data_parse = np.zeros(N*N)
	try:
		for i in range(N*N):
			data_parse[i] = float(paras[i])
	except:
		pass
	try:
		frame_idx = int(paras[N*N])
	except:
		frame_idx = -1
	try:
		time_stamp = int(paras[N*N+1]) / 1000000
		date_time = datetime.fromtimestamp(time_stamp)
	except:
		date_time = None
	return data_parse, frame_idx, date_time

def write_line(filename, data, tags=None, delim=',', override=False):
	items = [str(item) for item in data]
	if isinstance(tags, str):
		items.append(tags)
	elif isinstance(tags, Iterable):
		for tag in tags:
			items.append(str(tag))
	elif tags is not None:
		items.append(str(tags))
	content = delim.join(items) + "\n"
	write_file(filename, content, override=override)

def write_lines(filename, data, tags=None, delim=',', override=False):
	tags_list = []
	if isinstance(tags, str):
		tags_list.append(tags)
	elif isinstance(tags, Iterable):
		for tag in tags:
			tags_list.append(str(tag))
	elif tags is not None:
		tags_list.append(str(tags))
	lines = []
	for row in data:
		items = [str(item) for item in row]
		if tags_list:
			items += tags_list
		content = delim.join(items) + "\n"
		lines.append(content)
	writelines_file(filename, lines, override=override)

def clear_file(filename):
	check_root(filename)
	with open(filename, "w"):
		pass
