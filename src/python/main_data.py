import argparse
import numpy as np

from toolkit.filemanager import parse_line, write_line
from toolkit.process import Processor

N = 16
INTERP = 16
FPS = 194
TH = 0.15


def main(args):
	filename = args.filename
	print(f"reading file: {filename}")
	if args.output:
		print("Data process mode:")

		my_processor = Processor(args.interp, noblob=args.noblob, threshold=args.threshold)
		my_processor.print_info()

		print(f"writing to file: {args.output}")
		## clear file content
		with open(args.output, 'w') as fout:
			pass
		cnt = 0
		with open(filename, 'r') as fin:
			for line in fin:
				data_parse, frame_idx, date_time = parse_line(line, args.n, ',')
				data_out = my_processor.transform(data_parse, reshape=True)
				data_str = [f"{item:.6f}" for item in data_out]
				timestamp = int(date_time.timestamp()*1000000)
				write_line(args.output, data_str, tags=[frame_idx, timestamp])
				cnt += 1
		print(f"output {cnt} lines to {args.output}")
	else:
		print("Data visualization mode")
		content = ([], [])
		with open(filename, 'r') as fin:
			for line in fin:
				data_parse, frame_idx, date_time = parse_line(line, args.n, ',')
				data_reshape = data_parse.reshape(args.n, args.n)
				content[0].append(np.array(data_reshape))
				content[1].append(f"frame idx: {frame_idx}  {date_time}")

		if args.matplot:
			from toolkit.visual.player_matplot import Player3DMatplot as Player
		else:
			from toolkit.visual.player_pyqtgraph import Player3DPyqtgraph as Player
		my_player = Player(zlim=3, widgets=True, N=args.n)
		my_player.run_interactive(dataset=content[0], infoset=content[1], fps=args.fps)


if __name__ == '__main__':
	description = "Visualize data, or process data via -o flag"
	parser = argparse.ArgumentParser(description=description, formatter_class=argparse.ArgumentDefaultsHelpFormatter)
	parser.add_argument('filename', action='store')
	parser.add_argument('-n', dest='n', action='store', default=N, type=int, help="sensor side size")
	parser.add_argument('-f', dest='fps', action='store', default=FPS, type=int, help="frames per second")
	parser.add_argument('-m', '--matplot', dest='matplot', action='store_true', default=False, help="use mathplotlib to plot")
	parser.add_argument('-o', dest='output', action='store', default=None, help="output processed data to file")
	parser.add_argument('--interp', dest='interp', action='store', default=INTERP, type=int, help="interpolated side size")
	parser.add_argument('--noblob', dest='noblob', action='store_true', default=False, help="do not filter out blob")
	parser.add_argument('--th', dest='threshold', action='store', default=TH, type=float, help="blob filter threshold")
	args = parser.parse_args()

	main(args)
