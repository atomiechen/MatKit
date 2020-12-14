#ifndef FILE_ITERATOR_HPP
#define FILE_ITERATOR_HPP

#include "constants.h"

class FileIterator {
	const vector<string>& my_input;
	int file_idx = 0;
	ifstream * fin = nullptr;
	bool available = false;

public:
	string line = "";

	FileIterator(const vector<string>& input): 
		my_input(input), file_idx(0) {}

	~FileIterator() {
		delete fin;
	}

	int open_file(string filename) {
		fin = new ifstream(filename);
		if (!fin->is_open()) {
			fprintf(stderr, "File not open: %s\n", filename.c_str());
			return RET_FAILURE;
		}
		return RET_SUCCESS;
	}

	int put_line(string * line) {
		int ret = RET_SUCCESS;

		if (!fin && file_idx < my_input.size()) {
			if ((ret = open_file(my_input[file_idx++]))) {
				return ret;
			}
		}

		while (!getline(*fin, *line)) {  // extracted \n
			// end of file
			fin->close();
			if (file_idx == my_input.size()) {
				// no more file, no more data
				ret = RET_EOF;
				break;
			} else {
				//  fetch next file
				if ((ret = open_file(my_input[file_idx++]))) {
					break;
				}
			}
		}

		return ret;
	}

	int put_frame(double * data, int size, string * tags = nullptr) {
		int ret = RET_SUCCESS;
		if ((ret = put_line(&line))) {
			return ret;
		}
		ret = parse_line(line, data, size, tags);
		return ret;
	}

	int parse_line(const string& line, double * data, int size, string * tags = nullptr) {
		int ret = RET_SUCCESS;
		if (size > 0) {
			const char * ptr = line.c_str();
			data[0] = atof(ptr);
			for (int i = 1; i < size; i++) {
				while (1) {
					ptr++;
					if (*ptr == ',') {
						data[i] = atof(++ptr);
						break;
					}
				}
			}
			if (tags) {
				while (*(++ptr) != ',');
				if (*(ptr++) == ',') {
					*tags = line.substr(ptr-line.c_str());
				}
			}
		} else {
			ret = RET_FAILURE;
		}
		return ret;
	}
};

#endif