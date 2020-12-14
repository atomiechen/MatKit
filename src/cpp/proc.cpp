#include "header.h"
#include <ctime>
#include <cmath>
#include <fftw3.h>

// transmission delimiter
#define DELIM 0xff
// calculate the reciprocal of resistance
#define calReciprocalResistance(v, v0, r0_reci) ((double)(r0_reci)*(double)(v)/((double)(v0)-(double)(v)))
// #define R0_RECI (100.0/51.0)  // reference reciprocal value of R0 (double), a constant to multiply output value
#define R0_RECI 1  // reference reciprocal value of R0 (double), a constant to multiply output value
#define V0 255  // reference value of working voltage (double)

// for processing data
#define FPS_CHECK_TIME 1000000  // microseconds of interval to check fps
#define INIT_CALI_FRAMES 200  // number of calibration frames
#define WIN_SIZE 10000  // window size
#define ALPHA 0.11  // for first-order exponential smoothing
#define BETA 0.0  // for second-order exponential smoothing
#define MA_SIZE 15  // moving average kernel size
#define LP_SIZE 15  // sinc kernel size
#define LP_W 0.04  // sinc cut-off normalized frequency
#define SF_D0 3.5  // spatial filter radius
#define BUTTER_ORDER 2  // butterworth filter order

#define WARM_UP 1000000  // microseconds

// get index of (row x, col y)
#define getIdx(x, y, n) ((x)*(n)+(y))
#define getRow(idx, n) ((idx)/(n))
#define getCol(idx, n) ((idx)%(n))
// get next index in a fix-size round array
#define getNextIndex(idx, size) (((idx)==(size)-1)? (0):(idx+1))

const double PI = acos(-1);  // math constant PI = 3.141592653589793

// shared variables
double * data_out;
double * data_raw;
int my_INIT_CALI_FRAMES = INIT_CALI_FRAMES;
int my_WIN_SIZE = WIN_SIZE;
int my_filter = FILTER_LP;
int my_filter_s = FILTER_S_GAUSSIAN;
double my_SF_D0 = SF_D0;
int my_BUTTER_ORDER = BUTTER_ORDER;
double my_ALPHA = ALPHA;
double my_BETA = BETA;
int my_MA_SIZE = MA_SIZE;
int my_LP_SIZE = LP_SIZE;
double my_LP_W = LP_W;

// local variables
unsigned int total;  // n * n
unsigned char * buf;
double * data_win;
double * data_zero;
double * data_ref;
double * data_ref2;
double * data_filter;
double * data_tmp;
double * kernel_lp;  // kernel of sinc low-pass filter
unsigned int frame_idx_out;  // output frame index
unsigned int win_frame_idx = 0;  // calibration window frame index
unsigned int filter_frame_idx = 0;  // filter history frame index
long long cur_time;  // current frame timestamp
long long start_time;  // start timestamp
unsigned int last_frame_idx;
long long last_time;
string tags;
fftw_complex * freq;  // fft frequency data
fftw_plan plan_forward;  // fftw plan
fftw_plan plan_backward;  // fftw plan
unsigned int cols;  // fft data column number
double * kernel_sf;  // kernel of spatial filter
// function pointers
int (*reset)() = reset_serial;
int (*get_raw_frame)() = get_raw_frame_serial;
int (*post_action)() = post_action_serial;
int (*final_action)() = final_action_serial;

void init() {
	total = n * n;
	cols = n / 2 + 1;

	buf = new unsigned char[total]();
	data_win = new double[total * SAFE_WIN_SIZE]();
	data_zero = new double[total]();
	data_out = new double[total+1]();
	data_raw = new double[total+1]();
	data_ref = new double[total]();
	data_ref2 = new double[total]();
	data_filter = new double[total * SAFE_FILTER_SIZE]();
	data_tmp = new double[total]();
	kernel_lp = new double[SAFE_FILTER_SIZE]();
	kernel_sf = new double[n*cols];

	freq = (fftw_complex*) fftw_malloc(n * cols * sizeof(fftw_complex));
	plan_forward = fftw_plan_dft_r2c_2d(n, n, data_tmp, freq, FFTW_MEASURE);
	plan_backward = fftw_plan_dft_c2r_2d(n, n, freq, data_tmp, FFTW_MEASURE);
}

void quit() {
	delete [] buf;
	delete [] data_win;
	delete [] data_zero;
	delete [] data_out;
	delete [] data_raw;
	delete [] data_ref;
	delete [] data_ref2;
	delete [] data_filter;
	delete [] data_tmp;
	delete [] kernel_lp;
	delete [] kernel_sf;

	fftw_destroy_plan(plan_forward);
	fftw_destroy_plan(plan_backward);
	fftw_free(freq);
}

int prepare_temporal() {
	int ret = RET_SUCCESS;

	if (my_filter == FILTER_NONE) {  // no temporal filter
		return ret;
	}

	printf("Initiating temporal filter...\n");
	filter_frame_idx = 0;  // for filter data
	// init to 0
	for (int i = 0; i < total * SAFE_FILTER_SIZE; i++) {
		data_filter[i] = 0;
	}

	int need_cache = 0;
	switch (my_filter) {
	case FILTER_NONE:
		need_cache = 0;
		break;
	case FILTER_EXP:
		// need_cache = log(0.05) / log(1 - my_ALPHA);
		need_cache = 0;
		if ((ret = get_raw_frame())) {
			break;
		}
		printf("Cache 1 frame to initiate exponential smoothing.\n");
		// init history smooth data
		for (int i = 0; i < total; i++) {
			data_ref[i] = data_tmp[i];  // first value
			data_ref2[i] = 0;  // zero
		}
		break;
	case FILTER_MA:
		need_cache = my_MA_SIZE - 1;
		break;
	case FILTER_LP:
		need_cache = my_LP_SIZE - 1;
		// calculate kernel values
		double sum = 0;
		sum += kernel_lp[0];
		for (int t = 0; t < my_LP_SIZE; t++) {
			double shifted = t - (double)(my_LP_SIZE-1) / 2;
			if (shifted == 0) {
				kernel_lp[t] = 2 * PI * my_LP_W;  // limit: t -> 0, sin(t)/t -> 1
			} else {
				kernel_lp[t] = sin(2 * PI * my_LP_W * shifted) / shifted;
			}
			sum += kernel_lp[t];
		}
		for (int t = 0; t < my_LP_SIZE; t++) {
			kernel_lp[t] /= sum;
		}
		break;
	}

	if (need_cache > 0) {
		printf("Cache %d frames for filter.\n", need_cache);
		// drop some frames to cache filter data
		int frame_cnt = 0;
		while (frame_cnt < need_cache) {
			if ((ret = get_raw_frame())) {
				break;
			}
			filter();
			frame_cnt++;
		}
	}
	return ret;
}

int prepare_cali() {
	int ret = RET_SUCCESS;

	if (my_INIT_CALI_FRAMES <= 0) {  // no calibration
		return ret;
	}

	printf("Initiating calibration...\n");
	printf("Cache %d frames for calibration.\n", my_INIT_CALI_FRAMES);
	win_frame_idx = 0;  // for dynamic calibrate
	// init to 0
	for (int i = 0; i < total; i++) {
		data_zero[i] = 0;
	}

	// accumulate data
	int frame_cnt = 0;
	while (frame_cnt < my_INIT_CALI_FRAMES) {
		if ((ret = get_raw_frame())) {
			return ret;
		}
		filter();

		for (int i = 0; i < total; i++) {
			data_zero[i] += data_tmp[i];
		}
		frame_cnt++;
	}
	// get average
	for (int i = 0; i < total; i++) {
		data_zero[i] /= (double)frame_cnt;
	}
	// calculate data_win
	for (int j = 0, t = 0; t < my_WIN_SIZE; t++) {
		for (int i = 0; i < total; i++, j++) {
			data_win[j] = data_zero[i];
		}
	}
	return ret;
}

int prepare_spatial() {
	if (my_filter_s == FILTER_S_NONE) {
		return RET_SUCCESS;
	}

	// use lambda expression for frequency-domain (k-space) window function
	double (*freq_window)(double) = nullptr;
	switch (my_filter_s) {
	case FILTER_S_IDEAL:
		freq_window = [](double distance) -> double {
			if (distance <= my_SF_D0) {
				return 1;
			} else {
				return 0;
			}
		};
		break;
	case FILTER_S_BUTTERWORTH:
		freq_window = [](double distance) -> double {
			return 1 / (1 + pow((distance / my_SF_D0), (2 * my_BUTTER_ORDER)));
		};
		break;
	case FILTER_S_GAUSSIAN:
		freq_window = [](double distance) -> double {
			return exp(- distance * distance / (2 * my_SF_D0 * my_SF_D0));
		};
		break;
	}

	// calculate frequency-domain (k-space) window values
	int t = 0;
	int row_divide = n / 2;
	int i = 0;
	for (; i <= row_divide; i++) {
		for (int j = 0; j < cols; j++) {
			double distance = hypot((double)i, (double)j);
			kernel_sf[t++] = freq_window(distance);
		}
	}
	for (; i < n; i++) {
		for (int j = 0; j < cols; j++) {
			double distance = hypot((double)(n-i), (double)j);
			kernel_sf[t++] = freq_window(distance);
		}
	}
	return RET_SUCCESS;
}

void spatial_filter() {
	if (my_filter_s == FILTER_S_NONE) {
		return;
	}

	fftw_execute(plan_forward);
	int t = 0;
	for (int i = 0; i < n; i++) {
		for (int j = 0; j < cols; j++) {
			freq[t][0] *= kernel_sf[t];
			freq[t][1] *= kernel_sf[t];
			t++;
		}
	}
	fftw_execute(plan_backward);
	for (int i = 0; i < total; i++) {
		data_tmp[i] /= total;
	}
}

void temporal_filter() {
	switch (my_filter) {
	case FILTER_NONE:  // do nothing
		break;
	case FILTER_EXP:
		for (int i = 0; i < total; i++) {
			double modified = data_tmp[i];
			// // first-order exponential smoothing
			// modified = my_ALPHA * modified + (1 - my_ALPHA) * data_ref[i];
			// data_ref[i] = modified;

			// second-order exponential smoothing
			modified = my_ALPHA * modified + (1 - my_ALPHA) * (data_ref[i] + data_ref2[i]);
			data_ref2[i] = my_BETA * (modified - data_ref[i]) + (1 - my_BETA) * data_ref2[i];
			data_ref[i] = modified;

			// output
			data_tmp[i] = modified;
		}
		break;
	case FILTER_MA:
		for (int i = 0; i < total; i++) {
			double modified = data_tmp[i];
			int start = i * (my_MA_SIZE-1);
			// longest point in data_filter is firstly visited
			for (int t = 1; t < my_MA_SIZE; t++) {
				modified += data_filter[start + filter_frame_idx];
				filter_frame_idx = getNextIndex(filter_frame_idx, my_MA_SIZE-1);
			}
			modified /= my_MA_SIZE;
			data_filter[start + filter_frame_idx] = data_tmp[i];
			// output
			data_tmp[i] = modified;
		}
		// update to next index
		filter_frame_idx = getNextIndex(filter_frame_idx, my_MA_SIZE-1);
		break;
	case FILTER_LP:
		for (int i = 0; i < total; i++) {
			// convolve
			double modified = data_tmp[i] * kernel_lp[0];
			int start = i * (my_LP_SIZE-1);
			// oldest point in data_filter is firstly visited
			for (int t = 1; t < my_LP_SIZE; t++) {
				modified += data_filter[start + filter_frame_idx] * kernel_lp[t];
				filter_frame_idx = getNextIndex(filter_frame_idx, my_LP_SIZE-1);
			}
			data_filter[start + filter_frame_idx] = data_tmp[i];
			// output
			data_tmp[i] = modified;
		}
		// update to next index
		filter_frame_idx = getNextIndex(filter_frame_idx, my_LP_SIZE-1);
		break;
	}
}

void filter() {
	spatial_filter();
	temporal_filter();
}

void calibrate() {
	if (my_INIT_CALI_FRAMES <= 0)
		return;

	int start = win_frame_idx * total;
	for (int i = 0, j = start; i < total; i++, j++) {
		double reci_resistance = data_tmp[i];
		// calibrate
		double modified = reci_resistance - data_zero[i];
		// the value should be positive
		data_tmp[i] = (modified > 0)? modified : 0;

		// adjust window if using dynamic window
		if (my_WIN_SIZE > 0) {
			// update data_zero (zero position) and data_win (history data)
			data_zero[i] += (reci_resistance - data_win[j]) / (double)my_WIN_SIZE;
			data_win[j] = reci_resistance;
		}
	}
	// update frame index if using dynamic window
	if (my_WIN_SIZE > 0) {
		win_frame_idx = getNextIndex(win_frame_idx, my_WIN_SIZE);
	}
}

void print_proc() {
	printf("Data mode: %s\n", my_raw? "raw" : "processed");
	if (!my_raw) {
	vector<pair<string, double>> arg_list;
	switch (my_filter) {
	case FILTER_EXP:
		arg_list.push_back(make_pair("ALPHA for 1st smoothing", my_ALPHA));
		arg_list.push_back(make_pair("BETA for 2nd smoothing ", my_BETA));
		break;
	case FILTER_MA:
		arg_list.push_back(make_pair("kernel size", my_MA_SIZE));
		break;
	case FILTER_LP:
		arg_list.push_back(make_pair("kernel size                ", my_LP_SIZE));
		arg_list.push_back(make_pair("cut-off normalized freqency", my_LP_W));
		break;
	}

	vector<pair<string, double>> arg_list_s;
	switch (my_filter_s) {
	case FILTER_S_BUTTERWORTH:
		arg_list_s.push_back(make_pair("order", my_BUTTER_ORDER));
	case FILTER_S_IDEAL:
	case FILTER_S_GAUSSIAN:
		arg_list_s.push_back(make_pair("cut-off freqency", my_SF_D0));
		break;
	}

	printf("  Spatial filter: %s\n", filter_s_name(my_filter_s).c_str());
	for (auto item : arg_list_s) {
	printf("    %s: %f\n", item.first.c_str(), item.second);
	}

	printf("  Temporal filter: %s\n", filter_name(my_filter).c_str());
	for (auto item : arg_list) {
	printf("    %s: %f\n", item.first.c_str(), item.second);
	}

	if (my_INIT_CALI_FRAMES == 0) {
	printf("  Calibration: No\n");
	} else {
	printf("  Calibration:\n");
	printf("    Initializing frames:     %d\n", my_INIT_CALI_FRAMES);
	if (my_WIN_SIZE == 0) {
	printf("    Static calibration\n");
	} else {
	printf("    Dynamic calibration\n");
	printf("    Calibration window size: %d\n", my_WIN_SIZE);
	}
	}
	}
	printf("\n");
}

int check_source(int source) {
	int ret = RET_SUCCESS;
	switch (source) {
	case SOURCE_SERIAL:
		reset = reset_serial;
		get_raw_frame = get_raw_frame_serial;
		post_action = post_action_serial;
		final_action = final_action_serial;
		if (!my_serial) {
			ret = RET_FAILURE;
		}
		break;
	case SOURCE_FILE:
		reset = reset_file;
		get_raw_frame = get_raw_frame_file;
		post_action = post_action_file;
		final_action = final_action_file;
		if (!my_iterator) {
			ret = RET_FAILURE;
		}
		break;
	default:
		ret = RET_FAILURE;
	}
	return ret;
}

void copy_paras() {
	my_INIT_CALI_FRAMES = para_initcali;
	my_WIN_SIZE = para_win;
	my_filter = para_filter;
	switch (my_filter) {
	case FILTER_EXP:
		my_ALPHA = para_alpha;
		my_BETA = para_beta;
		break;
	case FILTER_MA:
		my_MA_SIZE = para_ma_size;
		break;
	case FILTER_LP:
		my_LP_SIZE = para_lp_size;
		my_LP_W = para_lp_w;
		break;
	}
}

int run(int source) {
	int ret = RET_SUCCESS;
	// check data source
	if ((ret = check_source(source))) {
		return ret;
	}

	init();

	if (source == SOURCE_SERIAL) {
		// make CPU schedule more time for serial reading
		if ((ret = warm_up())) {
			return ret;
		}
	}

	do {
		start_time =  time_stamp();

		if ((ret = reset())) {
			break;
		}
		print_proc();

		if (!my_raw) {
			if ((ret = prepare_spatial())) {
				fprintf(stderr, "Failed to initiate spatial filter!\n");
				break;
			}
			if ((ret = prepare_temporal())) {
				fprintf(stderr, "Frames not enough for filter!\n");
				break;
			}
			if ((ret = prepare_cali())) {
				fprintf(stderr, "Frames not enough for calibration!\n");
				break;
			}
		}

		printf("Running...\n");
		while (flag == FLAG_RUN) {
			if ((ret = get_raw_frame())) {
				if (ret == RET_TIMEOUT) {
					continue;  // serial too slow, loop to try
				}
				break;  // invalid raw frame or EOF, exit
			}
			cur_time = time_stamp();  // record timestamp
			// process data
			if (!my_raw) {
				// smooth filter
				filter();
				// calibration: make values relative to zeros
				calibrate();
			}
			// output (if raw data mode, same as data_raw)
			for (int i = 0; i < total; i++) {
				data_out[i] = data_tmp[i];
			}
			if ((ret = post_action())) {
				break;  // error, exit
			}
		}

		if ((ret = final_action())) {
			break;
		}

		if (flag == FLAG_RESTART) {
			// copy parameters
			copy_paras();
			flag = FLAG_RUN;
			printf("\nRestarting the process...\n");
		} else {
			break;
		}
	} while (1);

	printf("Stopping...\n");
	quit();

	return ret;
}

int warm_up() {
	printf("Warming up processing...\n");
	int ret = RET_SUCCESS;
	long long begin_time = time_stamp();
	while (time_stamp() - begin_time < WARM_UP) {
		if ((ret = get_raw_frame())) {
			if (ret == RET_TIMEOUT) {
				ret = RET_SUCCESS;
			} else {
				break;
			}
		}
	}
	return ret;
}

int fetch_serial_frame() {
	int ret = RET_SUCCESS;
	try {
		int recv = -1;
		unsigned char tmp;
		while (1) {
			recv = my_serial->read(&tmp, 1);
			if (recv != 1) {
				ret = RET_TIMEOUT;
				return ret;
			}
			if (tmp == DELIM) {
				recv = my_serial->read(buf, total);
				if (recv != total) {
					ret = RET_TIMEOUT;
				}
				break;
			}
		}
	} catch (exception& e) {
		fprintf(stderr, "%s\n", e.what());
		ret = RET_FAILURE;
	} catch (...) {
		fprintf(stderr, "Unknown error!\n");
		ret = RET_FAILURE;
	}
	return ret;
}

int reset_serial() {
	// for output
	frame_idx_out = 0;
	// for fps checking
	last_frame_idx = frame_idx_out;
	last_time = start_time;
	return RET_SUCCESS;
}

int get_raw_frame_serial() {
	int ret = RET_SUCCESS;
	if ((ret = fetch_serial_frame())) {
		return ret;
	}
	frame_idx_out++;

	// calculate reciprocal resistance
	for (int i = 0; i < total; i++) {
		data_raw[i] = calReciprocalResistance(buf[i], V0, R0_RECI);
		data_tmp[i] = data_raw[i];
	}
	*(int*)&data_raw[total] = frame_idx_out;  // for service

	return ret;
}

int post_action_serial() {
	*(int*)&data_out[total] = frame_idx_out;  // for service

	// check if to write to file
	double * data_ptr = nullptr;
	if (flag_record == FLAG_REC_DATA) {
		data_ptr = data_out;
	} else if (flag_record == FLAG_REC_RAW) {
		data_ptr = data_raw;
	}
	// write to file
	if (data_ptr) {
		tags = to_string(frame_idx_out) + "," + to_string(cur_time);
		flag_ret = write_frame(my_filename, data_ptr, total, tags);
		if (flag_ret != FLAG_REC_RET_SUCCESS) {  // fail to write, stop recording
			flag_record = FLAG_REC_STOP;
		}
	}
	// check fps
	if (cur_time - last_time >= FPS_CHECK_TIME) {
		double duration = (cur_time - last_time) / (double)1000000;
		double run_duration = (cur_time - start_time)/(double)1000000;
		int frames = frame_idx_out - last_frame_idx;
		printf("  frame rate: %f fps  running time: %f s\n", (double)frames / duration, run_duration);
		last_frame_idx = frame_idx_out;
		last_time = cur_time;
	}
	return RET_SUCCESS;
}

int final_action_serial() {
	return RET_SUCCESS;
}

int reset_file() {
	// clear file
	FILE* fp = fopen(my_filename.c_str(), "w");
	if (!fp) {
		fprintf(stderr, "File write failed: %s\n", my_filename.c_str());
		return RET_FAILURE;
	}
	fclose(fp);
	frame_idx_out = 0;  // for counting frames
	return RET_SUCCESS;
}

int get_raw_frame_file() {
	int ret = RET_SUCCESS;
	ret = my_iterator->put_frame(data_tmp, total, &tags);
	return ret;
}

int post_action_file() {
	int ret = RET_SUCCESS;
	// write to file
	flag_ret = write_frame(my_filename, data_out, total, tags);
	if (flag_ret != FLAG_REC_RET_SUCCESS) {  // fail to write, stop recording
		ret = RET_FAILURE;
	}
	frame_idx_out++;
	return ret;
}

int final_action_file() {
	printf("* Output file: %s\n", my_filename.c_str());
	printf("* Total output frames: %d\n", frame_idx_out);
	return RET_SUCCESS;
}

int write_line(const string& filename, const string& line) {
	FILE* fp = fopen(filename.c_str(), "a");
	if (!fp) {
		fprintf(stderr, "File append failed: %s\n", filename.c_str());
		return FLAG_REC_RET_FAIL;
	}
	fputs(line.c_str(), fp);
	fputc('\n', fp);
	fclose(fp);
	return FLAG_REC_RET_SUCCESS;
}

int write_frame(const string& filename, const double* data, int size, const string& tags, char delim) {
	string line = "";
	for (int i = 0; i < size; i++) {
		line += to_string(data[i]);
		line += delim;
	}
	line += tags;
	return write_line(filename, line);
}
