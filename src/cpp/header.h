#ifndef HEADER_H
#define HEADER_H

#include <cstdio>
#include <atomic>
#include <fstream>
using namespace std;

#include <sys/time.h>

#include <serial/serial.h>
using namespace serial;

#include "constants.h"
#include "FileIterator.hpp"

// for restarting service
#define SAFE_WIN_SIZE 10000  // max win size, upper bound of my_WIN_SIZE
// filters
#define SAFE_FILTER_SIZE 100  // max filter kernel size
#define NAME_NONE "None"
#define NAME_EXP "Exponential Smoothing"
#define NAME_MA "Moving Average"
#define NAME_LP "SINC Low-Pass filter"
#define NAME_IDEAL "Ideal"
#define NAME_BUTTERWORTH "Butterworth"
#define NAME_GAUSSIAN "Gaussian"

// shared variables
extern atomic<int> flag;
extern atomic<int> flag_record;
extern atomic<int> flag_ret;

extern int my_filter_s;
extern int my_filter;
extern Serial * my_serial;
extern FileIterator * my_iterator;

extern unsigned int n;
extern bool my_raw;
extern double * data_out;
extern double * data_raw;
extern string my_socket;
extern string my_filename;
extern int my_INIT_CALI_FRAMES;
extern int my_WIN_SIZE;
extern double my_ALPHA;
extern double my_BETA;
extern int my_MA_SIZE;
extern int my_LP_SIZE;
extern double my_LP_W;
extern double my_SF_D0;
extern int my_BUTTER_ORDER;

extern int para_initcali;
extern int para_win;
extern int para_filter;
extern double para_alpha;
extern double para_beta;
extern int para_ma_size;
extern int para_lp_size;
extern double para_lp_w;

// main.cpp
void print_usage();
void enumerate_ports();
int parse_args(int argc, char const *argv[]);
void print_serial();
void my_handler(int sig);
int task_serial();
int task_file();

// proc.cpp
void init();
void quit();
int prepare_temporal();
int prepare_spatial();
int prepare_cali();
void temporal_filter();
void spatial_filter();
void filter();
void calibrate();
int check_source(int source);
void copy_paras();
int run(int source);
void print_proc();
int write_line(const string& filename, const string& line);
int write_frame(const string& filename, const double* data, int size, const string& tags, char delim=',');
int fetch_serial_frame();
int reset_serial();
int get_raw_frame_serial();
int post_action_serial();
int final_action_serial();
int reset_file();
int get_raw_frame_file();
int post_action_file();
int final_action_file();
int warm_up();

// service.cpp
int run_service();

inline long long time_stamp() {
	struct timeval query_time;
	gettimeofday(&query_time, nullptr);
	return query_time.tv_sec * 1000000 + query_time.tv_usec;
}

inline string filter_name(int filter) {
	switch (filter) {
	case FILTER_NONE:
		return NAME_NONE;
	case FILTER_EXP:
		return NAME_EXP;
	case FILTER_MA:
		return NAME_MA;
	case FILTER_LP:
		return NAME_LP;
	default:
		return "";
	}
}

inline string filter_s_name(int filter_s) {
	switch (filter_s) {
	case FILTER_S_NONE:
		return NAME_NONE;
	case FILTER_S_IDEAL:
		return NAME_IDEAL;
	case FILTER_S_BUTTERWORTH:
		return NAME_BUTTERWORTH;
	case FILTER_S_GAUSSIAN:
		return NAME_GAUSSIAN;
	default:
		return "";
	}
}

inline int check_initcali(int para) {
	if (para >= 0) {
		return RET_SUCCESS;
	} else {
		return RET_FAILURE;
	}
}

inline int check_win(int para) {
	if (para >= 0 && para <= SAFE_WIN_SIZE) {
		return RET_SUCCESS;
	} else {
		return RET_FAILURE;
	}
}

inline int check_filter(int para) {
	if (para >= 0 && para < TOTAL_FILTERS) {
		return RET_SUCCESS;
	} else {
		return RET_FAILURE;
	}
}

inline int check_alpha(double para) {
	if (para >= 0 && para <= 1) {
		return RET_SUCCESS;
	} else {
		return RET_FAILURE;
	}
}

inline int check_beta(double para) {
	return check_alpha(para);
}

inline int check_ma_size(int para) {
	if (para > 0 && para <= SAFE_FILTER_SIZE) {
		return RET_SUCCESS;
	} else {
		return RET_FAILURE;
	}
}

inline int check_lp_size(int para) {
	return check_ma_size(para);
}

inline int check_lp_w(double para) {
	if (para > 0) {
		return RET_SUCCESS;
	} else {
		return RET_FAILURE;
	}
}

inline int check_filter_s(int para) {
	if (para >= 0 && para < TOTAL_S_FILTERS) {
		return RET_SUCCESS;
	} else {
		return RET_FAILURE;
	}
}

inline int check_sf_d0(double para) {
	if (para > 0) {
		return RET_SUCCESS;
	} else {
		return RET_FAILURE;
	}
}

inline int check_butter_order(int para) {
	if (para >= 1) {
		return RET_SUCCESS;
	} else {
		return RET_FAILURE;
	}
}
#endif