#ifndef CONSTANTS_H
#define CONSTANTS_H

// return type
enum RET {
	RET_SUCCESS = 0,
	RET_FAILURE = -1,
	RET_TIMEOUT = -2,
	RET_EOF = 1,
};

// data source
enum SOURCE {
	SOURCE_SERIAL = 0,
	SOURCE_FILE,
};

enum FLAG { 
	FLAG_RUN = 0,
	FLAG_STOP,
	FLAG_RESTART,
};

enum FLAG_REC {
	FLAG_REC_STOP = 0,
	FLAG_REC_DATA,
	FLAG_REC_RAW,
};

enum FLAG_REC_RET {
	FLAG_REC_RET_SUCCESS = 0,
	FLAG_REC_RET_STOP,
	FLAG_REC_RET_FAIL = -1,
};

enum FILTER_SPATIAL {
	FILTER_S_NONE,  // no spatial filter
	FILTER_S_IDEAL,  // ideal kernel
	FILTER_S_BUTTERWORTH,  // butterworth kernel
	FILTER_S_GAUSSIAN,  // gaussian kernel
	TOTAL_S_FILTERS = 4
};

enum FILTER_TEMPORAL {
	FILTER_NONE,  // no temporal filter
	FILTER_EXP,  // exponential smoothing
	FILTER_MA,  // moving average
	FILTER_LP,  // sinc low pass filter
	TOTAL_FILTERS = 4  // total number of filters
};


#endif