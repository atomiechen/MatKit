from enum import IntEnum


class FLAG(IntEnum):
	FLAG_RUN = 0
	FLAG_STOP = 1
	FLAG_RESTART = 2


class FLAG_REC(IntEnum):
	FLAG_REC_STOP = 0
	FLAG_REC_DATA = 1
	FLAG_REC_RAW = 2


class FLAG_REC_RET(IntEnum):
	FLAG_REC_RET_SUCCESS = 0
	FLAG_REC_RET_STOP = 1
	FLAG_REC_RET_FAIL = -1
