#include "header.h"

#include <sys/socket.h>
#include <unistd.h>
#include <sys/un.h>

extern int errno;

#define BUF_SIZE 2048

// shared variables
int para_initcali;
int para_win;
int para_filter;
double para_alpha;
double para_beta;
int para_ma_size;
int para_lp_size;
double para_lp_w;

// local variables
enum CMD { CLOSE = 0, DATA, RAW, REC_DATA, REC_RAW, REC_STOP, RESTART, PARAS };

int run_service() {
	// return value
	int ret = RET_SUCCESS;
	// frame size in bytes
	unsigned int frame_size = n*n * sizeof(double)+sizeof(int);
	// socket address
	struct sockaddr_un serverAddr;
	// for socket configure
	int on;
	struct timeval read_timeout;
	// for running service
	struct sockaddr_un from;
	socklen_t fromlen = sizeof(from);
	char buf[BUF_SIZE], buf_send[BUF_SIZE];
	int size;
	string tmp_filename;
	int pos;
	int error_cnt;

	int serverSocket = socket(AF_UNIX, SOCK_DGRAM, 0);
	if (serverSocket < 0) {
		fprintf(stderr, "Socket creation failed!\n");
		ret = RET_FAILURE;
		goto EXIT;
	}

	// configure protocol and address
	memset(&serverAddr, 0, sizeof(serverAddr));
	serverAddr.sun_family = AF_UNIX;
	strcpy(serverAddr.sun_path, my_socket.c_str());

	// configure address reuse
	if ((on = setsockopt(serverSocket, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on)))) {
		fprintf(stderr, "setsockopt SO_REUSEADDR error!\n");
		ret = RET_FAILURE;
		goto SOCK_ERROR;
	}
	// configure datagram max size
	on = 2 * frame_size;
	if (setsockopt(serverSocket, SOL_SOCKET, SO_SNDBUF, &on, sizeof(on))) {
		fprintf(stderr, "setsockopt SO_SNDBUF error!\n");
		ret = RET_FAILURE;
		goto SOCK_ERROR;
	}
	// configure non-blocking recvfrom using timeout = 1000 us = 1 ms
	read_timeout.tv_sec = 0;
	read_timeout.tv_usec = 1000;
	if (setsockopt(serverSocket, SOL_SOCKET, SO_RCVTIMEO, &read_timeout, sizeof(read_timeout))) {
		fprintf(stderr, "setsockopt SO_RCVTIMEO error!\n");
		ret = RET_FAILURE;
		goto SOCK_ERROR;
	}

	// bind address
	if (::bind(serverSocket, (struct sockaddr *)&serverAddr, sizeof(serverAddr))) {
		fprintf(stderr, "Bind error!\n");
		ret = RET_FAILURE;
		goto SOCK_ERROR;
	}

	try {
		printf("Running service...\n");
		while (flag != FLAG_STOP) {
			size = recvfrom(serverSocket, buf, BUF_SIZE, 0, (sockaddr*)&from, &fromlen);
			if (size <= 0) {  // error data or invalid size
				continue;
			}
			from.sun_path[fromlen] = '\0';
			buf[size] = '\0';

			switch (buf[0]) {
			case CLOSE:
				buf_send[0] = 0;
				size = sendto(serverSocket, buf_send, 1, 0, (sockaddr*)&from, fromlen);
				// need to break the while loop afterwards
				break;
			case DATA:
				size = sendto(serverSocket, data_out, frame_size, 0, (sockaddr*)&from, fromlen);
				break;
			case RAW:
				size = sendto(serverSocket, data_raw, frame_size, 0, (sockaddr*)&from, fromlen);
				break;
			case REC_DATA:
			case REC_RAW:
				tmp_filename = &buf[1];
				if (tmp_filename != "") {
					my_filename = tmp_filename;
				}
				strcpy(&buf_send[1], my_filename.c_str());
				if (buf[0] == REC_DATA) {
					flag_record = FLAG_REC_DATA;
				} else {
					flag_record = FLAG_REC_RAW;
				}
				while (flag_ret == FLAG_REC_RET_STOP);  // wait for writing operation
				if (flag_ret == FLAG_REC_RET_SUCCESS) {  // sucess
					buf_send[0] = 0;
					size = sendto(serverSocket, buf_send, my_filename.size()+2, 0, (sockaddr*)&from, fromlen);
				} else {  // fail
					flag_record = FLAG_REC_STOP;
					flag_ret = FLAG_REC_RET_STOP;
					buf_send[0] = -1;
					size = sendto(serverSocket, buf_send, my_filename.size()+2, 0, (sockaddr*)&from, fromlen);
				}
				break;
			case REC_STOP:
				flag_record = FLAG_REC_STOP;
				flag_ret = FLAG_REC_RET_STOP;
				buf_send[0] = 0;
				size = sendto(serverSocket, buf_send, 1, 0, (sockaddr*)&from, fromlen);
				break;
			case RESTART:
				if (size >= sizeof(char) + 3*sizeof(int)) {
					// parse arguments
					error_cnt = 0;
					pos = sizeof(char);
					para_initcali = *(int*)(&buf[pos]);
					para_initcali = (para_initcali == -1)? my_INIT_CALI_FRAMES : para_initcali;
					error_cnt += check_initcali(para_initcali);
					pos += sizeof(int);
					para_win = *(int*)(&buf[pos]);
					para_win = (para_win == -1)? my_WIN_SIZE : para_win;
					error_cnt += check_win(para_win);
					pos += sizeof(int);
					para_filter = *(int*)(&buf[pos]);
					para_filter = (para_filter == -1)? my_filter : para_filter;
					error_cnt += check_filter(para_filter);
					pos += sizeof(int);
					switch (para_filter) {
					case FILTER_EXP:
						if (size - pos >= 2 * sizeof(double)) {
							para_alpha = *(double*)(&buf[pos]);
							para_alpha = (para_alpha == -1)? my_ALPHA : para_alpha;
							error_cnt += check_alpha(para_alpha);
							pos += sizeof(double);
							para_beta = *(double*)(&buf[pos]);
							para_beta = (para_beta == -1)? my_BETA : para_beta;
							error_cnt += check_beta(para_beta);
							pos += sizeof(double);
						} else {
							para_alpha = my_ALPHA;
							para_beta = my_BETA;
						}
						break;
					case FILTER_MA:
						if (size - pos >= sizeof(int)) {
							para_ma_size = *(int*)(&buf[pos]);
							para_ma_size = (para_ma_size == -1)? my_MA_SIZE : para_ma_size;
							error_cnt += check_ma_size(para_ma_size);
							pos += sizeof(int);
						} else {
							para_ma_size = my_MA_SIZE;
						}
						break;
					case FILTER_LP:
						if (size - pos >= sizeof(int) + sizeof(double)) {
							para_lp_size = *(int*)(&buf[pos]);
							para_lp_size = (para_lp_size == -1)? my_LP_SIZE : para_lp_size;
							error_cnt += check_lp_size(para_lp_size);
							pos += sizeof(int);
							para_lp_w = *(double*)(&buf[pos]);
							para_lp_w = (para_lp_w == -1)? my_LP_W : para_lp_w;
							error_cnt += check_lp_w(para_lp_w);
							pos += sizeof(double);
						} else {
							para_lp_size = my_LP_SIZE;
							para_lp_w = my_LP_W;
						}
						break;
					}
					if (error_cnt != 0) {
						buf_send[0] = -1;
					} else {
						// signal run() to restart
						flag = FLAG_RESTART;
						while (flag == FLAG_RESTART);
						buf_send[0] = 0;
					}
				} else {  // wrong request format
					buf_send[0] = -1;
				}
			case PARAS:
				if (buf[0] == PARAS)
					buf_send[0] = 0;
				pos = sizeof(char);
				*(int*)(&buf_send[pos]) = my_INIT_CALI_FRAMES;
				pos += sizeof(int);
				*(int*)(&buf_send[pos]) = my_WIN_SIZE;
				pos += sizeof(int);
				*(int*)(&buf_send[pos]) = my_filter;
				pos += sizeof(int);
				switch (my_filter) {
				case FILTER_EXP:
					*(double*)(&buf_send[pos]) = my_ALPHA;
					pos += sizeof(double);
					*(double*)(&buf_send[pos]) = my_BETA;
					pos += sizeof(double);
					break;
				case FILTER_MA:
					*(int*)(&buf_send[pos]) = my_MA_SIZE;
					pos += sizeof(int);
					break;
				case FILTER_LP:
					*(int*)(&buf_send[pos]) = my_LP_SIZE;
					pos += sizeof(int);
					*(double*)(&buf_send[pos]) = my_LP_W;
					pos += sizeof(double);
					break;
				}
				size = sendto(serverSocket, buf_send, pos, 0, (sockaddr*)&from, fromlen);
				break;
			}
			if (size == -1) {
				printf("errno: %d  request: %s client: %s\n", errno, buf, from.sun_path);
			}
			if (buf[0] == CLOSE) {
				break;
			}
		}
	} catch (exception& e) {
		fprintf(stderr, "%s\n", e.what());
	} catch (...) {
		fprintf(stderr, "Unknown error!\n");
	}

END:
	unlink(my_socket.c_str());
SOCK_ERROR:
	close(serverSocket);
EXIT:
	flag = FLAG_STOP;
	printf("Stopping service...\n");
	return ret;
}