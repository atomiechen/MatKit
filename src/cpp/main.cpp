#include "header.h"

#include <exception>
#include <thread>
#include <signal.h>

#define N 16  // sensor side length
#define BAUDRATE 500000
#define TIMEOUT 1000  // in milliseconds (ms), must NOT be 0
#define SOCK_FILE "/var/tmp/unix.socket.server"  // UNIX domain socket address
#define DEFAULT_FILENAME "output.csv"

// shared variables
Serial * my_serial;
FileIterator * my_iterator;
unsigned int n = N;
atomic<int> flag(FLAG_RUN);
atomic<int> flag_record(FLAG_REC_STOP);
atomic<int> flag_ret(FLAG_REC_RET_STOP);
string my_socket = SOCK_FILE;
bool my_raw = false;
string my_filename = DEFAULT_FILENAME;

// local variables
unsigned int my_baudrate = BAUDRATE;
unsigned int my_timeout = TIMEOUT;
const vector<PortInfo> devices_found = list_ports();
string my_port = devices_found[devices_found.size()-1].port;  // the last port on the list
bool my_service = false;
thread my_thread;
vector<string> my_input;

int main(int argc, char const *argv[])
{
	if (parse_args(argc, argv) != 0) {
		return RET_FAILURE;
	}

	if (my_input.size() != 0) {
		return task_file();
	} else {
		return task_serial();
	}
}

void print_usage() {
	printf("READ and PROCESS Arduino pressure sensor data.\n");
	printf("\n");
	printf("Data flow:\n");
	printf("  Default: connect to Arduino board using serial port.\n");
	printf("  Alternative: read from && output to file.\n");
	printf("               input: positional argument\n");
	printf("               output: '-o' flag\n");
	printf("\n");
	printf("Usage options:\n");
	printf("  -h            print this help message and exit\n");
	printf("  -e            enumerate serial ports and exit\n");
	printf("  -r            raw data mode\n");
	printf("                  default: %s\n", my_raw? "true" : "false (processed)");
	printf("  -s            run service on demand using UNIX domain datagram socket\n");
	printf("                  default: %s\n", my_service? "true" : "false");
	printf("  -p PORT       sepcify port name\n");
	printf("                  default: last port i.e. %s\n", my_port.c_str());
	printf("  -b BAUDRATE   specify baudrate\n");
	printf("                  default: %u\n", my_baudrate);
	printf("  -t TIMEOUT    specify timeout in milliseconds\n");
	printf("                  default: %u\n", my_timeout);
	printf("  -n SIZE       specify side size of sensor\n");
	printf("                  default: %u\n", n);
	printf("  -u SOCK_FILE  specify UNIX domain socket address\n");
	printf("                  default: %s\n", my_socket.c_str());
	printf("  -o OUT_FILE   specify output filename\n");
	printf("                  default: %s\n", my_filename.c_str());
	printf("\n");
	printf("Options for processed data mode (i.e. ignored when raw data mode):\n");
	printf("  -i INIT_NUM   specify number of frames for initiating calibration\n");
	printf("                  0 for no calibration\n");
	printf("                  default: %d, value(integer) >= 0\n", my_INIT_CALI_FRAMES);
	printf("  -w WINDOW     specify window size for dynamic calibration,\n");
	printf("                  0 for static calibration\n");
	printf("                  default: %d, 0 <= value(integer) <= %d\n", my_WIN_SIZE, SAFE_WIN_SIZE);
	printf("  -fs S_FILTER  specify spatial filter window type to smooth data,\n");
	for (int i = 0; i < TOTAL_S_FILTERS; i++) {
	printf("                  %d: %s\n", i, filter_s_name(i).c_str());
	}
	printf("                  default: %d\n", my_filter_s);
	printf("  --d D0        specify spatial filter window radius D0\n");
	printf("                  default: %f, value(float) > 0\n", my_SF_D0);
	printf("\n");
	printf("  for %s:\n", NAME_BUTTERWORTH);
	printf("  --o ORDER     specify order of butterworth window\n");
	printf("                  default: %d, value(int) >= 1\n", my_BUTTER_ORDER);
	printf("\n");
	printf("  -ft T_FILTER  specify temporal filter to smooth data,\n");
	for (int i = 0; i < TOTAL_FILTERS; i++) {
	printf("                  %d: %s\n", i, filter_name(i).c_str());
	}
	printf("                  default: %d\n", my_filter);
	printf("\n");
	printf("  for %s:\n", NAME_EXP);
	printf("  --a ALPHA     specify ALPHA for first-order exponential smoothing\n");
	printf("                  default: %f, 0 <= value(float) <= 1\n", my_ALPHA);
	printf("  --b BETA      specify BETA for second-order exponential smoothing\n");
	printf("                  default: %f, 0 <= value(float) <= 1\n", my_BETA);
	printf("\n");
	printf("  for %s:\n", NAME_MA);
	printf("  --m MA_SIZE   specify kernel size for moving average\n");
	printf("                  default: %d, 0 < value(integer) <= %d\n", my_MA_SIZE, SAFE_FILTER_SIZE);
	printf("\n");
	printf("  for %s:\n", NAME_LP);
	printf("  --ls LP_SIZE  specify kernel size for sinc low-pass filter\n");
	printf("                  default: %d, 0 < value(integer) <= %d\n", my_LP_SIZE, SAFE_FILTER_SIZE);
	printf("  --lw LP_W      specify cut-off normalized frequency\n");
	printf("                  default: %f, value(float) > 0\n", my_LP_W);
	printf("\n");
	printf("Positional argument:\n");
	printf("  INPUT_FILE    (optional) specify input filename(s), reading each line\n");
	printf("                  as a raw frame and output processed data to OUT_FILE;\n");
	printf("                  multiple input files will be concatenated.\n");
	printf("                  NOTE: this argument will disable serial port reading\n");
	printf("\n");
}

void enumerate_ports() {
	printf("All serial ports:\n");
	char format[] = "%-30s\t%-15s\t%-15s\n";
	printf(format, "PORT", "DESCRIPTION", "HARDWARE-ID");
	for (PortInfo device : devices_found) {
		printf(format, device.port.c_str(), device.description.c_str(), device.hardware_id.c_str() );
	}
}

int parse_args(int argc, char const *argv[]) {
	// parse for -h and -e arguments
	for (int k = 1; k < argc; ++k) {
		if (argv[k][0] == '-') {
			switch(argv[k][1]) {
			case 'h':
				print_usage();
				exit(0);
			case 'e':
				enumerate_ports();
				exit(0);
			}
		}
	}
	// parse other arguments
	int ret = RET_SUCCESS;
	for (int k = 1; k < argc; ++k) {
		string full_arg = argv[k];
		if (argv[k][0] == '-') {
			if (full_arg == "-r") {
				my_raw = true;
			} else if (full_arg == "-s") {
				my_service = true;
			} else if (full_arg == "-p") {
				my_port = argv[++k];
			} else if (full_arg == "-u") {
				my_socket = argv[++k];
			} else if (full_arg == "-o") {
				my_filename = argv[++k];
			} else if (full_arg == "-b") {
				my_baudrate = atoi(argv[++k]);
			} else if (full_arg == "-t") {
				my_timeout = atoi(argv[++k]);
			} else if (full_arg == "-n") {
				n = atoi(argv[++k]);
			} else if (full_arg == "-i") {
				my_INIT_CALI_FRAMES = atoi(argv[++k]);
				if ((ret = check_initcali(my_INIT_CALI_FRAMES))) {
					printf("Invalid initiate number of frames!\n");
				}
			} else if (full_arg == "-w") {
				my_WIN_SIZE = atoi(argv[++k]);
				if ((ret = check_win(my_WIN_SIZE))) {
					printf("Invalid window size!\n");
				}
			} else if (full_arg == "-fs") {
				my_filter_s = atoi(argv[++k]);
				if ((ret = check_filter_s(my_filter_s))) {
					printf("Invalid spatial filter number!\n");
				}
			} else if (full_arg == "-ft") {
				my_filter = atoi(argv[++k]);
				if ((ret = check_filter(my_filter))) {
					printf("Invalid temporal filter number!\n");
				}
			}
			// filter arguments
			else if (full_arg == "--a") {
				my_ALPHA = atof(argv[++k]);
				if ((ret = check_alpha(my_ALPHA))) {
					printf("Invalid ALPHA for %s!\n", NAME_EXP);
				}
			} else if (full_arg == "--b") {
				my_BETA = atof(argv[++k]);
				if ((ret = check_beta(my_BETA))) {
					printf("Invalid BETA for %s!\n", NAME_EXP);
				}	
			} else if (full_arg == "--m") {
				my_MA_SIZE = atoi(argv[++k]);
				if ((ret = check_ma_size(my_MA_SIZE))) {
					printf("Invalid kernel size for %s!\n", NAME_MA);
				}	
			} else if (full_arg == "--ls") {
				my_LP_SIZE = atoi(argv[++k]);
				if ((ret = check_lp_size(my_LP_SIZE))) {
					printf("Invalid kernel size for %s!\n", NAME_LP);
				}
			} else if (full_arg == "--lw") {
				my_LP_W = atof(argv[++k]);
				if ((ret = check_lp_w(my_LP_W))) {
					printf("Invalid cut-off frequency for %s!\n", NAME_LP);
				}	
			} else if (full_arg == "--d") {
				my_SF_D0 = atof(argv[++k]);
				if ((ret = check_sf_d0(my_SF_D0))) {
					printf("Invalid D0 for spatial filter!\n");
				}
			} else if (full_arg == "--o") {
				my_BUTTER_ORDER = atoi(argv[++k]);
				if ((ret = check_butter_order(my_BUTTER_ORDER))) {
					printf("Invalid butterworth order!\n");
				}
			} else {
				ret = RET_FAILURE;
				printf("Unrecognized flag!\n");
			}
		} else {
			// positional arguments
			my_input.push_back(full_arg);
		}
	}
	return ret;
}

void print_serial() {
	printf("Serial info:\n");
	printf("  Sensor size: %u * %u\n", n, n);
	printf("  Port:        %s\n", my_serial->getPort().c_str());
	printf("  Baudrate:    %u\n", my_serial->getBaudrate());
	printf("  Timeout:     %u ms\n", my_serial->getTimeout().read_timeout_constant);
	printf("Service: %s\n", my_service? "Enabled" : "Disabled");
	if (my_service) {
	printf("  UNIX domain datagram socket\n");
	printf("  Address (file path): %s\n", my_socket.c_str());
	}
	printf("\n");
}

void my_handler(int sig) {
	flag = FLAG_STOP;
}

int task_serial() {
	my_serial = new Serial;

	// set port, baudrate, timeout to serial port object
	my_serial->setPort(my_port);
	my_serial->setBaudrate(my_baudrate);
	my_serial->setTimeout(Timeout::max(), my_timeout, 0, my_timeout, 0);

	print_serial();

	// open serial port
	bool error = false;
	try {
		my_serial->open();
	} catch (exception& e) {
		fprintf(stderr, "%s\n", e.what());
		error = true;
	} catch (...) {
		fprintf(stderr, "Unknown error!\n");
		error = true;
	}
	if (!my_serial->isOpen()) {
		fprintf(stderr, "Serial port %s not open!\n", my_port.c_str());
		error = true;
	}
	if (error) {
		delete my_serial;
		return RET_FAILURE;
	}

	// register interrupt signal
	struct sigaction sigIntHandler;
	sigIntHandler.sa_handler = my_handler;
	sigemptyset(&sigIntHandler.sa_mask);
	sigIntHandler.sa_flags = 0;
	sigaction(SIGINT, &sigIntHandler, NULL);

	// service in background
	if (my_service) {
		my_thread = thread(run_service);
	}

	// start running
	int ret = run(SOURCE_SERIAL);

	// stop service
	flag = FLAG_STOP;
	if (my_service) {
		my_thread.join();
	}

	// close serial port and exit
	my_serial->close();
	delete my_serial;
	return ret;
}

void print_file() {
	printf("File info:\n");
	printf("  Input:\n");
	for (const string& name : my_input) {
	printf("          %s\n", name.c_str());
	}
	printf("  Output:\n");
	printf("          %s\n", my_filename.c_str());
	printf("\n");
}

int task_file() {
	print_file();

	// prepare FileIterator
	my_iterator = new FileIterator(my_input);

	// register interrupt signal
	struct sigaction sigIntHandler;
	sigIntHandler.sa_handler = my_handler;
	sigemptyset(&sigIntHandler.sa_mask);
	sigIntHandler.sa_flags = 0;
	sigaction(SIGINT, &sigIntHandler, NULL);

	// start running
	run(SOURCE_FILE);

	delete my_iterator;
	return RET_SUCCESS;
}