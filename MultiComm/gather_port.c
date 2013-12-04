/*
 * gather.c
 *
 *  Created on: Dec 2, 2013
 *      Author: lifeng
 */

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/time.h>

#include "gather_port.h"
#include "frame_manager.h"

#define MAX_GATHER_NUM (10)

static int open_serial(char *port, int baudrate);
static speed_t get_speed(int baudrate);
static void * proc_work(void * data);
static int timeval_subtract(struct timeval* result, struct timeval* x,
		struct timeval* y);

static struct gather_port gather_array[MAX_GATHER_NUM];

struct gather_port * create_gather(char* serial_name, int baudrate) {

	int i;
	struct gather_port * pgather = NULL;
	for (i = 0; i < MAX_GATHER_NUM; i++) {
		if (gather_array[i].used == 0) {

			gather_array[i].used = 1;
			pgather = gather_array + i;
			strcpy(gather_array[i].serial_name, serial_name);
			gather_array[i].baudrate = baudrate;
			int serial_fd = open_serial(serial_name, baudrate);
			gather_array[i].serial_fd = serial_fd;
			gather_array[i].frame_manager = create_frame_manager(serial_fd);
			break;
		}
	}
	return pgather;

}

void start_gather_port(struct gather_port * pgather) {

	start_frame_manager(pgather->frame_manager);

	pthread_t * thread = &(pgather->thread_wk);

	pthread_create(thread, NULL, proc_work, pgather);
}

void stop_gather_port(struct gather_port * pgather) {

}

static void * proc_work(void * data) {
	int i = 0;
	char tx_frame[16];
	char rx_frame[1024 * 2];
	int length = 0;

	struct timeval start;
	struct timeval stop;
	struct timeval diff;

	int lost = 0;
	struct gather_port* pgather = (struct gather_port*) data;
	struct frame_manager *pManager = pgather->frame_manager;
	while (1) {

		lost = 0;
		gettimeofday(&start, 0);

		for (i = 20; i < 100; i++) {

			clear_all_frame(pManager);

			tx_frame[0] = 0; //src addr
			tx_frame[1] = (char) i; //dest addr
			tx_frame[2] = 0x20; //request digital data
			tx_frame[3] = 0xff; //all channel
			send_frame(pManager, tx_frame, 4);

			get_frame(pManager, rx_frame, &length, 10);
			if (length <= 0) {
				lost++;
			}
			//printf("OK\r\n");
		}
		gettimeofday(&stop, 0);
		if (lost == 0) {
			timeval_subtract(&diff, &start, &stop);
			printf("total time:%d us\r\n", diff.tv_usec);
		}
	}

	return NULL;
}

static int timeval_subtract(struct timeval* result, struct timeval* x,
		struct timeval* y) {
	//int nsec;

	if (x->tv_sec > y->tv_sec)
		return -1;

	if ((x->tv_sec == y->tv_sec) && (x->tv_usec > y->tv_usec))
		return -1;

	result->tv_sec = (y->tv_sec - x->tv_sec);
	result->tv_usec = (y->tv_usec - x->tv_usec);

	if (result->tv_usec < 0) {
		result->tv_sec--;
		result->tv_usec += 1000000;
	}

	return 0;
}

static int open_serial(char *port, int baudrate) {
	int serial_fd = open(port, O_RDWR | O_NONBLOCK | O_NDELAY);

	if (serial_fd < 0) {
		return serial_fd;
	}
	/* *** Configure Port *** */
	struct termios tty;
	memset(&tty, 0, sizeof tty);

	/* Error Handling */
	if (tcgetattr(serial_fd, &tty) != 0) {

	}
	/* Set Baud Rate */
	speed_t speed = get_speed(baudrate);
	cfsetospeed(&tty, speed);
	cfsetispeed(&tty, speed);

	/* Setting other Port Stuff */
	tty.c_cflag &= ~PARENB; // Make 8n1
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CSIZE;
	tty.c_cflag |= CS8;
	tty.c_cflag &= ~CRTSCTS; // no flow control
	tty.c_lflag = 0; // no signaling chars, no echo, no canonical processing
	tty.c_oflag = 0; // no remapping, no delays
	//tty.c_cc[VMIN] = 0; // read doesn't block
	//tty.c_cc[VTIME] = 5; // 0.5 seconds read timeout

	tty.c_cc[VMIN] = 5;
	tty.c_cc[VTIME] = 1;

	tty.c_cflag |= CREAD | CLOCAL; // turn on READ & ignore ctrl lines
	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl
	tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
	tty.c_oflag &= ~OPOST; // make raw

	/* Flush Port, then applies attributes */
	tcflush(serial_fd, TCIFLUSH);

	if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) {

	}

	fcntl(serial_fd, F_SETFL, 0); //block read
	return serial_fd;
}

static speed_t get_speed(int baudrate) {

	static speed_t speed_list[] =
			{ B50, B75, B110, B134, B150, B200, B300, B600, B1200, B1800, B2400,
					B4800, B9600, B19200, B38400, B57600, B115200, B230400,
					B460800, B500000, B576000, B921600, B1000000, B1152000,
					B1500000, B2000000, B2500000, B3000000, B3500000, B4000000 };
	static int baudrate_list[] = { 50, 75, 110, 134, 150, 200, 300, 600, 1200,
			1800, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800,
			500000, 576000, 921600, 1000000, 1152000, 1500000, 2000000, 2500000,
			3000000, 3500000, 4000000 };

	int i;
	int index = 0;
	int diff_min = baudrate - baudrate_list[0];

	for (i = 1; i < sizeof(speed_list); i++) {

		int diff = baudrate - baudrate_list[i];
		if (diff == 0) {
			index = i;
			break;
		} else if (diff < 0) {
			diff = -diff;
		}

		if (diff < diff_min) {
			diff_min = diff;
			index = i;
		}

	}

	return speed_list[index];
}

