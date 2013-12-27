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
#include "port_manager.h"

#define MAX_GATHER_NUM (10)

#define QUERY_INTERVAL	(1000000) //1S
#define WAIT_TIMEOUT (30)	//30ms

static int open_serial(char *port, int baudrate);
static speed_t get_speed(int baudrate);
static void * proc_work(void * data);
static int timeval_subtract(struct timeval* result, struct timeval* x,
		struct timeval* y);

static void query_digital(struct smart_sensor *sensor);
static void query_analog(struct smart_sensor* sensor);
static void query_curve(struct smart_sensor *sensor);

static struct gather_port gather_array[MAX_GATHER_NUM];

enum query_type {
	TYPE_DIGITAL = 0x20, TYPE_ANALOG = 0x30, TYPE_CURVE = 0x60,
};

struct gather_port * create_gather(char* serial_name, int baudrate) {

	int i;
	struct gather_port * pgather = NULL;
	for (i = 0; i < MAX_GATHER_NUM; i++) {
		if (gather_array[i].used == 0) {

			gather_array[i].used = 1;
			pgather = gather_array + i;
			strcpy(gather_array[i].serial_name, serial_name);
			gather_array[i].baudrate = baudrate;
			gather_array[i].sensor_num = 0;
			int serial_fd = open_serial(serial_name, baudrate);
//			if(serial_fd!=0)
//			{
//				printf("%s\r\n",serial_name);
//			}
			gather_array[i].serial_fd = serial_fd;
			gather_array[i].frame_manager = create_frame_manager(serial_fd);
			break;
		}
	}
	return pgather;

}

void add_sensor_II(struct gather_port *port, int addr, int type) {
	if (port == NULL)
		return;
	if (port->sensor_num >= MAX_SENSOR)
		return;

	int i;
	for (i = 0; i < port->sensor_num; i++) {
		if (port->sensors[i].addr == addr) //sensor address repeat
			return;
	}
	//printf("sensor Num:%d\r\n",port->sensor_num);
	struct smart_sensor* sensor = port->sensors + port->sensor_num;
	sensor->port = port;
	sensor->addr = addr;
	sensor->type = type;
	sensor->tx_data = port->tx_data;
	sensor->rx_data = port->rx_data;
	sensor->query_digit = NULL;
	sensor->query_analog = NULL;
	sensor->query_curve = NULL;
	switch (type) {
	case 16: //Type II 交流道岔表示
		sensor->query_analog = query_analog;
		port->sensor_num++;
		break;
	case 17: //Type II 直流道岔表示
		sensor->query_analog = query_analog;
		port->sensor_num++;
		break;
	case 18: //Type II 轨道电路传感器

		break;
	case 19: //交流转辙机智能传感器
		sensor->query_digit = query_digital;
		sensor->query_analog = query_analog;
		sensor->query_curve = query_curve;
		port->sensor_num++;
		break;
	case 21: //高压不对称 ,怎么查询曲线??
		sensor->query_analog = query_analog;
		port->sensor_num++;
		break;
	case 23: // 半自动闭塞传感器
		sensor->query_analog = query_analog;
		port->sensor_num++;
		break;
	case 24: //站联电压智能传感器
		sensor->query_analog = query_analog;
		port->sensor_num++;
		break;
	case 25: //直流道岔传感器
		sensor->query_digit = query_digital;
		sensor->query_curve = query_curve;
		port->sensor_num++;

		break;
	case 26: //站内点码化
		sensor->query_analog = query_analog;
		port->sensor_num++;
		break;
	case 29: //无绝缘移频发送传感器
		sensor->query_analog = query_analog;
		port->sensor_num++;
		break;

	case 30: //无绝缘移频接收传感器
		sensor->query_analog = query_analog;
		port->sensor_num++;
		break;

	}

}

static void query_data(struct smart_sensor *sensor, char type) {

	struct gather_port* pgather = sensor->port;
	struct frame_manager *pManager = pgather->frame_manager;
	struct port_manager * portManager = get_port_manager();

	char *tx_frame = sensor->tx_data;
	char *rx_frame = sensor->rx_data;
	struct timeval tm;
	int length = 0;

	clear_all_frame(pManager);
	tx_frame[0] = 0; //src addr
	tx_frame[1] = (char) sensor->addr; //dest addr
	tx_frame[2] = type; //request [type-digital,analog etc] data
	tx_frame[3] = 0xff; //all channel
	send_frame(pManager, tx_frame, 4);

	if (type == TYPE_ANALOG || type == TYPE_DIGITAL) {
		get_frame(pManager, rx_frame + 10, &length, WAIT_TIMEOUT);
		if (length > 0) {
			gettimeofday(&tm, NULL);
			rx_frame[0] = 0x02; //Serial-data
			rx_frame[1] = pgather->portIndex; //COM Num

			long tv_sec = (long) (tm.tv_sec);
			rx_frame[2] = (tv_sec & 0xff);
			rx_frame[3] = ((tv_sec >> 8) & 0xff);
			rx_frame[4] = ((tv_sec >> 16) & 0xff);
			rx_frame[5] = ((tv_sec >> 24) & 0xff);
			long tv_usec = (long) (tm.tv_usec);
			rx_frame[6] = (tv_usec & 0xff);
			rx_frame[7] = ((tv_usec >> 8) & 0xff);
			rx_frame[8] = ((tv_usec >> 16) & 0xff);
			rx_frame[9] = ((tv_usec >> 24) & 0xff);

			send_network_data(portManager, rx_frame, 0, length + 10);
		}
	} else if (type == TYPE_CURVE) {
		while (1) {

			get_frame(pManager, rx_frame + 10, &length, WAIT_TIMEOUT);
			if (length > 0) {
				gettimeofday(&tm, NULL);
				rx_frame[0] = 0x02; //Serial-data
				rx_frame[1] = pgather->portIndex; //COM Num

				long tv_sec = (long) (tm.tv_sec);
				rx_frame[2] = (tv_sec & 0xff);
				rx_frame[3] = ((tv_sec >> 8) & 0xff);
				rx_frame[4] = ((tv_sec >> 16) & 0xff);
				rx_frame[5] = ((tv_sec >> 24) & 0xff);
				long tv_usec = (long) (tm.tv_usec);
				rx_frame[6] = (tv_usec & 0xff);
				rx_frame[7] = ((tv_usec >> 8) & 0xff);
				rx_frame[8] = ((tv_usec >> 16) & 0xff);
				rx_frame[9] = ((tv_usec >> 24) & 0xff);

				send_network_data(portManager, rx_frame, 0, length + 10);

				if ((rx_frame[14] == TYPE_CURVE) && (rx_frame[15] == 0xff)) //no curve data
						{
					break;
				}
			}

			int interval = serial_rx_timeout(pManager);
						if (interval > WAIT_TIMEOUT)
							break;
		}
	}
}

static void query_digital(struct smart_sensor *sensor) {

	query_data(sensor, 0x20);
}

static void query_analog(struct smart_sensor* sensor) {
	query_data(sensor, 0x30);
}
static void query_curve(struct smart_sensor *sensor) {
	query_data(sensor, 0x60);
}

void start_gather_port(struct gather_port * pgather) {

	start_frame_manager(pgather->frame_manager);

	pthread_t * thread = &(pgather->thread_wk);

	pthread_create(thread, NULL, proc_work, pgather);
}

void stop_gather_port(struct gather_port * pgather) {

}

static void * proc_work(void * data) {

	struct timeval start;
	struct timeval stop;
	struct timeval diff;

	int i;
	struct gather_port* pgather = (struct gather_port*) data;

	while (1) {

		//printf("q:%d\r\n",pgather->sensor_num);
		gettimeofday(&start, NULL);

		for (i = 0; i < pgather->sensor_num; i++) {
			struct smart_sensor * sensor = pgather->sensors + i;

			if (sensor->query_digit != NULL) //query digital data
			{
				sensor->query_digit(sensor);
			}
			if (sensor->query_analog != NULL) //query analog data
			{
				sensor->query_analog(sensor);
			}
			if (sensor->query_curve != NULL) //query curve data
			{
				sensor->query_curve(sensor);
			}

		}

		gettimeofday(&stop, NULL);
		timeval_subtract(&diff, &start, &stop);

		__useconds_t interval = diff.tv_sec * 1000000 + diff.tv_usec + 10000;

		if (interval < QUERY_INTERVAL) {
			usleep(QUERY_INTERVAL - interval);
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

	//tty.c_cc[VMIN] = 0;
	//tty.c_cc[VTIME] = 1;

	tty.c_cflag |= CREAD | CLOCAL; // turn on READ & ignore ctrl lines
	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl
	tty.c_iflag &= ~(INLCR | IGNCR | ICRNL);
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

	static speed_t speed_list[] = { B50, B75, B110, B134, B150, B200, B300,
	B600, B1200, B1800, B2400,
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

