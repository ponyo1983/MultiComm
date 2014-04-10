/*
 * can_port.c
 *
 *  Created on: Dec 3, 2013
 *      Author: lifeng
 */

#include "libsocketcan.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include <pthread.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/time.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>

#include "can_port.h"
#include "port_manager.h"

#define MAX_CAN_NUM (3)
#define LED_DELAY "30"

static void * can_rx_proc(void * arg);

static struct can_port can_ports[MAX_CAN_NUM];

struct can_port * create_can_port(char *name, int baudrate) {
	int i, j;
	struct can_port * pPort = NULL;
	int length = strlen(name);
	for (i = 0; i < MAX_CAN_NUM; i++) {
		if (can_ports[i].used == 0) {
			can_ports[i].used = 1;
			pPort = can_ports + i;
			for (j = 0; j < length; j++) {
				pPort->name[j] = tolower(name[j]);
			}
			pPort->name[length] = '\0';
			pPort->bandrate = baudrate;
			pPort->portIndex = atoi(name + 3);
			if(pPort->portIndex==3) //CAN3->"can0"
			{
				pPort->name[3]='0';
			}
			break;
		}
	}
	return pPort;
}


static void config_led(struct can_port * port) {
	char led_name[30];
	int fd;
	int pIndex=((port->portIndex)%3+2)%3;
	sprintf(led_name, "/sys/class/leds/can%d_tx/delay_on", pIndex);
	fd = open(led_name, O_WRONLY);
	if (fd > 0) {
		write(fd, LED_DELAY, 2);
	}
	sprintf(led_name, "/sys/class/leds/can%d_tx/delay_off", pIndex);
	fd = open(led_name, O_WRONLY);
	if (fd > 0) {
		write(fd, LED_DELAY, 2);
	}
	sprintf(led_name, "/sys/class/leds/can%d_rx/delay_on", pIndex);
	fd = open(led_name, O_WRONLY);
	if (fd > 0) {
		write(fd, LED_DELAY, 2);
	}
	sprintf(led_name, "/sys/class/leds/can%d_rx/delay_off", pIndex);
	fd = open(led_name, O_WRONLY);
	if (fd > 0) {
		write(fd, LED_DELAY, 2);
	}
	sprintf(led_name, "/sys/class/leds/can%d_rx/shot", pIndex);
	port->rx_led_fd = open(led_name, O_WRONLY);

	sprintf(led_name, "/sys/class/leds/can%d_tx/shot", pIndex);
	port->tx_led_fd = open(led_name, O_WRONLY);
}

void start_can_port(struct can_port * port) {
	if (port == NULL)
		return;
	config_led(port);
	if (can_do_stop(port->name) < 0) {
		perror("can not stop can\n");
		return;
	}
	if (can_set_bitrate(port->name, port->bandrate) < 0) {
		perror("can not set can bitrate\n");
		return;
	}
	if (can_do_start(port->name) < 0) {
		perror("can not start can");
		return;
	}

	pthread_create(&(port->threadRx), NULL, can_rx_proc, port);
}

static void trigger_tx(struct can_port *port) {
	if (port->tx_led_fd > 0) {
		write(port->tx_led_fd, "1", 1);
	}
}
static void trigger_rx(struct can_port *port) {
	if (port->rx_led_fd > 0) {
		write(port->rx_led_fd, "1", 1);
	}
}


void stop_can_port(struct can_port * port) {

}

void send_can_data(struct can_port *port, char * buffer) {
	struct can_frame frame = { .can_id = 1, };
	int i;
	int id = buffer[0] | (buffer[1] << 8) | (buffer[2] << 16)
			| ((buffer[3] & 0x7f) << 24);
	frame.can_id = id;
	if (buffer[3] & 0x80) { //扩展祯
		frame.can_id &= CAN_EFF_MASK;
		frame.can_id |= CAN_EFF_FLAG;
	} else {
		frame.can_id &= CAN_SFF_MASK;
	}
	char len = buffer[4];
	if (len > 8) {
		len = 8;
	}
	frame.can_dlc = len;
	for (i = 0; i < len; i++) {
		frame.data[i] = buffer[5 + i];
	}
	int ret = write(port->socket, &frame, sizeof(frame));
	if (ret != -1) {
		trigger_tx(port);
	}

}

static void * can_rx_proc(void * arg) {

	struct port_manager * manager = get_port_manager();
	struct can_port * port = (struct can_port *) arg;

	struct sockaddr_can addr;
	int family = PF_CAN, type = SOCK_RAW, proto = CAN_RAW;
	char *interface = port->name;
	struct ifreq ifr;
	struct can_frame frames[10];

	int nbytes;
	int i, j;
	char can_buffer[24];
	int socket_can;
	struct timeval timestamp;

	socket_can = socket(family, type, proto);
	port->socket = socket_can;
	if (socket_can < 0) {
		perror("can not create can socket\n");
		return NULL;
	}

	addr.can_family = family;
	strncpy(ifr.ifr_name, interface, sizeof(ifr.ifr_name));
	if (ioctl(socket_can, SIOCGIFINDEX, &ifr)) {
		perror("ioctl");
		return NULL;
	}
	addr.can_ifindex = ifr.ifr_ifindex;

	if (bind(socket_can, (struct sockaddr *) &addr, sizeof(addr)) < 0) {
		perror("bind");
		return NULL;
	}

	while (1) {
		int id = 0;
		if ((nbytes = read(socket_can, frames, sizeof(struct can_frame) * 10))
				< 0) {
			perror("read");
			break;
		} else {

			int count = nbytes / sizeof(struct can_frame);
			if(count>0)
			{
				trigger_rx(port);
			}
			for (j = 0; j < count; j++) {
				gettimeofday(&timestamp, NULL);
				can_buffer[0] = 1; //CAN Port
				can_buffer[1] = port->portIndex; //Port Index
				long tv_sec = (long) (timestamp.tv_sec);
				can_buffer[2] = (tv_sec & 0xff);
				can_buffer[3] = ((tv_sec >> 8) & 0xff);
				can_buffer[4] = ((tv_sec >> 16) & 0xff);
				can_buffer[5] = ((tv_sec >> 24) & 0xff);
				long tv_usec = (long) (timestamp.tv_usec);
				can_buffer[6] = (tv_usec & 0xff);
				can_buffer[7] = ((tv_usec >> 8) & 0xff);
				can_buffer[8] = ((tv_usec >> 16) & 0xff);
				can_buffer[9] = ((tv_usec >> 24) & 0xff);
				if (frames[j].can_id & CAN_EFF_FLAG) { //扩展祯
					id = frames[j].can_id & CAN_EFF_MASK;
					can_buffer[10] = id & 0xff;
					can_buffer[11] = (id >> 8) & 0xff;
					can_buffer[12] = (id >> 16) & 0xff;
					can_buffer[13] = ((id >> 24) & 0xff) | 0x80;
				} else {
					id = frames[j].can_id & CAN_SFF_MASK;
					can_buffer[10] = id & 0xff;
					can_buffer[11] = (id >> 8) & 0xff;
					can_buffer[12] = (id >> 16) & 0xff;
					can_buffer[13] = 0;
				}
				can_buffer[14] = frames[j].can_dlc;
				for (i = 0; i < frames[j].can_dlc; i++) {
					can_buffer[15 + i] = frames[j].data[i];
				}
				if (frames[j].can_id & CAN_RTR_FLAG) {
				}

				send_network_data(manager, can_buffer, 0, 24);
			}

		}
	}

	return NULL;
}

