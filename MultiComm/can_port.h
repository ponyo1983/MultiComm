/*
 * can_port.h
 *
 *  Created on: Dec 3, 2013
 *      Author: lifeng
 */

#ifndef CAN_PORT_H_
#define CAN_PORT_H_

#include <pthread.h>

struct can_port {

	int used;
	char portIndex;
	char name[8];
	char type[8];
	int bandrate;
	int tx_led_fd;
	int rx_led_fd;
	int socket;
	int snd_num;
	pthread_t threadRx;

};

struct can_port * create_can_port(char* name,int baudrate);
void start_can_port(struct can_port * port);
void stop_can_port(struct can_port * port);
void send_can_data(struct can_port *port, char * buffer);

#endif /* CAN_PORT_H_ */
