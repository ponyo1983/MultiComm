/*
 * port_manager.h
 *
 *  Created on: Dec 3, 2013
 *      Author: lifeng
 */

#ifndef PORT_MANAGER_H_
#define PORT_MANAGER_H_

#include "config.h"
#include "gather_port.h"
#include "can_port.h"
#include "network.h"


#define MAX_GATHER_NUM (10)
#define MAX_CAN_NUM	(3)

struct port_manager
{

	struct config_global* config;

	struct network * network;

	int can_num;
	struct can_port* can_ports[MAX_CAN_NUM];
	int gather_num;
	struct gather_port * gathers[MAX_GATHER_NUM];

};

struct port_manager * get_port_manager();

void start_port_manager(struct port_manager * manager);
void stop_port_manager(struct port_manager * manager);

void send_network_data(struct port_manager * manager,char* buffer,int offset,int length);
void to_can_data(struct port_manager* manager,char* buffer);
void to_serial_data(struct port_manager* manager,char* buffer);
#endif /* PORT_MANAGER_H_ */
