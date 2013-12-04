/*
 * port_manager.c
 *
 *  Created on: Dec 3, 2013
 *      Author: lifeng
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"
#include "network.h"
#include "gather_port.h"
#include "can_port.h"
#include "port_manager.h"

static char *get_sys_port(char * port);
static void insert_can_port(struct port_manager* manager, struct can_port *port);
static void insert_gather_port(struct port_manager* manager,
		struct gather_port *port);

static struct port_manager * manager_global = NULL;

struct port_manager * get_port_manager() {
	if (manager_global == NULL) {
		int i;
		manager_global = (struct port_manager*) malloc(
				sizeof(struct port_manager));

		bzero(manager_global, sizeof(struct port_manager));

		struct config_global * config = get_global_config();

		manager_global->config = config;

		//network
		manager_global->network = create_network(config->networkPort);

		//CAN,RS485 Ports
		int portNum = config->portNum;

		for (i = 0; i < portNum; i++) {
			struct port_config* port = config->ports + i;

			if (strncmp(port->type, "CAN", 3) == 0) {
				struct can_port* canPort = create_can_port(port->name,
						port->baudrate);
				insert_can_port(manager_global, canPort);
			} else if (strncmp(port->type, "RS", 2) == 0) {
				if (port->workMode == 1) //smart sensor
						{
					char * serial = get_sys_port(port->name);
					struct gather_port* gather_port = create_gather(serial,
							port->baudrate);
					insert_gather_port(manager_global, gather_port);

				}

			}
		}

	}
	return manager_global;
}

void start_port_manager(struct port_manager * manager) {
	if (manager == NULL)
		return;

	int i;
	if (manager->network != NULL) {
		start_network(manager->network);
	}

	for(i=0;i<manager->can_num;i++)
	{
		start_can_port(manager->can_ports[i]);
	}
	for(i=0;i<manager->gather_num;i++)
	{
		//start_gather_port(manager->gathers[i]);
	}


}

void stop_port_manager(struct port_manager * manager) {

}

void send_network_data(struct port_manager * manager,char* buffer,int offset,int length)
{
	if(manager==NULL) return;

	network_send(manager->network,buffer,offset,length);
}

/*
 * data struct :byte[0]-portIndex, byte[1-4] can-ID ,byte[5]-data-length ,byte[6-N] can-data
 * */
void to_can_data(struct port_manager* manager,char* buffer)
{
	char portIndex=buffer[0];

	int i;
	for(i=0;i<manager->can_num;i++)
	{
		if(manager->can_ports[i]->portIndex==portIndex)
		{
			send_can_data(manager->can_ports[i],buffer+1);
			break;
		}
	}


}



static char *get_sys_port(char * port) {
	static char* sys_port[] = { "/dev/ttySAC5", "/dev/ttySAC3", "/dev/ttySAC2",
			"/dev/ttySAC1", "/dev/ttySAC0", "/dev/ttyS2", "/dev/ttyS1",
			"/dev/ttyS0", "", "" };
	static char* board_port[] = { "COM1", "COM2", "COM3", "COM4", "COM5",
			"COM6", "COM7", "COM8", "COM9", "COM10" };
	int i;
	char * name = NULL;
	for (i = 0; i < 10; i++) {
		if (strcmp(port, board_port[i]) == 0) {
			name = sys_port[i];
			break;
		}
	}
	return name;
}

static void insert_can_port(struct port_manager* manager, struct can_port *port) {

	if (manager == NULL)
		return;
	if (manager->can_num >= MAX_CAN_NUM)
		return;

	manager->can_ports[manager->can_num] = port;
	manager->can_num = manager->can_num + 1;

}

static void insert_gather_port(struct port_manager* manager,
		struct gather_port *port) {

	if (manager == NULL)
		return;
	if (manager->gather_num >= MAX_GATHER_NUM)
		return;

	manager->gathers[manager->gather_num] = port;
	manager->gather_num = manager->gather_num + 1;

}

