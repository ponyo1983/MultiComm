/*
 * config.c
 *
 *  Created on: Dec 3, 2013
 *      Author: lifeng
 */

#include <stdio.h>
#include <stdlib.h>

#include "ini_doc.h"
#include "config.h"

static struct port_config* insert_port_config(struct config_global * gconfig,
		char* name, char* type, int baudrate, int workMode);
static void insert_sensor_config(struct port_config * port,int addr,int type);


static struct config_global *global_config = NULL;

struct config_global * get_global_config() {
	if (global_config == NULL) {
		int i,j;
		char buffer[128];
		global_config = (struct config_global*) malloc(
				sizeof(struct config_global));
		struct ini_doc * config_doc = create_ini_doc("./config.ini");
		global_config->pDoc = config_doc;
		//[General]
		int portNum = get_ini_int(config_doc, "General", "PortNum");
		global_config->portNum = 0;//portNum;
		int networkPort = get_ini_int(config_doc, "General", "NetworkPort");
		global_config->networkPort = networkPort;
		//[Port*]
		for (i = 1; i <= portNum; i++) {
			sprintf(buffer, "Port%d", i);
			char * portName = get_ini_string(config_doc, buffer, "Name");
			char * portType = get_ini_string(config_doc, buffer, "Type");
			int baudrate = get_ini_int(config_doc, buffer, "Baudrate");
			int workMode = get_ini_int(config_doc, buffer, "WorkMode");
			int sensorNum=get_ini_int(config_doc, buffer, "SensorNum");
			struct port_config * pPort = insert_port_config(global_config, portName,
					portType, baudrate, workMode);
			if(pPort==NULL) break;
			for(j=1;j<=sensorNum;j++)
			{
				sprintf(buffer, "Port%d/Sensor%d", i,j);
				int sensorAddr=get_ini_int(config_doc, buffer, "Addr");
				int sensorType=get_ini_int(config_doc, buffer, "Type");
				insert_sensor_config(pPort,sensorAddr,sensorType);

			}

		}

	}
	return global_config;

}

static struct port_config* insert_port_config(struct config_global * gconfig,
		char* name, char* type, int baudrate, int workMode) {

	if(gconfig->portNum>=MAX_PORT_NUM)
		return NULL;

	struct port_config *port=gconfig->ports+(gconfig->portNum);


	port->name=name;
	port->type=type;
	port->baudrate=baudrate;
	port->workMode=workMode;
	port->sensorNum=0;


	gconfig->portNum=gconfig->portNum+1;

	return port;

}

static void insert_sensor_config(struct port_config * port,int addr,int type)
{
	if(port==NULL) return;

	if(port->sensorNum>=MAX_SENSORS_PER_PORT) return;

	struct sensor_config * pSensor=port->sensors+(port->sensorNum);

	pSensor->addr=addr;
	pSensor->type=type;
	port->sensorNum=port->sensorNum+1;

}


