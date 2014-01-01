/*
 * config.c
 *
 *  Created on: Dec 3, 2013
 *      Author: lifeng
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ini_doc.h"
#include "config.h"

static struct port_config* insert_port_config(struct config_global * gconfig,
		char* name, char* type, int baudrate, int workMode);
static void insert_sensor_config(struct port_config * port, int addr, int type);

static struct config_global *global_config = NULL;

static void insert_sensor_config(struct port_config * port, int addr, int type) {
	if (port == NULL)
		return;

	if (port->sensorNum >= MAX_SENSORS_PER_PORT)
		return;

	struct sensor_config * pSensor = port->sensors + (port->sensorNum);

	pSensor->addr = addr;
	pSensor->type = type;
	port->sensorNum = port->sensorNum + 1;

}

static void insert_sensors_segment(struct port_config * pPort, char *segment,
		int startIndex, int midIndex) {

	int i;
	int startNum, endNum;
	if (midIndex > 0) {
		startNum = atoi(segment + startIndex);
		endNum = atoi(segment + midIndex+1);

	} else {
		startNum = atoi(segment + startIndex);
		endNum = startNum;
	}
	for(i=startNum;i<=endNum;i++)
	{

		insert_sensor_config(pPort,i,-1);
	}
}

static void insert_sensors_list(struct port_config * pPort, char *list) {
	char sensorList[100];
	int startIndex, endIndex, midIndex;


	strcpy(sensorList, list);

	startIndex = 0;
	endIndex = 0;
	midIndex = -1;

	while (sensorList[endIndex] != '\0') {
		if (sensorList[endIndex] == ',') {
			sensorList[endIndex] = '\0';

			if (endIndex > startIndex) {

				insert_sensors_segment(pPort, sensorList, startIndex, midIndex);

				midIndex = -1;
			}
			startIndex = endIndex + 1;
		} else if (sensorList[endIndex] == '-') {
			sensorList[endIndex] = '\0';
			midIndex = endIndex;
		}
		endIndex++;
	}
	if (endIndex > startIndex) {

		insert_sensors_segment(pPort, sensorList, startIndex, midIndex);
		midIndex = -1;
	}
}

struct config_global * get_global_config() {
	if (global_config == NULL) {
		int i;

		char buffer[128];
		global_config = (struct config_global*) malloc(
				sizeof(struct config_global));
		struct ini_doc * config_doc = create_ini_doc("./config.ini");
		global_config->pDoc = config_doc;
		//[General]
		int portNum = get_ini_int(config_doc, "General", "PortNum");
		global_config->portNum = 0; //portNum;
		int networkPort = get_ini_int(config_doc, "General", "NetworkPort");
		global_config->networkPort = networkPort;
		//[Port*]
		for (i = 1; i <= portNum; i++) {
			sprintf(buffer, "Port%d", i);

			char * portName = get_ini_string(config_doc, buffer, "Name");

			char * portType = get_ini_string(config_doc, buffer, "Type");
			int baudrate = get_ini_int(config_doc, buffer, "Baudrate");
			int sensorType = get_ini_int(config_doc, buffer, "SensorType");

			char* sensorList = get_ini_string(config_doc, buffer, "SensorList");


			struct port_config * pPort = insert_port_config(global_config,
					portName, portType, baudrate, sensorType);

			if (pPort == NULL)
				break;

			if(sensorList!=NULL)
			{
				insert_sensors_list(pPort, sensorList);
			}

		}

	}
	return global_config;

}

static struct port_config* insert_port_config(struct config_global * gconfig,
		char* name, char* type, int baudrate, int workMode) {

	if (gconfig->portNum >= MAX_PORT_NUM)
		return NULL;

	struct port_config *port = gconfig->ports + (gconfig->portNum);

	port->name = name;
	port->type = type;
	port->baudrate = baudrate;
	port->workMode = workMode;
	port->sensorNum = 0;

	gconfig->portNum = gconfig->portNum + 1;

	return port;

}



