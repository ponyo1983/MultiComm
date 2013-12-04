/*
 * gather.h
 *
 * one serial port to one gather
 *
 *  Created on: Dec 2, 2013
 *      Author: lifeng
 */

#ifndef GATHER_H_
#define GATHER_H_

#include <pthread.h>
#include "frame_manager.h"

struct gather_port
{
  int used;
  int serial_fd;
  char  serial_name[50];
  char portIndex;
  int baudrate;

  struct frame_manager * frame_manager;
  pthread_t thread_wk;


};

struct smart_sensor
{
	int addr;
	int type;
};



struct gather_port* create_gather(char*serial_name,int baudrate);

void gather_add_module(int addr,int type);

void start_gather_port(struct gather_port* pgather);
void stop_gather_port(struct gather_port* pgather);


#endif /* GATHER_H_ */
