/*
 * frame_manager.h
 *
 *  Created on: Dec 2, 2013
 *      Author: lifeng
 */

#ifndef FRAME_MANAGER_H_
#define FRAME_MANAGER_H_

#include <pthread.h>

#define MAX_RX_FRAME_LENGTH  (2*1024)
#define MAX_FRAME_COUNT (4)
#define MAX_TX_FRAME_LENGTH	(128)

#define RX_BUFFER_SIZE (32)

#define DLE (0x10)
#define STX (0x02)
#define ETX (0x03)


struct frame_manager
{
	int used;
	int wrIndex;
	int frame_count;
	char frame_array [MAX_FRAME_COUNT][MAX_RX_FRAME_LENGTH];
	char rx_buffer[RX_BUFFER_SIZE];
	char frame_buffer[MAX_RX_FRAME_LENGTH];
	char tx_buffer[MAX_TX_FRAME_LENGTH];
	int serial_fd;

	pthread_mutex_t mutext;
	pthread_cond_t cond;

	pthread_t thread_rx;

	struct timeval last_rx_time;

};


struct frame_manager * create_frame_manager(int serial_fd);

void start_frame_manager(struct frame_manager* manager);
void stop_frame_manager(struct frame_manager* manager);

void clear_all_frame(struct frame_manager * manager);
void send_frame(struct frame_manager *manager,char* frame,int length);
void get_frame(struct frame_manager * manager, char * buffer, int *length,
		int timeout_ms);

int serial_rx_timeout(struct frame_manager* manager);

#endif /* FRAME_MANAGER_H_ */
