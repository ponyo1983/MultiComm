/*
 * main.c
 *
 *  Created on: Dec 2, 2013
 *      Author: lifeng
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <libgen.h>


#include "port_manager.h"



int main(int argc, char**argv) {


	if(daemon(1, 1)==-1)
	{
		exit(-1);
		perror("daemon error\r\n");
	}


	chdir(dirname(argv[0])); //change current dir to application dir


	struct port_manager* manager= get_port_manager();

	start_port_manager(manager);



	pthread_exit(NULL);

	exit(0);
}
