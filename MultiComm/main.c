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

	chdir(dirname(argv[0])); //change current dir to application dir


	struct port_manager* manager= get_port_manager();

	start_port_manager(manager);

//	struct gather* pgather1 = create_gather("/dev/ttySAC0", 57600);
//
//	start_gather(pgather1);
//
//	struct gather* pgather2 = create_gather("/dev/ttySAC1", 57600);
//
//	start_gather(pgather2);
//
//	struct gather* pgather3 = create_gather("/dev/ttySAC2", 57600);
//
//	start_gather(pgather3);

	pthread_exit(NULL);

	exit(0);
}
