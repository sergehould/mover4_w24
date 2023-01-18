/****************************************************************************
 *  main.cpp
 *
 *	Author				Date			Version
 *	Serge Hould			16 May 2022		v1.0.0	
 *	Serge Hould			25 Dec. 2022	v2.0.0	The project was converted into cpp
 *	Serge Hould			2 Jan. 2023		v2.0.1	Test trajectory Generation for smooth movement
 *
 *****************************************************************************/

#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#include <ncurses.h>
#include <sys/ioctl.h>
#endif
#include "../header/can.h"
#include "../header/task_controller.h"
#include "../header/ncurses_init.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <curses.h>
#include <stdint.h>
#include <ctype.h>
#include <errno.h>
#include <sys/types.h>

#ifdef _WIN32
#define HAVE_STRUCT_TIMESPEC  // for win32 only. Because TIMESPEC is re-defined inside pthread.h
#endif
#include <pthread.h>


int main(void)
{
	/*Ncurse config */
	init_ncurses();

	/**** rx-tx CAN init ***********/
#ifndef _WIN32
	open_socket(); // CAN socket init
#endif


	startTasksControllerRx();// combined tasks controller and Tx (AKA task1 and task3)

	pthread_joinControllerRx();

	close_files();

	exit(EXIT_SUCCESS);
}
