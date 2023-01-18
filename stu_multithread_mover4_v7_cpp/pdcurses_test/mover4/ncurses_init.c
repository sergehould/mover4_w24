/****************************************************************************
*	ncurses_init.c
* 	Contains definitions of useful  ncurses library functions
*
*	Author				Date			Version
*	Serge Hould			27 Dec. 2020	1.0.0
*
*****************************************************************************/
 
 #ifdef _WIN32
#include <Windows.h>

#else
#include <unistd.h>
#include <ncurses.h>
#include <sys/ioctl.h>
#endif

#include "header/ncurses_init.h"
#include <math.h>	
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

/* Initializes Ncurses*/
void init_ncurses(void) {
	/*Ncurse config */
	initscr();			/* Start curses mode 		  */
	keypad(stdscr, TRUE);  // enable keyboard mapping
	nonl();         // tell curses not to do NL->CR/NL on output
	cbreak();       // take input chars one at a time, no wait for \n ; necessary for moving the robot
	noecho();        // do not echo input
	start_color();
	init_pair(1, COLOR_WHITE, COLOR_BLACK);
	init_pair(2, COLOR_RED, COLOR_WHITE);
	init_pair(3, COLOR_GREEN, COLOR_WHITE);
	init_pair(4, COLOR_BLUE, COLOR_WHITE);
	init_pair(5, COLOR_RED, COLOR_BLACK);
	init_pair(6, COLOR_GREEN, COLOR_BLACK);
	init_pair(7, COLOR_BLUE, COLOR_BLACK);
	//attron(COLOR_PAIR(3)); // BLUE
}

