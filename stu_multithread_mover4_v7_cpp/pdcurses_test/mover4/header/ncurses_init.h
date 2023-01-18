 /****************************************************************************
 *	ncurses_init.h
 * 	Contains useful public functions for the ncurses library
 *
 *	Author				Date			Version
 *	Serge Hould			27 Dec. 2020	1.0.0
 *
 *****************************************************************************/

#ifndef	NCURSE_INIT_H
#define NCURSE_INIT_H
#ifdef	__cplusplus
extern "C" {
#endif

#define	WHITE_BLACK		COLOR_PAIR(1)
#define	RED_WHITE		COLOR_PAIR(2)
#define	GREEN_WHITE		COLOR_PAIR(3)
#define	BLUE_WHITE		COLOR_PAIR(4)
#define	RED_BLACK		COLOR_PAIR(5)
#define	GREEN_BLACK		COLOR_PAIR(6)
#define	BLUE_BLACK		COLOR_PAIR(7)

/*Prototype Area*/
void init_ncurses(void);
#ifdef	__cplusplus
}
#endif
#endif
