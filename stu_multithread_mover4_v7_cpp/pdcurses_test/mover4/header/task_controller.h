 /*
 *	File: task_controller.h
 * 		Move all prototypes and macros pertaining to task_controller from public.h into task_controller.h
 *		public.h now contains only public prototypes excluding task_controller
 *
 *	Author				Date			Version
 *	Serge Hould			16 May 2022		2.0.0		
 *	Serge Hould			24 May 2022		2.0.1 add print_time() & void set_time()
 */

#ifndef TASK_CONTROLLER_H
#define TASK_CONTROLLER_H


#define JOG_MODE	0
#define NORM_MODE	1
#define TEST_MODE	2
#define RESET_ERROR	0xd0
#define EN_MOTOR	0xd1
#define	GRIP_DIS	0x00
#define	GRIP_CLOSE	0x02
#define	GRIP_OPEN	0x03
#define	ONE_DEGREE	65	//joints: 1° = 65 encoder tics

 /*		Speed parameters															*/
#define 	SPEED_MAX	60.0	// high speed
//#define 	SPEED_MAX	120.0	// too much speed  - overcurrent
#define		SPEED_MED	30.0
#define		SPEED_SLOW	10.0
#define		SPEED_MIN	SPEED_SLOW
#define 	SPEED_300	180.0
#define 	SPEED_200	120.0	
#define 	SPEED_100	60.0	// high speed
#define 	SPEED_90	54.0
#define 	SPEED_80	48.0	
#define 	SPEED_70	42.0
#define 	SPEED_60	36.0
#define 	SPEED_50	30.0	
#define 	SPEED_40	24.0
#define 	SPEED_30	18.0	
#define 	SPEED_20	12.0
#define 	SPEED_10	6.0	
#define 	SPEED_5		3.0	
//#define		set_speed			set_speed_max  //an alias

typedef struct
{
	double data[4];		//angle 
}kin_f;

typedef struct
{
	int data[4];		// position
}kin_i;

enum { EXTRAPOL, TRAJECT, IDLE };

/*Prototype Area*/
int get_keyb_f(int s);
void set_keyb_f(int , int);
void set_gripper(int val);
int get_gripper(void);
//kin_f get_pv_angles(void);
void clear_error(void);
void enable_motor(void);
void resetJointsToZero(void);
kin_f get_all_pv_angles(void);
void set_pv_angle(int i, double _angle);
double get_pv_angle(int nb);
double get_sp_angle(int nb);
kin_f get_all_sp_angles(void);
kin_f get_all_pv_angles(void);
double get_pv_angle(int nb);
void startTasksControllerRx(void);
void pthread_joinControllerRx(void);
void delay_ms(int);
void set_warnings(char* str);
int get_warnings(char* str); 
void init_files(void);
void print_warnings(int v, int h);
void set_errors(char* str);
int get_errors(char* str);
void print_errors(int v, int h);
void close_files(void);
void speed_set(double base, double shld, double elbow, double wrist);;
void print_time(int v, int h);
void set_time(int t);
void set_all_sp_angles(double* ptr, int max);

#endif
