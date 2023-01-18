 /*
 * 	can.c 
 *	Provided
 *
 *	Author				Date			Version
 *	Serge Hould			26 Feb 2019		
 */
 #include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <ncurses.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <stdint.h>
#include <ctype.h>
#include <libgen.h>
#include <errno.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/uio.h>
#include <net/if.h>
#include "header/can.h"
#include "../can-utils/lib.h"
/********/
/*Limits the movements of the joints*/
// #define HIGH_LIM1 	0x9200 	0d37376// base about +90 degrees right/left?
// #define LOW_LIM1 	0x6800	0d26624// base about -90 degrees
// //#define HIGH_LIM2 	0x8a00	// temp
// //#define HIGH_LIM2 	0x8600	0d34304// Shoulder about 45 degrees  forward
// //#define LOW_LIM2 	0x7400	0d29696// Shoulder about -45 degrees backward
// #define HIGH_LIM2 	0x8800
// #define LOW_LIM2 	0x7100	
// #define HIGH_LIM3 	0x9810	// Elbow about -100 degrees backward
// #define LOW_LIM3	0x61F0	// Elbow about +100 degrees forward
// #define HIGH_LIM4 	0x9f00	// Wrist about  +90 degrees forward
// #define LOW_LIM4 	0x5B00	// Wrist about  -90 degrees backward

/* Sets angles for joints 0 to 3 */ 
/* 	Sets angles for joints 0 to 3 */ 
/*This is the limits tested with CPRog*/
/*	Base (0) -150 to +150 degrees	*/
/*	Shoulder(1)-50 to +65*/
/*	Elbow (2) -110 to +140 */
/*	Wrist (3) -140 to +140*/

// To review in can.c (redundant, to remove and set limits only in set_sp_angles())
/*Limits the movements of the joints*/
/*Tested with CPRog*/
#define HIGH_LIM1 	0xA41C 		// -150 degrees 
#define LOW_LIM1 	0x55E3		// +150 degrees 	
#define HIGH_LIM2 	0X89DD 		// -50 degrees 
#define LOW_LIM2 	0X6C47	//  +65degrees 
#define HIGH_LIM3 	0XA105 	//  +140 degrees 
#define LOW_LIM3	0X60B5	//  -110 degrees 
#define HIGH_LIM4 	0XB402 	//  -140 degrees 
#define LOW_LIM4 	0X45C9	//  +140 degrees 	


/*Limits the movements of the joints*/
// #define HIGH_LIM1 	40000 	// about 180 degrees measured
// #define LOW_LIM1 	24000	// about -180 degrees measured
// #define HIGH_LIM2 	40000 // about 45 degrees measured
// #define LOW_LIM2 	24000 // about -45 degrees measured
// #define HIGH_LIM3 	40000  // about 100 degrees measured
// #define LOW_LIM3	24000	// about -100 degrees measured
// #define HIGH_LIM4 	40000 	 // about 0 degrees measured
// #define LOW_LIM4 	24000	// about 160 degrees measured
#define MAXSOCK 16    /* max. number of CAN interfaces given on the cmdline */


const int canfd_on = 1;
int required_mtu;
struct canfd_frame frame;
//int s; /* can raw socket */ 
int mtu;
int enable_canfd = 1;
struct sockaddr_can addr;
struct ifreq ifr;
int s[MAXSOCK];
fd_set rdfs;
char *ptr, *nptr;
char ctrlmsg[CMSG_SPACE(sizeof(struct timeval) + 3*sizeof(struct timespec) + sizeof(__u32))];
struct iovec iov;
struct msghdr msg;
int nbytes;


int open_socket(void){
	if ((s[0] = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror("socket");
		//return 1;
		exit(EXIT_FAILURE);
	}
	/* selects can0 bus */
	strncpy(ifr.ifr_name, "can0", IFNAMSIZ - 1);  // SH
	
	ifr.ifr_name[IFNAMSIZ - 1] = '\0';
	ifr.ifr_ifindex = if_nametoindex(ifr.ifr_name);
	if (!ifr.ifr_ifindex) {
		perror("if_nametoindex");
		//return 1;
		exit(EXIT_FAILURE);
	}
	/* Sets address*/
	memset(&addr, 0, sizeof(addr));
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;
	setsockopt(s[0], SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &canfd_on, sizeof(canfd_on));

	if (bind(s[0], (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror("bind");
		//return 1;
		exit(EXIT_FAILURE);
	}
	/********** Rx CAN ********/
/* these settings are static and can be held out of the hot path */
	iov.iov_base = &frame;
	msg.msg_name = &addr;
	msg.msg_iov = &iov;
	msg.msg_iovlen = 1;
	msg.msg_control = &ctrlmsg;
}


can_frame_ get_can_mess(void){
			int i;
			can_frame_ fr;
			FD_ZERO(&rdfs);
			FD_SET(s[0], &rdfs);
			if (FD_ISSET(s[0], &rdfs)) {
				/* these settings may be modified by recvmsg() */
				iov.iov_len = sizeof(frame);
				msg.msg_namelen = sizeof(addr);
				msg.msg_controllen = sizeof(ctrlmsg);  
				msg.msg_flags = 0;
				nbytes = recvmsg(s[0], &msg, 0);
//				printf("Frame1%03x, %02x %02x %02x %02x %02x %02x\n",frame.can_id,frame.data[0],frame.data[1],frame.data[2],frame.data[3],frame.data[4],frame.data[5]);
				/*Filter frames*/
			} 
			fr.id = frame.can_id;
			for(i =0;i<6;i++){
				fr.data[i] = frame.data[i];
			}
			//printf("Frame_fr%03x, %02x %02x %02x %02x %02x %02x\n",fr.id,fr.data[0],fr.data[1],fr.data[2],fr.data[3],fr.data[4],fr.data[5]);
			return fr;
		
}

void write_can_mess(char buff[]){
	required_mtu = parse_canframe(buff , &frame); 
	if (write(s[0], &frame, required_mtu) != required_mtu) {  
				perror("write");
				//return 1;
				exit(EXIT_FAILURE);
	}
}

/* Sets the frame content and limits the mover4 movements of the joints*/
/* Returns -1 if exceeds limits */
int setFrame6(int id, int data1, int data2,int data3, int data4,int data5, int data6){
	char buff[30]={0};
	long sum=0;
	switch(id){
		case 0x10:
		case 0x11:
			sum = data3*256+data4;
//			printf("sum: %x\n",sum);
			if((sum > HIGH_LIM1)  || (sum < LOW_LIM1)){
				mvprintw(20,0,"Joint 1 exceeds limits\n\n");
				refresh();
				return -1;
			}
			//mvprintw(21,0,"Joint 1 sum %d limit H %d  limit L %d \n\n",sum , HIGH_LIM1, LOW_LIM1);
			//refresh();
			sprintf(buff,"%03x#%02x.%02x.%02x.%02x.%02x.%02x.%02x\0",id,data1,data2,data3,data4,data5,data6);
			buff[21]='\0';
			//printf(buff);
			//printf("\n");
		break;
		case 0x20:
		case 0x21:
			sum = data3*256+data4;
			if((sum > HIGH_LIM2)  || (sum < LOW_LIM2)){
				mvprintw(20,0,"Joint 2 exceeds limits\n\n");
				refresh();
			//	printf("sum: %x\n",sum);
				return -1;
			}
		//	mvprintw(22,0,"Joint 2 sum %d limit H %d  limit L %d \n\n",sum , HIGH_LIM2, LOW_LIM2);
			//refresh();
			sprintf(buff,"%03x#%02x.%02x.%02x.%02x.%02x.%02x.%02x\0",id,data1,data2,data3,data4,data5,data6);
			//printf("joint 2: %03x#%02x.%02x.%02x.%02x.%02x.%02x.%02x\0",id,data1,data2,data3,data4,data5,data6);
			buff[21]='\0';
		break;
		case 0x30:
		case 0x31:
			sum = data3*256+data4;
			if((sum > HIGH_LIM3)  || (sum < LOW_LIM3)){
				mvprintw(20,0,"Joint 3 exceeds limits\n\n");
				refresh();
			//	mvprintw(17,0,"Joint angle : %x\n\n",angle3);
				return -1;
			}
			//mvprintw(23,0,"Joint 3 sum %d limit H %d  limit L %d \n\n",sum , HIGH_LIM3, LOW_LIM3);
			//refresh();
			sprintf(buff,"%03x#%02x.%02x.%02x.%02x.%02x.%02x.%02x",id,data1,data2,data3,data4,data5,data6);
			buff[21]='\0'; 
		break;
		case 0x40:
		case 0x41:
			sum = data3*256+data4;
			if((sum > HIGH_LIM4)  || (sum < LOW_LIM4)){
				mvprintw(20,0,"Joint 4 exceeds limits\n\n");
				refresh();
				return -1;
			}
		//	mvprintw(24,0,"Joint 4 sum %d limit H %d  limit L %d \n\n",sum , HIGH_LIM4, LOW_LIM4);
		//	refresh();
			sprintf(buff,"%03x#%02x.%02x.%02x.%02x.%02x.%02x.%02x\0",id,data1,data2,data3,data4,data5,data6);
			buff[21]='\0';
		break;
	}
	

	write_can_mess(buff);
	return 0;
}

/* Sets a 2 bytes of data frame */
int setFrame2(int id, int data1, int data2){
	char buff[30]={0};
	sprintf(buff,"%03x#%02x.%02x",id,data1,data2);
	buff[9]='\0';
	//printf(buff);
	//printf("\n");
	write_can_mess(buff);

	return 0;
}

/* Sets a 3 bytes of data frame */
int setFrame3(int id, int data1, int data2, int data3){
	char buff[30]={0};
	sprintf(buff,"%03x#%02x.%02x.%02x",id,data1,data2, data3);
	buff[12]='\0';
	//printf(buff);
	//printf("\n");
	write_can_mess(buff);
	return 0;
}

/* Sets a 3 bytes of data frame */
int setFrame4(int id, int data1, int data2, int data3, int data4){
	char buff[30]={0};
	sprintf(buff,"%03x#%02x.%02x.%02x.%02x",id,data1,data2, data3, data4);
	buff[15]='\0';
	//printf(buff);
	//printf("\n");
	write_can_mess(buff);
	return 0;
}

