/****************************************************************************
 *  UDP.cpp 
 *
 *	Author			Date				Version
 *	Serge Hould		27 Feb. 2023		v1.0.0	    Tested in loopback mode - crude 
 *                                                  packets are retured to the client. No CAN 
 *                                                  packets involved. See server bbb_server_udp-can.cpp
 * SH               3 Mar. 2023         v2.0.0      Populates setFrame2(), setFrame3() and setFrame4()
 *                                                  Add limits to setFrame6() - the limits inside setFrame6() 
 *                                                  inside can.c were removed - see 247-607\projects\visual_studio
 *                                                  \multithread_mover4_udp\pdcurses_test\mover4
 *													Tested succesfully with the real robot.
 *	SH				6 Mar.2023						Was tested successfully on 3 robots running simulteanously
 *
 *  TODO: test with robot in none loopback mode - must undefine LOOPBACK
 *****************************************************************************/

#ifdef _WIN32
//#include <Windows.h>
#else
#include <unistd.h>
#include <ncurses.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <libgen.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <sys/uio.h>
#include "../can-utils/terminal.h"
#include <fcntl.h>   /* File control definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include "../can-utils/lib.h"
#endif
#include <curses.h>

#define _WINSOCK_DEPRECATED_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <ctype.h>
#include <errno.h>
#include <sys/types.h>

#include <winsock2.h>
#include <stdio.h>
#include <tchar.h>

#pragma comment(lib,"WS2_32")

#include "header/udp.h"
#include "header/config.h"

#define HIGH_LIM1 	0xA41C 		// -150 degrees 
#define LOW_LIM1 	0x55E3		// +150 degrees 	
#define HIGH_LIM2 	0X89DD 		// -50 degrees 
#define LOW_LIM2 	0X6C47	//  +65degrees 
#define HIGH_LIM3 	0XA105 	//  +140 degrees 
#define LOW_LIM3	0X60B5	//  -110 degrees 
#define HIGH_LIM4 	0XB402 	//  -140 degrees 
#define LOW_LIM4 	0X45C9	//  +140 degrees 	

#define BUF_SIZE 1024
#define PORT 5150


WSADATA wsa;
SOCKET sockfd;
struct sockaddr_in server_addr;

char send_buf[10];
char rec_buf[100];
int cnt = 0;

int udp_init(void) {

    // Initialize Winsock
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
        printf("WSAStartup failed. Error Code : %d\n", WSAGetLastError());
        return NULL;
    }

    // Creating socket file descriptor
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == INVALID_SOCKET) {
        printf("socket creation failed. Error Code : %d\n", WSAGetLastError());
        WSACleanup();
        return NULL;
    }

    memset(&server_addr, 0, sizeof(server_addr));

    // Filling server information
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(PORT);
    server_addr.sin_addr.s_addr = inet_addr(ADDRESS);
}

int setFrame6(int id, int data1, int data2, int data3, int data4, int data5, int data6) {
        char buff[30] = { 0 };
        long sum = 0;
        switch (id) {
            case 0x10:
            case 0x11:
                sum = data3 * 256 + data4;
                if ((sum > HIGH_LIM1) || (sum < LOW_LIM1)) {
                    mvprintw(20, 0, "Joint 1 exceeds limits\n\n");
                    refresh();
                    return -1;
                }
                break;
            case 0x20:
            case 0x21:
                sum = data3 * 256 + data4;
                if ((sum > HIGH_LIM2) || (sum < LOW_LIM2)) {
                    mvprintw(20, 0, "Joint 2 exceeds limits\n\n");
                    refresh();
                    //	printf("sum: %x\n",sum);
                    return -1;
                }
                break;
            case 0x30:
            case 0x31:
                sum = data3 * 256 + data4;
                if ((sum > HIGH_LIM3) || (sum < LOW_LIM3)) {
                    mvprintw(20, 0, "Joint 3 exceeds limits\n\n");
                    refresh();
                    //	mvprintw(17,0,"Joint angle : %x\n\n",angle3);
                    return -1;
                }
                break;
            case 0x40:
            case 0x41:
                sum = data3 * 256 + data4;
                if ((sum > HIGH_LIM4) || (sum < LOW_LIM4)) {
                    mvprintw(20, 0, "Joint 4 exceeds limits\n\n");
                    refresh();
                    return -1;
                }
                break;
        }

        /*Fills the buffer*/
        send_buf[0] = id;
        send_buf[1] = data1;
        send_buf[2] = data2;
        send_buf[3] = data3;
        send_buf[4] = data4;
        send_buf[5] = data5;
        send_buf[6] = data6;

        // Send datagram  to server
        if (sendto(sockfd, (char*)send_buf, 7, 0, (struct sockaddr*)&server_addr, sizeof(server_addr)) == SOCKET_ERROR) {
            printf("sendto failed. Error Code : %d\n", WSAGetLastError());
            closesocket(sockfd);
            WSACleanup();
            return NULL;
        }
        return NULL;

}


int setFrame3(int id, int data1, int data2, int data3) { 
    send_buf[0] = id;
    send_buf[1] = data1;
    send_buf[2] = data2;
    send_buf[3] = data3;

    // Send datagram  to server
    if (sendto(sockfd, (char*)send_buf, 4, 0, (struct sockaddr*)&server_addr, sizeof(server_addr)) == SOCKET_ERROR) {
        printf("sendto failed. Error Code : %d\n", WSAGetLastError());
        closesocket(sockfd);
        WSACleanup();
        return NULL;
    }
    return NULL;


}
int setFrame2(int id, int data1, int data2) { 
    send_buf[0] = id;
    send_buf[1] = data1;
    send_buf[2] = data2;

    // Send datagram  to server
    if (sendto(sockfd, (char*)send_buf, 3, 0, (struct sockaddr*)&server_addr, sizeof(server_addr)) == SOCKET_ERROR) {
        printf("sendto failed. Error Code : %d\n", WSAGetLastError());
        closesocket(sockfd);
        WSACleanup();
        return NULL;
    }
    return NULL;


}
int setFrame4(int id, int data1, int data2, int data3, int data4) { 
    send_buf[0] = id;
    send_buf[1] = data1;
    send_buf[2] = data2;
    send_buf[3] = data3;
    send_buf[4] = data4;

    // Send datagram  to server
    if (sendto(sockfd, (char*)send_buf, 5, 0, (struct sockaddr*)&server_addr, sizeof(server_addr)) == SOCKET_ERROR) {
        printf("sendto failed. Error Code : %d\n", WSAGetLastError());
        closesocket(sockfd);
        WSACleanup();
        return NULL;
    }
    return NULL;

}
//void write_can_mess(char*) {}

can_frame_ get_can_mess(void) {
	    can_frame_ the_frame;
        // Receive acknowledgement from server
        int server_addr_len = sizeof(server_addr);
        int n;
        if ((n = recvfrom(sockfd, rec_buf, BUF_SIZE, 0, (struct sockaddr*)&server_addr, &server_addr_len)) == SOCKET_ERROR) {
            printf("recvfrom failed. Error Code : %d\n", WSAGetLastError());
            closesocket(sockfd);
            WSACleanup();
            return the_frame;
        }
        the_frame.id = rec_buf[0]; //id
        the_frame.data[0] = rec_buf[1];  // error code
        the_frame.data[1] = rec_buf[2];  // velocity
        the_frame.data[2] = rec_buf[3];  // posH
        the_frame.data[3] = rec_buf[4];  // posL
        the_frame.data[4] = rec_buf[5];  // shunt
        the_frame.data[5] = rec_buf[6];  // timestamp
        the_frame.data[6] = rec_buf[7]; //divValue
        the_frame.data[7] = rec_buf[8]; //digitalInputs
		the_frame.data[8] = rec_buf[9]; //adc LSB
        the_frame.data[9] = rec_buf[10]; //adc LSB

        return the_frame;
        //printf("n= %d \n", n);
        //rec_buf[n] = '\0';
        //printf("Just received from server at time  %d ms %s\n", current_ms, buffer);
}
