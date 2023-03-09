 /*
 * 	udp.h 
 *	Provided
 *
 *	Author				Date			Version
 *	Serge Hould			26 Feb 2019		
 */
 
#ifndef UDP_H
#define UDP_H
typedef struct
{
	int data[10];		//data
	int id;
}can_frame_;

int setFrame6(int id, int data1, int data2,int data3, int data4,int data5, int data6);
int setFrame3(int id, int data1, int data2, int data3);
int setFrame2(int id, int data1, int data2);
int setFrame4(int id, int data1, int data2, int data3, int data4);
void write_can_mess(char*);
can_frame_ get_can_mess(void);
int udp_init(void);

#endif  // UDP_H