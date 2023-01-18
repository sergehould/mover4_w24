 /*
 * 	can.h 
 *	Provided
 *
 *	Author				Date			Version
 *	Serge Hould			26 Feb 2019		
 */
 
#ifndef CAN_H
#define CAN_H
typedef struct
{
	int data[6];		//data
	int id;
}can_frame_;

int setFrame6(int id, int data1, int data2,int data3, int data4,int data5, int data6);
int setFrame3(int id, int data1, int data2, int data3);
int setFrame2(int id, int data1, int data2);
int setFrame4(int id, int data1, int data2, int data3, int data4);
void write_can_mess(char*);
can_frame_ get_can_mess(void);
int open_socket(void);

#endif  // CAN_H