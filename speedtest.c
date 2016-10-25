/* speedtest.c */ 
#include <stdlib.h> 
#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"
#include "math.h"

/* OSEK declarations */
DeclareTask(Task1);

/* OSEK hooks */
void StartupHook(void){}
void ShutdownHook(StatusType ercd){}
void PreTaskHook(void){}
void PostTaskHook(void){}
void ErrorHook(StatusType ercd){}

/* LEJOS OSEK hooks */
void ecrobot_device_initialize()
{
	nxt_motor_set_speed(NXT_PORT_B, 0, 1);
	nxt_motor_set_speed(NXT_PORT_C, 0, 1);
}
void ecrobot_device_terminate()
{
	nxt_motor_set_speed(NXT_PORT_B, 0, 1);
	nxt_motor_set_speed(NXT_PORT_C, 0, 1);
}
void user_1ms_isr_type2(void){}

void disp(int row, char *str, int val)
{ 
	if (row == 0) display_clear(0);
	display_goto_xy(0, row);
	display_string(str);
	display_int(val, 0);
	if (row == 7) display_update();
}
/* Task for speed test
 * If you want to see the performance of LCD display,
 * remove // in the loop
 */

#define rows 10
#define cols 12
#define key 0.305

int map[rows][cols];//={{0}};
int path[20];
double start[2];
double goal[2];
int path_count;
int count;
int val[4];
int face;
int index1[2];

int i;
int j;
int k;

int found;

int r_rev=0;
int l_rev=0;
float R=0;
float L=0;

int REV;
int R_REV;
int L_REV;

int time_out;

void indexes(double ptr[],int ret[])
{	

	for(i=0;i<2;i++)
	{
		double temp=ptr[i]/key;
		double val=temp;
		temp=ceil(temp);

		if(abs(val - temp)>0.2)
			temp=temp+1;

		ret[i]=temp;
	}
}

void generate_path()
{
path_count=0;
count=0;
int index=-1;
 
 face=1;

	int start_index[2];
	indexes(start,start_index);
	found=0;
	while(count!=rows*cols)
	{
		if(start_index[1]+1<rows)
			val[0]=map[start_index[1]+1][start_index[0]];
		if(start_index[1]-1>=0)
			val[1]=map[start_index[1]-1][start_index[0]];
		if(start_index[0]+1<cols)
			val[2]=map[start_index[1]][start_index[0]+1];
		if(start_index[0]-1>=0)
			val[3]=map[start_index[1]][start_index[0]-1];

		for(i=0;i<4;i++)
		{
			if(val[i]==99)
			{
				index=i;
				found=1;
			}
		}
		
		//east		+1
		//west		-1
		//north		+2
		//south		-2

		int min=99;
		
		//int index=-1;

		if(found==0)
		{
		if(val[0]!=0 && val[0]!=-1)
		{
			if(val[1]!=0 && val[1]!=-1)
			{			
				if(val[0]<=val[1])
				{				
					min=val[0];
					index=0;
				}
				else
				{
					min=val[1];
					index=1;
				}
			}
			else
			{
				min=val[0];
				index=0;
			}
		}
		else if(val[1]!=0 && val[1]!=-1)
		{
			min=val[1];
			index=1;
		}
		if(min>val[2] && val[2]!=-1 && val[2]!=0)
		{
			min=val[2];
			index=2;
		}
		if(min>val[3] && val[3]!=-1 && val[3]!=0)
		{
			min=val[3];
			index=3;
		}

		if(index==0)
			start_index[1]++;
		if(index==1)
			start_index[1]--;
		if(index==2)
			start_index[0]++;
		if(index==3)
			start_index[0]--;
		}
			//0 = forward
			//1 = left
			//2 = right
			//3 = rotate
		if(face==1)
		{
			if(index==0)
			{
				path[path_count++]=1;
				face=2;
			}
			else if(index==1)
			{
				path[path_count++]=2;
				face=-2;
			}
			else if(index==2)
			{
				path[path_count++]=0;
				face=1;
			}
			else
			{
				path[path_count++]=3;
				face=-1;
			}
		}
		else if(face==-1)
		{
			if(index==0)
			{
				path[path_count++]=2;
				face=2;
			}
			else if(index==1)
			{
				path[path_count++]=1;
				face=-2;
			}
			else if(index==2)
			{
				path[path_count++]=3;
				face=1;
			}
			else
			{
				path[path_count++]=0;
				face=-1;
			}
		}
		else if(face==2)
		{
			if(index==0)
			{
				path[path_count++]=0;
				face=2;
			}
			else if(index==1)
			{
				path[path_count++]=3;
				face=-2;
			}
			else if(index==2)
			{
				path[path_count++]=2;
				face=1;
			}
			else
			{
				path[path_count++]=1;
				face=-1;
			}
		}
		else
		{
			if(index==0)
			{
				path[path_count++]=3;
				face=2;
			}
			else if(index==1)
			{
				path[path_count++]=0;
				face=-2;
			}
			else if(index==2)
			{
				path[path_count++]=1;
				face=1;
			}
			else
			{
				path[path_count++]=2;
				face=-1;
			}			
		}
		count++;

		if(found==1)
			break;
	}
	path[path_count]=-1;
}

void move_straight()
{
 int hold=systick_get_ms()+10;
 int hold2=systick_get_ms()+50;
L=60;
R=60;

do
	{		
		r_rev = nxt_motor_get_count(NXT_PORT_B);	 			/* Read Rotation Sensor           */
		l_rev = nxt_motor_get_count(NXT_PORT_C);	 			/* Read Rotation Sensor           */
				
			if(hold<systick_get_ms())
			{
			 if(l_rev-r_rev>=10)
				{
					L=L-4;
					//R=R+2;
					if(L<20)
						L=20;
					// if(R>40)
						// R=40;
				}		
				else if(l_rev-r_rev>5)
				 {
					L=L-3;
					//R=R+1;
					if(L<20)
						L=20;
				}		
				else if(l_rev-r_rev>3)
				{
					L=L-1;
					//R=R+1;
					if(L<i)
						L=i;
				}		
				else if(r_rev-l_rev>=10)
				{					
					R=R-3;
					//L=L+1;
					 if(R<20)
						R=20;									
				}
				else if (l_rev-r_rev>5)
				{
					L=L-1;
					if(L<20)
						L=20;
				
				}				
				
				hold=systick_get_ms()+20;
			 }						

			 
		nxt_motor_set_speed(NXT_PORT_B, R, 1); 			/* Set motor speed for B and C to RN */
		nxt_motor_set_speed(NXT_PORT_C, L, 1);
		
		if(systick_get_ms()>hold2)
			{
				L=64;
				R=60;
				hold2=systick_get_ms()+50;
			}
						
	}while(r_rev<=REV);	
}

void move_left()
{
	do
	{
		r_rev = nxt_motor_get_count(NXT_PORT_B);	 			/* Read Rotation Sensor           */
		l_rev = nxt_motor_get_count(NXT_PORT_C);	 			/* Read Rotation Sensor           */
		
		
		nxt_motor_set_speed(NXT_PORT_B, R, 1); 			/* Set motor speed for B and C to RN */
		nxt_motor_set_speed(NXT_PORT_C, L, 1);	
	}while(R_REV>r_rev && L_REV<l_rev);
	
	nxt_motor_set_speed(NXT_PORT_B, 0, 1); 			/* Set motor speed for B and C to RN */
	nxt_motor_set_speed(NXT_PORT_C, 0, 1);				
}

void move_right()
{
	do
	{
		r_rev = nxt_motor_get_count(NXT_PORT_B);	 			/* Read Rotation Sensor           */
		l_rev = nxt_motor_get_count(NXT_PORT_C);	 			/* Read Rotation Sensor           */
		
		
		nxt_motor_set_speed(NXT_PORT_B, R, 1); 			/* Set motor speed for B and C to RN */
		nxt_motor_set_speed(NXT_PORT_C, L, 1);	
	}while(R_REV<r_rev && L_REV>l_rev);
	
	nxt_motor_set_speed(NXT_PORT_B, 0, 1); 			/* Set motor speed for B and C to RN */
	nxt_motor_set_speed(NXT_PORT_C, 0, 1);				
}

#define MAX_OBSTACLES 25

TASK(Task1)
{

	// int light;
	// int sonar;

	//int num_obstacles=13;

//	const int cols=12;
//	const int rows=10;

	double obstacle[MAX_OBSTACLES][2] = /* obstacle locations */
	{{0.305, 0.305},{0.305, 0.61},{0.61, 0.305},{0.61, 0.61},
	{0.61, 2.134},{ 0.61, 2.438},
	{1.524, 1.219},{1.524, 1.524},{1.524, 1.829},
	{2.438, 0.61},{2.743, 0.61},{3.048, 0.61},
	{2.438, 2.438},
	{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
	{-1,-1},{-1,-1},{-1,-1}};

	 start[0] = 0.305;
	 start[1]= 1.524; /* start location */
	 goal[0] = 2.743; 
	 goal[1]=1.524; /* goal location */

	 for(i=rows-1;i<=0;i--)
		for(j=0;j<cols;j++)
			map[i][j]=0;
	//int map[rows][cols]={{0}};
	
	for(i=0;i<MAX_OBSTACLES;i++)
	{
		int hold1=-1;
		int hold2=-1;
		for(j=0;j<2;j++)
		{
			if(obstacle[i][j]==-1)
				continue;

			double temp=obstacle[i][j]/key;
			double val=temp;
			temp=ceil(temp);

			if(abs(val - temp)>0.2)
				temp=temp+1;

			if(j==0)
				hold1=temp;
			else
				hold2=temp;
		}
		if(hold1!=-1)
			map[hold1][hold2]=-1;
			
		if(hold1+1<rows)
		{
			map[hold1+1][hold2]=-1;
		}
		if(hold1-1>=0)
		{
			map[hold1-1][hold2]=-1;
		}
		if(hold2+1<cols)
		{
			map[hold1][hold2+1]=-1;
		}
		if(hold2-1>=0)
		{
			map[hold1][hold2-1]=-1;
		}

		hold1=-1;
		hold2=-1;
	}
	
	indexes(start,index1);
	map[index1[1]][index1[0]]=88;
	
	indexes(goal,index1);
	map[index1[1]][index1[0]]=99;

	//print();

	if(index1[1]+1<rows)
		map[index1[1]+1][index1[0]]=1;
	if(index1[1]-1>=0)
		map[index1[1]-1][index1[0]]=1;
	
	if(index1[0]+1<cols)
		map[index1[1]][index1[0]+1]=1;
	if(index1[0]-1>=0)
		map[index1[1]][index1[0]-1]=1;
		
	int search_num=1;
	for(k=0;k<120;k++)
	{
		for(i=rows-1;i>=0;i--)
		{
			for(j=0;j<cols;j++)
			{
				/*if( map[i][j]  !=0			&&
					map[i][j] !=88		&&
					map[i][j] !=99		&&
					map[i][j] !=-1)*/
				
				if(map[i][j]==search_num)
				{
					if(i+1<rows)
					{					
						if(map[i+1][j]==88)
							{ 
								found=1;
								break;
							}
						if(map[i+1][j]==0)
							map[i+1][j]=map[i][j]+1;
					}
					if(i-1>=0)
					{
						if(map[i-1][j]==88)
						{ 
								found=1;
								break;
						}

						if(map[i-1][j]==0)
							map[i-1][j]=map[i][j]+1;
					}
					if(j+1<cols)
					{
						if(map[i][j+1]==88)
						{ 
							found=1;
							break;
						}

						if(map[i][j+1]==0)
							map[i][j+1]=map[i][j]+1;
					}
					if(j-1>=0)
					{
						if(map[i][j-1]==88)
							{ 
								found=1;
								break;
							}
						if(map[i][j-1]==0)
							map[i][j-1]=map[i][j]+1;				
					}
					//print(map);
				}
			}			
			if(found==1)
			
			break;
		}
			if(found==1)
			break;
			search_num++;
	}
	generate_path();	
	path_count=0;
	// ////////////////////////////////
	// //print();

	// //int *path=(int*)malloc( sizeof(int)*search_num);
	 
	// path_count=0;
	
	// path[0]=0;
	// path[1]=0;	
	// path[2]=0;
	// path[3]=0;
	// path[4]=0;
	// path[5]=0;
	// path[6]=0;
	// path[7]=0;	
	// path[8]=0;
	// path[9]=0;
	// path[10]=1;
	// path[11]=0;
	// path[12]=1;
	// path[13]=2;
	// path[14]=0;
	
	// path[15]=-1;
	
	 
	 
	////////////////////////////////////////	
	// disp(0,"0:",path[0]);
	//disp(7,"1:",path[1]);
	// systick_wait_ms(2000);
	
	////////////////////////////////////////
	// while(path[path_count]!=-1)
	// {
		// disp(7,"path:",path[path_count]);
		// systick_wait_ms(400);
		// path_count++;
	// }
	
	path_count=0;	
	
	disp(7,"Start",0);	
	systick_wait_ms(2000);									/* Show display for 10 seconds       */
	
	int temp =0;
	
	REV=0;
	time_out=0;
	L=0;
	R=0;
	
 while(path[path_count]!=-1)	
	{
	R=30;
	L=30;
		
	disp(0,"path: ", path[path_count]);
	disp(7,"count", path_count);	
	//systick_wait_ms(500);									/* Show display for 10 seconds       */
	
	if(path[path_count]==0)
	{
		L=30;
		R=30;
		nxt_motor_set_count(NXT_PORT_B,0);	 			/* Read Rotation Sensor           */
		nxt_motor_set_count(NXT_PORT_C,0);	 			/* Read Rotation Sensor           */
		REV=1250;
	//time_out = systick_get_ms() + 5000;
		temp=0;
	}
		else if (path[path_count]==1)						//Turn left
	{
		//R = 40;
		R=40;
		L = -1*40;
		temp=1;
		
		// disp(0,"R:",R);
		// disp(1,"L:",L);
		// disp(7,"temp",temp);
		
		// systick_wait_ms(6000);
		
		L_REV=-535;
		R_REV=535;
		nxt_motor_set_count(NXT_PORT_B,0);	 			/* Read Rotation Sensor           */
		nxt_motor_set_count(NXT_PORT_C,0);	 			/* Read Rotation Sensor           */
		//time_out = systick_get_ms() + 1950;
	}
	
	else if (path[path_count]==2)						//Turn right
	{
		R=-1*40;
		L = 40;
		temp=2;
		
		// disp(0,"R:",R);
		// disp(1,"L:",L);
		// disp(7,"temp",temp);
		
		// systick_wait_ms(6000);
		
		L_REV=535;
		R_REV=-535;
		nxt_motor_set_count(NXT_PORT_B,0);	 			/* Read Rotation Sensor           */
		nxt_motor_set_count(NXT_PORT_C,0);	 			/* Read Rotation Sensor           */
		
		// R = -51;//-44;//
		// L = 18;
		// temp = 2;
		// //time_out = systick_get_ms() + 1950;
		// R_REV=-785;
		// L_REV=179;
		// nxt_motor_set_count(NXT_PORT_B,0);	 			/* Read Rotation Sensor           */
		// nxt_motor_set_count(NXT_PORT_C,0);	 			/* Read Rotation Sensor           */
	}
	
	else if (path[path_count]==3)
	{
		R = -51;//-44;//
		L = 18;
		temp = 2;
		//time_out = systick_get_ms() + 1950;
		R_REV=-785*2;
		L_REV=179*2;
		nxt_motor_set_count(NXT_PORT_B,0);	 			/* Read Rotation Sensor           */
		nxt_motor_set_count(NXT_PORT_C,0);	 			/* Read Rotation Sensor           */
	}

	if(temp==0)
	{	
		move_straight();
	}
	else
	{
		if(temp==1)
			move_left();					
		else
			move_right();
			
		systick_wait_ms(400);
		
		// L=40;
		// R=40;		

		// nxt_motor_set_count(NXT_PORT_B,0);	 			/* Read Rotation Sensor           */
		// nxt_motor_set_count(NXT_PORT_C,0);	 			/* Read Rotation Sensor           */
		// REV=220;
		// move_straight();	
		// systick_wait_ms(2000);
		//////////////////////
		L=30;
		R=30;		

		nxt_motor_set_count(NXT_PORT_B,0);	 			/* Read Rotation Sensor           */
		nxt_motor_set_count(NXT_PORT_C,0);	 			/* Read Rotation Sensor           */
		REV=1250;
		move_straight();		
	}		
		nxt_motor_set_count(NXT_PORT_B,0);	 			/* Read Rotation Sensor           */
		nxt_motor_set_count(NXT_PORT_C,0);	 			/* Read Rotation Sensor           */
		
		nxt_motor_set_speed(NXT_PORT_B, 0, 1);
		nxt_motor_set_speed(NXT_PORT_C, 0, 1);
				
		path_count++;		
	//	systick_wait_ms(3000);									/* Show display for 10 seconds       */
	}

	disp(0,"count: ",path[path_count]);	
	disp(7,"END",0);
	systick_wait_ms(20000);									/* Show display for 10 seconds       */
	
	TerminateTask();
}
