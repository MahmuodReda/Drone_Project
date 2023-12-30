#ifndef _PID_H
#define _PID_H


#include "main.h"

extern float max (float x ,float y , float z) ;
typedef enum
{

	pid_OK= 0x00U,
	pid_error= 0x01U,
	Out_of_range= 0x02U,
	Control_done= 0x03U,
} pid_Status;

typedef enum
{

	pidA_OK= 0x00U,
	pidA_error= 0x01U,
	Out_of_rangeA= 0x02U,
	ControlA_done= 0x03U,
} pidA_Status;

typedef enum
{

	pidB_OK= 0x00U,
	pidB_error= 0x01U,
	Out_of_rangeB= 0x02U,
	ControlB_done= 0x03U,
} pidB_Status;
typedef enum
{

	pidC_OK= 0x00U,
	pidC_error= 0x01U,
	Out_of_rangeC= 0x02U,
	ControlC_done= 0x03U,
} pidC_Status;

pid_Status PID_CONTROL(float kp, float ki, float kd,int Target_Vaue ,int Real_Value,int Resolution ,float dt, float *p ,float *i,float *d,float *error);
pidA_Status PIDA_CONTROL(float kp, float ki, float kd,int Target_Vaue ,int Real_Value,int Resolution ,float dt, float *p ,float *i,float *d,float *error);
pidC_Status PIDC_CONTROL(float kp, float ki, float kd,int Target_Vaue ,int Real_Value,int Resolution ,float dt, float *p ,float *i,float *d,float *error);
pidB_Status PIDB_CONTROL(float kp, float ki, float kd,int Target_Vaue ,int Real_Value,int Resolution ,float dt, float *p ,float *i,float *d,float *error);



#endif
