#ifndef _PID_Tune_Online_H
#define _PID_Tune_Online_H

#include "main.h"
extern UART_HandleTypeDef huart5;
extern int8_t dw[5];

extern void PID_Tune_Online(float *kpa,float *kia ,float *kda ,float *kpb ,float *kib ,float *kdb );


#endif
