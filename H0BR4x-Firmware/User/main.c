/*
 BitzOS (BOS) V0.2.7 - Copyright (C) 2017-2022 Hexabitz
 All rights reserved

 File Name     : main.c
 Description   : Main program body.
 */
/* Includes ------------------------------------------------------------------*/
#include "BOS.h"
#include <math.h>

#define ok2 0
#define ok1 0
#define ok3 0

/* Private variables ---------------------------------------------------------*/
#if (ok1==1)
 float  xa, ya, za,accy,c,accx;
 float phihat_acc,thetahat_acc,phihat_x,phihat_y;
float x, y, z,phihat,thetahat,tt;

int xaa,zaa,yaa;
float phi_X,theta_y;
float gyrox ,gyroy,gyroz ;
#endif
//float phidot_rps, thetadot_rps,cc,end_time,Loop_time,start_time,zz,yz,xz,kl,kkk ,hh,h,g,kk,aaa,bb;
//float x,y,z,xa,ya,za;
//float q,w,qq,ww,rt,rrt;
//HAL_StatusTypeDef j;
//uint8_t mn[2];
int we;
/* Private function prototypes -----------------------------------------------*/

/* Main function ------------------------------------------------------------*/

#if (ok3==1)



 typedef struct {

	float phi_r;
	float theta_r;

	float P[2][2];

	float Q[2];
	float R[3];


} EKF;
EKF ekf;

void EKF_Init(EKF *ekf, float P[2], float Q[2], float R[3]) {

	/* Reset state estimates */
	ekf->phi_r		= 0;
	ekf->theta_r 	= 0;

	/* Initialise state covariance matrix */
	ekf->P[0][0] = P[0]; ekf->P[0][1] = 0.0f;
	ekf->P[1][0] = 0.0f; ekf->P[1][1] = P[1];

	/* Set process and measurement noise */
	ekf->Q[0] = Q[0];
	ekf->Q[1] = Q[1];

	ekf->R[0] = R[0];
	ekf->R[1] = R[1];
	ekf->R[2] = R[2];

}

void EKF_Predict(EKF *ekf, float p_rps, float q_rps, float r_rps, float sampleTime_s) {
//	ekf->phi_r		= 1;
//	ekf->theta_r 	= 1;
//	SampleGyroDPS(&x, &y, &z);
//	p_rps=x-3.5;
//	q_rps=y+4.8;
//	r_rps=z+3.6;
//	p_rps=10;
//	q_rps=10;
//
//	r_rps=10;

	rrt=p_rps;
	/* Pre-compute trigonometric quantities */
	float sp = sinf(ekf->phi_r);
	float cp = cosf(ekf->phi_r);
	float tt = tanf(ekf->theta_r);

	/* Compute state transition function dx/dt = f(x,u) */
	float dphidt	= p_rps + tt * (q_rps * sp + r_rps * cp);
	float dthetadt	= 				q_rps * cp - r_rps * sp;

	/* Update state estimates (x(n+1) = x(n) + T * dx/dt) */
	ekf->phi_r 		+= sampleTime_s * dphidt;
	ekf->theta_r	+= sampleTime_s * dthetadt;

	/* Re-compute trigonometric quantities */
	sp 			= sinf(ekf->phi_r);
	cp 			= cosf(ekf->phi_r);
	tt 			= tanf(ekf->theta_r);
	float ctInv = 1.0f / cosf(ekf->theta_r);

	/* Compute Jacobian of state transition function A(x,u) = df(x,u)/dx */
	float A[2][2] =
	{ 	{ tt * (q_rps * cp - r_rps * sp), (q_rps * sp + r_rps * cp) * ctInv * ctInv },
		{ -(q_rps * sp + r_rps * cp), 	  0.0f } };

	/* Update state covariance matrix P(n+1) = P(n) + T * (A * P(n) + P(n) * A' + Q) (note that A[1][1] = 0!) */
	float Pnew[2][2] =
	{	{A[0][0] * ekf->P[0][0] + A[0][1] * ekf->P[1][0] + ekf->P[0][0] * A[0][0] + ekf->P[0][1] * A[1][0] + ekf->Q[0], A[0][0] * ekf->P[0][1] + A[0][1] * ekf->P[1][1] + ekf->P[0][0] * A[0][1]},
		{A[1][0] * ekf->P[0][0] + 						   ekf->P[1][0] * A[0][0] + ekf->P[1][1] * A[1][0], 			A[1][0] * ekf->P[0][1] + 						  ekf->P[1][0] * A[0][1] + ekf->Q[1]} };

	ekf->P[0][0] += sampleTime_s * Pnew[0][0]; ekf->P[0][1] += sampleTime_s * Pnew[0][1];
	ekf->P[1][0] += sampleTime_s * Pnew[1][0]; ekf->P[1][1] += sampleTime_s * Pnew[1][1];
	q=ekf->phi_r *57;
	w=ekf->theta_r *57;
}

void EKF_Update(EKF *ekf, float ax_mps2, float ay_mps2, float az_mps2) {

	/* Normalise accelerometer readings */
	float accNormFactor = 1.0f / sqrtf(ax_mps2 * ax_mps2 + ay_mps2 * ay_mps2 + az_mps2 * az_mps2);
rt=az_mps2;
	float ax_norm 		= ax_mps2 * accNormFactor;
	float ay_norm 		= ay_mps2 * accNormFactor;
	float az_norm 		= az_mps2 * accNormFactor;

	/* Compute Jacobian of output function C(x,u) = dh(x,u)/dx */
	float sp = sinf(ekf->phi_r);
	float cp = cosf(ekf->phi_r);
	float st = sinf(ekf->theta_r);
	float ct = cosf(ekf->theta_r);


	float C[3][2] = {	{0.0f, 		ct},
						{-cp * ct, 	sp * st},
						{sp * ct, 	cp * st} };

	/* Compute Kalman gain K = P * C' * (R + C * P * C ')^-1 in steps (note that C[0][0] = 0!) */

	/* P * C'*/
	float PCt[2][3] = { {ekf->P[0][1] * C[0][1], ekf->P[0][0] * C[1][0] + ekf->P[0][1] * C[1][1], ekf->P[0][0] * C[2][0] + ekf->P[0][1] * C[2][1]},
						{ekf->P[1][1] * C[0][1], ekf->P[1][0] * C[1][0] + ekf->P[1][1] * C[1][1], ekf->P[1][0] * C[2][0] + ekf->P[1][1] * C[2][1]} };

	/* R + C * P * C' */
	float RCPCt[3][3] = { 	{C[0][1] * PCt[1][0] + ekf->R[0],			C[0][1] * PCt[1][1], 									C[0][1] * PCt[1][2]},
							{C[1][0] * PCt[0][0] + C[1][1] * PCt[1][0],	C[1][0] * PCt[0][1] + C[1][1] * PCt[1][1] + ekf->R[1],	C[1][0] * PCt[0][2] + C[1][1] * PCt[1][2]},
							{C[2][0] * PCt[0][0] + C[2][1] * PCt[1][0],	C[2][0] * PCt[0][1] + C[2][1] * PCt[1][1],				C[2][0] * PCt[0][2] + C[2][1] * PCt[1][2] + ekf->R[2]} };

	/* inv(R + C * P * C') */
	float detMatInv = 1.0f / (RCPCt[0][0] * (RCPCt[2][2] * RCPCt[1][1] - RCPCt[2][1] * RCPCt[1][2])
					- RCPCt[1][0] * (RCPCt[2][2] * RCPCt[0][1] - RCPCt[2][1] * RCPCt[0][2])
					+ RCPCt[2][0] * (RCPCt[1][2] * RCPCt[0][1] - RCPCt[1][1] * RCPCt[0][2]));

	float matInv[3][3] = { 	{	RCPCt[2][2] * RCPCt[1][1] - RCPCt[2][1] * RCPCt[1][2], -(	RCPCt[2][2] * RCPCt[0][1] - RCPCt[2][1] * RCPCt[0][2]), 	RCPCt[1][2] * RCPCt[0][1] - RCPCt[1][1] * RCPCt[0][2] },
							{-(	RCPCt[2][2] * RCPCt[1][0] - RCPCt[2][0] * RCPCt[1][2]), 	RCPCt[2][2] * RCPCt[0][0] - RCPCt[2][0] * RCPCt[0][2], -(	RCPCt[1][2] * RCPCt[0][0] - RCPCt[1][0] * RCPCt[0][2]) },
							{	RCPCt[2][1] * RCPCt[1][0] - RCPCt[2][0] * RCPCt[1][1], -(	RCPCt[2][1] * RCPCt[0][0] - RCPCt[2][0] * RCPCt[0][1]), 	RCPCt[1][1] * RCPCt[0][0] - RCPCt[1][0] * RCPCt[0][1]} };

	for (unsigned int i = 0; i < 3; i++) {

		for (unsigned int j = 0; j < 3; j++) {

			matInv[i][j] *= detMatInv;

		}

	}

	/* C' * inv(R + C * P * C') */
	float CtmatInv[2][3] = { 	{						  C[1][0] * matInv[1][0] + C[2][0] * matInv[2][0], 							C[1][0] * matInv[1][1] + C[2][0] * matInv[2][1], 						  C[1][0] * matInv[1][2] + C[2][0] * matInv[2][2]},
								{C[0][1] * matInv[0][0] + C[1][1] * matInv[1][0] + C[2][1] * matInv[2][0], C[0][1] * matInv[0][1] + C[1][1] * matInv[1][1] + C[2][1] * matInv[2][1], C[0][1] * matInv[0][2] + C[1][1] * matInv[1][2] + C[2][1] * matInv[2][2]} };

	/* K = P * C' * inv(R + C * P * C') */
	float K[2][3] = { 	{ekf->P[0][0] * CtmatInv[0][0] + ekf->P[0][1] * CtmatInv[1][0], ekf->P[0][0] * CtmatInv[0][1] + ekf->P[0][1] * CtmatInv[1][1], ekf->P[0][0] * CtmatInv[0][2] + ekf->P[0][1] * CtmatInv[1][2]},
						{ekf->P[1][0] * CtmatInv[0][0] + ekf->P[1][1] * CtmatInv[1][0], ekf->P[1][0] * CtmatInv[0][1] + ekf->P[1][1] * CtmatInv[1][1], ekf->P[1][0] * CtmatInv[0][2] + ekf->P[1][1] * CtmatInv[1][2]} };

	/* Update state covariance matrix P(n+1) = (I - K * C) * P(n) */
	float IminKC[2][2] = { 	{1.0f - (K[0][1] * C[1][0] + K[1][0] * C[2][0]), -(K[0][1] * C[1][1] + K[1][0] * C[2][1])},
							{-(K[1][1] * C[1][0] + K[1][2] * C[2][0]), 1.0f - (K[1][1] * C[1][1] + K[1][2] * C[2][1])} };

	float Pnew[2][2] = { 	{IminKC[0][0] * ekf->P[0][0] + IminKC[0][1] * ekf->P[1][0], IminKC[0][0] * ekf->P[0][1] + IminKC[0][1] * ekf->P[1][1]},
							{IminKC[1][0] * ekf->P[0][0] + IminKC[1][1] * ekf->P[1][0], IminKC[1][0] * ekf->P[0][1] + IminKC[1][1] * ekf->P[1][1]} };

	ekf->P[0][0] = Pnew[0][0]; ekf->P[0][1] = Pnew[0][1];
	ekf->P[1][0] = Pnew[1][0]; ekf->P[1][1] = Pnew[1][1];

	/* Compute output function h(x,u) */
	float h[3] = {	 sinf(ekf->theta_r),
					-cosf(ekf->theta_r) * sinf(ekf->phi_r),
					-cosf(ekf->theta_r) * cosf(ekf->phi_r) };

	/* Update state estimate x(n+1) = x(n) + K * (y - h) */
	ekf->phi_r 		= K[0][0] * (ax_norm - h[0]) + K[0][1] * (ay_norm - h[1]) + K[0][2] * (az_norm - h[2]);
	ekf->theta_r 	= K[1][0] * (ax_norm - h[0]) + K[1][1] * (ay_norm - h[1]) + K[1][2] * (az_norm - h[2]);

}
#endif
//float o[2][2],i[3],u[2];
//float a ,b;
//float dthetadt;
//float dphidt;
//float xx,yy,zz ;
int main(void){
	
Module_Init( );		//Initialize Module &  BitzOS
	
	//Don't place your code here.
	for(;;){
	}
}

/*--------------------* ---------------------------------------*/

/* User Task */
void UserTask(void *argument){
#if (ok3==1)
o[0][0]=1;
o[0][1]=1;
o[1][0]=1;
o[1][1]=1;
i[0]=100;
i[1]=100;
i[2]=0.1;
u[0]=0.1;
u[1]=0.1;
	EKF_Init(&ekf, o, u, i);
#endif

#if (ok1==1)

SampleAccG(&x, &y, &z);
phihat_x=atan2(y,z)*57.3;
thetahat_acc=atan2(x,z)*57.3;
gyrox=phihat_x;
gyroy=thetahat_acc;
Loop_time=0;
start_time=0;


#endif





	// put your code here, to run repeatedly.
	while(1){
		we++;

#if (ok3==1)
SampleAccG(&x, &y, &z);
a=atan2(y,z)*57.3;
b=atan2(x,z)*57.3;
		start_time=HAL_GetTick();
		Loop_time=start_time-end_time;
		end_time=HAL_GetTick();
			SampleGyroDPS(&x, &y, &z);
			xx=x-3.5;
			yy=y+4.8;
			zz=z+3.6;

EKF_Predict(&ekf , xx, yy, zz, Loop_time/1000);

SampleAccG(&xa, &ya, &za);
EKF_Update(&ekf, xa, ya, za);
//q= ekf->phi_r  *57;
//w= ekf->theta_r  *57;
qq=q;
ww=w;

//SampleGyroDPS(&x, &y, &z);
////a=0;
////b=0;
//float sp = sinf(a);
//float cp = cosf(a);
//float tt = tanf(b);
//
///* Compute state transition function dx/dt = f(x,u) */
// dphidt	 = (x + tt * (y* sp + z* cp))-3.9;
// dthetadt 	= 				(y * cp - z * sp)+4.8;
//
//a 		+= (Loop_time/1000) * dphidt;
//b	+= (Loop_time/1000) * dthetadt;
#endif



#if (ok1==1)

//j=HAL_UART_Transmit(&huart1, &mn, 2, 100);
start_time=HAL_GetTick();
Loop_time=start_time-end_time;
end_time=HAL_GetTick();
SampleAccG(&x, &y, &z);
accx=atan2(y,z)*57.3;
accy=-atan2(x,z)*57.3;


//		//_______________________________________________________________________//


SampleAccG (&x, &y, &z);
kl=powf(x,2)+powf(z,2);
kk=kl+powf(y,2);
g=sqrtf(kk);
			if(g<1.018&&g>0.99)
			{

                Delay_ms(10);
                SampleAccG (&x, &y, &z);
                kl=powf(x,2)+powf(z,2);
                kk=kl+powf(y,2);
                g=sqrtf(kk);
				if(g<1.018&&g>0.99){
					phi_X=atan2(y,z)*57.3;
					theta_y=-atan2(x,z)*57.3;

				}
}
	//_____________________________________________________________________________//

		SampleGyroDPS(&xa, &ya, &za);
			    xaa=xa-3.5;
		        yaa=ya+4.8;
		        zaa=za+3.6;
		        phi_X=phi_X+(Loop_time/1000)*xaa;
		        theta_y=theta_y+(Loop_time/1000)*yaa;



bb=bb+xaa;

            gyrox=gyrox+(Loop_time/1000)*xaa;
            gyroy=gyroy+(Loop_time/1000)*yaa;
            zz=zz+(Loop_time/1000)*zaa;
//            mn[0]=phi_X;
//            mn[1]=theta_y;

//  		 // ___________________________________________________________________________//

//            b=tanf(thetahat);
//
//phidot_rps=xaa + tanf(thetahat) * (sinf(phihat) * yaa + cosf(phihat) * zaa);
//
//thetadot_rps=                      cosf(phihat) * yaa - sinf(phihat) * zaa;
//
//
//
//		        	 			phihat=phihat+(nn/1000)* phidot_rps;
//
//		        				thetahat=thetahat+(nn/1000)* thetadot_rps;
//
//		        				a=0.5 * phihat_x + phihat * 0.5;




  		  //__________________________________________________________________________//

#endif

}
}




























#if (ok2==1)
int aaa;
uint32_t *o ;
float x, y, z,phihat,thetahat,tt;
volatile uint32_t mynum = 0x12ABCDEF;


uint8_t *data ;
// Define local variables: LED state (ON/OFF), intensity and color (global or static if inside a function)

//
	volatile bool mybool = true;
BOS_Status l ;
uint8_t flag = 0;
#endif
//	  AddBOSvar(FMT_UINT8,  (uint32_t) &flag);
#if (ok2==1)

SampleAccG(&x, &y, &z);
flag = z*10;
l=WriteRemote(2, (uint32_t) &flag, 1, FMT_UINT8, 0);
aaa++;
//	Delay_ms(1000);  aaa++;
//writePxMutex(5, &data, 8, 0xff, 0xffff);
#endif



//
//if(nn>50){

////					bb=powf(y,2);
			//	aaa=bb*(kl/90);
//		        	    phidot_rps=xaa +tanf(thetahat) * (sinf(phihat) * yaa + cosf(phihat) * zaa);
//		        		thetadot_rps=                   cosf(phihat) * yaa - sinf(phihat) * zaa;
//		        			    a= cosf(phihat);
//		        		        b=tanf(thetahat);
//		        			    c=sinf(phihat);
//		        				phihat=phihat+(nn/1000)* phidot_rps;
//		        				thetahat=thetahat+(nn/1000)* thetadot_rps;
//		        		nn=0;
//		        			   cc=phihat*57;
////		        			   ca=thetahat*57;
//}



//}




/*-----------------------------------------------------------*/

//		tt=HAL_GetTick();
//
//if(nn>=7)
//{


//	thetahat_acc=asinf(x/9.81);

//	hh=1.01849592-g;
//	h=sqrtf(hh);

//	h=kkk - (powf(z,2)+powf(x,2));
//	hh=sqrtf(h);

//	phihat_acc=atan2(hh,z)*57.3;

//	    SampleGyroDPS (&xa, &ya, &za);
//	    xaa=xa-3.5;
//        yaa=ya+5.7;
//        zaa=za+3.6;
//    xz=xz+(nn/1000)*xaa;
//	    yz=yz+(nn/1000)*yaa;
//	    zz=zz+(nn/1000)*zaa;
//	    a=0.5 * phihat_acc + xz * 0.5;


//	    a= cosf(phihat);
//        b=tanf(thetahat);
//	    c=sinf(phihat);
//	    phidot_rps=xaa +tanf(thetahat) * (sinf(phihat) * yaa + cosf(phihat) * zaa);
//		thetadot_rps=                   cosf(phihat) * yaa - sinf(phihat) * zaa;
//		phihat=phihat+(nn/1000)* phidot_rps;
//		thetahat=thetahat+(nn/1000)* thetadot_rps;
 //phihat=zz*0.5+phihat_acc*0.95;
//		phihat=0.04*phihat_acc+0.96*(phihat+(nn/1000)*phidot_rps);
//		thetahat=0.04*thetahat_acc+0.96*(thetahat+(nn/1000)*thetadot_rps);
	//	Delay_ms(500);
//	   cc=phihat*57;
//	   nn=0;}
//
//pr=HAL_GetTick();
//		nn+=pr-tt;
