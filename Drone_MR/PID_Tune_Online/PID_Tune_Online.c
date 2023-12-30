#include "‏‏PID_Tune_Online .h"

float z=0,x=0,y=0;
int8_t dw[5];


void PID_Tune_Online(float *kpa,float *kia ,float *kda ,float *kpb ,float *kib ,float *kdb )
{
	  HAL_UART_Receive_DMA(&huart5, &dw[0], 5);


		  if(dw[0]==112)
		  {


			  if(dw[1]==97)
			  {
				  x=dw[2]-48;
					  x=x;

					  y=dw[3]-48;
					  y=y/10;
					  z=dw[4]-48;

					  z=z/100;
					  *kpa=(x+y+z);
			  }

			  if(dw[1]==98)
				  {
					  x=dw[2]-48;
						  x=x;

						  y=dw[3]-48;
						  y=y/10;
						  z=dw[4]-48;

						  z=z/100;
						  *kpb=(x+y+z);
				  }








		  }
		  else if(dw[0]==105)
		  {


			  if(dw[1]==97)
			  {
				  x=dw[2]-48;
					  x=x;

					  y=dw[3]-48;
					  y=y/10;
					  z=dw[4]-48;

					  z=z/100;
					 *kia=(x+y+z);
			  }

			  if(dw[1]==98)
				  {
					  x=dw[2]-48;
						  x=x;

						  y=dw[3]-48;
						  y=y/10;
						  z=dw[4]-48;

						  z=z/100;
						  *kib=(x+y+z);
				  }


		  }

		  else if(dw[0]==100)
		  {
			  if(dw[1]==97)
			  {
				  x=dw[2]-48;
					  x=x;

					  y=dw[3]-48;
					  y=y/10;
					  z=dw[4]-48;

					  z=z/100;
					  *kda=(x+y+z);
			  }

			  if(dw[1]==98)
				  {
					  x=dw[2]-48;
						  x=x;

						  y=dw[3]-48;
						  y=y/10;
						  z=dw[4]-48;

						  z=z/100;
						  *kdb=(x+y+z);
				  }

		  }

		  else {
			  dw[3]=dw[4]=dw[2]=48;


		  }






}

