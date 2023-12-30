#include "../ESC_Initializationn/ESC_Initialization.h"

//#include "ESC_Initialization.h"

int counter ;



void ESC_Initialization(void)
{



	  htim2.Instance->CCR1 = 999;
	  htim2.Instance->CCR2 = 999;
	  htim2.Instance->CCR3 = 999;
	  htim2.Instance->CCR4 = 999;
	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);


	  while( counter<1)
	  {

	  	 				  HAL_Delay(5000);
	  	 				  if(htim2.Instance->CCR1==999)
	  	 			 	  {
	  	 			 	  htim2.Instance->CCR1 = 500;
	  	 			 	  htim2.Instance->CCR2 = 500;
	  	 			 	  htim2.Instance->CCR3 = 500;
	  	 			 	  htim2.Instance->CCR4 = 500;
	  	 			 	  }
	  	 			 	  else
	  	 			 	  {
//	  	 			 	  htim2.Instance->CCR1 = 999;
//	  	 			 	  htim2.Instance->CCR2 = 999;
//	  	 			 	  htim2.Instance->CCR3 = 999;
//	  	 			 	  htim2.Instance->CCR4 = 999;
//	  	 			 		  HAL_Delay(1000);
//
//	  	 			 	  htim2.Instance->CCR1 = 800;
//	  	 			 	  htim2.Instance->CCR2 = 800;
//	  	 			 	  htim2.Instance->CCR3 = 800;
//	  	 			 	  htim2.Instance->CCR4 = 800;
//	  	 			 		  HAL_Delay(1000);
//	  	 			 	  htim2.Instance->CCR1 = 600;
//	  	 			 	  htim2.Instance->CCR2 = 700;
//	  	 			 	  htim2.Instance->CCR3 = 700;
//	  	 			 	  htim2.Instance->CCR4 = 700;
	  	 			 		 HAL_Delay(1000);
	  	 			 	  htim2.Instance->CCR1 = 600;
	  	 			 	  htim2.Instance->CCR2 = 600;
	  	 			 	  htim2.Instance->CCR3 = 600;
	  	 			 	  htim2.Instance->CCR4 = 600;
	  	 			 	 	 HAL_Delay(1000);
	  	 			 	  htim2.Instance->CCR1 = 550;
	  	 			 	  htim2.Instance->CCR2 = 550;
	  	 			 	  htim2.Instance->CCR3 = 550;
	  	 			 	  htim2.Instance->CCR4 = 550;
	  	 			 	  HAL_Delay(2000);
	  	 			 	  htim2.Instance->CCR1 = 600;HAL_Delay(100);
	  	 			 	  htim2.Instance->CCR2 = 600;HAL_Delay(100);
	  	 			 	  htim2.Instance->CCR3 = 600;HAL_Delay(100);
	  	 			 	  htim2.Instance->CCR4 = 600;HAL_Delay(100);
	  	 			 	  HAL_Delay(1000);
	  	 			 	  htim2.Instance->CCR1 = final_speed; HAL_Delay(100);
	  	 			 	  htim2.Instance->CCR2 = final_speed; HAL_Delay(100);
	  	 			 	  htim2.Instance->CCR3 = final_speed; HAL_Delay(100);
	  	 			 	  htim2.Instance->CCR4 = final_speed; HAL_Delay(100);
	  	 			 	counter++;
	  	 			 	  }

	  }
}
