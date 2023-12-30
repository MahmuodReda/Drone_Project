#include "PID.h"

	int errorr,perror ;

	float iu,du,doo;


	//-----------------------------------------------------/
	int errorra,perrora ;

	float iua,dua,dooa;

	//----------------------------------------------------/

	int errorrc,perrorc ;

	float iuc,duc,dooc;

	//----------------------------------------------------/

	int errorrb,perrorb ;

	float iub,dub,doob;

	//----------------------------------------------------/

float max (float x ,float y , float z)
{
		 if(x>=y)

			 {x=y;}
		 else {x=x;}

		 if(x<=z)

		 { x=z;}
		 else {x=x;}
		 return x;

	}

pid_Status PID_CONTROL(float kp, float ki, float kd,int Target_Vaue ,int Real_Value,int Resolution ,float dt, float *p ,float *i,float *d,float *error)

{



	if(dt==0||Resolution==0)
	{

		return pid_error ;
	}

	if(Target_Vaue>=20||Target_Vaue<=-20)
	{

		return Out_of_range;
	}

	 errorr=((Target_Vaue-Real_Value)/Resolution)*Resolution;

				*p=errorr*kp;
				iu+=errorr*dt;
				*i=ki*iu;
				iu=max(iu, 50, -50);
				du=(errorr-perror)/dt;
				doo=du*kd;
				doo=max(doo, 100,-100);
				*error=errorr;
				perror=errorr;
				*d=doo;
				if(errorr==0)
				{

					return Control_done;
				}


	  	 		return pid_OK ;

}

pidA_Status PIDA_CONTROL(float kp, float ki, float kd,int Target_Vaue ,int Real_Value,int Resolution ,float dt, float *p ,float *i,float *d,float *error)

{



	if(dt==0||Resolution==0)
	{

		return pid_error ;
	}

	if(Target_Vaue>=20||Target_Vaue<=-20)
	{

		return Out_of_range;
	}

	 errorra=((Target_Vaue-Real_Value)/Resolution)*Resolution;

	 if (errorra>=2||errorra<=-2)
	 {
				*p=errorra*kp;
				iua+=errorra*dt;
				iua=max(iua, 75,-75);
				*i=ki*iua;

				dua=(errorra-perrora)/dt;
				doo=dua*kd;
				doo=max(doo, 100,-100);
				*error=errorra;
				perrora=errorra;
				*d=doo;
	 }
				if(errorra==0)
				{

					return Control_done;
				}


	  	 		return pid_OK ;

}

pidB_Status PIDB_CONTROL(float kp, float ki, float kd,int Target_Vaue ,int Real_Value,int Resolution ,float dt, float *p ,float *i,float *d,float *error)

{



	if(dt==0||Resolution==0)
	{

		return pid_error ;
	}

	if(Target_Vaue>=20||Target_Vaue<=-20)
	{

		return Out_of_range;
	}

	 errorrb=((Target_Vaue-Real_Value)/Resolution)*Resolution;

	 if (errorrb>=2||errorrb<=-2)
	 {
				*p=errorrb*kp;
				iub+=errorrb*dt;
				iub=max(iub, 75,-75);
				*i=ki*iua;

				dub=(errorrb-perrorb)/dt;
				doo=dub*kd;
				doo=max(doo, 100,-100);
				*error=errorrb;
				perrorb=errorrb;
				*d=doo;
	 }
				if(errorrb==0)
				{

					return Control_done;
				}


	  	 		return pid_OK ;

}

pidC_Status PIDC_CONTROL(float kp, float ki, float kd,int Target_Vaue ,int Real_Value,int Resolution ,float dt, float *p ,float *i,float *d,float *error)

{



	if(dt==0||Resolution==0)
	{

		return pid_error ;
	}

	if(Target_Vaue>=20||Target_Vaue<=-20)
	{

		return Out_of_range;
	}

	 errorrc=((Target_Vaue-Real_Value)/Resolution)*Resolution;

	 if (errorrc>=2||errorrc<=-2)
	 {
				*p=errorrc*kp;
				iuc+=errorrc*dt;
				iuc=max(iuc, 75,-75);
				*i=ki*iuc;

				duc=(errorrc-perrorc)/dt;
				doo=duc*kd;
				doo=max(doo, 100,-100);
				*error=errorrc;
				perrorc=errorrc;
				*d=doo;
	 }
				if(errorrc==0)
				{

					return Control_done;
				}


	  	 		return pid_OK ;

}



