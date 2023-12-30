#include "Draw_Parameter.h"

int p_parameter;
int dr[number];
int tt[number];
int i ;

int p_parameter1;
int dr1[number];
int tt1[number];
int i1 ;

int p_parameter2;
int dr2[number];
int tt2[number];
int i2 ;

void drow (int parameter,int t)
{
	if(parameter!=p_parameter)
	{
		dr[i]=parameter;
		tt[i]=t;
		i++;
		if(i==number)i=0;

	}
	p_parameter=parameter;
}







void drow1 (int parameter1,int t1)
{
	if(parameter1!=p_parameter1)
	{
		dr1[i]=parameter1;
		tt1[i]=t1;
		i1++;
		if(i1==number)i1=0;

	}
	p_parameter1=parameter1;
}







void drow2 (int parameter2,int t2)
{
	if(parameter2!=p_parameter2)
	{
		dr2[i]=parameter2;
		tt2[i]=t2;
		i2++;
		if(i2==number)i2=0;

	}
	p_parameter2=parameter2;
}



