#include "controller.h"
#include "StateMachine.h"
#include <memory.h>

void PIDInit (PID *pp)
{
	memset (pp,0,sizeof(PID));
}

/*全局函数定义*/
double PIDCalculate(PID* pp, double error)
{
	double vOut;    
	pp->sumError = pp->sumError + error;
	pp->dError = error - pp->error;
	pp->error = error;  
	vOut =	pp->Proportion * pp->error + pp->Integral * pp->sumError*0.0002 + pp->Derivative * pp->dError;
	return vOut;	
}

void PIDClear(PID* pp)
{
	pp->sumError = 0.0;
	pp->dError = 0.0;
	pp->error = 0.0;
}
/*
double LLCCalculate(LLC* pp, double error)
{
	int i = 0;
	pp->pIn[0] = error;
	pp->pUout[0] = 0.0;
	//控制量计算
	for(i=0;i< pp->order;i++)
	{
		pp->pUout[0] += pp->pB[i]*pp->pIn[i]; 
	}		  
	for(i=1;i<pp->order;i++)
	{
		pp->pUout[0] -= pp->pA[i]*pp->pUout[i]; 
	}
	//更新数据
	for(i=pp->order-1; i>0; i--)
	{
		pp->pUout[i] = pp->pUout[i-1];
		pp->pIn[i] = pp->pIn[i-1];
	}
	return pp->pUout[0];
}
*/
double LLCCalculate(LLC* pp, double error)
{
	int i = 0;
	pp->pIn[0] = error;
	pp->pUout[0] = 0.0;
	//控制量计算
	for(i=1;i< pp->order;i++)
	{
		pp->pUout[0] -= pp->pB[i]*pp->pUout[i]; 
	}		  
	for(i=1;i<pp->order;i++)
	{
		pp->pUout[0] += pp->pA[i]*pp->pIn[i]; 
	}
	//更新数据
	for(i=pp->order-1; i>0; i--)
	{
		pp->pUout[i] = pp->pUout[i-1];
		pp->pIn[i] = pp->pIn[i-1];
	}
	return pp->pUout[0];
	}

double LLCFilter(LLC* pp, double pIn)
{
	int i = 0;
	pp->pIn[0] = pIn;
	pp->pUout[0] = 0.0;
	//控制量计算
	for(i=0;i< pp->order;i++)
	{
		pp->pUout[0] += pp->pB[i]*pp->pIn[i]; 
	}
	for(i=1;i<pp->order;i++)
	{
		pp->pUout[0] -= pp->pA[i]*pp->pUout[i]; 
	}
	//更新数据
	for(i=pp->order-1; i>0; i--)
	{
		pp->pUout[i] = pp->pUout[i-1];
		pp->pIn[i] = pp->pIn[i-1];
	}
	return pp->pUout[0];
}

double yk = 0,yk_1 = 0, uk = 0, uk_1 = 0,ykplus1 = 0;
int filterflag = 0;

double LPFilter(double errorphai)
{
    uk = errorphai;
	ykplus1 = 0.5692*yk - 0.081*yk_1 + 0.3577*uk + 0.154*uk_1;
	yk_1 = yk;
	yk = ykplus1;
	uk_1 = uk;
	return ykplus1;
}


void LLCClear(LLC* pp)
{
	int i = 0;
	for(i=0; i<pp->order; i++)
	{
		pp->pIn[i] = 0.0;
		pp->pUout[i] = 0.0;
	}
}

