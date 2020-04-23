#ifndef CONTROLLER_H
#define CONTROLLER_H
/*类型定义*/
typedef struct PID
{
	double Proportion;
	double Integral;
	double Derivative;
	double error;
	double dError;
	double sumError;
}PID;
typedef struct LLC
{
	double *pIn;         
	double *pUout;
	double *pA;
	double *pB;
	int order;
}LLC;

typedef struct  
{
    double in[3];
    double out[3];
    double a[3];
    double b[3];
}ControlObject;


/*全局函数声明*/
extern PID sCogPID;

extern double PIDCalculate(PID* pp, double error);
extern void PIDClear(PID* pp);
extern double LLCCalculate(LLC* pp, double error);
extern void LLCClear(LLC* pp);

extern double PID_Calc_Balance(PID *pp,long NextPoint);

extern void PIDInit (PID *pp);


#endif
