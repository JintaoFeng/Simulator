#ifndef TRAJ_HANDLER_H
#define TRAJ_HANDLER_H

#include "StateMachine.h"

#define pi 3.1415926

typedef struct
{
	int R;
	int cirNum;
	float angle;
	float circleStep;
}CIRCLE;

extern CIRCLE circleParam;

extern int Coarse_Count;
extern TrajLast DXPosLast_Fine;
extern TrajLast DYPosLast_Fine;
extern TrajLast DXNegLast_Fine;
extern TrajLast DYNegLast_Fine;

extern int Fine_Count;
extern TrajLast DXPosLast_Coarse;
extern TrajLast DYPosLast_Coarse;
extern TrajLast DXNegLast_Coarse;
extern TrajLast DYNegLast_Coarse;

extern char* CharResoluted_Mode1(int field);
extern char* CharResoluted_Mode2(int field);



extern void Close(Axis *pAxis,int iAxis);
extern void Open(Axis *pAxis,int iAxis);
extern void StepFor(Axis *pAxis,int iAxis);
extern void StepRev(Axis *pAxis,int iAxis);
extern void JogFor(Axis *pAxis,int iAxis);
extern void JogRev(Axis *pAxis,int iAxis);
extern void Ident(Axis *pAxis,int iAxis);
extern void XYRun_Forward(Axis *pAxis,int iAxis);
extern void XYRun_Reverse(Axis *pAxis,int iAxis);
extern void Home(Axis *pAxis,int iAxis);
extern void Home_Coarse(Axis *pAxis,int iAxis);


extern void Traj_Coarse();
extern void Traj_Fine();
 extern void CounterClose(Axis *pAxis,int iAxis);
extern void LaserSwitch(Axis *pAxis, int iAxis);
extern void GetTrajData(s_TrajParam_4_Order_Poly *pp_TrajParam,volatile double *pp_TrajData);
extern void GetRunData(s_TrajParam_4_Order_Poly *pp_TrajParam,volatile double *pp_TrajData);
extern void GetSineTrajData(s_TrajParam_Sine_Poly *pp_TrajParam,volatile double *pp_TrajData,int m_iAxis);
extern void GetSineTrajP2PData(s_TrajParam_Sine_Poly *pp_TrajParam,volatile double *pp_TrajData);
void AbsolutMove();
void absolute(double* temp);

void Circle(Stage  *pStage);

#endif

