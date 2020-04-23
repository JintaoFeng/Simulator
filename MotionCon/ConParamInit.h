
#ifndef CON_PARAM_INIT_H
#define CON_PARAM_INIT_H


#include "StateMachine.h"

#define ORDER_LP   3
//////////////////////////////////////Counter//////////////////////////
#define ORDER_COARSE_X      5
#define ORDER_COARSE_Y      5
#define ORDER_COARSE_XTZ    3
#define ORDER_COARSE_YTZ    3

#define SWITCH_ORDER_COARSE_X     5
#define SWITCH_ORDER_COARSE_Y     3
#define SWITCH_ORDER_COARSE_XTZ   3
#define SWITCH_ORDER_COARSE_YTZ   3

#define DY_ORDER_COARSE_FIR  	      7
#define DY_ORDER_COARSE_FIR_SWITCH	  7

#define DX_ORDER_COARSE_FIR			  7
#define DX_ORDER_COARSE_FIR_SWITCH    7

#define ORDER_FINE_X        5
#define ORDER_FINE_Y        5
#define ORDER_FINE_TZ       5
#define ORDER_FINE_Z        5
#define ORDER_FINE_TX       5
#define ORDER_FINE_TY       5

#define ORDER_FINE_FILTER_X		5
#define ORDER_FINE_FILTER_Y		5
#define ORDER_FINE_FILTER_TZ	5
#define ORDER_FINE_FILTER_Z		5
#define ORDER_FINE_FILTER_TX	5
#define ORDER_FINE_FILTER_TY	5



#define SWITCH_ORDER_FINE_X       5
#define SWITCH_ORDER_FINE_Y       5
#define SWITCH_ORDER_FINE_TZ      5
#define SWITCH_ORDER_FINE_Z       5
#define SWITCH_ORDER_FINE_TX      5
#define SWITCH_ORDER_FINE_TY      5





#define DY_ORDER_FINE_FIR      	  	 7
#define DY_ORDER_FINE_FIR_SWITCH     7

#define DX_ORDER_FINE_FIR            7
#define DX_ORDER_FINE_FIR_SWITCH     7

//Coarse
//DX
extern double Coarse_bx[ORDER_COARSE_X];
extern double Coarse_ax[ORDER_COARSE_X];
extern double Coarse_UXout[ORDER_COARSE_X];
extern int Coarse_order_x;

extern double Switch_Coarse_bx[SWITCH_ORDER_COARSE_X];
extern double Switch_Coarse_ax[SWITCH_ORDER_COARSE_X];
extern double Switch_Coarse_UXout[SWITCH_ORDER_COARSE_X];
extern int Switch_Coarse_order_x;

//Dy
extern double Coarse_by[ORDER_COARSE_Y];
extern double Coarse_ay[ORDER_COARSE_Y];
extern double Coarse_UYout[ORDER_COARSE_Y];
extern int Coarse_order_y;

extern double Switch_Coarse_by[SWITCH_ORDER_COARSE_Y];
extern double Switch_Coarse_ay[SWITCH_ORDER_COARSE_Y];
extern double Switch_Coarse_UYout[SWITCH_ORDER_COARSE_Y];
extern int Switch_Coarse_order_y;
//XTZ
extern double Coarse_bxtz[ORDER_COARSE_XTZ];
extern double Coarse_axtz[ORDER_COARSE_XTZ];
extern double Coarse_UXTZout[ORDER_COARSE_XTZ];
extern int Coarse_order_xtz;

extern double Switch_Coarse_bxtz[SWITCH_ORDER_COARSE_XTZ];
extern double Switch_Coarse_axtz[SWITCH_ORDER_COARSE_XTZ];
extern double Switch_Coarse_UXTZout[SWITCH_ORDER_COARSE_XTZ];
extern int Switch_Coarse_order_xtz;

//YTZ
extern double Coarse_bytz[ORDER_COARSE_YTZ];
extern double Coarse_aytz[ORDER_COARSE_YTZ];
extern double Coarse_UYTZout[ORDER_COARSE_YTZ];
extern int Coarse_order_ytz;

extern double Switch_Coarse_bytz[SWITCH_ORDER_COARSE_YTZ];
extern double Switch_Coarse_aytz[SWITCH_ORDER_COARSE_YTZ];
extern double Switch_Coarse_UYTZout[SWITCH_ORDER_COARSE_YTZ];
extern int Switch_Coarse_order_ytz;

//Fine
//DX
extern double Fine_bx[ORDER_FINE_X];
extern double Fine_ax[ORDER_FINE_X];
extern double Fine_UXout[ORDER_FINE_X];
extern int Fine_order_x;

extern double Switch_Fine_bx[SWITCH_ORDER_FINE_X];
extern double Switch_Fine_ax[SWITCH_ORDER_FINE_X];
extern double Switch_Fine_UXout[SWITCH_ORDER_FINE_X];
extern int Switch_Fine_order_x;

//DY
extern double Fine_by[ORDER_FINE_Y];
extern double Fine_ay[ORDER_FINE_Y];
extern double Fine_UYout[ORDER_FINE_Y];
extern int Fine_order_y;

extern double Switch_Fine_by[SWITCH_ORDER_FINE_Y];
extern double Switch_Fine_ay[SWITCH_ORDER_FINE_Y];
extern double Switch_Fine_UYout[SWITCH_ORDER_FINE_Y];
extern int Switch_Fine_order_y;

//TZ
extern double Fine_btz[ORDER_FINE_TZ];
extern double Fine_atz[ORDER_FINE_TZ];
extern double Fine_UTZout[ORDER_FINE_TZ];
extern int Fine_order_tz;

extern double Switch_Fine_btz[SWITCH_ORDER_FINE_TZ];
extern double Switch_Fine_atz[SWITCH_ORDER_FINE_TZ];
extern double Switch_Fine_UTZout[SWITCH_ORDER_FINE_TZ];
extern int Switch_Fine_order_tz;

//DZ
extern double Fine_bx[ORDER_FINE_Z];
extern double Fine_ax[ORDER_FINE_Z];
extern double Fine_UXout[ORDER_FINE_Z];
extern int Fine_order_z;

extern double Switch_Fine_bx[SWITCH_ORDER_FINE_Z];
extern double Switch_Fine_ax[SWITCH_ORDER_FINE_Z];
extern double Switch_Fine_UXout[SWITCH_ORDER_FINE_Z];
extern int Switch_Fine_order_z;


//TX
extern double Fine_btz[ORDER_FINE_TX];
extern double Fine_atz[ORDER_FINE_TX];
extern double Fine_UTZout[ORDER_FINE_TX];
extern int Fine_order_tz;

extern double Switch_Fine_btz[SWITCH_ORDER_FINE_TX];
extern double Switch_Fine_atz[SWITCH_ORDER_FINE_TX];
extern double Switch_Fine_UTZout[SWITCH_ORDER_FINE_TX];
extern int Switch_Fine_order_tx;

//TY
extern double Fine_btz[ORDER_FINE_TY];
extern double Fine_atz[ORDER_FINE_TY];
extern double Fine_UTZout[ORDER_FINE_TY];
extern int Fine_order_tz;

extern double Switch_Fine_btz[SWITCH_ORDER_FINE_TY];
extern double Switch_Fine_atz[SWITCH_ORDER_FINE_TY];
extern double Switch_Fine_UTZout[SWITCH_ORDER_FINE_TY];
extern int Switch_Fine_order_ty;

void InitFineStage();
void InitCoarseStage();
void InitStage();
void InitCoarseAxis(Axis *pAxis,int iAxisNum);
void InitFineAxis(Axis *pAxis,int iAxisNum);
void InitAxisStatus(Axis *pAxis,int iAxisNum);
void InitStageStatus(Stage *pStage,int iStageType);

void InitControlObject(Axis *pAxis);





//ÂË²¨Æ÷
extern double  ILP_b[ORDER_LP];
extern double  ILP_a[ORDER_LP];

#define COARSE_AXIS_NUM		4
#define FINE_AXIS_NUM 		6//3yht
///////////////////////////////Sensor/////////////////////////////////



#endif
