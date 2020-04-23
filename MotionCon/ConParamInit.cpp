/*********************************************
文件名：ConParam.c
文件功能：设置所有反馈控制器与前馈控制器的参数
函数表：void SetPID_Counter(int iAxisNum)
		void SetLLCCounter(int iAxisNum)
		void SetPID_Switch(int iAxisNum)
		void SetLLC_Switch(int iAxisNum)
		void SetLP(int iAxisNum)
*********************************************/
#include "ConParamInit.h"
#include "controller.h"
#include "rtcontrol.h"
#include "StateMachine.h"
#include "TrajHandler.h"
#include "ConCalc.h"
#include "GetData.h"
#include <stdlib.h>
#include <memory.h>
//#include "TrajGen_4OrderPoly.h"

////////////////////////////////////////////////////////x////////////////////////////////////////////////
//偏置电压
double CoarseOffset[4] = {0.0,0.0,0.0,0.0};
double FineOffset[6] = {0.0,0.0,0.0,0.0,0.0,0.0};
double CoarseLimit[4] = {3.0*3276.8/2,3.0*3276.8/2,3.0*3276.8/2,3.0*3276.8/2};
double FineLimit[6] = {3.0*3276.8/2,3.0*3276.8/2,3.0*3276.8/2,3.0*3276.8/2,3.0*3276.8/2,3.0*3276.8/2};

int Coarse_CounterPIDType[COARSE_AXIS_NUM] = {0,0,0,0};
int Coarse_SwitchPIDType[COARSE_AXIS_NUM] = {0,0,0,0};
int Fine_CounterPIDType[FINE_AXIS_NUM] = {0,0,0,0,0,0};
int Fine_SwitchPIDType[FINE_AXIS_NUM] = {0,0,0,0,0,0};

/////////////////////////////////////粗动台////////////////////////////////////////////
//DX
double Coarse_bx[ORDER_COARSE_X] = {1.000000000000,0.0,0.0,0.0,0.0}; 
double Coarse_ax[ORDER_COARSE_X] = {1.000000000000,0.0,0.0,0.0,0.0};
double Coarse_UXout[ORDER_COARSE_X] = {0.0};
double Coarse_Xin[ORDER_COARSE_X] = {0.0};
int Coarse_order_x = ORDER_COARSE_X;

//DY
double Coarse_by[ORDER_COARSE_Y]={1.000000000000, 0.0, 0.0, 0.0, 0.0};
double Coarse_ay[ORDER_COARSE_Y]={1.000000000000, 0.0, 0.0, 0.0, 0.0};
double Coarse_UYout[ORDER_COARSE_Y]={0.0};
double Coarse_Yin[ORDER_COARSE_Y]={0.0};
int Coarse_order_y=ORDER_COARSE_Y;

//XTZ
double Coarse_bxtz[ORDER_COARSE_XTZ]={1.000000000000, 0.0, 0.0};
double Coarse_axtz[ORDER_COARSE_XTZ]={1.000000000000, 0.0, 0.0};
double Coarse_UXTZout[ORDER_COARSE_XTZ]={0.0};
double Coarse_XTZin[ORDER_COARSE_XTZ]={0.0};
int Coarse_order_xtz=ORDER_COARSE_XTZ;

//YTZ
double Coarse_bytz[ORDER_COARSE_YTZ]={1.000000000000, 0.0, 0.0};
double Coarse_aytz[ORDER_COARSE_YTZ]={1.000000000000, 0.0, 0.0};
double Coarse_UYTZout[ORDER_COARSE_YTZ]={0.0};
double Coarse_YTZin[ORDER_COARSE_YTZ]={0.0};
int Coarse_order_ytz=ORDER_COARSE_YTZ;

/////////////////////////////////////粗动台切换////////////////////////////////////////////
//DX
double Switch_Coarse_bx[SWITCH_ORDER_COARSE_X]={1.000000000000, 0.0, 0.0, 0.0, 0.0};
double Switch_Coarse_ax[SWITCH_ORDER_COARSE_X]={1.000000000000, 0.0, 0.0, 0.0, 0.0};
double Switch_Coarse_UXout[SWITCH_ORDER_COARSE_X]={0.0};
double Switch_Coarse_Xin[SWITCH_ORDER_COARSE_X]={0.0};
int Switch_Coarse_order_x=SWITCH_ORDER_COARSE_X;

//DY
double Switch_Coarse_by[SWITCH_ORDER_COARSE_Y]={1.000000000000, 0.0, 0.0};
double Switch_Coarse_ay[SWITCH_ORDER_COARSE_Y]={1.000000000000, 0.0, 0.0};
double Switch_Coarse_UYout[SWITCH_ORDER_COARSE_Y]={0.0};
double Switch_Coarse_Yin[SWITCH_ORDER_COARSE_Y]={0.0};
int Switch_Coarse_order_y=SWITCH_ORDER_COARSE_Y;

//XTZ
double Switch_Coarse_bxtz[ORDER_COARSE_XTZ]={1.000000000000, 0.0, 0.0};
double Switch_Coarse_axtz[ORDER_COARSE_XTZ]={1.000000000000, 0.0, 0.0};
double Switch_Coarse_UXTZout[ORDER_COARSE_XTZ]={0.0};
double Switch_Coarse_XTZin[ORDER_COARSE_XTZ]={0.0};
int Switch_Coarse_order_xtz=ORDER_COARSE_XTZ;

//YTZ
double Switch_Coarse_bytz[SWITCH_ORDER_COARSE_YTZ]={1.000000000000, 0.0, 0.0};
double Switch_Coarse_aytz[SWITCH_ORDER_COARSE_YTZ]={1.000000000000, 0.0, 0.0};
double Switch_Coarse_UYTZout[SWITCH_ORDER_COARSE_YTZ]={0.0};
double Switch_Coarse_YTZin[SWITCH_ORDER_COARSE_YTZ]={0.0};
int Switch_Coarse_order_ytz=SWITCH_ORDER_COARSE_YTZ;

////////////////////////////////////////////////////////滤波器参数////////////////////////////////////////////////
double  ILP_b[ORDER_LP]={1.0,-1.4814478,0.5905994};
double  ILP_a[ORDER_LP]={0.0272879,0.0545758,0.0272879};

/////////////////////////////////////微动台////////////////////////////////////////////
//DX
double Fine_bx[ORDER_FINE_X]={1.000000000000, 0.0, 0.0, 0.0, 0.0};
double Fine_ax[ORDER_FINE_X]={1.000000000000, 0.0, 0.0, 0.0, 0.0};
double Fine_UXout[ORDER_FINE_X]={0.0};
double Fine_Xin[ORDER_FINE_X]={0.0};
int Fine_order_x=ORDER_FINE_X;
//DY
double Fine_by[ORDER_FINE_Y]={1.000000000000, 0.0, 0.0, 0.0, 0.0};
double Fine_ay[ORDER_FINE_Y]={1.000000000000, 0.0, 0.0, 0.0, 0.0};
double Fine_UYout[ORDER_FINE_Y]={0.0};
double Fine_Yin[ORDER_FINE_Y]={0.0};
int Fine_order_y=ORDER_FINE_Y;
//TZ
double Fine_btz[ORDER_FINE_TZ]={1.000000000000, 0.0, 0.0, 0.0, 0.0};
double Fine_atz[ORDER_FINE_TZ]={1.000000000000, 0.0, 0.0, 0.0, 0.0};
double Fine_UTZout[ORDER_FINE_TZ]={0.0};
double Fine_TZin[ORDER_FINE_TZ]={0.0};
int Fine_order_tz=ORDER_FINE_TZ;

//DZ
double Fine_bz[ORDER_FINE_Z]={1.000000000000, 0.0, 0.0, 0.0, 0.0};
double Fine_az[ORDER_FINE_Z]={1.000000000000, 0.0, 0.0, 0.0, 0.0};
double Fine_UZout[ORDER_FINE_Z]={0.0};
double Fine_Zin[ORDER_FINE_Z]={0.0};
int Fine_order_z=ORDER_FINE_Z;

//TX
double Fine_btx[ORDER_FINE_TX]={1.000000000000, 0.0, 0.0, 0.0, 0.0};
double Fine_atx[ORDER_FINE_TX]={1.000000000000, 0.0, 0.0, 0.0, 0.0};
double Fine_UTXout[ORDER_FINE_TX]={0.0};
double Fine_TXin[ORDER_FINE_TX]={0.0};
int Fine_order_tx=ORDER_FINE_TX;
//TY
double Fine_bty[ORDER_FINE_TY]={1.000000000000, 0.0, 0.0, 0.0, 0.0};
double Fine_aty[ORDER_FINE_TY]={1.000000000000, 0.0, 0.0, 0.0, 0.0};
double Fine_UTYout[ORDER_FINE_TY]={0.0};
double Fine_TYin[ORDER_FINE_TY]={0.0};
int Fine_order_ty=ORDER_FINE_TY;

///////////////微动台滤波
//DX
//double FineFilter_bx[ORDER_FINE_FILTER_X]={1.000000000000, 0.0, 0.0, 0.0, 0.0};
//double FineFilter_ax[ORDER_FINE_FILTER_X]={1.000000000000, 0.0, 0.0, 0.0, 0.0};
// 300 Hz lowpass filter
//double FineFilter_bx[ORDER_FINE_FILTER_X]={ 0.02728790, 0.05457580, 0.02728790, 0.00000000, 0.00000000 };
//double FineFilter_ax[ORDER_FINE_FILTER_X]={ 1.00000000, -1.48144783, 0.59059944, 0.00000000, 0.00000000 };

// 200 Hz lowpass filter
double FineFilter_bx[ORDER_FINE_FILTER_X]={0.01323136, 0.02646273, 0.01323136, 0.00000000, 0.00000000};
double FineFilter_ax[ORDER_FINE_FILTER_X]={1.00000000, -1.64930918, 0.70223464, 0.00000000, 0.00000000};

// 120 Hz lowpass filter
//double FineFilter_bx[ORDER_FINE_FILTER_X]={0.00511094, 0.01022189, 0.00511094, 0.00000000, 0.00000000};
//double FineFilter_ax[ORDER_FINE_FILTER_X]={1.00000000, -1.78785744, 0.80830121, 0.00000000, 0.00000000};



double FineFilter_UXout[ORDER_FINE_FILTER_X]={0.0};
double FineFilter_Xin[ORDER_FINE_FILTER_X]={0.0};
int FineFilter_order_x=ORDER_FINE_FILTER_X;
//DY
//double FineFilter_by[ORDER_FINE_FILTER_Y]={ 0.02728790, 0.05457580, 0.02728790, 0.00000000, 0.00000000 };
//double FineFilter_ay[ORDER_FINE_FILTER_Y]={ 1.00000000, -1.48144783, 0.59059944, 0.00000000, 0.00000000 };
// 120 Hz lowpass filter
double FineFilter_by[ORDER_FINE_FILTER_X]={0.01323136, 0.02646273, 0.01323136, 0.00000000, 0.00000000};
double FineFilter_ay[ORDER_FINE_FILTER_X]={1.00000000, -1.64930918, 0.70223464, 0.00000000, 0.00000000};

double FineFilter_UYout[ORDER_FINE_FILTER_Y]={0.0};
double FineFilter_Yin[ORDER_FINE_FILTER_Y]={0.0};
int FineFilter_order_y=ORDER_FINE_FILTER_Y;
//TZ
//double FineFilter_btz[ORDER_FINE_FILTER_TZ]={ 0.02728790, 0.05457580, 0.02728790, 0.00000000, 0.00000000 };
//double FineFilter_atz[ORDER_FINE_FILTER_TZ]={ 1.00000000, -1.48144783, 0.59059944, 0.00000000, 0.00000000 };
// 120 Hz lowpass filter
double FineFilter_btz[ORDER_FINE_FILTER_X]={0.01323136, 0.02646273, 0.01323136, 0.00000000, 0.00000000};
double FineFilter_atz[ORDER_FINE_FILTER_X]={1.00000000, -1.64930918, 0.70223464, 0.00000000, 0.00000000};

// double FineFilter_btz[ORDER_FINE_FILTER_TZ]={0.00511094, 0.01022189, 0.00511094, 0.00000000, 0.00000000};
// double FineFilter_atz[ORDER_FINE_FILTER_TZ]={1.00000000, -1.78785744, 0.80830121, 0.00000000, 0.00000000};
double FineFilter_UTZout[ORDER_FINE_FILTER_TZ]={0.0};
double FineFilter_TZin[ORDER_FINE_FILTER_TZ]={0.0};
int FineFilter_order_tz=ORDER_FINE_FILTER_TZ;

//DZ
//double FineFilter_bz[ORDER_FINE_FILTER_Z]={ 0.02728790, 0.05457580, 0.02728790, 0.00000000, 0.00000000 };
//double FineFilter_az[ORDER_FINE_FILTER_Z]={ 1.00000000, -1.48144783, 0.59059944, 0.00000000, 0.00000000 };
// 120 Hz lowpass filter
double FineFilter_bz[ORDER_FINE_FILTER_X]={0.01323136, 0.02646273, 0.01323136, 0.00000000, 0.00000000};
double FineFilter_az[ORDER_FINE_FILTER_X]={1.00000000, -1.64930918, 0.70223464, 0.00000000, 0.00000000};

double FineFilter_UZout[ORDER_FINE_FILTER_Z]={0.0};
double FineFilter_Zin[ORDER_FINE_FILTER_Z]={0.0};
int FineFilter_order_z=ORDER_FINE_FILTER_Z;
//TX
//double FineFilter_btx[ORDER_FINE_FILTER_TX]={ 0.02728790, 0.05457580, 0.02728790, 0.00000000, 0.00000000 };
//double FineFilter_atx[ORDER_FINE_FILTER_TX]={ 1.00000000, -1.48144783, 0.59059944, 0.00000000, 0.00000000 };
// 120 Hz lowpass filter
double FineFilter_btx[ORDER_FINE_FILTER_X]={0.01323136, 0.02646273, 0.01323136, 0.00000000, 0.00000000};
double FineFilter_atx[ORDER_FINE_FILTER_X]={1.00000000, -1.64930918, 0.70223464, 0.00000000, 0.00000000};

double FineFilter_UTXout[ORDER_FINE_FILTER_TX]={0.0};
double FineFilter_TXin[ORDER_FINE_FILTER_TX]={0.0};
int FineFilter_order_tx=ORDER_FINE_FILTER_TX;
//TY
//double FineFilter_bty[ORDER_FINE_FILTER_TY]={ 0.02728790, 0.05457580, 0.02728790, 0.00000000, 0.00000000 };
//double FineFilter_aty[ORDER_FINE_FILTER_TY]={ 1.00000000, -1.48144783, 0.59059944, 0.00000000, 0.00000000 };
// 120 Hz lowpass filter
double FineFilter_bty[ORDER_FINE_FILTER_X]={0.01323136, 0.02646273, 0.01323136, 0.00000000, 0.00000000};
double FineFilter_aty[ORDER_FINE_FILTER_X]={1.00000000, -1.64930918, 0.70223464, 0.00000000, 0.00000000};

double FineFilter_UTYout[ORDER_FINE_FILTER_TY]={0.0};
double FineFilter_TYin[ORDER_FINE_FILTER_TY]={0.0};
int FineFilter_order_ty=ORDER_FINE_FILTER_TY;



/////////////////////////////////////微动台切换////////////////////////////////////////////
//DX
double Switch_Fine_bx[SWITCH_ORDER_FINE_X]={1.000000000000, 0.0, 0.0, 0.0, 0.0};
double Switch_Fine_ax[SWITCH_ORDER_FINE_X]={1.000000000000, 0.0, 0.0, 0.0, 0.0};
double Switch_Fine_UXout[SWITCH_ORDER_FINE_X]={0.0};
double Switch_Fine_Xin[SWITCH_ORDER_FINE_X]={0.0};
int Switch_Fine_order_x=SWITCH_ORDER_FINE_X;
//DY
double Switch_Fine_by[SWITCH_ORDER_FINE_Y]={1.000000000000, 0.0, 0.0, 0.0, 0.0};
double Switch_Fine_ay[SWITCH_ORDER_FINE_Y]={1.000000000000, 0.0, 0.0, 0.0, 0.0};
double Switch_Fine_UYout[SWITCH_ORDER_FINE_Y]={0.0};
double Switch_Fine_Yin[SWITCH_ORDER_FINE_Y]={0.0};
int Switch_Fine_order_y=SWITCH_ORDER_FINE_Y;
//TZ
double Switch_Fine_btz[SWITCH_ORDER_FINE_TZ]={1.000000000000, 0.0, 0.0, 0.0, 0.0};
double Switch_Fine_atz[SWITCH_ORDER_FINE_TZ]={1.000000000000, 0.0, 0.0, 0.0, 0.0};
double Switch_Fine_UTZout[SWITCH_ORDER_FINE_TZ]={0.0};
double Switch_Fine_TZin[SWITCH_ORDER_FINE_TZ]={0.0};
int Switch_Fine_order_tz=SWITCH_ORDER_FINE_TZ;

//DZ
double Switch_Fine_bz[SWITCH_ORDER_FINE_Z]={1.000000000000, 0.0, 0.0, 0.0, 0.0};
double Switch_Fine_az[SWITCH_ORDER_FINE_Z]={1.000000000000, 0.0, 0.0, 0.0, 0.0};
double Switch_Fine_UZout[SWITCH_ORDER_FINE_Z]={0.0};
double Switch_Fine_Zin[SWITCH_ORDER_FINE_Z]={0.0};
int Switch_Fine_order_z=SWITCH_ORDER_FINE_Z;
//TX
double Switch_Fine_btx[SWITCH_ORDER_FINE_TX]={1.000000000000, 0.0, 0.0, 0.0, 0.0};
double Switch_Fine_atx[SWITCH_ORDER_FINE_TX]={1.000000000000, 0.0, 0.0, 0.0, 0.0};
double Switch_Fine_UTXout[SWITCH_ORDER_FINE_TX]={0.0};
double Switch_Fine_TXin[SWITCH_ORDER_FINE_TX]={0.0};
int Switch_Fine_order_tx=SWITCH_ORDER_FINE_TX;
//TY
double Switch_Fine_bty[SWITCH_ORDER_FINE_TY]={1.000000000000, 0.0, 0.0, 0.0, 0.0};
double Switch_Fine_aty[SWITCH_ORDER_FINE_TY]={1.000000000000, 0.0, 0.0, 0.0, 0.0};
double Switch_Fine_UTYout[SWITCH_ORDER_FINE_TY]={0.0};
double Switch_Fine_TYin[SWITCH_ORDER_FINE_TY]={0.0};
int Switch_Fine_order_ty=SWITCH_ORDER_FINE_TY;

/*********************************微动台AFC控制器******************************/
//DX方向系数
double DX_Fine_x_in[4][6] = {
						 	{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						 	{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						 	{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						 	{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						 	};
double DX_Fine_y_out[4][6] = {
						  	 {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						  	 {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						  	 {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						  	 {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						  	 };
double DX_Fine_b[4][6] = {
					  	 {0.0,    0.0,   0.0, 0.0, 0.0, 0.0},
					  	 {0.0,    0.0,   0.0, 0.0, 0.0, 0.0},
					  	 {0.0,    0.0,   0.0, 0.0, 0.0, 0.0},
					  	 {0.0,    0.0,   0.0, 0.0, 0.0, 0.0},
					  	 };
double DX_Fine_a[4][6] = {
					  	 {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	 {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	 {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	 {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	 };
//DY方向系数
double DY_Fine_x_in[4][6] = {
						 	{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						 	{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						 	{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						 	{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						 	};
double DY_Fine_y_out[4][6] = {
						  	 {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						  	 {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						  	 {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						  	 {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						  	 };
double DY_Fine_b[4][6] = {
					  	 {0.0,    0.0,   0.0, 0.0, 0.0, 0.0},
					  	 {0.0,    0.0,   0.0, 0.0, 0.0, 0.0},
					  	 {0.0,    0.0,   0.0, 0.0, 0.0, 0.0},
					  	 {0.0,    0.0,   0.0, 0.0, 0.0, 0.0},
						 };
double DY_Fine_a[4][6] = {
					  	 {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	 {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	 {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	 {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	 };
//TZ方向系数
double TZ_Fine_x_in[4][6] = {
						 	{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						 	{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						 	{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						 	{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						 	};
double TZ_Fine_y_out[4][6] = {
						  	 {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						  	 {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						  	 {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						  	 {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						  	 };
double TZ_Fine_b[4][6] = {
					  	 {0.0,    0.0,   0.0, 0.0, 0.0, 0.0},
					  	 {0.0,    0.0,   0.0, 0.0, 0.0, 0.0},
					  	 {0.0,    0.0,   0.0, 0.0, 0.0, 0.0},
					  	 {0.0,    0.0,   0.0, 0.0, 0.0, 0.0},
					  	 };
double TZ_Fine_a[4][6] = {
					  	 {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	 {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	 {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	 {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	 };


//DZ方向系数
double DZ_Fine_x_in[4][6] = {
						 	{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						 	{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						 	{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						 	{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						 	};
double DZ_Fine_y_out[4][6] = {
						  	 {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						  	 {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						  	 {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						  	 {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						  	 };
double DZ_Fine_b[4][6] = {
					  	 {0.0,    0.0,   0.0, 0.0, 0.0, 0.0},
					  	 {0.0,    0.0,   0.0, 0.0, 0.0, 0.0},
					  	 {0.0,    0.0,   0.0, 0.0, 0.0, 0.0},
					  	 {0.0,    0.0,   0.0, 0.0, 0.0, 0.0},
					  	 };
double DZ_Fine_a[4][6] = {
					  	 {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	 {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	 {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	 {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	 };

//TX方向系数
double TX_Fine_x_in[4][6] = {
						 	{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						 	{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						 	{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						 	{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						 	};
double TX_Fine_y_out[4][6] = {
						  	 {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						  	 {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						  	 {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						  	 {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						  	 };
double TX_Fine_b[4][6] = {
					  	 {0.0,    0.0,   0.0, 0.0, 0.0, 0.0},
					  	 {0.0,    0.0,   0.0, 0.0, 0.0, 0.0},
					  	 {0.0,    0.0,   0.0, 0.0, 0.0, 0.0},
					  	 {0.0,    0.0,   0.0, 0.0, 0.0, 0.0},
					  	 };
double TX_Fine_a[4][6] = {
					  	 {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	 {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	 {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	 {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	 };
//TY方向系数
double TY_Fine_x_in[4][6] = {
						 	{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						 	{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						 	{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						 	{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						 	};
double TY_Fine_y_out[4][6] = {
						  	 {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						  	 {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						  	 {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						  	 {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						  	 };
double TY_Fine_b[4][6] = {
					  	 {0.0,    0.0,   0.0, 0.0, 0.0, 0.0},
					  	 {0.0,    0.0,   0.0, 0.0, 0.0, 0.0},
					  	 {0.0,    0.0,   0.0, 0.0, 0.0, 0.0},
					  	 {0.0,    0.0,   0.0, 0.0, 0.0, 0.0},
					  	 };
double TY_Fine_a[4][6] = {
					  	 {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	 {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	 {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	 {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	 };

/*********************************切换后的AFC控制器******************************/
//DX方向系数
double DX_Fine_Switch_x_in[4][6] = {
						           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
						           };
double DX_Fine_Switch_y_out[4][6] = {
						            {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	            {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	            {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	            {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
						            };
double DX_Fine_Switch_b[4][6] = {
						           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
						        };
double DX_Fine_Switch_a[4][6] = {
						           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
						        };
//DY方向系数
double DY_Fine_Switch_x_in[4][6] = {		           
						           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
						           };
						           
double DY_Fine_Switch_y_out[4][6] = {
						           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
						           };
double DY_Fine_Switch_b[4][6] = {
						           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
						        };
double DY_Fine_Switch_a[4][6] = {
						           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
						        };
//TZ方向系数
double TZ_Fine_Switch_x_in[4][6] = {
						           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
						           };
double TZ_Fine_Switch_y_out[4][6] = {
						           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
						           };
double TZ_Fine_Switch_b[4][6] = {
						           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
						         };
double TZ_Fine_Switch_a[4][6] = {
						           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
						        };

//DZ方向系数
double DZ_Fine_Switch_x_in[4][6] = {
						           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
						           };
double DZ_Fine_Switch_y_out[4][6] = {
						           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
						           };
double DZ_Fine_Switch_b[4][6] = {
						           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
						         };
double DZ_Fine_Switch_a[4][6] = {
						           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
						        };
//TX方向系数
double TX_Fine_Switch_x_in[4][6] = {
						           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
						           };
double TX_Fine_Switch_y_out[4][6] = {
						           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
						           };
double TX_Fine_Switch_b[4][6] = {
						           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
						         };
double TX_Fine_Switch_a[4][6] = {
						           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
						        };
//TY方向系数
double TY_Fine_Switch_x_in[4][6] = {
						           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
						           };
double TY_Fine_Switch_y_out[4][6] = {
						           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
						           };
double TY_Fine_Switch_b[4][6] = {
						           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
						         };
double TY_Fine_Switch_a[4][6] = {
						           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
					  	           {0.0,   0.0,    0.0, 0.0, 0.0, 0.0},
						        };

/******************************************
函数名：SetPID_Coarse_Counter
返回值：无
参数表：int iAxisNum	粗动台四自由度轴号
功能说明：设置电涡流闭环下的粗动台四自由度的PID参数
******************************************/
void SetPID_Coarse_Counter(int iAxis)
{
	switch(iAxis)
	{
		case CDX:
			Machine.coarseStage.arrAxis[CDX].sPID.Proportion = 0.000025;
			Machine.coarseStage.arrAxis[CDX].sPID.Integral = 0.0;
			Machine.coarseStage.arrAxis[CDX].sPID.Derivative = 0.00025;
		break;
		case CDY:
			Machine.coarseStage.arrAxis[CDY].sPID.Proportion = 0.000025;
			Machine.coarseStage.arrAxis[CDY].sPID.Integral = 0.0;
			Machine.coarseStage.arrAxis[CDY].sPID.Derivative = 0.00025;
		break;
		case XTZ:
			Machine.coarseStage.arrAxis[XTZ].sPID.Proportion = 0.000025;
			Machine.coarseStage.arrAxis[XTZ].sPID.Integral = 0.0;
			Machine.coarseStage.arrAxis[XTZ].sPID.Derivative = 0.00025;
		break;
		case YTZ:
			Machine.coarseStage.arrAxis[YTZ].sPID.Proportion = 0.000025;
			Machine.coarseStage.arrAxis[YTZ].sPID.Integral = 0.0;
			Machine.coarseStage.arrAxis[YTZ].sPID.Derivative = 0.00025;
		break;
		default:	break;
	}
}

/******************************************
函数名：SetLLC_Coarse_Counter
返回值：无
参数表：int iAxisNum	粗动台四自由度轴号
功能说明：设置电涡流闭环下的粗动台四自由度的超前滞后控制器参数
******************************************/
void SetLLC_Coarse_Counter(int iAxis)
{
	switch(iAxis)
	{
		case CDX:
			Machine.coarseStage.arrAxis[CDX].sLLC.pUout = Coarse_UXout;
			Machine.coarseStage.arrAxis[CDX].sLLC.pA = Coarse_ax;
			Machine.coarseStage.arrAxis[CDX].sLLC.pB = Coarse_bx;
			Machine.coarseStage.arrAxis[CDX].sLLC.pIn = Coarse_Xin;
			Machine.coarseStage.arrAxis[CDX].sLLC.order = Coarse_order_x;
		break;
		case CDY:
			Machine.coarseStage.arrAxis[CDY].sLLC.pUout = Coarse_UYout;
			Machine.coarseStage.arrAxis[CDY].sLLC.pA = Coarse_ay;
			Machine.coarseStage.arrAxis[CDY].sLLC.pB = Coarse_by;
			Machine.coarseStage.arrAxis[CDY].sLLC.pIn = Coarse_Yin;
			Machine.coarseStage.arrAxis[CDY].sLLC.order = Coarse_order_y;
		break;
		case XTZ:
			Machine.coarseStage.arrAxis[XTZ].sLLC.pUout = Coarse_UXTZout;
			Machine.coarseStage.arrAxis[XTZ].sLLC.pA = Coarse_axtz;
			Machine.coarseStage.arrAxis[XTZ].sLLC.pB = Coarse_bxtz;
			Machine.coarseStage.arrAxis[XTZ].sLLC.pIn = Coarse_XTZin;
			Machine.coarseStage.arrAxis[XTZ].sLLC.order = Coarse_order_xtz;
		break;
		case YTZ:
			Machine.coarseStage.arrAxis[YTZ].sLLC.pUout = Coarse_UYTZout;
			Machine.coarseStage.arrAxis[YTZ].sLLC.pA = Coarse_aytz;
			Machine.coarseStage.arrAxis[YTZ].sLLC.pB = Coarse_bytz;
			Machine.coarseStage.arrAxis[YTZ].sLLC.pIn = Coarse_YTZin;
			Machine.coarseStage.arrAxis[YTZ].sLLC.order = Coarse_order_ytz;
		break;
		default:	break;
	}	
}

/******************************************
函数名：SetPID_Coarse_Switch
返回值：无
参数表：int iAxisNum	粗动台四自由度轴号
功能说明：设置切换后的粗动台四自由度PID参数
******************************************/
void SetPID_Coarse_Switch(int iAxis)
{
	switch(iAxis)
	{
		case CDX:
			Machine.coarseStage.arrAxis[CDX].sPID_Switch.Proportion = 1.0;
			Machine.coarseStage.arrAxis[CDX].sPID_Switch.Integral = 0.0;
			Machine.coarseStage.arrAxis[CDX].sPID_Switch.Derivative = 0.0;
		break;
		case CDY:
			Machine.coarseStage.arrAxis[CDY].sPID_Switch.Proportion = 1.0;
			Machine.coarseStage.arrAxis[CDY].sPID_Switch.Integral = 0.0;
			Machine.coarseStage.arrAxis[CDY].sPID_Switch.Derivative = 0.0;
		break;
		case XTZ:
			Machine.coarseStage.arrAxis[XTZ].sPID_Switch.Proportion = 1.0;
			Machine.coarseStage.arrAxis[XTZ].sPID_Switch.Integral = 0.0;
			Machine.coarseStage.arrAxis[XTZ].sPID_Switch.Derivative = 0.0;
		break;
		case YTZ:
			Machine.coarseStage.arrAxis[YTZ].sPID_Switch.Proportion = 1.0;
			Machine.coarseStage.arrAxis[YTZ].sPID_Switch.Integral = 0.0;
			Machine.coarseStage.arrAxis[YTZ].sPID_Switch.Derivative = 0.0;
		break;
		default:	break;
	}
}
/******************************************
函数名：SetLLC_Coarse_Switch
返回值：无
参数表：int iAxisNum	粗动台四自由度轴号
功能说明：设置切换后的粗动台四自由度超前滞后控制器参数
******************************************/
void SetLLC_Coarse_Switch(int iAxis)
{
	switch(iAxis)
	{
		case CDX:
			Machine.coarseStage.arrAxis[CDX].sLLC_Switch.pUout = Switch_Coarse_UXout;
			Machine.coarseStage.arrAxis[CDX].sLLC_Switch.pA = Switch_Coarse_ax;
			Machine.coarseStage.arrAxis[CDX].sLLC_Switch.pB = Switch_Coarse_bx;
			Machine.coarseStage.arrAxis[CDX].sLLC_Switch.pIn = Switch_Coarse_Xin;
			Machine.coarseStage.arrAxis[CDX].sLLC_Switch.order = Switch_Coarse_order_x;
		break;
		case CDY:
			Machine.coarseStage.arrAxis[CDY].sLLC_Switch.pUout = Switch_Coarse_UYout;
			Machine.coarseStage.arrAxis[CDY].sLLC_Switch.pA = Switch_Coarse_ay;
			Machine.coarseStage.arrAxis[CDY].sLLC_Switch.pB = Switch_Coarse_by;
			Machine.coarseStage.arrAxis[CDY].sLLC_Switch.pIn = Switch_Coarse_Yin;
			Machine.coarseStage.arrAxis[CDY].sLLC_Switch.order = Switch_Coarse_order_y;
		break;
		case XTZ:
			Machine.coarseStage.arrAxis[XTZ].sLLC_Switch.pUout = Switch_Coarse_UXTZout;
			Machine.coarseStage.arrAxis[XTZ].sLLC_Switch.pA = Switch_Coarse_axtz;
			Machine.coarseStage.arrAxis[XTZ].sLLC_Switch.pB = Switch_Coarse_bxtz;
			Machine.coarseStage.arrAxis[XTZ].sLLC_Switch.pIn = Switch_Coarse_XTZin;
			Machine.coarseStage.arrAxis[XTZ].sLLC_Switch.order = Switch_Coarse_order_xtz;
		break;
		case YTZ:
			Machine.coarseStage.arrAxis[YTZ].sLLC_Switch.pUout = Switch_Coarse_UYTZout;
			Machine.coarseStage.arrAxis[YTZ].sLLC_Switch.pA = Switch_Coarse_aytz;
			Machine.coarseStage.arrAxis[YTZ].sLLC_Switch.pB = Switch_Coarse_bytz;
			Machine.coarseStage.arrAxis[YTZ].sLLC_Switch.pIn = Switch_Coarse_YTZin;
			Machine.coarseStage.arrAxis[YTZ].sLLC_Switch.order = Switch_Coarse_order_ytz;
		break;
		default:	break;
	}
}

/******************************************
函数名：SetPID_Fine_Counter
返回值：无
参数表：int iAxisNum	微动台三自由度轴号
功能说明：设置电涡流闭环下的微动台三自由度的PID参数
******************************************/
void SetPID_Fine_Counter(int iAxis)
{
	switch(iAxis)
	{
		case FDX:
			Machine.fineStage.arrAxis[FDX].sPID.Proportion = 0.000001;//0.000025;
			Machine.fineStage.arrAxis[FDX].sPID.Integral = 0.000005;//0.00003;//0.0;
			Machine.fineStage.arrAxis[FDX].sPID.Derivative = 0.000;//0.0;
		break;
		case FDY:
			Machine.fineStage.arrAxis[FDY].sPID.Proportion = 0.000001;//0.000025;
			Machine.fineStage.arrAxis[FDY].sPID.Integral = 0.000005;//0.00003;//0.0;
			Machine.fineStage.arrAxis[FDY].sPID.Derivative = 0.0000;//0.0;
		break;
		case FTZ:
			Machine.fineStage.arrAxis[FTZ].sPID.Proportion = 0.000001;//0.000025;
			Machine.fineStage.arrAxis[FTZ].sPID.Integral = 0.000005;
			Machine.fineStage.arrAxis[FTZ].sPID.Derivative = 0.0000;//0.0;
		break;
		case FDZ:
			Machine.fineStage.arrAxis[FDZ].sPID.Proportion = 0.000026;
			Machine.fineStage.arrAxis[FDZ].sPID.Integral = 0.00005;
			Machine.fineStage.arrAxis[FDZ].sPID.Derivative = 0.0003;//0.0;
		break;
		case FTX:
			Machine.fineStage.arrAxis[FTX].sPID.Proportion = 0.000025;
			Machine.fineStage.arrAxis[FTX].sPID.Integral = 0.00006;
			Machine.fineStage.arrAxis[FTX].sPID.Derivative = 0.00015;
		break;
		case FTY:
			Machine.fineStage.arrAxis[FTY].sPID.Proportion = 0.00002;
			Machine.fineStage.arrAxis[FTY].sPID.Integral = 0.000035;
			Machine.fineStage.arrAxis[FTY].sPID.Derivative = 0.0002;	
		break;
	}
}

void SetLLC_Fine_Filter(int iAxis)
{
	switch(iAxis)
	{
		case FDX:
			Machine.fineStage.arrAxis[FDX].sFilter.pUout = FineFilter_UXout;
			Machine.fineStage.arrAxis[FDX].sFilter.pA = FineFilter_ax;
			Machine.fineStage.arrAxis[FDX].sFilter.pB = FineFilter_bx;
			Machine.fineStage.arrAxis[FDX].sFilter.pIn = FineFilter_Xin;
			Machine.fineStage.arrAxis[FDX].sFilter.order = FineFilter_order_x;
		break;
		case FDY:
			Machine.fineStage.arrAxis[FDY].sFilter.pUout = FineFilter_UYout;
			Machine.fineStage.arrAxis[FDY].sFilter.pA = FineFilter_ay;
			Machine.fineStage.arrAxis[FDY].sFilter.pB = FineFilter_by;
			Machine.fineStage.arrAxis[FDY].sFilter.pIn = FineFilter_Yin;
			Machine.fineStage.arrAxis[FDY].sFilter.order = FineFilter_order_y;
		break;
		case FTZ:
			Machine.fineStage.arrAxis[FTZ].sFilter.pUout = FineFilter_UTZout;
			Machine.fineStage.arrAxis[FTZ].sFilter.pA = FineFilter_atz;
			Machine.fineStage.arrAxis[FTZ].sFilter.pB = FineFilter_btz;
			Machine.fineStage.arrAxis[FTZ].sFilter.pIn = FineFilter_TZin;
			Machine.fineStage.arrAxis[FTZ].sFilter.order = FineFilter_order_tz;
			break;
		case FDZ:
			Machine.fineStage.arrAxis[FDZ].sFilter.pUout = FineFilter_UZout;
			Machine.fineStage.arrAxis[FDZ].sFilter.pA = FineFilter_az;
			Machine.fineStage.arrAxis[FDZ].sFilter.pB = FineFilter_bz;
			Machine.fineStage.arrAxis[FDZ].sFilter.pIn = FineFilter_Zin;
			Machine.fineStage.arrAxis[FDZ].sFilter.order = FineFilter_order_z;
			break;
		case FTX:
			Machine.fineStage.arrAxis[FTX].sFilter.pUout = FineFilter_UTXout;
			Machine.fineStage.arrAxis[FTX].sFilter.pA = FineFilter_atx;
			Machine.fineStage.arrAxis[FTX].sFilter.pB = FineFilter_btx;
			Machine.fineStage.arrAxis[FTX].sFilter.pIn = FineFilter_TXin;
			Machine.fineStage.arrAxis[FTX].sFilter.order = FineFilter_order_tx;
			break;
		case FTY:
			Machine.fineStage.arrAxis[FTY].sFilter.pUout = FineFilter_UTYout;
			Machine.fineStage.arrAxis[FTY].sFilter.pA = FineFilter_aty;
			Machine.fineStage.arrAxis[FTY].sFilter.pB = FineFilter_bty;
			Machine.fineStage.arrAxis[FTY].sFilter.pIn = FineFilter_TYin;
			Machine.fineStage.arrAxis[FTY].sFilter.order = FineFilter_order_ty;
			break;
	}
}

/******************************************
函数名：SetLLC_Fine_Counter
返回值：无
参数表：int iAxisNum	微动台三自由度轴号
功能说明：设置电涡流闭环下的微动台三自由度的超前滞后控制器参数
******************************************/
void SetLLC_Fine_Counter(int iAxis)
{
	switch(iAxis)
	{
		case FDX:
			Machine.fineStage.arrAxis[FDX].sLLC.pUout = Fine_UXout;
			Machine.fineStage.arrAxis[FDX].sLLC.pA = Fine_ax;
			Machine.fineStage.arrAxis[FDX].sLLC.pB = Fine_bx;
			Machine.fineStage.arrAxis[FDX].sLLC.pIn = Fine_Xin;
			Machine.fineStage.arrAxis[FDX].sLLC.order = Fine_order_x;
		break;
		case FDY:
			Machine.fineStage.arrAxis[FDY].sLLC.pUout = Fine_UYout;
			Machine.fineStage.arrAxis[FDY].sLLC.pA = Fine_ay;
			Machine.fineStage.arrAxis[FDY].sLLC.pB = Fine_by;
			Machine.fineStage.arrAxis[FDY].sLLC.pIn = Fine_Yin;
			Machine.fineStage.arrAxis[FDY].sLLC.order = Fine_order_y;
		break;
		case FTZ:
			Machine.fineStage.arrAxis[FTZ].sLLC.pUout = Fine_UTZout;
			Machine.fineStage.arrAxis[FTZ].sLLC.pA = Fine_atz;
			Machine.fineStage.arrAxis[FTZ].sLLC.pB = Fine_btz;
			Machine.fineStage.arrAxis[FTZ].sLLC.pIn = Fine_TZin;
			Machine.fineStage.arrAxis[FTZ].sLLC.order = Fine_order_tz;
			break;
		case FDZ:
			Machine.fineStage.arrAxis[FDZ].sLLC.pUout = Fine_UZout;
			Machine.fineStage.arrAxis[FDZ].sLLC.pA = Fine_az;
			Machine.fineStage.arrAxis[FDZ].sLLC.pB = Fine_bz;
			Machine.fineStage.arrAxis[FDZ].sLLC.pIn = Fine_Zin;
			Machine.fineStage.arrAxis[FDZ].sLLC.order = Fine_order_z;
			break;
		case FTX:
			Machine.fineStage.arrAxis[FTX].sLLC.pUout = Fine_UTXout;
			Machine.fineStage.arrAxis[FTX].sLLC.pA = Fine_atx;
			Machine.fineStage.arrAxis[FTX].sLLC.pB = Fine_btx;
			Machine.fineStage.arrAxis[FTX].sLLC.pIn = Fine_TXin;
			Machine.fineStage.arrAxis[FTX].sLLC.order = Fine_order_tx;
			break;
		case FTY:
			Machine.fineStage.arrAxis[FTY].sLLC.pUout = Fine_UTYout;
			Machine.fineStage.arrAxis[FTY].sLLC.pA = Fine_aty;
			Machine.fineStage.arrAxis[FTY].sLLC.pB = Fine_bty;
			Machine.fineStage.arrAxis[FTY].sLLC.pIn = Fine_TYin;
			Machine.fineStage.arrAxis[FTY].sLLC.order = Fine_order_ty;
		break;
	}
}
/******************************************
函数名：SetPID_Fine_Switch
返回值：无
参数表：int iAxisNum	微动台三自由度轴号
功能说明：设置切换后的微动台三自由度PID参数
******************************************/
void SetPID_Fine_Switch(int iAxis)
{
	switch(iAxis)
	{
		/*case FDX:
			Machine.fineStage.arrAxis[FDX].sPID_Switch.Proportion = 0.00015;//0.0001;
			Machine.fineStage.arrAxis[FDX].sPID_Switch.Integral = 0.002;//0.0;
			Machine.fineStage.arrAxis[FDX].sPID_Switch.Derivative = 0.001;//0.0005;
		break;
		case FDY:
			Machine.fineStage.arrAxis[FDY].sPID_Switch.Proportion = 0.00015;//0.0001;
			Machine.fineStage.arrAxis[FDY].sPID_Switch.Integral = 0.0025;//0.0;
			Machine.fineStage.arrAxis[FDY].sPID_Switch.Derivative = 0.001;//0.0005;
		break;
		case FTZ:
			Machine.fineStage.arrAxis[FTZ].sPID_Switch.Proportion = 0.0002;//0.0001;
			Machine.fineStage.arrAxis[FTZ].sPID_Switch.Integral = 0.0;//0.0;
			Machine.fineStage.arrAxis[FTZ].sPID_Switch.Derivative = 0.0005;//0.0005;
		break;*/

		case FDX:
			Machine.fineStage.arrAxis[FDX].sPID_Switch.Proportion = 0.00008;//0.0001;
			Machine.fineStage.arrAxis[FDX].sPID_Switch.Integral = 0.0;//0.002;
			Machine.fineStage.arrAxis[FDX].sPID_Switch.Derivative = 0.0005;//0.0005;
		break;
		case FDY:
			Machine.fineStage.arrAxis[FDY].sPID_Switch.Proportion = 0.00008;//0.0001;
			Machine.fineStage.arrAxis[FDY].sPID_Switch.Integral = 0.0;//0.002;
			Machine.fineStage.arrAxis[FDY].sPID_Switch.Derivative = 0.0005;//0.0005;
		break;
		case FTZ:
			Machine.fineStage.arrAxis[FTZ].sPID_Switch.Proportion = 0.0001;//0.0001;
			Machine.fineStage.arrAxis[FTZ].sPID_Switch.Integral = 0.0001;//0.0;
			Machine.fineStage.arrAxis[FTZ].sPID_Switch.Derivative = 0.0001;//0.0005;
			break;
		case FDZ:
			Machine.fineStage.arrAxis[FDZ].sPID_Switch.Proportion = 0.0001;//0.0001;
			Machine.fineStage.arrAxis[FDZ].sPID_Switch.Integral = 0.0;
			Machine.fineStage.arrAxis[FDZ].sPID_Switch.Derivative = 0.0;//0.0005;
			break;
		case FTX:
			Machine.fineStage.arrAxis[FTX].sPID_Switch.Proportion = 0.0001;//0.0001;
			Machine.fineStage.arrAxis[FTX].sPID_Switch.Integral = 0.0;
			Machine.fineStage.arrAxis[FTX].sPID_Switch.Derivative = 0.0;//0.0005;
			break;
		case FTY:
			Machine.fineStage.arrAxis[FTY].sPID_Switch.Proportion = 0.0001;//0.0001;
			Machine.fineStage.arrAxis[FTY].sPID_Switch.Integral = 0.0;
			Machine.fineStage.arrAxis[FTY].sPID_Switch.Derivative = 0.0;//0.0005;
		break;

		
	}
}
/******************************************
函数名：SetLLC_Fine_Switch
返回值：无
参数表：int iAxisNum	微动台三自由度轴号
功能说明：设置切换后的微动台三自由度超前滞后控制器参数
******************************************/
void SetLLC_Fine_Switch(int iAxis)
{
	switch(iAxis)
	{
		case FDX:
			Machine.fineStage.arrAxis[FDX].sLLC_Switch.pUout = Switch_Fine_UXout;
			Machine.fineStage.arrAxis[FDX].sLLC_Switch.pA = Switch_Fine_ax;
			Machine.fineStage.arrAxis[FDX].sLLC_Switch.pB = Switch_Fine_bx;
			Machine.fineStage.arrAxis[FDX].sLLC_Switch.pIn = Switch_Fine_Xin;
			Machine.fineStage.arrAxis[FDX].sLLC_Switch.order = Switch_Fine_order_x;
		break;
		case FDY:
			Machine.fineStage.arrAxis[FDY].sLLC_Switch.pUout = Switch_Fine_UYout;
			Machine.fineStage.arrAxis[FDY].sLLC_Switch.pA = Switch_Fine_ay;
			Machine.fineStage.arrAxis[FDY].sLLC_Switch.pB = Switch_Fine_by;
			Machine.fineStage.arrAxis[FDY].sLLC_Switch.pIn = Switch_Fine_Yin;
			Machine.fineStage.arrAxis[FDY].sLLC_Switch.order = Switch_Fine_order_y;
		break;
		case FTZ:
			Machine.fineStage.arrAxis[FTZ].sLLC_Switch.pUout = Switch_Fine_UTZout;
			Machine.fineStage.arrAxis[FTZ].sLLC_Switch.pA = Switch_Fine_atz;
			Machine.fineStage.arrAxis[FTZ].sLLC_Switch.pB = Switch_Fine_btz;
			Machine.fineStage.arrAxis[FTZ].sLLC_Switch.pIn = Switch_Fine_TZin;
			Machine.fineStage.arrAxis[FTZ].sLLC_Switch.order = Switch_Fine_order_tz;
		break;
		case FDZ:
			Machine.fineStage.arrAxis[FDZ].sLLC_Switch.pUout = Switch_Fine_UZout;
			Machine.fineStage.arrAxis[FDZ].sLLC_Switch.pA = Switch_Fine_az;
			Machine.fineStage.arrAxis[FDZ].sLLC_Switch.pB = Switch_Fine_bz;
			Machine.fineStage.arrAxis[FDZ].sLLC_Switch.pIn = Switch_Fine_Zin;
			Machine.fineStage.arrAxis[FDZ].sLLC_Switch.order = Switch_Fine_order_z;
		break;
		case FTX:
			Machine.fineStage.arrAxis[FTX].sLLC_Switch.pUout = Switch_Fine_UTXout;
			Machine.fineStage.arrAxis[FTX].sLLC_Switch.pA = Switch_Fine_atx;
			Machine.fineStage.arrAxis[FTX].sLLC_Switch.pB = Switch_Fine_btx;
			Machine.fineStage.arrAxis[FTX].sLLC_Switch.pIn = Switch_Fine_TXin;
			Machine.fineStage.arrAxis[FTX].sLLC_Switch.order = Switch_Fine_order_tx;
		break;
		case FTY:
			Machine.fineStage.arrAxis[FTY].sLLC_Switch.pUout = Switch_Fine_UTYout;
			Machine.fineStage.arrAxis[FTY].sLLC_Switch.pA = Switch_Fine_atx;
			Machine.fineStage.arrAxis[FTZ].sLLC_Switch.pB = Switch_Fine_btx;
			Machine.fineStage.arrAxis[FTY].sLLC_Switch.pIn = Switch_Fine_TYin;
			Machine.fineStage.arrAxis[FTY].sLLC_Switch.order = Switch_Fine_order_tx;
		break;
	}
}

void InitCoarseConParam()
{
	int i = 0;
	for(i=0;i<4;i++)
	{
		SetPID_Coarse_Counter(i);
		SetLLC_Coarse_Counter(i);
		SetPID_Coarse_Switch(i);
		SetLLC_Coarse_Switch(i);

	}
}

void InitFineConParam()
{
	int i = 0;
	for(i=0;i<6;i++)
	{
		SetPID_Fine_Counter(i);
		SetLLC_Fine_Counter(i);
		SetLLC_Fine_Filter(i);
		SetPID_Fine_Switch(i);
		SetLLC_Fine_Switch(i);
	}
}



void InitMachine()
{
	InitBuff();
	InitStage();
}


void InitStage()
{
	InitCoarseStage();
	InitFineStage();
	InitTrajParam();
	Machine.connectType = TCP;
}

void InitCoarseStage()
{
	int i = 0;
	for(i=0;i<4;i++)
	{
		Machine.coarseStage.DA_Struct[i].iOffset = CoarseOffset[i];
		Machine.coarseStage.DA_Struct[i].iLimit = FineLimit[i];
	}
	Machine.coarseStage.Traj = Traj_Coarse;

	Machine.coarseStage.DDX_Flag=0;
	InitStageStatus(&Machine.coarseStage,CoarseStage);

	InitCoarseAxis(Machine.coarseStage.arrAxis,COARSE_AXIS_NUM);
}
void InitFineStage()
{
	int i = 0;
	for(i=0;i<6;i++)
	{
		Machine.fineStage.DA_Struct[i].iOffset = FineOffset[i];
		Machine.fineStage.DA_Struct[i].iLimit = FineLimit[i];
		
	}

	Machine.fineStage.Traj = Traj_Fine;

	InitStageStatus(&Machine.fineStage,FineStage);
	InitFineAxis(Machine.fineStage.arrAxis,FINE_AXIS_NUM);
}


void InitCoarseAxis(Axis *pAxis,int iAxisNum)
{
	int i = 0;
	for(i=0;i<iAxisNum;i++)
	{
		(pAxis+i)->controllerType = (ControllerType)Coarse_CounterPIDType[i];
		(pAxis+i)->LaserControllerType = (ControllerType)Coarse_SwitchPIDType[i];
		pAxis[i].targetPos = 100;
		pAxis[i].targetVel= 10;
		pAxis[i].targetAcc= 100;
		pAxis[i].targetJerk= 100000;
		pAxis[i].targetSnap= 10000000;

		InitControlObject(pAxis);
	}

	
	(pAxis+0)->OutCalc = Concalc_Coarse_DX;
	(pAxis+1)->OutCalc = Concalc_Coarse_DY;
	(pAxis+2)->OutCalc = Concalc_Coarse_XTZ;
	(pAxis+3)->OutCalc = Concalc_Coarse_YTZ;

	

	(pAxis+0)->TrajSine_Poly = TrajSine_Poly_Coarse;
	(pAxis+1)->TrajSine_Poly = TrajSine_Poly_Coarse;


	(pAxis+0)->TrajGen4Order = TrajGen4OrderX1;
	(pAxis+1)->TrajGen4Order = TrajGen4OrderX2;
	(pAxis+2)->TrajGen4Order = TrajGen4OrderY1;
	(pAxis+3)->TrajGen4Order = TrajGen4OrderY2;

	(pAxis+0)->dFeedForwardCoef = 0;//0.000001;//加速度前馈系数
	(pAxis+1)->dFeedForwardCoef = 0;//0.000001;
	(pAxis+2)->dFeedForwardCoef = 0;//0.000001;
	(pAxis+3)->dFeedForwardCoef = 0;//0.000001;

	(pAxis+0)->dStepDis = 100;
	(pAxis+1)->dStepDis = 100;
	(pAxis+2)->dStepDis = 100;
	(pAxis+3)->dStepDis = 100;

	pAxis[0].dJogSpd = 1000;//100单轴不行
	pAxis[1].dJogSpd = 1000;
	pAxis[2].dJogSpd = 1000;
	pAxis[3].dJogSpd = 1000;

	pAxis[0].dIdentGain =1e-8;// 0.3;
	pAxis[1].dIdentGain =1e-8;// 0.3;
	pAxis[2].dIdentGain = 1e-8;//0.03;
	pAxis[3].dIdentGain =1e-8;// 0.3;
	
	pAxis[4].dIdentGain =1e-8;// 0.3;
	pAxis[5].dIdentGain = 1e-8;//0.03;
	

	//Axis Common Status
	InitAxisStatus(Machine.coarseStage.arrAxis,COARSE_AXIS_NUM);	
	InitCoarseConParam();
}

void InitFineAxis(Axis *pAxis,int iAxisNum)
{
	int i = 0;
	for(i=0;i<iAxisNum;i++)
	{
		(pAxis+i)->controllerType = (ControllerType)Fine_CounterPIDType[i];
		(pAxis+i)->LaserControllerType = (ControllerType)Fine_SwitchPIDType[i];
		pAxis[i].targetPos = 100;
		pAxis[i].targetVel= 10;
		pAxis[i].targetAcc= 100;
		pAxis[i].targetJerk= 100000;
		pAxis[i].targetSnap= 10000000;
		pAxis[i].trajUpdate = 1;
	}
	(pAxis+0)->OutCalc = Concalc_Fine_DX;
	(pAxis+1)->OutCalc = Concalc_Fine_DY;
	(pAxis+2)->OutCalc = Concalc_Fine_TZ;

	//YHT
	(pAxis+3)->OutCalc = Concalc_Fine_DZ;
	(pAxis+4)->OutCalc = Concalc_Fine_TX;
	(pAxis+5)->OutCalc = Concalc_Fine_TY;
	

	(pAxis+0)->TrajSine_Poly = TrajSine_Poly_Fine;
	(pAxis+1)->TrajSine_Poly = TrajSine_Poly_Fine;
	(pAxis+2)->TrajSine_Poly = TrajSine_Poly_Fine;
	//YHT
	(pAxis+3)->TrajSine_Poly = TrajSine_Poly_Fine;
	(pAxis+4)->TrajSine_Poly = TrajSine_Poly_Fine;
	(pAxis+5)->TrajSine_Poly = TrajSine_Poly_Fine;
	
	(pAxis+0)->TrajGen4Order = TrajGen4OrderFineX;
	(pAxis+1)->TrajGen4Order = TrajGen4OrderFineY;
	(pAxis+2)->TrajGen4Order = TrajGen4OrderFineTz;
	
	

	(pAxis+0)->dFeedForwardCoef = 0.000001;
	(pAxis+1)->dFeedForwardCoef = 0.000001;
	(pAxis+2)->dFeedForwardCoef = 0.000001;
     //YHT
	(pAxis+3)->dFeedForwardCoef = 0.000001;
	(pAxis+4)->dFeedForwardCoef = 0.000001;
	(pAxis+5)->dFeedForwardCoef = 0.000001;

	(pAxis+0)->dStepDis = 100;
	(pAxis+1)->dStepDis = 100;
	(pAxis+2)->dStepDis = 100;
	//YHT
	(pAxis+3)->dStepDis = 100;
	(pAxis+4)->dStepDis = 100;
	(pAxis+5)->dStepDis = 100;

	(pAxis+0)->dJogSpd = 50;
	(pAxis+1)->dJogSpd = 50;
	(pAxis+2)->dJogSpd = 50;
	//YHT
	(pAxis+3)->dJogSpd = 50;
	(pAxis+4)->dJogSpd = 50;
	(pAxis+5)->dJogSpd = 50;


	pAxis[0].dIdentGain =1e-9;// 0.3;
	pAxis[1].dIdentGain =1e-9;// 0.3;
	pAxis[2].dIdentGain = 1e-9;//0.03;

	pAxis[3].dIdentGain =1e-9;// 0.3;
	pAxis[4].dIdentGain =1e-9;// 0.3;
	pAxis[5].dIdentGain = 1e-9;//0.03;

	pAxis[0].iDir = 1;
	pAxis[1].iDir = 1;
	pAxis[2].iDir = 1;
	pAxis[3].iDir = 1;
	pAxis[4].iDir = 1;
	pAxis[5].iDir = 1;
	
	pAxis[0].iLaserDir = 1;
	pAxis[1].iLaserDir = 1;
	pAxis[2].iLaserDir = 1;
	pAxis[3].iLaserDir = 1;
	pAxis[4].iLaserDir = 1;
	pAxis[5].iLaserDir = 1;
	
	//Axis Common Status
	InitAxisStatus(Machine.fineStage.arrAxis,FINE_AXIS_NUM);
	InitFineConParam();	
}

void InitAxisStatus(Axis *pAxis,int iAxisNum)
{
	int i = 0;
	for(i=0;i<iAxisNum;i++)
	{
		(pAxis+i)->axisCMD = C_Aixs_Open;
		(pAxis+i)->axisStatus = S_Aixs_Open;
		(pAxis+i)->closeStatus = S_Counter_Close;
		(pAxis+i)->trajType = isTraj4OrderPoly;//isTrajSineAccT;
		(pAxis+i)->trajMode = IsTrajIndependent;
	}
}

void InitStageStatus(Stage *pStage,int iStageType)
{
    pStage->stageCMD = C_ALL_Nothing;
	pStage->stageStatus = S_Stage_ALL_Open;
	pStage->stageType = (StageType)iStageType;
}


void InitControlObject(Axis *pAxis)
{
	pAxis->controlObject.order=7;
    pAxis->controlObject.pA = (double *)malloc((size_t)(pAxis->controlObject.order * (int)sizeof(double)));
    pAxis->controlObject.pB = (double *)malloc((size_t)(pAxis->controlObject.order * (int)sizeof(double)));
    pAxis->controlObject.pIn = (double *)malloc((size_t)(pAxis->controlObject.order * (int)sizeof(double)));
    pAxis->controlObject.pUout = (double *)malloc((size_t)(pAxis->controlObject.order * (int)sizeof(double)));
	pAxis->controlObject.pA[0] = 0;
	pAxis->controlObject.pA[1] = -0.9587;
	pAxis->controlObject.pA[2] = 10.88;
	pAxis->controlObject.pA[3] = -2.296;
	pAxis->controlObject.pA[4] = 6.509;
	pAxis->controlObject.pA[5] = -9.549;
	pAxis->controlObject.pA[6] = -4.317;
	
	pAxis->controlObject.pB[0] = 0;
	pAxis->controlObject.pB[1] = -4.497;
	pAxis->controlObject.pB[2] = 8.752;
	pAxis->controlObject.pB[3] = -9.655;
	pAxis->controlObject.pB[4] = 6.433;
	pAxis->controlObject.pB[5] = -2.422;
	pAxis->controlObject.pB[6] = 0.3896;

/*
	pAxis->controlObject.order=3;
	pAxis->controlObject.pA = (double *)malloc(pAxis->controlObject.order*sizeof(double));
	pAxis->controlObject.pB = (double *)malloc(pAxis->controlObject.order*sizeof(double));
	pAxis->controlObject.pIn = (double *)malloc(pAxis->controlObject.order*sizeof(double));
	pAxis->controlObject.pUout = (double *)malloc(pAxis->controlObject.order*sizeof(double));
	pAxis->controlObject.pA[0] = 0;
	pAxis->controlObject.pA[1] = 2e-8;
	pAxis->controlObject.pA[2] = 2e-8;

	
	pAxis->controlObject.pB[0] = 0;
	pAxis->controlObject.pB[1] = -2;
	pAxis->controlObject.pB[2] = 1;
*/	
    memset(pAxis->controlObject.pIn,0,pAxis->controlObject.order * sizeof(double));
    memset(pAxis->controlObject.pUout,0,pAxis->controlObject.order * sizeof(double));
}




