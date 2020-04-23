#ifndef GETDATA_H
#define GETDATA_H
#include "TrajGen_4OrderPoly.h"

/******************************end*******************************************/

extern int DirParam[4];

extern double XTrajParam[10];

extern double YTrajParam[10];


extern double XExpoParam[10];
extern double YExpoParam[10];
extern double YExpoRevParam[10];


typedef struct _node 
{
	double *data;
	struct _node* next;
}node;

extern node* head;
extern node* tail;
extern int memoryStype;
void nodeInsert(double* addr);
void nodeRemove(double* add);


typedef enum
{
	coarseAxisActPos_X1=0,
	coarseAxisSetPoint_X1=1,
	coarseAxisError_X1=2,
	coarseAxisActPos_X2=3,
	coarseAxisSetPoint_X2=4,
	coarseAxisError_X2=5,
	coarseAxisActPos_Y1=6,
	coarseAxisSetPoint_Y1=7,
	coarseAxisError_Y1=8,
	coarseAxisActPos_Y2=9,
	coarseAxisSetPoint_Y2=10,
	coarseAxisError_Y2=11,
	fineAxisActPos_DX=12,
	fineAxisSetPoint_DX=13,
	fineAxisError_DX=14,
	fineAxisActPos_DY=15,
	fineAxisSetPoint_DY=16,
	fineAxisError_DY=17,
	fineAxisActPos_TZ=18,
	fineAxisSetPoint_TZ=19,
	fineAxisError_TZ=20,
	fineAxisActPos_DZ=21,
	fineAxisSetPoint_DZ=22,
	fineAxisError_DZ=23,
	fineAxisActPos_TX=24,
	fineAxisSetPoint_TX=25,
	fineAxisError_TX=26,
	fineAxisActPos_TY=27,
	fineAxisSetPoint_TY=28,
	fineAxisError_TY=29,
	fineAxisConcalc_DX=48,
	fineAxisConcalc_DY=49,
	fineAxisConcalc_TZ=50,
	fineAxisConcalc_DZ=51,
	fineAxisConcalc_TX=52,
	fineAxisConcalc_TY=53,
	dIdentBuff_d = 54
}RECORD_INDEX;

typedef void(*OPERATE_FUNC)(double,int);

typedef struct
{
	RECORD_INDEX  index;
	void (*func)(double*,int);//对应的函数
	double* arg1;
	int arg2;
}RECORD_PROCESS;

extern RECORD_PROCESS recordMap[];

void recordFun(double* add,int Flag);

#endif
