#include "GetData.h"
#include "rtcontrol.h"
#include "TrajGen_4OrderPoly.h"
#include "stdio.h"

#include "StateMachine.h"


//0.15g，25mm/s,10mm
/*double XTrajParam[10]={	
												0.0002000000000000,
												21089.7851203973987140,
												0.0084000000000000,
												0.0000000000000000,
												0.0000000000000000,
												0.3664000000000000,
												0.2000000000000000,
												0.0000000000000000,
												0.0000000000000000,
												0.0000000000000000
										 };


//0.15g，25mm/s,10mm
double YTrajParam[10]={	
												0.0002000000000000,
												21089.7851203973987140,
												0.0084000000000000,
												0.0000000000000000,
												0.0000000000000000,
												0.3664000000000000,
												0.2000000000000000,
												0.0000000000000000,
												0.0000000000000000,
												0.0000000000000000
										 };*/


//0.01g，25mm/s,10mm


int DirParam[4]={1,1,1,1};


double XTrajParam[10]={	
												0.0002000000000000,
												17.7546895794486979,
												0.0750000000000000,
												0.0000000000000000,
												0.1002000000000000,
												0.0000000000000000,
												0.2000000000000000,
												0.0000000000000000,
												0.0000000000000000,
												0.0000000000000000
										 };


//0.01g，25mm/s,10mm
double YTrajParam[10]={	
												0.0002000000000000,
												17.7546895794486979,
												0.0750000000000000,
												0.0000000000000000,
												0.1002000000000000,
												0.0000000000000000,
												0.2000000000000000,
												0.0000000000000000,
												0.0000000000000000,
												0.0000000000000000

										 };




















/*//0.01g，25mm/s,10mm
double XExpoParam[10]={	
												0.0002000000000000,
												17.7546895794486979,
												0.0750000000000000,
												0.0000000000000000,
												0.1002000000000000,
												0.0000000000000000,
												2.0000000000000000,
												0.0000000000000000,
												0.0000000000000000,
												0.0000000000000000
										 };

//0.01g，25mm/s,10mm
double YExpoParam[10]={	
												0.0002000000000000,
												17.7546895794486979,
												0.0750000000000000,
												0.0000000000000000,
												0.1002000000000000,
												0.0000000000000000,
												2.0000000000000000,
												0.0000000000000000,
												0.0000000000000000,
												0.0000000000000000
										 };*/




//0.01g，5mm/s,10mm
double XExpoParam[10]={	
												0.0002000000000000,
												156.2206305214619988,
												0.0252000000000000,
												0.0000000000000000,
												0.0000000000000000,
												1.8992000000000000,
												1.0000000000000000,
												0.0000000000000000,
												0.0000000000000000,
												0.0000000000000000
										 };





//0.01g，15mm/s,300mm
double YExpoParam[10]={	
												0.0002000000000000,
												17.6363107163153003,
												0.0752000000000000,
												0.0000000000000000,
												0.0000000000000000,
												19.6992000000000012,
												1.0000000000000000,
												0.0000000000000000,
												0.0000000000000000,
												0.0000000000000000

										 };
double YExpoRevParam[10]={0};







/*********************************************
函数名：GetTrajData
返回值：无
参数表：S_TrajParam *pp_TrajParam	存放曝光轨迹、测量轨迹与点对点轨迹参数
		volatile double *pp_TrajData	读取曝光轨迹、测量轨迹与点对点轨迹的地址
功能说明：从指定内存中读取曝光轨迹、测量轨迹与点对点轨迹的参数
*********************************************/
void GetTrajData(s_TrajParam_4_Order_Poly *pp_TrajParam,volatile double *pp_TrajData)
{
	int i=0;
	int ParamCount = 0;
	double *pp_Expo = (double*)pp_TrajParam;
	ParamCount = (int)(*pp_TrajData);
	for(i=0;i<10*ParamCount;i++)
	{
		*(pp_Expo+i)=*(pp_TrajData+1+i);
	}
}

/******************************************
函数名：GetRunData
返回值：无
参数表：S_TrajParam *pp_TrajParam	存放单独轨迹的参数
		volatile double *pp_TrajData	读取单独轨迹参数的地址
功能说明：从指定内存中读取单独轨迹的参数
******************************************/
void GetRunData(s_TrajParam_4_Order_Poly *pp_TrajParam,volatile double *pp_TrajData)
{
	int i=0;
	double *pp_Expo = (double*)pp_TrajParam;
	for(i=0;i<10;i++)
	{
		*(pp_Expo+i)=*(pp_TrajData+i);
	}
//	printf("hah\n");
}



/*********************************************
函数名：GetSineTrajData
返回值：无
参数表：TrajSine_Poly *pp_TrajParam	存放曝光轨迹、测量轨迹与点对点轨迹参数
		volatile double *pp_TrajData	读取曝光轨迹、测量轨迹与点对点轨迹的地址
功能说明：从指定内存中读取曝光轨迹、测量轨迹与点对点轨迹的参数
*********************************************/
void GetSineTrajData(s_TrajParam_Sine_Poly *pp_TrajParam,volatile double *pp_TrajData,int iAxis)
{
	int i=0;
	int pCount = 0;
	double *pp_Expo = (double*)pp_TrajParam;
	pCount = (int)(*pp_TrajData);
	for(i=0;i<3;i++)
	{
		*(pp_Expo+i)=*(pp_TrajData+1+3*iAxis+i);
	}
	if(iAxis == 1)
	{
		pp_TrajParam->WaitCyc = (int)(*(pp_TrajData+7));
	}
	else
	{
		pp_TrajParam->WaitCyc = 0;
	}
}
/*****************************************
函数名：GetSineTrajP2PData
返回值：无
参数表：无
功能说明：获取点对点轨迹参数函数
******************************************/
void GetSineTrajP2PData(s_TrajParam_Sine_Poly *pp_TrajParam,volatile double *pp_TrajData)
{   		
	int i = 0;
	double *pp_Expo = (double*)pp_TrajParam;
	for(i=0;i<3;i++)
	{
		*(pp_Expo+i)=*(pp_TrajData+i);
	}
}



void InitTrajParam()
{
	GetRunData(&Machine.coarseStage.arrAxis[0].sTP_Run,XTrajParam);
	GetRunData(&Machine.coarseStage.arrAxis[1].sTP_Run,XTrajParam);
	GetRunData(&Machine.coarseStage.arrAxis[2].sTP_Run,YTrajParam);
	GetRunData(&Machine.coarseStage.arrAxis[3].sTP_Run,YTrajParam);

//yht
	GetRunData(&Machine.fineStage.arrAxis[0].sTP_Run,XTrajParam);
	GetRunData(&Machine.fineStage.arrAxis[1].sTP_Run,YTrajParam);

    GetRunData(&Machine.fineStage.arrAxis[3].sTP_Run,XTrajParam);
	GetRunData(&Machine.fineStage.arrAxis[4].sTP_Run,YTrajParam);

    GetRunData(&Machine.fineStage.arrAxis[5].sTP_Run,XTrajParam);
	GetRunData(&Machine.fineStage.arrAxis[6].sTP_Run,YTrajParam);
}

node* head=NULL;
node* tail=NULL;
void nodeInsert(double* addr)
{
   	node *p=NULL,*q=NULL;
	q=head;
	p= (node*)malloc(sizeof(node));
	p->data =(double*) addr;
	p->next= NULL;
	if(head == NULL)
	{
		head = p;
		tail=head;
//		printf("first\n");
		return;
	}
	while(q!=NULL)
	{
		if(q->data == addr)
		{
			free(p);
			logMsg("The value has already existed\n",0,0,0,0,0,0);
			return;
		}
		if(q->next==NULL)
		{
			q->next = p;
			tail=p;
			return;
		}
		q=q->next;
	}
}

int memoryStype=1;
void nodeRemove(double* add)
{
	node *q = NULL,*temp = NULL;
	q= head;
	temp =head;
	if(head==NULL)
	{
		logMsg("list is empty\n",0,0,0,0,0,0);
		return;
	}
//	printf("%d\n",add);
	while(q!=NULL)	//确保当前节点不为空
	{
		if((double*)(q->data) == add)		//判断当前节点的值是输入的值
		{
			if(q == head)
			{
				if(head == tail)
					tail = head = NULL;
				else
					head=q->next;
				free(q);
				logMsg("Head Removed Successfully\n",0,0,0,0,0,0);
			}
			else if(q == tail)
			{
				free(q);
				tail=temp;
				logMsg("Tail Removed Successfully\n",0,0,0,0,0,0);
			}
			else
			{
				temp->next=q->next;
				free(q);
				logMsg("Removed Successfully\n",0,0,0,0,0,0);
			}
		}
		temp = q;
		q=q->next;
	}
}

RECORD_PROCESS recordMap[]=
{
	{coarseAxisActPos_X1,recordFun,(double*)(&Machine.coarseStage.arrAxis[0].dActPos),0},
	{coarseAxisSetPoint_X1,recordFun,(double*)&Machine.coarseStage.arrAxis[0].dSetPoint,0},
	{coarseAxisError_X1,recordFun,(double*)(&Machine.coarseStage.arrAxis[0].dError),0},
	{coarseAxisActPos_X2,recordFun,(double*)&Machine.coarseStage.arrAxis[1].dActPos,0},
	{coarseAxisSetPoint_X2,recordFun,(double*)(&Machine.coarseStage.arrAxis[1].dSetPoint),0},
	{coarseAxisError_X2,recordFun,(double*)(&Machine.coarseStage.arrAxis[1].dError),0},
	{coarseAxisActPos_Y1,recordFun,(double*)&Machine.coarseStage.arrAxis[2].dActPos,0},
	{coarseAxisSetPoint_Y1,recordFun,(double*)(&Machine.coarseStage.arrAxis[2].dSetPoint),0},
	{coarseAxisError_Y1,recordFun,(double*)(&Machine.coarseStage.arrAxis[2].dError),0},
	{coarseAxisActPos_Y2,recordFun,(double*)&Machine.coarseStage.arrAxis[3].dActPos,0},
	{coarseAxisSetPoint_Y2,recordFun,(double*)(&Machine.coarseStage.arrAxis[3].dSetPoint),0},
	{coarseAxisError_Y2,recordFun,(double*)(&Machine.coarseStage.arrAxis[3].dError),0},
	{fineAxisActPos_DX,recordFun,(double*)&Machine.fineStage.arrAxis[0].dActPos,0},
	{fineAxisSetPoint_DX,recordFun,(double*)(&Machine.fineStage.arrAxis[0].dSetPoint),0},
	{fineAxisError_DX,recordFun,(double*)(&Machine.fineStage.arrAxis[0].dError),0},
	{fineAxisActPos_DY,recordFun,(double*)&Machine.fineStage.arrAxis[1].dActPos,0},
	{fineAxisSetPoint_DY,recordFun,(double*)(&Machine.fineStage.arrAxis[1].dSetPoint),0},
	{fineAxisError_DY,recordFun,(double*)(&Machine.fineStage.arrAxis[1].dError),0},
	{fineAxisActPos_TZ,recordFun,(double*)&Machine.fineStage.arrAxis[2].dActPos,0},
	{fineAxisSetPoint_TZ,recordFun,(double*)(&Machine.fineStage.arrAxis[2].dSetPoint),0},
	{fineAxisError_TZ,recordFun,(double*)(&Machine.fineStage.arrAxis[2].dError),0},
	{fineAxisActPos_DZ,recordFun,(double*)&Machine.fineStage.arrAxis[3].dActPos,0},
	{fineAxisSetPoint_DZ,recordFun,(double*)(&Machine.fineStage.arrAxis[3].dSetPoint),0},
	{fineAxisError_DZ,recordFun,(double*)(&Machine.fineStage.arrAxis[3].dError),0},
	{fineAxisActPos_TX,recordFun,(double*)&Machine.fineStage.arrAxis[4].dActPos,0},
	{fineAxisSetPoint_TX,recordFun,(double*)(&Machine.fineStage.arrAxis[4].dSetPoint),0},
	{fineAxisError_TX,recordFun,(double*)(&Machine.fineStage.arrAxis[4].dError),0},
	{fineAxisActPos_TY,recordFun,(double*)&Machine.fineStage.arrAxis[5].dActPos,0},
	{fineAxisSetPoint_TY,recordFun,(double*)(&Machine.fineStage.arrAxis[5].dSetPoint),0},
	{fineAxisError_TY,recordFun,(double*)(&Machine.fineStage.arrAxis[5].dError),0},
	{fineAxisConcalc_DX,recordFun,(double*)(&Machine.fineStage.arrAxis[0].dConCalc),0},
	{fineAxisConcalc_DY,recordFun,(double*)(&Machine.fineStage.arrAxis[1].dConCalc),0},
	{fineAxisConcalc_TZ,recordFun,(double*)(&Machine.fineStage.arrAxis[2].dConCalc),0},
	{fineAxisConcalc_DZ,recordFun,(double*)(&Machine.fineStage.arrAxis[3].dConCalc),0},
	{fineAxisConcalc_TX,recordFun,(double*)(&Machine.fineStage.arrAxis[4].dConCalc),0},
	{fineAxisConcalc_TY,recordFun,(double*)(&Machine.fineStage.arrAxis[5].dConCalc),0},
	{dIdentBuff_d,recordFun,(double*)(&Machine.dIdentData),0}
};



void recordFun(double* add,int Flag)
{
	printf("%d\n",add);
	if(Flag == 1)
	{
		nodeInsert(add);
		printf("%d\n",add);
	}
	else if(Flag==0)
	{
		nodeRemove(add);
	}
}