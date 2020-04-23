#include <vxworks.h>
#include <vme.h>
#include <taskLib.h>

#include <sockLib.h>
#include <ioLib.h>
#include <inetLib.h>
#include <logLib.h>
#include <string.h>
#include <fioLib.h>
#include <stdio.h>
#include <memLib.h>
#include <stdLib.h>
#include <semLib.h>
#include <netinet\\tcp.h>

#include "ConParamHandler.h"
#include "ComWithWin.h"
#include "Controller.h"
#include "TrajHandler.h"
#include "main.h"
#include "SingleTPG.h"
#include "GetData.h"

double* bParam;
double* aParam;
double LLCin[12] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
double LLCout[12] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};

void SetPIDParam(PID *pPID,int iAxis)
{
	pPID->Proportion = *(comData.transConParam.dParamter[iAxis]);
	pPID->Integral = *(comData.transConParam.dParamter[iAxis]+1);
	pPID->Derivative = *(comData.transConParam.dParamter[iAxis]+2);
}

void SetLLCParam(LLC *pLLC,int iAxis)
{
	int i = 0;
	
	for(i=0;i<pLLC->order;i++)
	{
		*(aParam+i) = *(comData.transConParam.dParamter[iAxis]+i);
		*(bParam+i) = *(comData.transConParam.dParamter[iAxis]+pLLC->order+i);
	}
	pLLC->pA = aParam;
	pLLC->pB = bParam;
	pLLC->pIn = LLCin;
	pLLC->pUout = LLCout;
}

void SetDXFeedForward(Axis *arrAxis,int iAxis)
{
	arrAxis->SingleFeedForward_x = *(comData.transConParam.dParamter[iAxis]);
}

void SetDXFeedForward_Switch(Axis *arrAxis,int iAxis)
{
	arrAxis->SingleFeedForward_x_Switch = *(comData.transConParam.dParamter[iAxis]);
}

void SetDYFeedForward(Axis *arrAxis,int iAxis)
{
	arrAxis->SingleFeedForward_y = *(comData.transConParam.dParamter[iAxis]);
}

void SetDYFeedForward_Switch(Axis *arrAxis,int iAxis)
{
	arrAxis->SingleFeedForward_y_Switch = *(comData.transConParam.dParamter[iAxis]);
}

void SetLimit(DAStruct *pDAlimit)
{
	int i = 0;
	for(i=0;i<4;i++)
	{
		(pDAlimit+i)->iLimit = comData.transConParam.dParamter[0][0];
	}
}

void GetPIDParam(PID *pPID,int iAxis)
{
	*(comData.transConParam.dParamter[iAxis]+0) = pPID->Proportion;
	*(comData.transConParam.dParamter[iAxis]+1) = pPID->Integral;
	*(comData.transConParam.dParamter[iAxis]+2) = pPID->Derivative;
}

void GetLLCParam(LLC *pLLC,int iAxis)
{
	int i = 0;
	int order = 0;
	order = pLLC->order;
	for(i=0;i<12;i++)
	{
		if(i<order)
		{
			*(comData.transConParam.dParamter[iAxis]+i) = *(pLLC->pA + i);
			*(comData.transConParam.dParamter[iAxis]+12+i) = *(pLLC->pB + i);
		}
		else
		{
			*(comData.transConParam.dParamter[iAxis]+i) = 0.0;
			*(comData.transConParam.dParamter[iAxis]+12+i) = 0.0;
		}
	}
}


void GetDXFeedForward(Axis *arrAxis,int iAxis)
{
	*(comData.transConParam.dParamter[iAxis]) = arrAxis->SingleFeedForward_x;
}

void GetDXFeedForward_Switch(Axis *arrAxis,int iAxis)
{
	*(comData.transConParam.dParamter[iAxis]) = arrAxis->SingleFeedForward_x_Switch;
}

void GetDYFeedForward(Axis *arrAxis,int iAxis)
{
	*(comData.transConParam.dParamter[iAxis]) = arrAxis->SingleFeedForward_y;
}

void GetDYFeedForward_Switch(Axis *arrAxis,int iAxis)
{
	*(comData.transConParam.dParamter[iAxis]) = arrAxis->SingleFeedForward_y_Switch;
}

void GetLimit(DAStruct *pDAlimit)
{
	int i = 0;
	comData.transConParam.dParamter[0][0] = (pDAlimit+0)->iLimit;
}
/*
void SetParam()
{
	int CAL_MIN_J_AND_D =1;
	double p_dd,p_ddY,p_ddRev;
	double Td, Tj, Ta, Tv;
	double TdY, TjY, TaY, TvY;
	int i=0;
	int iStageType = 0;
	int iDof = 0;
	int iParamType = 0;
	iStageType = recvData.stageType;
	iDof = recvData.iDof - 1;
	iParamType = recvData.paramType;
	switch(iStageType)
	{
		case 101:
		    TP.p = fabs (recvData.dParamData[0]);
			TP.v = recvData.dParamData[1];
			TP.a = recvData.dParamData[2];
			TP.j = recvData.dParamData[3];
			TP.d = recvData.dParamData[4];
			TP.s = 15;
			TP.r = 1e-9;
			TP.Ts = 0.0002;

		
		    TPY.p = fabs (recvData.dParamData[5]);
			TPY.v = recvData.dParamData[6];
			TPY.a = recvData.dParamData[7];
			TPY.j = recvData.dParamData[8];
			TPY.d = recvData.dParamData[9];
			TPY.s = 15;
			TPY.r = 1e-9;
			TPY.Ts = 0.0002;		
			
	//		TPGen_4Order_P2P(&TP, &p_dd, &Td, &Tj,&Ta, &Tv, CAL_MIN_J_AND_D);
	//		TPGen_4Order_P2P(&TPY, &p_ddY, &TdY, &TjY,&TaY, &TvY, CAL_MIN_J_AND_D);
			TPGen_4Order_P2P(&TP, CAL_MIN_J_AND_D,&Machine.coarseStage.arrAxis[0].sTP_Run);
			TPGen_4Order_P2P(&TP, CAL_MIN_J_AND_D,&Machine.coarseStage.arrAxis[0].sTP_Run);
			TPGen_4Order_P2P(&TPY,CAL_MIN_J_AND_D,&Machine.coarseStage.arrAxis[0].sTP_Run);
			TPGen_4Order_P2P(&TPY,CAL_MIN_J_AND_D,&Machine.coarseStage.arrAxis[0].sTP_Run);
/*
			XExpoParam[0]= TP.Ts;
			XExpoParam[1]= p_dd;
			XExpoParam[2]= Td;
			XExpoParam[3]= Tj;
			XExpoParam[4]= Ta;
			XExpoParam[5]= Tv;
			XExpoParam[6]= 0;
			XExpoParam[7]= 0;
			XExpoParam[8]= 0;
			XExpoParam[9]= 0;

			YExpoParam[0]= TPY.Ts;
			YExpoParam[1]= p_ddY;
			YExpoParam[2]= TdY;
			YExpoParam[3]= TjY;
			YExpoParam[4]= TaY;
			YExpoParam[5]= TvY;
			YExpoParam[6]= 0;
			YExpoParam[7]= 0;
			YExpoParam[8]= 0;
			YExpoParam[9]= 0;

			for(i=0;i<10;i++)
			{
				printf("%lf\n",XExpoParam[i]);
			}
			for(i=0;i<10;i++)
			{
				printf("%lf\n",YExpoParam[i]);

			}

			GetRunData(&Machine.coarseStage.arrAxis[0].sTP_Run,XExpoParam);
			GetRunData(&Machine.coarseStage.arrAxis[1].sTP_Run,XExpoParam);
				
			GetRunData(&Machine.coarseStage.arrAxis[2].sTP_Run,YExpoParam);
			GetRunData(&Machine.coarseStage.arrAxis[3].sTP_Run,YExpoParam);
				
			GetRunData(&Machine.fineStage.arrAxis[0].sTP_Run,XExpoParam);
			GetRunData(&Machine.fineStage.arrAxis[1].sTP_Run,YExpoParam); 
	//
        break;
		case CoarseStage:
			switch((AxisCoarseIndex)iDof)
			{
				case CDX:
					switch(iParamType)
					{
						case PID_INDEPENDENT_TYPE:
							//SetPIDParam(&Machine.coarseStage.arrAxis[CDX].sPID,CDX);							
							Machine.coarseStage.arrAxis[CDX].sPID.Proportion = recvData.dParamData[0];
							Machine.coarseStage.arrAxis[CDX].sPID.Integral = recvData.dParamData[1];
							Machine.coarseStage.arrAxis[CDX].sPID.Derivative = recvData.dParamData[2];
						break;
						case PID_FOLLOW_TYPE:
							//SetPIDParam(&Machine.coarseStage.arrAxis[CDX].sPID_Switch,CDX);
							Machine.coarseStage.arrAxis[CDX].sPID_Switch.Proportion = recvData.dParamData[0];
							Machine.coarseStage.arrAxis[CDX].sPID_Switch.Integral = recvData.dParamData[1];
							Machine.coarseStage.arrAxis[CDX].sPID_Switch.Derivative = recvData.dParamData[2];														
						break;
						case LLC_INDEPENDENT_TYPE:
							SetLLCParam(&Machine.coarseStage.arrAxis[CDX].sLLC,CDX);
						break;
						case LLC_FOLLOW_TYPE:
							SetLLCParam(&Machine.coarseStage.arrAxis[CDX].sLLC_Switch,CDX);
						break;
						case ACC_COEFFICIENT_INDEPENDENT_DXMOVE:
							SetDXFeedForward(&Machine.coarseStage.arrAxis[CDX],CDX);
						break;
						case ACC_COEFFICIENT_FOLLOW_DXMOVE:
							SetDXFeedForward_Switch(&Machine.coarseStage.arrAxis[CDX],CDX);
						break;
						case ACC_COEFFICIENT_INDEPENDENT_DYMOVE:
							SetDYFeedForward(&Machine.coarseStage.arrAxis[CDX],CDX);
						break;
						case ACC_COEFFICIENT_FOLLOW_DYMOVE:
							SetDYFeedForward_Switch(&Machine.coarseStage.arrAxis[CDX],CDX);
						break;
						case LIMIT_TYPE:

							Machine.coarseStage.dDALimit = recvData.dParamData[0]/1000;
							
							//SetLimit(Machine.coarseStage.DA_Struct);
						break;
					}
				break;
				case CDY:
					switch(iParamType)
					{
						case PID_INDEPENDENT_TYPE:
							//SetPIDParam(&Machine.coarseStage.arrAxis[CDY].sPID,CDY);
							Machine.coarseStage.arrAxis[CDY].sPID.Proportion=recvData.dParamData[0];
							Machine.coarseStage.arrAxis[CDY].sPID.Integral=recvData.dParamData[1];
							Machine.coarseStage.arrAxis[CDY].sPID.Derivative=recvData.dParamData[2];
						break;
						case PID_FOLLOW_TYPE:
							//SetPIDParam(&Machine.coarseStage.arrAxis[CDY].sPID_Switch,CDY);
							Machine.coarseStage.arrAxis[CDY].sPID_Switch.Proportion=recvData.dParamData[0];
							Machine.coarseStage.arrAxis[CDY].sPID_Switch.Integral=recvData.dParamData[1];
							Machine.coarseStage.arrAxis[CDY].sPID_Switch.Derivative=recvData.dParamData[2];							
						break;
						case LLC_INDEPENDENT_TYPE:
							SetLLCParam(&Machine.coarseStage.arrAxis[CDY].sLLC,CDY);
						break;
						case LLC_FOLLOW_TYPE:
							SetLLCParam(&Machine.coarseStage.arrAxis[CDY].sLLC_Switch,CDY);
						break;
						case ACC_COEFFICIENT_INDEPENDENT_DXMOVE:
							SetDXFeedForward(&Machine.coarseStage.arrAxis[CDY],CDY);
						break;
						case ACC_COEFFICIENT_FOLLOW_DXMOVE:
							SetDXFeedForward_Switch(&Machine.coarseStage.arrAxis[CDY],CDY);
						break;
						case ACC_COEFFICIENT_INDEPENDENT_DYMOVE:
							SetDYFeedForward(&Machine.coarseStage.arrAxis[CDY],CDY);
						break;
						case ACC_COEFFICIENT_FOLLOW_DYMOVE:
							SetDYFeedForward_Switch(&Machine.coarseStage.arrAxis[CDY],CDY);
						break;
						case LIMIT_TYPE:
							Machine.coarseStage.dDALimit = recvData.dParamData[0]/1000;
							logMsg("CDY\n",0,0,0,0,0,0);
							//SetLimit(Machine.coarseStage.DA_Struct);
						break;
					}
				break;
				case XTZ:
					switch(iParamType)
					{
						case PID_INDEPENDENT_TYPE:
							//SetPIDParam(&Machine.coarseStage.arrAxis[XTZ].sPID,XTZ);
							Machine.coarseStage.arrAxis[XTZ].sPID.Proportion=recvData.dParamData[0];
							Machine.coarseStage.arrAxis[XTZ].sPID.Integral=recvData.dParamData[1];
							Machine.coarseStage.arrAxis[XTZ].sPID.Derivative=recvData.dParamData[2];							
						break;
						case PID_FOLLOW_TYPE:
							//SetPIDParam(&Machine.coarseStage.arrAxis[XTZ].sPID_Switch,XTZ);
							Machine.coarseStage.arrAxis[XTZ].sPID_Switch.Proportion=recvData.dParamData[0];
							Machine.coarseStage.arrAxis[XTZ].sPID_Switch.Integral=recvData.dParamData[1];
							Machine.coarseStage.arrAxis[XTZ].sPID_Switch.Derivative=recvData.dParamData[2];							
	
					
						break;
						case LLC_INDEPENDENT_TYPE:
							SetLLCParam(&Machine.coarseStage.arrAxis[XTZ].sLLC,XTZ);
						break;
						case LLC_FOLLOW_TYPE:
							SetLLCParam(&Machine.coarseStage.arrAxis[XTZ].sLLC_Switch,XTZ);
						break;
						case ACC_COEFFICIENT_INDEPENDENT_DXMOVE:
							SetDXFeedForward(&Machine.coarseStage.arrAxis[XTZ],XTZ);
						break;
						case ACC_COEFFICIENT_FOLLOW_DXMOVE:
							SetDXFeedForward_Switch(&Machine.coarseStage.arrAxis[XTZ],XTZ);
						break;
						case ACC_COEFFICIENT_INDEPENDENT_DYMOVE:
							SetDYFeedForward(&Machine.coarseStage.arrAxis[XTZ],XTZ);
						break;
						case ACC_COEFFICIENT_FOLLOW_DYMOVE:
							SetDYFeedForward_Switch(&Machine.coarseStage.arrAxis[XTZ],XTZ);
						break;
					}
				break;
				case YTZ:
					switch(iParamType)
					{
						case PID_INDEPENDENT_TYPE:
							//SetPIDParam(&Machine.coarseStage.arrAxis[YTZ].sPID,YTZ);
							Machine.coarseStage.arrAxis[YTZ].sPID.Proportion = recvData.dParamData[0];
							Machine.coarseStage.arrAxis[YTZ].sPID.Integral = recvData.dParamData[1];
							Machine.coarseStage.arrAxis[YTZ].sPID.Derivative = recvData.dParamData[2];
						break;
						case PID_FOLLOW_TYPE:
							//SetPIDParam(&Machine.coarseStage.arrAxis[YTZ].sPID_Switch,YTZ);
							Machine.coarseStage.arrAxis[YTZ].sPID_Switch.Proportion=recvData.dParamData[0];
							Machine.coarseStage.arrAxis[YTZ].sPID_Switch.Integral=recvData.dParamData[1];
							Machine.coarseStage.arrAxis[YTZ].sPID_Switch.Derivative=recvData.dParamData[2];
						break;
						case LLC_INDEPENDENT_TYPE:
							SetLLCParam(&Machine.coarseStage.arrAxis[YTZ].sLLC,YTZ);
						break;
						case LLC_FOLLOW_TYPE:
							SetLLCParam(&Machine.coarseStage.arrAxis[YTZ].sLLC_Switch,YTZ);
						break;
						case ACC_COEFFICIENT_INDEPENDENT_DXMOVE:
							SetDXFeedForward(&Machine.coarseStage.arrAxis[YTZ],YTZ);
						break;
						case ACC_COEFFICIENT_FOLLOW_DXMOVE:
							SetDXFeedForward_Switch(&Machine.coarseStage.arrAxis[YTZ],YTZ);
						break;
						case ACC_COEFFICIENT_INDEPENDENT_DYMOVE:
							SetDYFeedForward(&Machine.coarseStage.arrAxis[YTZ],YTZ);
						break;
						case ACC_COEFFICIENT_FOLLOW_DYMOVE:
							SetDYFeedForward_Switch(&Machine.coarseStage.arrAxis[YTZ],YTZ);
						break;
					}
				break;
			}
		break;
		case FineStage:
			switch((AxisFineIndex)iDof)
			{
				case FDX:
					switch(iParamType)
					{
						case PID_INDEPENDENT_TYPE:
							//SetPIDParam(&Machine.fineStage.arrAxis[FDX].sPID,FDX);
							Machine.fineStage.arrAxis[FDX].sPID.Proportion=recvData.dParamData[0];
							Machine.fineStage.arrAxis[FDX].sPID.Integral=recvData.dParamData[1];
							Machine.fineStage.arrAxis[FDX].sPID.Derivative=recvData.dParamData[2];
						break;
						case PID_FOLLOW_TYPE:
							//SetPIDParam(&Machine.fineStage.arrAxis[FDX].sPID_Switch,FDX);
							Machine.fineStage.arrAxis[FDX].sPID_Switch.Proportion=recvData.dParamData[0];
							Machine.fineStage.arrAxis[FDX].sPID_Switch.Integral=recvData.dParamData[1];
							Machine.fineStage.arrAxis[FDX].sPID_Switch.Derivative=recvData.dParamData[2];
						break;
						case LLC_INDEPENDENT_TYPE:
							SetLLCParam(&Machine.fineStage.arrAxis[FDX].sLLC,FDX);
						break;
						case LLC_FOLLOW_TYPE:
							SetLLCParam(&Machine.fineStage.arrAxis[FDX].sLLC_Switch,FDX);
					break;
					case ACC_COEFFICIENT_INDEPENDENT_DXMOVE:
							SetDXFeedForward(&Machine.fineStage.arrAxis[FDX],FDX);
						break;
						case ACC_COEFFICIENT_FOLLOW_DXMOVE:
							SetDXFeedForward_Switch(&Machine.fineStage.arrAxis[FDX],FDX);
						break;
						case ACC_COEFFICIENT_INDEPENDENT_DYMOVE:
							SetDYFeedForward(&Machine.fineStage.arrAxis[FDX],FDX);
						break;
						case ACC_COEFFICIENT_FOLLOW_DYMOVE:
							SetDYFeedForward_Switch(&Machine.fineStage.arrAxis[FDX],FDX);
						break;
						case LIMIT_TYPE:
							SetLimit(Machine.fineStage.DA_Struct);
						break;
					}
				break;
				case FDY:
					switch(iParamType)
					{
						case PID_INDEPENDENT_TYPE:
							//SetPIDParam(&Machine.fineStage.arrAxis[FDY].sPID,FDY);
							Machine.fineStage.arrAxis[FDY].sPID.Proportion=recvData.dParamData[0];
							Machine.fineStage.arrAxis[FDY].sPID.Integral=recvData.dParamData[1];
							Machine.fineStage.arrAxis[FDY].sPID.Derivative=recvData.dParamData[2];
						break;
						case PID_FOLLOW_TYPE:							
							//SetPIDParam(&Machine.fineStage.arrAxis[FDY].sPID_Switch,FDY);
							Machine.fineStage.arrAxis[FDY].sPID_Switch.Proportion=recvData.dParamData[0];
							Machine.fineStage.arrAxis[FDY].sPID_Switch.Integral=recvData.dParamData[1];
							Machine.fineStage.arrAxis[FDY].sPID_Switch.Derivative=recvData.dParamData[2];

						break;
						case LLC_INDEPENDENT_TYPE:
							SetLLCParam(&Machine.fineStage.arrAxis[FDY].sLLC,FDY);
						break;
						case LLC_FOLLOW_TYPE:
							SetLLCParam(&Machine.fineStage.arrAxis[FDY].sLLC_Switch,FDY);
						break;
						case ACC_COEFFICIENT_INDEPENDENT_DXMOVE:
							SetDXFeedForward(&Machine.fineStage.arrAxis[FDY],FDY);
						break;
						case ACC_COEFFICIENT_FOLLOW_DXMOVE:
							SetDXFeedForward_Switch(&Machine.fineStage.arrAxis[FDY],FDY);
						break;
						case ACC_COEFFICIENT_INDEPENDENT_DYMOVE:
							SetDYFeedForward(&Machine.fineStage.arrAxis[FDY],FDY);
						break;
						case ACC_COEFFICIENT_FOLLOW_DYMOVE:
							SetDYFeedForward_Switch(&Machine.fineStage.arrAxis[FDY],FDY);
						break;
					}
				break;



				case 2://FDZ:
					switch(iParamType)
					{
						case PID_INDEPENDENT_TYPE:
							//SetPIDParam(&Machine.fineStage.arrAxis[FDX].sPID,FDX);
							logMsg("RevNumber is \n",2,0,0,0,0,0);
							Machine.fineStage.arrAxis[FDZ].sPID.Proportion=recvData.dParamData[0];
							Machine.fineStage.arrAxis[FDZ].sPID.Integral=recvData.dParamData[1];
							Machine.fineStage.arrAxis[FDZ].sPID.Derivative=recvData.dParamData[2];
						break;
						case PID_FOLLOW_TYPE:
							//SetPIDParam(&Machine.fineStage.arrAxis[FDX].sPID_Switch,FDX);
							Machine.fineStage.arrAxis[FDZ].sPID_Switch.Proportion=recvData.dParamData[0];
							Machine.fineStage.arrAxis[FDZ].sPID_Switch.Integral=recvData.dParamData[1];
							Machine.fineStage.arrAxis[FDZ].sPID_Switch.Derivative=recvData.dParamData[2];
						break;
						case LLC_INDEPENDENT_TYPE:
							SetLLCParam(&Machine.fineStage.arrAxis[FDZ].sLLC,FDZ);
						break;
						case LLC_FOLLOW_TYPE:
							SetLLCParam(&Machine.fineStage.arrAxis[FDZ].sLLC_Switch,FDZ);
						break;
						case ACC_COEFFICIENT_INDEPENDENT_DXMOVE:
							SetDXFeedForward(&Machine.fineStage.arrAxis[FDZ],FDZ);
						break;
						case ACC_COEFFICIENT_FOLLOW_DXMOVE:
							SetDXFeedForward_Switch(&Machine.fineStage.arrAxis[FDZ],FDZ);
						break;
						case ACC_COEFFICIENT_INDEPENDENT_DYMOVE:
							SetDYFeedForward(&Machine.fineStage.arrAxis[FDZ],FDZ);
						break;
						case ACC_COEFFICIENT_FOLLOW_DYMOVE:
							SetDYFeedForward_Switch(&Machine.fineStage.arrAxis[FDZ],FDZ);
						break;
						case LIMIT_TYPE:
							SetLimit(Machine.fineStage.DA_Struct);
						break;
					}
				break;

				case 3://FTX:
					switch(iParamType)
					{
						case PID_INDEPENDENT_TYPE:
							//SetPIDParam(&Machine.fineStage.arrAxis[FDX].sPID,FDX);
							Machine.fineStage.arrAxis[FTX].sPID.Proportion=recvData.dParamData[0];
							Machine.fineStage.arrAxis[FTX].sPID.Integral=recvData.dParamData[1];
							Machine.fineStage.arrAxis[FTX].sPID.Derivative=recvData.dParamData[2];
						break;
						case PID_FOLLOW_TYPE:
							//SetPIDParam(&Machine.fineStage.arrAxis[FDX].sPID_Switch,FDX);
							Machine.fineStage.arrAxis[FTX].sPID_Switch.Proportion=recvData.dParamData[0];
							Machine.fineStage.arrAxis[FTX].sPID_Switch.Integral=recvData.dParamData[1];
							Machine.fineStage.arrAxis[FTX].sPID_Switch.Derivative=recvData.dParamData[2];
						break;
						case LLC_INDEPENDENT_TYPE:
							SetLLCParam(&Machine.fineStage.arrAxis[FTX].sLLC,FTX);
						break;
						case LLC_FOLLOW_TYPE:
							SetLLCParam(&Machine.fineStage.arrAxis[FTX].sLLC_Switch,FTX);
						break;
						case ACC_COEFFICIENT_FOLLOW_DXMOVE:
							SetDXFeedForward_Switch(&Machine.fineStage.arrAxis[FTX],FTX);
						break;
						case ACC_COEFFICIENT_INDEPENDENT_DYMOVE:
							SetDYFeedForward(&Machine.fineStage.arrAxis[FTX],FTX);
						break;
						case ACC_COEFFICIENT_FOLLOW_DYMOVE:
							SetDYFeedForward_Switch(&Machine.fineStage.arrAxis[FTX],FTX);
						break;
						case LIMIT_TYPE:
							SetLimit(Machine.fineStage.DA_Struct);
						break;
					}
				break;


				case 4://FTY:
					switch(iParamType)
					{
						case PID_INDEPENDENT_TYPE:
							//SetPIDParam(&Machine.fineStage.arrAxis[FDX].sPID,FDX);
							Machine.fineStage.arrAxis[FTY].sPID.Proportion=recvData.dParamData[0];
							Machine.fineStage.arrAxis[FTY].sPID.Integral=recvData.dParamData[1];
							Machine.fineStage.arrAxis[FTY].sPID.Derivative=recvData.dParamData[2];
						break;
						case PID_FOLLOW_TYPE:
							//SetPIDParam(&Machine.fineStage.arrAxis[FDX].sPID_Switch,FDX);
							Machine.fineStage.arrAxis[FTY].sPID_Switch.Proportion=recvData.dParamData[0];
							Machine.fineStage.arrAxis[FTY].sPID_Switch.Integral=recvData.dParamData[1];
							Machine.fineStage.arrAxis[FTY].sPID_Switch.Derivative=recvData.dParamData[2];
						break;
						case LLC_INDEPENDENT_TYPE:
							SetLLCParam(&Machine.fineStage.arrAxis[FTY].sLLC,FTY);
						break;
						case LLC_FOLLOW_TYPE:
							SetLLCParam(&Machine.fineStage.arrAxis[FTY].sLLC_Switch,FTY);
						break;
						case ACC_COEFFICIENT_INDEPENDENT_DXMOVE:
							SetDXFeedForward(&Machine.fineStage.arrAxis[FTY],FTY);
						break;
						case ACC_COEFFICIENT_FOLLOW_DXMOVE:
							SetDXFeedForward_Switch(&Machine.fineStage.arrAxis[FTY],FTY);
						break;
						case ACC_COEFFICIENT_INDEPENDENT_DYMOVE:
							SetDYFeedForward(&Machine.fineStage.arrAxis[FTY],FTY);
						break;
						case ACC_COEFFICIENT_FOLLOW_DYMOVE:
							SetDYFeedForward_Switch(&Machine.fineStage.arrAxis[FTY],FTY);
						break;
						case LIMIT_TYPE:
							SetLimit(Machine.fineStage.DA_Struct);
						break;
					}
				break;



				
				case 5://FTZ:
					switch(iParamType)
					{
						case PID_INDEPENDENT_TYPE:
							//SetPIDParam(&Machine.fineStage.arrAxis[FTZ].sPID,FTZ);							
							Machine.fineStage.arrAxis[FTZ].sPID.Proportion=recvData.dParamData[0];
							Machine.fineStage.arrAxis[FTZ].sPID.Integral=recvData.dParamData[1];
							Machine.fineStage.arrAxis[FTZ].sPID.Derivative=recvData.dParamData[2];
						break;
						case PID_FOLLOW_TYPE:
							//SetPIDParam(&Machine.fineStage.arrAxis[FTZ].sPID_Switch,FTZ);
							Machine.fineStage.arrAxis[FTZ].sPID_Switch.Proportion=recvData.dParamData[0];
							Machine.fineStage.arrAxis[FTZ].sPID_Switch.Integral=recvData.dParamData[1];
							Machine.fineStage.arrAxis[FTZ].sPID_Switch.Derivative=recvData.dParamData[2];
						
						break;
						case LLC_INDEPENDENT_TYPE:
							SetLLCParam(&Machine.fineStage.arrAxis[FTZ].sLLC,FTZ);
						break;
						case LLC_FOLLOW_TYPE:
							SetLLCParam(&Machine.fineStage.arrAxis[FTZ].sLLC_Switch,FTZ);
						break;
						case ACC_COEFFICIENT_INDEPENDENT_DXMOVE:
							SetDXFeedForward(&Machine.fineStage.arrAxis[FTZ],FTZ);
						break;
						case ACC_COEFFICIENT_FOLLOW_DXMOVE:
							SetDXFeedForward_Switch(&Machine.fineStage.arrAxis[FTZ],FTZ);
						break;
						case ACC_COEFFICIENT_INDEPENDENT_DYMOVE:
							SetDYFeedForward(&Machine.fineStage.arrAxis[FTZ],FTZ);
						break;
						case ACC_COEFFICIENT_FOLLOW_DYMOVE:
							SetDYFeedForward_Switch(&Machine.fineStage.arrAxis[FTZ],FTZ);
						break;
					}
				break;
			}
		break;
	}

	
}

void GetParam()
{
	int iStageType = FineStage;
	int iDof = 99;
	int iParamType = PID_INDEPENDENT_TYPE;
	int i = 0;

	iStageType = recvData.stageType;
	iDof = recvData.iDof - 1;
	iParamType = recvData.paramType;
	recvData.stageType=0;
	switch(iStageType)
	{
//		logMsg("SendNumber0 is %d\n",iStageType,0,0,0,0,0);
		case CoarseStage:
		logMsg("SendNumber1 is %d\n",comData.iCoarseData[10],0,0,0,0,0);
//		printf("SendNumber1 is %d\n",comData.iCoarseData[10]);
			switch((AxisCoarseIndex)iDof)
			{
				case CDX:
					switch(iParamType)
					{
						case PID_INDEPENDENT_TYPE:
							//GetPIDParam(&Machine.coarseStage.arrAxis[CDX].sPID,CDX);
							comData.dFineData[0]=Machine.coarseStage.arrAxis[CDX].sPID.Proportion;
							comData.dFineData[1]=Machine.coarseStage.arrAxis[CDX].sPID.Integral;
							comData.dFineData[2]=Machine.coarseStage.arrAxis[CDX].sPID.Derivative;	
							break;
						case PID_FOLLOW_TYPE:							
							//GetPIDParam(&Machine.coarseStage.arrAxis[CDX].sPID_Switch,CDX);
							comData.dFineData[0]=Machine.coarseStage.arrAxis[CDX].sPID_Switch.Proportion;
							comData.dFineData[1]=Machine.coarseStage.arrAxis[CDX].sPID_Switch.Integral;
							comData.dFineData[2]=Machine.coarseStage.arrAxis[CDX].sPID_Switch.Derivative;	
	
							break;
						case LLC_INDEPENDENT_TYPE:
							GetLLCParam(&Machine.coarseStage.arrAxis[CDX].sLLC,CDX);
							break;
						case LLC_FOLLOW_TYPE:
							GetLLCParam(&Machine.coarseStage.arrAxis[CDX].sLLC_Switch,CDX);
							break;
						case ACC_COEFFICIENT_INDEPENDENT_DXMOVE:
							GetDXFeedForward(&Machine.coarseStage.arrAxis[CDX],CDX);
							break;
						case ACC_COEFFICIENT_FOLLOW_DXMOVE:
							GetDXFeedForward_Switch(&Machine.coarseStage.arrAxis[CDX],CDX);
							break;
						case ACC_COEFFICIENT_INDEPENDENT_DYMOVE:
							GetDYFeedForward(&Machine.coarseStage.arrAxis[CDX],CDX);
							break;
						case ACC_COEFFICIENT_FOLLOW_DYMOVE:
							GetDYFeedForward_Switch(&Machine.coarseStage.arrAxis[CDX],CDX);
							break;
					}
					break;
			case CDY:
				switch(iParamType)
				{
					case PID_INDEPENDENT_TYPE:
						//GetPIDParam(&Machine.coarseStage.arrAxis[CDY].sPID,CDY);
						comData.dFineData[0]=Machine.coarseStage.arrAxis[CDY].sPID.Proportion;
						comData.dFineData[1]=Machine.coarseStage.arrAxis[CDY].sPID.Integral;
						comData.dFineData[2]=Machine.coarseStage.arrAxis[CDY].sPID.Derivative;		
						break;
					case PID_FOLLOW_TYPE:
						//GetPIDParam(&Machine.coarseStage.arrAxis[CDY].sPID_Switch,CDY);
						comData.dFineData[0]=Machine.coarseStage.arrAxis[CDY].sPID_Switch.Proportion;
						comData.dFineData[1]=Machine.coarseStage.arrAxis[CDY].sPID_Switch.Integral;
						comData.dFineData[2]=Machine.coarseStage.arrAxis[CDY].sPID_Switch.Derivative;							
						
						break;
					case LLC_INDEPENDENT_TYPE:
						GetLLCParam(&Machine.coarseStage.arrAxis[CDY].sLLC,CDY);
						break;
					case LLC_FOLLOW_TYPE:
						GetLLCParam(&Machine.coarseStage.arrAxis[CDY].sLLC_Switch,CDY);
						break;
	
					case ACC_COEFFICIENT_INDEPENDENT_DXMOVE:
						GetDXFeedForward(&Machine.coarseStage.arrAxis[CDY],CDY);
						break;
					case ACC_COEFFICIENT_FOLLOW_DXMOVE:
						GetDXFeedForward_Switch(&Machine.coarseStage.arrAxis[CDY],CDY);
						break;
					case ACC_COEFFICIENT_INDEPENDENT_DYMOVE:
						GetDYFeedForward(&Machine.coarseStage.arrAxis[CDY],CDY);
						break;
					case ACC_COEFFICIENT_FOLLOW_DYMOVE:
						GetDYFeedForward_Switch(&Machine.coarseStage.arrAxis[CDY],CDY);
						break;
				}
				break;
			case XTZ:
					switch(iParamType)
					{
						case PID_INDEPENDENT_TYPE:
							//GetPIDParam(&Machine.coarseStage.arrAxis[XTZ].sPID,XTZ);
							comData.dFineData[0]=Machine.coarseStage.arrAxis[XTZ].sPID.Proportion;
							comData.dFineData[1]=Machine.coarseStage.arrAxis[XTZ].sPID.Integral;
							comData.dFineData[2]=Machine.coarseStage.arrAxis[XTZ].sPID.Derivative;	
						break;
						case PID_FOLLOW_TYPE:
							//GetPIDParam(&Machine.coarseStage.arrAxis[XTZ].sPID_Switch,XTZ);
							comData.dFineData[0]=Machine.coarseStage.arrAxis[XTZ].sPID_Switch.Proportion;
							comData.dFineData[1]=Machine.coarseStage.arrAxis[XTZ].sPID_Switch.Integral;
							comData.dFineData[2]=Machine.coarseStage.arrAxis[XTZ].sPID_Switch.Derivative;	
														
						break;
						case LLC_INDEPENDENT_TYPE:
							GetLLCParam(&Machine.coarseStage.arrAxis[XTZ].sLLC,XTZ);
						break;
						case LLC_FOLLOW_TYPE:
							GetLLCParam(&Machine.coarseStage.arrAxis[XTZ].sLLC_Switch,XTZ);
						break;
						case ACC_COEFFICIENT_INDEPENDENT_DXMOVE:
							GetDXFeedForward(&Machine.coarseStage.arrAxis[XTZ],XTZ);
						break;
						case ACC_COEFFICIENT_FOLLOW_DXMOVE:
							GetDXFeedForward_Switch(&Machine.coarseStage.arrAxis[XTZ],XTZ);
						break;
						case ACC_COEFFICIENT_INDEPENDENT_DYMOVE:
							GetDYFeedForward(&Machine.coarseStage.arrAxis[XTZ],XTZ);
						break;
						case ACC_COEFFICIENT_FOLLOW_DYMOVE:
							GetDYFeedForward_Switch(&Machine.coarseStage.arrAxis[XTZ],XTZ);
						break;
					}
				break;
				case YTZ:
					switch(iParamType)
					{
						case PID_INDEPENDENT_TYPE:
							//GetPIDParam(&Machine.coarseStage.arrAxis[YTZ].sPID,YTZ);
							comData.dFineData[0]=Machine.coarseStage.arrAxis[YTZ].sPID.Proportion;
							comData.dFineData[1]=Machine.coarseStage.arrAxis[YTZ].sPID.Integral;
							comData.dFineData[2]=Machine.coarseStage.arrAxis[YTZ].sPID.Derivative;
						break;
						case PID_FOLLOW_TYPE:
							//GetPIDParam(&Machine.coarseStage.arrAxis[YTZ].sPID_Switch,YTZ);
							comData.dFineData[0]=Machine.coarseStage.arrAxis[YTZ].sPID_Switch.Proportion;
							comData.dFineData[1]=Machine.coarseStage.arrAxis[YTZ].sPID_Switch.Integral;
							comData.dFineData[2]=Machine.coarseStage.arrAxis[YTZ].sPID_Switch.Derivative;
							
						break;
						case LLC_INDEPENDENT_TYPE:
							GetLLCParam(&Machine.coarseStage.arrAxis[YTZ].sLLC,YTZ);
						break;
						case LLC_FOLLOW_TYPE:
							GetLLCParam(&Machine.coarseStage.arrAxis[YTZ].sLLC_Switch,YTZ);
						break;
						case ACC_COEFFICIENT_INDEPENDENT_DXMOVE:
							GetDXFeedForward(&Machine.coarseStage.arrAxis[YTZ],YTZ);
						break;
						case ACC_COEFFICIENT_FOLLOW_DXMOVE:
							GetDXFeedForward_Switch(&Machine.coarseStage.arrAxis[YTZ],YTZ);
						break;
						case ACC_COEFFICIENT_INDEPENDENT_DYMOVE:
							GetDYFeedForward(&Machine.coarseStage.arrAxis[YTZ],YTZ);
						break;
						case ACC_COEFFICIENT_FOLLOW_DYMOVE:
							GetDYFeedForward_Switch(&Machine.coarseStage.arrAxis[YTZ],YTZ);
						break;
					}
				break;
				case ALLREFRESH_COARSE:
					switch(iParamType)
					{
						case PID_INDEPENDENT_TYPE:
							comData.dFineData[0]=Machine.coarseStage.arrAxis[CDX].sPID.Proportion;
							comData.dFineData[1]=Machine.coarseStage.arrAxis[CDX].sPID.Integral;
							comData.dFineData[2]=Machine.coarseStage.arrAxis[CDX].sPID.Derivative;	

							comData.dFineData[3]=Machine.coarseStage.arrAxis[CDY].sPID.Proportion;
							comData.dFineData[4]=Machine.coarseStage.arrAxis[CDY].sPID.Integral;
							comData.dFineData[5]=Machine.coarseStage.arrAxis[CDY].sPID.Derivative;	

							comData.dFineData[6]=Machine.coarseStage.arrAxis[XTZ].sPID.Proportion;
							comData.dFineData[7]=Machine.coarseStage.arrAxis[XTZ].sPID.Integral;
							comData.dFineData[8]=Machine.coarseStage.arrAxis[XTZ].sPID.Derivative;	
							
							comData.dFineData[9]=Machine.coarseStage.arrAxis[YTZ].sPID.Proportion;
							comData.dFineData[10]=Machine.coarseStage.arrAxis[YTZ].sPID.Integral;
							comData.dFineData[11]=Machine.coarseStage.arrAxis[YTZ].sPID.Derivative;	
						break;
						case PID_FOLLOW_TYPE:
							comData.dFineData[0]=Machine.coarseStage.arrAxis[CDX].sPID_Switch.Proportion;
							comData.dFineData[1]=Machine.coarseStage.arrAxis[CDX].sPID_Switch.Integral;
							comData.dFineData[2]=Machine.coarseStage.arrAxis[CDX].sPID_Switch.Derivative;	

							comData.dFineData[3]=Machine.coarseStage.arrAxis[CDY].sPID_Switch.Proportion;
							comData.dFineData[4]=Machine.coarseStage.arrAxis[CDY].sPID_Switch.Integral;
							comData.dFineData[5]=Machine.coarseStage.arrAxis[CDY].sPID_Switch.Derivative;	

							comData.dFineData[6]=Machine.coarseStage.arrAxis[XTZ].sPID_Switch.Proportion;
							comData.dFineData[7]=Machine.coarseStage.arrAxis[XTZ].sPID_Switch.Integral;
							comData.dFineData[8]=Machine.coarseStage.arrAxis[XTZ].sPID_Switch.Derivative;	


							comData.dFineData[9]=Machine.coarseStage.arrAxis[YTZ].sPID_Switch.Proportion;
							comData.dFineData[10]=Machine.coarseStage.arrAxis[YTZ].sPID_Switch.Integral;
							comData.dFineData[11]=Machine.coarseStage.arrAxis[YTZ].sPID_Switch.Derivative;	
						break;
						case LLC_INDEPENDENT_TYPE:
							for(i=0;i<4;i++)
							{
								GetLLCParam(&Machine.coarseStage.arrAxis[i].sLLC,i);
							}
						break;
						case LLC_FOLLOW_TYPE:
							for(i=0;i<4;i++)
							{
								GetLLCParam(&Machine.coarseStage.arrAxis[i].sLLC_Switch,i);
							}
						break;
						case ACC_COEFFICIENT_INDEPENDENT_DXMOVE:
							for(i=0;i<4;i++)
							{
								GetDXFeedForward(&Machine.coarseStage.arrAxis[i],i);
							}
						break;
						case ACC_COEFFICIENT_FOLLOW_DXMOVE:
							for(i=0;i<4;i++)
							{
								GetDXFeedForward_Switch(&Machine.coarseStage.arrAxis[i],i);
							}
						break;
						case ACC_COEFFICIENT_INDEPENDENT_DYMOVE:
							for(i=0;i<4;i++)
							{
								GetDYFeedForward(&Machine.coarseStage.arrAxis[i],i);
							}
						break;
						case ACC_COEFFICIENT_FOLLOW_DYMOVE:
							for(i=0;i<4;i++)
							{
								GetDYFeedForward_Switch(&Machine.coarseStage.arrAxis[i],i);
							}
						break;
						case LIMIT_TYPE:							
							comData.dFineData[0]=Machine.coarseStage.dDALimit*1000;
							//GetLimit(Machine.coarseStage.DA_Struct);
						break;
					}
				break;
			}
		break;
		case FineStage:
			switch((AxisFineIndex)iDof)
			{
				case FDX:
					switch(iParamType)
					{
						case PID_INDEPENDENT_TYPE:
							//GetPIDParam(&Machine.fineStage.arrAxis[FDX].sPID,FDX);
							comData.dFineData[0]=Machine.fineStage.arrAxis[FDX].sPID.Proportion;
							comData.dFineData[1]=Machine.fineStage.arrAxis[FDX].sPID.Integral;
							comData.dFineData[2]=Machine.fineStage.arrAxis[FDX].sPID.Derivative;
						break;
						case PID_FOLLOW_TYPE:
							//GetPIDParam(&Machine.fineStage.arrAxis[FDX].sPID_Switch,FDX);
							comData.dFineData[0]=Machine.fineStage.arrAxis[FDX].sPID_Switch.Proportion;
							comData.dFineData[1]=Machine.fineStage.arrAxis[FDX].sPID_Switch.Integral;
							comData.dFineData[2]=Machine.fineStage.arrAxis[FDX].sPID_Switch.Derivative;	
						break;
						case LLC_INDEPENDENT_TYPE:
							GetLLCParam(&Machine.fineStage.arrAxis[FDX].sLLC,FDX);
						break;
						case LLC_FOLLOW_TYPE:
							GetLLCParam(&Machine.fineStage.arrAxis[FDX].sLLC_Switch,FDX);
						break;
						case ACC_COEFFICIENT_INDEPENDENT_DXMOVE:
							GetDXFeedForward(&Machine.fineStage.arrAxis[FDX],FDX);
						break;
						case ACC_COEFFICIENT_FOLLOW_DXMOVE:
							GetDXFeedForward_Switch(&Machine.fineStage.arrAxis[FDX],FDX);
						break;
						case ACC_COEFFICIENT_INDEPENDENT_DYMOVE:
							GetDYFeedForward(&Machine.fineStage.arrAxis[FDX],FDX);
						break;
						case ACC_COEFFICIENT_FOLLOW_DYMOVE:
							GetDYFeedForward_Switch(&Machine.fineStage.arrAxis[FDX],FDX);
						break;
					}
				break;
				case FDY:
					switch(iParamType)
					{
						case PID_INDEPENDENT_TYPE:
							//GetPIDParam(&Machine.fineStage.arrAxis[FDY].sPID,FDY);
							comData.dFineData[0]=Machine.fineStage.arrAxis[FDY].sPID.Proportion;
							comData.dFineData[1]=Machine.fineStage.arrAxis[FDY].sPID.Integral;
							comData.dFineData[2]=Machine.fineStage.arrAxis[FDY].sPID.Derivative;
						break;
						case PID_FOLLOW_TYPE:
							//GetPIDParam(&Machine.fineStage.arrAxis[FDY].sPID_Switch,FDY);
							comData.dFineData[0]=Machine.fineStage.arrAxis[FDY].sPID_Switch.Proportion;
							comData.dFineData[1]=Machine.fineStage.arrAxis[FDY].sPID_Switch.Integral;
							comData.dFineData[2]=Machine.fineStage.arrAxis[FDY].sPID_Switch.Derivative;
						
						break;
						case LLC_INDEPENDENT_TYPE:
							GetLLCParam(&Machine.fineStage.arrAxis[FDY].sLLC,FDY);
						break;
						case LLC_FOLLOW_TYPE:
							GetLLCParam(&Machine.fineStage.arrAxis[FDY].sLLC_Switch,FDY);
						break;
	
						case ACC_COEFFICIENT_INDEPENDENT_DXMOVE:
							GetDXFeedForward(&Machine.fineStage.arrAxis[FDY],FDY);
						break;
						case ACC_COEFFICIENT_FOLLOW_DXMOVE:
							GetDXFeedForward_Switch(&Machine.fineStage.arrAxis[FDY],FDY);
						break;
						case ACC_COEFFICIENT_INDEPENDENT_DYMOVE:
							GetDYFeedForward(&Machine.fineStage.arrAxis[FDY],FDY);
						break;
						case ACC_COEFFICIENT_FOLLOW_DYMOVE:
							GetDYFeedForward_Switch(&Machine.fineStage.arrAxis[FDY],FDY);
						break;
					}
				break;

				case 2://FDZ:
					switch(iParamType)
					{
						case PID_INDEPENDENT_TYPE:
							//GetPIDParam(&Machine.fineStage.arrAxis[FDY].sPID,FDY);
							comData.dFineData[0]=Machine.fineStage.arrAxis[FDZ].sPID.Proportion;
							comData.dFineData[1]=Machine.fineStage.arrAxis[FDZ].sPID.Integral;
							comData.dFineData[2]=Machine.fineStage.arrAxis[FDZ].sPID.Derivative;
						break;
						case PID_FOLLOW_TYPE:
							//GetPIDParam(&Machine.fineStage.arrAxis[FDY].sPID_Switch,FDY);
							comData.dFineData[0]=Machine.fineStage.arrAxis[FDZ].sPID_Switch.Proportion;
							comData.dFineData[1]=Machine.fineStage.arrAxis[FDZ].sPID_Switch.Integral;
							comData.dFineData[2]=Machine.fineStage.arrAxis[FDZ].sPID_Switch.Derivative;
						
						break;
						case LLC_INDEPENDENT_TYPE:
							GetLLCParam(&Machine.fineStage.arrAxis[FDZ].sLLC,FDZ);
						break;
						case LLC_FOLLOW_TYPE:
							GetLLCParam(&Machine.fineStage.arrAxis[FDZ].sLLC_Switch,FDZ);
						break;

						case ACC_COEFFICIENT_INDEPENDENT_DXMOVE:
							GetDXFeedForward(&Machine.fineStage.arrAxis[FDZ],FDZ);
						break;
						case ACC_COEFFICIENT_FOLLOW_DXMOVE:
							GetDXFeedForward_Switch(&Machine.fineStage.arrAxis[FDZ],FDZ);
						break;
						case ACC_COEFFICIENT_INDEPENDENT_DYMOVE:
							GetDYFeedForward(&Machine.fineStage.arrAxis[FDZ],FDZ);
						break;
						case ACC_COEFFICIENT_FOLLOW_DYMOVE:
							GetDYFeedForward_Switch(&Machine.fineStage.arrAxis[FDZ],FDZ);
						break;
					}
				break;
				case 3://FTX:
					switch(iParamType)
					{
						case PID_INDEPENDENT_TYPE:
							//GetPIDParam(&Machine.fineStage.arrAxis[FDY].sPID,FDY);
							comData.dFineData[0]=Machine.fineStage.arrAxis[FTX].sPID.Proportion;
							comData.dFineData[1]=Machine.fineStage.arrAxis[FTX].sPID.Integral;
							comData.dFineData[2]=Machine.fineStage.arrAxis[FTX].sPID.Derivative;
						break;
						case PID_FOLLOW_TYPE:
							//GetPIDParam(&Machine.fineStage.arrAxis[FDY].sPID_Switch,FDY);
							comData.dFineData[0]=Machine.fineStage.arrAxis[FTX].sPID_Switch.Proportion;
							comData.dFineData[1]=Machine.fineStage.arrAxis[FTX].sPID_Switch.Integral;
							comData.dFineData[2]=Machine.fineStage.arrAxis[FTX].sPID_Switch.Derivative;
						
						break;
						case LLC_INDEPENDENT_TYPE:
							GetLLCParam(&Machine.fineStage.arrAxis[FTX].sLLC,FTX);
						break;
						case LLC_FOLLOW_TYPE:
							GetLLCParam(&Machine.fineStage.arrAxis[FTX].sLLC_Switch,FTX);
						break;
			
						case ACC_COEFFICIENT_INDEPENDENT_DXMOVE:
							GetDXFeedForward(&Machine.fineStage.arrAxis[FTX],FTX);
						break;
						case ACC_COEFFICIENT_FOLLOW_DXMOVE:
							GetDXFeedForward_Switch(&Machine.fineStage.arrAxis[FTX],FTX);
						break;
						case ACC_COEFFICIENT_INDEPENDENT_DYMOVE:
							GetDYFeedForward(&Machine.fineStage.arrAxis[FTX],FTX);
						break;
						case ACC_COEFFICIENT_FOLLOW_DYMOVE:
							GetDYFeedForward_Switch(&Machine.fineStage.arrAxis[FTX],FTX);
						break;
					}
				break;
				case 4://FTY:
					switch(iParamType)

					{
						case PID_INDEPENDENT_TYPE:
							//GetPIDParam(&Machine.fineStage.arrAxis[FDY].sPID,FDY);
							comData.dFineData[0]=Machine.fineStage.arrAxis[FTY].sPID.Proportion;
							comData.dFineData[1]=Machine.fineStage.arrAxis[FTY].sPID.Integral;
							comData.dFineData[2]=Machine.fineStage.arrAxis[FTY].sPID.Derivative;
						break;
						case PID_FOLLOW_TYPE:
							//GetPIDParam(&Machine.fineStage.arrAxis[FDY].sPID_Switch,FDY);
							comData.dFineData[0]=Machine.fineStage.arrAxis[FTY].sPID_Switch.Proportion;
							comData.dFineData[1]=Machine.fineStage.arrAxis[FTY].sPID_Switch.Integral;
							comData.dFineData[2]=Machine.fineStage.arrAxis[FTY].sPID_Switch.Derivative;
						
						break;
						case LLC_INDEPENDENT_TYPE:
							GetLLCParam(&Machine.fineStage.arrAxis[FTY].sLLC,FTY);
						break;
						case LLC_FOLLOW_TYPE:
							GetLLCParam(&Machine.fineStage.arrAxis[FTY].sLLC_Switch,FTY);
						break;
	
						case ACC_COEFFICIENT_INDEPENDENT_DXMOVE:
							GetDXFeedForward(&Machine.fineStage.arrAxis[FTY],FTY);
						break;
						case ACC_COEFFICIENT_FOLLOW_DXMOVE:
							GetDXFeedForward_Switch(&Machine.fineStage.arrAxis[FTY],FTY);
						break;
						case ACC_COEFFICIENT_INDEPENDENT_DYMOVE:
							GetDYFeedForward(&Machine.fineStage.arrAxis[FTY],FTY);
						break;
						case ACC_COEFFICIENT_FOLLOW_DYMOVE:
							GetDYFeedForward_Switch(&Machine.fineStage.arrAxis[FTY],FTY);
						break;
					}
				break;
				case 5://FTZ:
					switch(iParamType)
					{
						case PID_INDEPENDENT_TYPE:
							//GetPIDParam(&Machine.fineStage.arrAxis[FTZ].sPID,FTZ);
							comData.dFineData[0]=Machine.fineStage.arrAxis[FTZ].sPID.Proportion;
							comData.dFineData[1]=Machine.fineStage.arrAxis[FTZ].sPID.Integral;
							comData.dFineData[2]=Machine.fineStage.arrAxis[FTZ].sPID.Derivative;
						break;
						case PID_FOLLOW_TYPE:
							//GetPIDParam(&Machine.fineStage.arrAxis[FTZ].sPID_Switch,FTZ);
							comData.dFineData[0]=Machine.fineStage.arrAxis[FTZ].sPID_Switch.Proportion;
							comData.dFineData[1]=Machine.fineStage.arrAxis[FTZ].sPID_Switch.Integral;
							comData.dFineData[2]=Machine.fineStage.arrAxis[FTZ].sPID_Switch.Derivative;
						break;
						case LLC_INDEPENDENT_TYPE:
							GetLLCParam(&Machine.fineStage.arrAxis[FTZ].sLLC,FTZ);
						break;
						case LLC_FOLLOW_TYPE:
							GetLLCParam(&Machine.fineStage.arrAxis[FTZ].sLLC_Switch,FTZ);
						break;

						case ACC_COEFFICIENT_INDEPENDENT_DXMOVE:
							GetDXFeedForward(&Machine.fineStage.arrAxis[FTZ],FTZ);
						break;
						case ACC_COEFFICIENT_FOLLOW_DXMOVE:
							GetDXFeedForward_Switch(&Machine.fineStage.arrAxis[FTZ],FTZ);
						break;
						case ACC_COEFFICIENT_INDEPENDENT_DYMOVE:
							GetDYFeedForward(&Machine.fineStage.arrAxis[FTZ],FTZ);
						break;
						case ACC_COEFFICIENT_FOLLOW_DYMOVE:
							GetDYFeedForward_Switch(&Machine.fineStage.arrAxis[FTZ],FTZ);
						break;
					}
				break;

				case ALLREFRESH_FINE:
					switch(iParamType)
					{
						case PID_INDEPENDENT_TYPE:
							//0
							comData.dFineData[0]=Machine.fineStage.arrAxis[FDX].sPID.Proportion;
							comData.dFineData[1]=Machine.fineStage.arrAxis[FDX].sPID.Integral;
							comData.dFineData[2]=Machine.fineStage.arrAxis[FDX].sPID.Derivative;
							// 1
							comData.dFineData[3]=Machine.fineStage.arrAxis[FDY].sPID.Proportion;
							comData.dFineData[4]=Machine.fineStage.arrAxis[FDY].sPID.Integral;
							comData.dFineData[5]=Machine.fineStage.arrAxis[FDY].sPID.Derivative;
							// 2 
							comData.dFineData[6]=Machine.fineStage.arrAxis[3].sPID.Proportion;
							comData.dFineData[7]=Machine.fineStage.arrAxis[3].sPID.Integral;
							comData.dFineData[8]=Machine.fineStage.arrAxis[3].sPID.Derivative;
							// 3
							comData.dFineData[9]=Machine.fineStage.arrAxis[4].sPID.Proportion;
							comData.dFineData[10]=Machine.fineStage.arrAxis[4].sPID.Integral;
							comData.dFineData[11]=Machine.fineStage.arrAxis[4].sPID.Derivative;
							 // 4
							comData.dFineData[12]=Machine.fineStage.arrAxis[5].sPID.Proportion;
							comData.dFineData[13]=Machine.fineStage.arrAxis[5].sPID.Integral;
							comData.dFineData[14]=Machine.fineStage.arrAxis[5].sPID.Derivative;
							// 5
							comData.dFineData[15]=Machine.fineStage.arrAxis[2].sPID.Proportion;
							comData.dFineData[16]=Machine.fineStage.arrAxis[2].sPID.Integral;
							comData.dFineData[17]=Machine.fineStage.arrAxis[2].sPID.Derivative;

						break;
						case PID_FOLLOW_TYPE:
							comData.dFineData[0]=Machine.fineStage.arrAxis[FDX].sPID_Switch.Proportion;
							comData.dFineData[1]=Machine.fineStage.arrAxis[FDX].sPID_Switch.Integral;
							comData.dFineData[2]=Machine.fineStage.arrAxis[FDX].sPID_Switch.Derivative;
							
							comData.dFineData[3]=Machine.fineStage.arrAxis[FDY].sPID_Switch.Proportion;
							comData.dFineData[4]=Machine.fineStage.arrAxis[FDY].sPID_Switch.Integral;
							comData.dFineData[5]=Machine.fineStage.arrAxis[FDY].sPID_Switch.Derivative;
							
							comData.dFineData[6]=Machine.fineStage.arrAxis[FTZ].sPID_Switch.Proportion;
							comData.dFineData[7]=Machine.fineStage.arrAxis[FTZ].sPID_Switch.Integral;
							comData.dFineData[8]=Machine.fineStage.arrAxis[FTZ].sPID_Switch.Derivative;
						break;
						case LLC_INDEPENDENT_TYPE:
							for(i=0;i<3;i++)
							{
								GetLLCParam(&Machine.fineStage.arrAxis[i].sLLC,i);
							}
						break;
						case LLC_FOLLOW_TYPE:
							for(i=0;i<3;i++)
							{
								GetLLCParam(&Machine.fineStage.arrAxis[i].sLLC_Switch,i);
							}
						break;
						case ACC_COEFFICIENT_INDEPENDENT_DXMOVE:
							for(i=0;i<3;i++)
							{
								GetDXFeedForward(&Machine.fineStage.arrAxis[i],i);
							}
						break;
						case ACC_COEFFICIENT_FOLLOW_DXMOVE:
							for(i=0;i<3;i++)
							{
								GetDXFeedForward_Switch(&Machine.fineStage.arrAxis[i],i);
							}
						break;
						case ACC_COEFFICIENT_INDEPENDENT_DYMOVE:
							for(i=0;i<3;i++)
							{
								GetDYFeedForward(&Machine.fineStage.arrAxis[i],i);
							}
						break;
						case ACC_COEFFICIENT_FOLLOW_DYMOVE:
							for(i=0;i<3;i++)
							{
								GetDYFeedForward_Switch(&Machine.fineStage.arrAxis[i],i);
							}
						break;
						case LIMIT_TYPE:
							GetLimit(Machine.fineStage.DA_Struct);
						break;
					}
				break;
			}
		break;
	}
	comData.iFineData[0]= iStageType;	
	comData.iFineData[1]= iDof+1;	
	comData.iFineData[2]= iParamType; 	
	comData.iFineData[3]= 1; 
		
	comData.iCMD= 5001;
}

*/