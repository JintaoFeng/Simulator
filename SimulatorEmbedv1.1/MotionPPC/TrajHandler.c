#include "TrajHandler.h"
#include "rtcontrol.h"
#include "GetData.h"
#include "CMD.h"
#include "TrajGen_4OrderPoly.h"
#include "StateMachine.h"
#include "math.h"
#include "ast_M58.h"
#include "SingleTPG.h"
#include <stdio.h>



//char T_Coarse[19] = {'Y','+','X','+','Y','+','X','-','Y','+','X','+','Y','+','X','-','Y','+','F'};
//char T_Fine[19] = 	{'Y','+','X','+','Y','+','X','-','Y','+','X','+','Y','+','X','-','Y','+','F'};
char TT[50] = 		{'Y','+','Y','+','Y','+','X','+','Y','-','Y','-','Y','-','F'};

/*char T_Coarse[50] = {'Y','+','X','+','Y','-','X','+','Y','+','X','+','Y','-','X','+','Y','+',
					 'Y','-','X','-','Y','+','X','-','Y','-','X','-','Y','+','X','-','Y','-',


					'F'};*/

char T_Coarse[1000] = {'Y','+','X','+','Y','-','X','-','Y','+','X','+','Y','-','X','-',
                     'Y','+','X','+','Y','-','X','-','Y','+','X','+','Y','-','X','-',
	                 'Y','+','X','+','Y','-','X','-','Y','+','X','+','Y','-','X','-',
					 'Y','+','X','+','Y','-','X','-','Y','+','X','+','Y','-','X','-',


					'F'};



char T_Fine[1000] = 	{'Y','+','X','+','Y','-','X','+','Y','+','X','+','Y','-','X','+','Y','+',
					 'Y','-','X','-','Y','+','X','-','Y','-','X','-','Y','+','X','-','Y','-',

					'F'};

char ExpoPlanChar[1000]={'X','+','Y','+','X','-','Y','+','X','+','Y','+','X','-','Y','+','X','+',
					 'X','-','Y','-','X','+','Y','-','X','-','Y','-','X','+','Y','-','X','-',

					'F'};


int g_iField=0;



//char* T_Coarse;
//char* T_Fine;

int Coarse_Count = 0;
TrajLast DXPosLast_Fine = FirstIn;
TrajLast DYPosLast_Fine = FirstIn;
TrajLast DXNegLast_Fine = FirstIn;
TrajLast DYNegLast_Fine = FirstIn;

int Fine_Count = 0;
TrajLast DXPosLast_Coarse = FirstIn;
TrajLast DYPosLast_Coarse = FirstIn;
TrajLast DXNegLast_Coarse = FirstIn;
TrajLast DYNegLast_Coarse = FirstIn;


/*****************************************
函数名：CharResoluted_Mode1
返回值：无
参数表：无
功能说明：获取点对点轨迹参数函数
******************************************/
char* CharResoluted_Mode1(int field)
{
	char* string = '\0';
	int i = 0;
	for(i=0;i<field;i++)
	{
		if(i < field-1)
		{
			*(string+i*4) = 'Y';
			if(i%2 == 0)
			{
				*(string+1+i*4) = '+';
			}
			else if(i%2 == 1)
			{
				*(string+1+i*4) = '-';
			}
			*(string+2+i*4) = 'X';
			*(string+3+i*4) = '+';
		}
		else
		{
			*(string+i*4) = 'Y';
			if(i%2 == 0)
			{
				*(string+1+i*4) = '+';
			}
			else if(i%2 == 1)
			{
				*(string+1+i*4) = '-';
			}
			*(string+2+i*4) = 'F';
		}
	}
	return(string);
}
/*****************************************
函数名：CharResoluted_Mode2
返回值：无
参数表：无
功能说明：获取点对点轨迹参数函数
******************************************/
char* CharResoluted_Mode2(int field)
{
	char* string = '\0';
	int i = 0;
	for(i=0;i<field;i++)
	{
		*(string+i*2) = 'Y';
		*(string+1+i*2) = '+';
	}
	*(string+2*field) = 'X';
	*(string+2*field+1) = '+';
	for(i=0;i<field;i++)
	{
		*(string+2*(field+1)+i*2) = 'Y';
		*(string+2*(field+1)+i*2+1) = '-';
	}
	*(string+2*(2*field+1)) = 'F';
	return(string);
}
void ExpoPlanGen(int field)
{
	int i = 0;

	char* string = T_Coarse;
	for(i=0;i<field;i++)
	{
		if(i < field-1)
		{
			*(string+i*4) = 'X';
			if(i%2 == 0)
			{
				*(string+1+i*4) = '+';
			}
			else if(i%2 == 1)
			{
				*(string+1+i*4) = '-';
			}
			*(string+2+i*4) = 'Y';
			*(string+3+i*4) = '+';
		}
		else
		{
			*(string+i*4) = 'X';
			if(i%2 == 0)
			{
				*(string+1+i*4) = '+';
			}
			else if(i%2 == 1)
			{
				*(string+1+i*4) = '-';
			}
			*(string+2+i*4) = 'F';
		}
	}
	//return(string);

}

void Close(Axis *pAxis,int iAxis)
{
	SwitchStateMachine(pAxis,iAxis);
}

//bSwitch在开环和初始化清零,保证开环后先用Counter闭环
void CounterClose(Axis *pAxis,int iAxis)
{
	if(pAxis->bSwitch==0)
	{
		pAxis->dSetPoint = pAxis->dActPos;
		//防止来回切换时未清零
		PIDClear(&(pAxis->sPID));
		LLCClear(&(pAxis->sLLC));
		LLCClear(&(pAxis->sFilter));
		pAxis->bSwitch = 1;	
	}
}

void LaserSwitch(Axis *pAxis, int iAxis)
{
	if(pAxis->bSwitch==1)
	{
		pAxis->dSetPoint = pAxis->dActPos;

		//防止来回切换时未清零
		PIDClear(&(pAxis->sPID_Switch));
		LLCClear(&(pAxis->sLLC_Switch));
		LLCClear(&(pAxis->sFilter));
		pAxis->bSwitch = 0;
	}
}

void Open(Axis *pAxis,int iAxis)
{

	//开环后切换为轴初始化状态，注意要积分清零，并且可以把不确定的东西都赋值
	
	pAxis->dSetPoint = pAxis->dActPos;//这个只能保证PD控制器输出为零，所以不保险

	LLCClear(&(pAxis->sLLC));
	LLCClear(&(pAxis->sLLC_Switch));
	
	PIDClear(&(pAxis->sPID));
	PIDClear(&(pAxis->sPID_Switch));
	
	pAxis->dError = 0.0;
	pAxis->dAffData = 0.0;
	pAxis->iTrajRealTimeFlag = 0;		
	pAxis->bSwitch = 0;
	pAxis->dConCalc=0.0;
	pAxis->closeCMD = C_Close_Nothing;
	pAxis->closeStatus = S_Counter_Close;
}

void StepFor(Axis *pAxis,int iAxis)
{	
	if(pAxis->closeStatus == S_Laser_Close)
	{
		pAxis->dSetPoint = pAxis->dSetPoint+pAxis->dStepDis;
	}
	else
	{
		pAxis->dSetPoint = pAxis->dSetPoint+pAxis->dStepDis;		
	}
	pAxis->axisCMD = C_Nothing;	
}

void StepRev(Axis *pAxis,int iAxis)
{
	if(pAxis->closeStatus == S_Laser_Close)
	{
		pAxis->dSetPoint = pAxis->dSetPoint-pAxis->dStepDis;	
	}
	else
	{
		pAxis->dSetPoint = pAxis->dSetPoint-pAxis->dStepDis;
	}	
	pAxis->axisCMD = C_Nothing;
}

void JogFor(Axis *pAxis,int iAxis)
{
	if(pAxis->closeStatus == S_Laser_Close)
	{		
		pAxis->dSetPoint = pAxis->dSetPoint+pAxis->dJogSpd;
	}
	else
	{
		
		pAxis->dSetPoint = pAxis->dSetPoint+pAxis->dJogSpd;
	}
}

void JogRev(Axis *pAxis,int iAxis)
{
	if(pAxis->closeStatus == S_Laser_Close)
	{		
		pAxis->dSetPoint = pAxis->dSetPoint-pAxis->dJogSpd;
	}
	else
	{		
		pAxis->dSetPoint = pAxis->dSetPoint-pAxis->dJogSpd;
	}
}

void Ident(Axis *pAxis,int iAxis)
{
	
}

void Home(Axis *pAxis,int iAxis)
{
//Coarse_home

  if(pAxis->HomeFlagCorse == 1.0)
  {
	  //Y轴
    if( Machine.stepFlagYQ == 1)
    {
		 EnableIntr3();
		Machine.coarseStage.arrAxis[2].bIntrEnable=1;
	} 
	  
     //X轴
	if( Machine.stepFlagXQ == 1)
	{
		EnableIntr1();
		Machine.coarseStage.arrAxis[0].bIntrEnable=1;
	}

	if(pAxis->stepHome == 1)
	{
	 	pAxis->dSetPoint = pAxis->dSetPoint-1000;
	}
	else if(pAxis->stepHome == 3)
    {
    	pAxis->dSetPoint = pAxis->dSetPoint+1000;
	}
//	else if(pAxis->stepHome==2)
//	{
//		pAxis->dSetPoint=pAxis->dActPos;
//	}
/*
	if(1==pAxis->bHome)
	{
		pAxis->axisStatus = S_Aixs_Close;	
		pAxis->dSetPoint=0.0;
		pAxis->bIntr=0;
		pAxis->bHome = 0;
		pAxis->stepHome = 0;
		/*Machine.stepHome = 0;
		Machine.coarseStage.arrAxis[2].bIntrEnable=0;
		Machine.coarseStage.arrAxis[0].bIntrEnable=0;
		Machine.HomeFlagCorse = 0;*

		pAxis->HomeFlagCorse = 0;
		pAxis->bHomeDone = 2;
		pAxis->bIntrEnable=0;=*/

		 //logMsg("value1=%x,value2=%x\n",9,9,0,0,0,0);
//

   }

	/*if(pAxis->closeStatus == S_Laser_Close)
	{
		//pAxis->LaserPos.dSetPoint = pAxis->LaserPos.dSetPoint-1;
		//pAxis->LaserPos.dError = pAxis->LaserPos.dSetPoint - pAxis->LaserPos.dActPos;
	}
	else
	{
		pAxis->dSetPoint = pAxis->dSetPoint+500;
	}
	if(1==pAxis->bHome)
	{
		pAxis->axisStatus = S_Aixs_Close;	
		pAxis->dSetPoint=0.0;
		pAxis->bIntr=0;
		pAxis->bHome = 0;
	}*/

//fine_home
else
 {
	if(pAxis->dSetPoint >0) 
	{ 
	  pAxis->dSetPoint=pAxis->dSetPoint-50;// fine_home step=100		  
	}
	if(pAxis->dSetPoint <0)
	{
	   pAxis->dSetPoint=pAxis->dSetPoint+50;// fine_home step=100			  
	}


	if(fabs(pAxis->dSetPoint) < 50)
	{
		pAxis->dSetPoint=0;
		pAxis->axisStatus = S_Aixs_Close;	
		pAxis->bHome = 0;	  
	}
  }
	
}

/*void Home_Coarse(Axis *pAxis,int iAxis)
{
	if(Port_B_A == 0x01)
	{
     pAxis->dSetPoint =  pAxis->dActPos;
	 	EnableIntr3();
		Machine.coarseStage.arrAxis[2].bIntrEnable=1;
        Machine.step =  1;
	}


	 if(Machine.step != 1)
		 {
	   
		 pAxis->dSetPoint = pAxis->dSetPoint-pAxis->dJogSpd;
	 
		 }
     if(Machine.step == 1)
     {
      pAxis->dSetPoint = pAxis->dSetPoint+500;
	 }

	//Coarse_home
	
	if(1==pAxis->bHome)
	{
		pAxis->axisStatus = S_Aixs_Close;	
		pAxis->dSetPoint=0.0;
		pAxis->bIntr=0;
		pAxis->bHome = 0;
		Machine.step = 0;
	}


}
	
*/

void XYRun_Forward(Axis *pAxis,int iAxis)
{
	if(pAxis->trajType == isTraj4OrderPoly)
	{	
 		pAxis->trajStatus = (TrajStatus)pAxis->TrajGen4Order(&pAxis->sTP_Run,&pAxis->snap,&pAxis->jerk,&pAxis->acc,&pAxis->vel,&pAxis->shift,&pAxis->cTrig);
	}
	else if(pAxis->trajType == isTrajSineAccT)
	{
		pAxis->trajStatus = (TrajStatus)pAxis->TrajSine_Poly(&pAxis->sine_Run,&pAxis->acc,&pAxis->vel,&pAxis->shift);
	}
	
	if(pAxis->trajStatus != Stop)
	{
		if(0 == pAxis->iTrajRealTimeFlag)
		{
			pAxis->iTrajStartPoint = pAxis->dSetPoint;
			pAxis->iTrajRealTimeFlag = 1;
		}
		pAxis->dAffData = 1*pAxis->acc;
		if(pAxis->trajMode == IsTrajIndependent)
		{
			pAxis->dSetPoint = pAxis->iTrajStartPoint + pAxis->shift;
			CLEAR_MOTOR_INPOS_FLAG(pAxis->flag);
			SET_MOTOR_MOVING_FLAG(pAxis->flag);
		}
	}
	else
	{
		pAxis->iTrajRealTimeFlag = 0;
		pAxis->axisCMD = C_Nothing;
		pAxis->dAffData = 0.0;

		pAxis->iTrajStartPoint=0;
		if(pAxis->axisStatus == S_Aixs_RunFor)
		{
			pAxis->axisStatus = S_Aixs_Close;
			CLEAR_MOTOR_MOVING_FLAG(pAxis->flag);
			SET_MOTOR_INPOS_FLAG(pAxis->flag);
		}
	}
}

void XYRun_Reverse(Axis *pAxis,int iAxis)
{
	
	if(pAxis->trajType == isTraj4OrderPoly)
	{	
 		pAxis->trajStatus = (TrajStatus)pAxis->TrajGen4Order(&pAxis->sTP_Run,&pAxis->snap,&pAxis->jerk,&pAxis->acc,&pAxis->vel,&pAxis->shift,&pAxis->cTrig);
	}
	else if(pAxis->trajType == isTrajSineAccT)
	{
		pAxis->trajStatus = (TrajStatus)pAxis->TrajSine_Poly(&pAxis->sine_Run,&pAxis->acc,&pAxis->vel,&pAxis->shift);
	}


	
	if(pAxis->trajStatus != Stop)
	{
		if(0 == pAxis->iTrajRealTimeFlag)
		{
			pAxis->iTrajStartPoint = pAxis->dSetPoint;
			pAxis->iTrajRealTimeFlag = 1;
		}
		pAxis->dAffData = -1*pAxis->acc;
		if(pAxis->trajMode == IsTrajIndependent)
		{
			pAxis->dSetPoint = pAxis->iTrajStartPoint - pAxis->shift;
			CLEAR_MOTOR_INPOS_FLAG(pAxis->flag);
			SET_MOTOR_MOVING_FLAG(pAxis->flag);
		}
	}
	else
	{
		pAxis->axisCMD = C_Nothing;
		pAxis->iTrajRealTimeFlag = 0;
		pAxis->dAffData = 0.0;

		pAxis->iTrajStartPoint=0;
		if(pAxis->axisStatus == S_Aixs_RunRev)
		{
			pAxis->axisStatus = S_Aixs_Close;
			CLEAR_MOTOR_MOVING_FLAG(pAxis->flag);
			SET_MOTOR_INPOS_FLAG(pAxis->flag);
		}
	}
}


/*****************************************
函数名：Traj_Coarse
返回值：无
参数表：无
功能说明：粗动台曝光轨迹函数
******************************************/
void Traj_Coarse()
{
	Stage* EXPO_Stage = &Machine.coarseStage;	
	static char num = 0;

	
	switch(*(T_Coarse+Coarse_Count))
	{

		case 'X':

			if(*(T_Coarse+Coarse_Count+1) == '+')
			{
				if(DXPosLast_Coarse == FirstIn)
				{ 
					Machine.coarseStage.arrAxis[0].cTrig = 1;
					Machine.coarseStage.arrAxis[1].cTrig = 1;
					EXPO_Stage->arrAxis[0].axisStatus = S_Aixs_RunFor;
//					EXPO_Stage->arrAxis[1].axisStatus = S_Aixs_RunFor;
					DXPosLast_Coarse = RunLast;
				}
				if(EXPO_Stage->arrAxis[0].axisStatus == S_Aixs_Close)
				{
					DXPosLast_Coarse = FirstIn;
					Coarse_Count+=2;
				}
			}
			else if(*(T_Coarse+Coarse_Count+1) == '-')
			{
				if(DXNegLast_Coarse == FirstIn)
				{
					Machine.coarseStage.arrAxis[0].cTrig = 1;
					Machine.coarseStage.arrAxis[1].cTrig = 1;
					EXPO_Stage->arrAxis[0].axisStatus = S_Aixs_RunRev;
//					EXPO_Stage->arrAxis[1].axisStatus = S_Aixs_RunRev;
					DXNegLast_Coarse = RunLast;
				}
				if(EXPO_Stage->arrAxis[0].axisStatus == S_Aixs_Close)
				{
					DXNegLast_Coarse = FirstIn;
					Coarse_Count+=2;
				}
			}
			else
			{
				Coarse_Count = 0;
				EXPO_Stage->stageStatus = S_Stage_ALL_Close;
				EXPO_Stage->arrAxis[0].axisStatus = S_Aixs_Close;
//				EXPO_Stage->arrAxis[1].axisStatus = S_Aixs_Close;
				DXPosLast_Coarse = FirstIn;
				DXNegLast_Coarse = FirstIn;
			}
		break;
		case 'Y':			 
			if(*(T_Coarse+Coarse_Count+1) == '+')
			{
				if(DYPosLast_Coarse == FirstIn)
				{
					Machine.coarseStage.arrAxis[2].cTrig = 1;	
					Machine.coarseStage.arrAxis[3].cTrig = 1;
					EXPO_Stage->arrAxis[2].axisStatus = S_Aixs_RunFor;
//					EXPO_Stage->arrAxis[3].axisStatus = S_Aixs_RunFor;
					DYPosLast_Coarse = RunLast;
				}
				if(EXPO_Stage->arrAxis[2].axisStatus == S_Aixs_Close)
				{
					DYPosLast_Coarse = FirstIn;
					Coarse_Count+=2;
				}
			}
			else if(*(T_Coarse+Coarse_Count+1) == '-')
			{
				if(DYNegLast_Coarse == FirstIn)
				{
					Machine.coarseStage.arrAxis[2].cTrig = 1;	
					Machine.coarseStage.arrAxis[3].cTrig = 1;	
					EXPO_Stage->arrAxis[2].axisStatus = S_Aixs_RunRev;
//					EXPO_Stage->arrAxis[3].axisStatus = S_Aixs_RunRev;
					DYNegLast_Coarse = RunLast;
				}
				if(EXPO_Stage->arrAxis[2].axisStatus == S_Aixs_Close)
				{
					DYNegLast_Coarse = FirstIn;
					Coarse_Count+=2;
				}
			}
			else
			{
				logMsg("Error!!!!\n",0,0,0,0,0,0);
				Coarse_Count = 0;
				EXPO_Stage->stageStatus = S_Stage_ALL_Close;
				EXPO_Stage->arrAxis[2].axisStatus = S_Aixs_Close;
//				EXPO_Stage->arrAxis[3].axisStatus = S_Aixs_Close;
				DYPosLast_Coarse = FirstIn;
				DYNegLast_Coarse = FirstIn;
			}
		break;
		case 'F':			
//			Coarse_Count = 0;
//			EXPO_Stage->stageStatus = S_Stage_ALL_Close;
//			EXPO_Stage->stageCMD = C_All_Nothing;
//			EXPO_Stage->arrAxis[0].axisStatus = S_Aixs_Close;
//			EXPO_Stage->arrAxis[1].axisStatus = S_Aixs_Close;
//			EXPO_Stage->arrAxis[2].axisStatus = S_Aixs_Close;
//			EXPO_Stage->arrAxis[3].axisStatus = S_Aixs_Close;
			if(DYNegLast_Coarse == FirstIn)
			{
				GetRunData(&Machine.coarseStage.arrAxis[2].sTP_Run,YExpoRevParam);
				EXPO_Stage->arrAxis[2].axisStatus = S_Aixs_RunRev;
				EXPO_Stage->arrAxis[2].cTrig=1;
				DYNegLast_Coarse = RunLast;
			}
			if(EXPO_Stage->arrAxis[2].axisStatus == S_Aixs_Close)
			{
				DYNegLast_Coarse = FirstIn;
				GetRunData(&Machine.coarseStage.arrAxis[2].sTP_Run,YExpoParam);
				Coarse_Count = 0;
			}
			
			
//			DXPosLast_Coarse = FirstIn;
//			DXNegLast_Coarse = FirstIn;
//			DYPosLast_Coarse = FirstIn;
//			DYNegLast_Coarse = FirstIn;
			
		break;
	}
}
/*****************************************
函数名：Traj_Fine
返回值：无
参数表：无
功能说明：粗动台曝光轨迹函数
******************************************/
void Traj_Fine()
{
	Stage* EXPO_Stage = &Machine.fineStage;

	
	switch(*(T_Fine+Fine_Count))
	{
		case 'X':
			if(*(T_Fine+Fine_Count+1) == '+')
			{
				if(DXPosLast_Fine == FirstIn)
				{
					//设置轨迹参数
					Machine.fineStage.arrAxis[FDX].cTrig = 1;			
					EXPO_Stage->arrAxis[FDX].axisStatus = S_Aixs_RunFor;					
					DXPosLast_Fine = RunLast;
				}
				if(EXPO_Stage->arrAxis[DX].axisStatus == S_Aixs_Close)
				{
					DXPosLast_Fine = FirstIn;
					Fine_Count+=2;
				}
			}
			else if(*(T_Fine+Fine_Count+1) == '-')
			{
				if(DXNegLast_Fine == FirstIn)
				{
					//设置轨迹参数
					Machine.fineStage.arrAxis[FDX].cTrig = 1;
					EXPO_Stage->arrAxis[FDX].axisStatus = S_Aixs_RunRev;
					DXNegLast_Fine = RunLast;
				}
				if(EXPO_Stage->arrAxis[FDX].axisStatus == S_Aixs_Close)
				{
					DXNegLast_Fine = FirstIn;
					Fine_Count+=2;
				}
			}
			else
			{
				Fine_Count = 0;
				EXPO_Stage->stageStatus = S_Stage_ALL_Close;
				EXPO_Stage->arrAxis[FDX].axisStatus = S_Aixs_Close;
				DXPosLast_Fine = FirstIn;
				DXNegLast_Fine = FirstIn;
			}
		break;
		case 'Y':
			if(*(T_Fine+Fine_Count+1) == '+')
			{
				if(DYPosLast_Fine == FirstIn)
				{

					//设置轨迹参数
					Machine.fineStage.arrAxis[FDY].cTrig = 1;
					EXPO_Stage->arrAxis[FDY].axisStatus = S_Aixs_RunFor;
					DYPosLast_Fine = RunLast;
				}
				if(EXPO_Stage->arrAxis[FDY].axisStatus == S_Aixs_Close)
				{
					DYPosLast_Fine = FirstIn;
					Fine_Count+=2;
				}
			}
			else if(*(T_Fine+Fine_Count+1) == '-')
			{
				if(DYNegLast_Fine == FirstIn)
				{
					//设置轨迹参数
					Machine.fineStage.arrAxis[FDY].cTrig = 1;
					EXPO_Stage->arrAxis[FDY].axisStatus = S_Aixs_RunRev;
					DYNegLast_Fine = RunLast;
				}
				if(EXPO_Stage->arrAxis[FDY].axisStatus == S_Aixs_Close)
				{
					DYNegLast_Fine = FirstIn;
					Fine_Count+=2;
				}
			}
			else
			{
				logMsg("Error!!!\n",0,0,0,0,0,0);
				Fine_Count = 0;
				EXPO_Stage->stageStatus = S_Stage_ALL_Close;
				EXPO_Stage->arrAxis[FDY].axisStatus = S_Aixs_Close;
				DYPosLast_Fine = FirstIn;
				DYNegLast_Fine = FirstIn;
			}
		break;
		case 'F':
//			Fine_Count = 0;
//			EXPO_Stage->stageStatus = S_Stage_ALL_Close;
//			EXPO_Stage->stageCMD = C_All_Nothing;
//			EXPO_Stage->arrAxis[DX].axisStatus = S_Aixs_Close;
//			EXPO_Stage->arrAxis[DY].axisStatus = S_Aixs_Close;
			if(DYNegLast_Fine == FirstIn)
			{
				GetRunData(&Machine.fineStage.arrAxis[FDY].sTP_Run,YExpoRevParam);
				EXPO_Stage->arrAxis[FDY].axisStatus = S_Aixs_RunRev;
				EXPO_Stage->arrAxis[FDY].cTrig=1;
				DYNegLast_Fine = RunLast;
			}
			if(EXPO_Stage->arrAxis[FDY].axisStatus == S_Aixs_Close)
			{
				DYNegLast_Fine = FirstIn;
				GetRunData(&Machine.fineStage.arrAxis[FDY].sTP_Run,YExpoParam);
				Fine_Count = 0;
			}
//			DXPosLast_Fine = FirstIn;
//			DXNegLast_Fine = FirstIn;
//			DYPosLast_Fine = FirstIn;
//			DYNegLast_Fine = FirstIn;
		break;
	}
}

//CIRCLE circleParam;

double X = 0,Y = 0,XLast = 0,YLast = 0;
CIRCLE circleParam={0,0,0,0.01};

//circleParam.circleCount = 0;
//circleParam.circleStep = 0.01

void Circle(Stage  *pStage)
{
	static int i=0;
	if(i < circleParam.cirNum)
	{
		if(circleParam.angle == 0)
		{
			circleParam.angle += circleParam.circleStep;
			XLast = X;
			YLast = circleParam.R;
			printf("%f\n%d\n%d\n",circleParam.circleStep,circleParam.R,circleParam.cirNum);
		}
		else if(circleParam.angle > 0 && circleParam.angle <= 360)
		{
			X = sin(pi/180*circleParam.angle) * circleParam.R;
			Y = cos(pi/180*circleParam.angle) * circleParam.R;
			pStage->arrAxis[0].dSetPoint += X - XLast;
			pStage->arrAxis[2].dSetPoint += Y - YLast;
			circleParam.angle += circleParam.circleStep;
			XLast = X;
			YLast = Y;
		}
		else if(circleParam.angle > 360)
		{
			circleParam.angle = 0;
			i++;
			X = 0,Y = 0,XLast = 0,YLast = 0;
		}
	}
	else if(i == circleParam.cirNum)
	{
	i=0;
		X = 0,Y = 0,XLast = 0,YLast = 0;
		pStage->stageStatus = S_Stage_ALL_Close;	
	}
	if(pStage->arrAxis[CDX].axisStatus == S_Aixs_Open || pStage->arrAxis[CDY].axisStatus == S_Aixs_Open || pStage->arrAxis[XTZ].axisStatus == S_Aixs_Open )
	{	 			
		pStage->stageStatus = S_Stage_ALL_Open;
		circleParam.angle = 0;
	}
}

//void AbsolutMove(s_TParam* pTP,Axis* pAxis)
void AbsolutMove(double pos,Axis* pAxis)
{
	double posDistance;
//	double ExpoParam[10];
//	printf("%lf",pTP->p);
	posDistance = pos * 1000000 - pAxis->dSetPoint;
//	printf("%lf\n",posDistance);
//	absolute(&posDistance);
	pos = fabs(posDistance)/1000000;
	pAxis->targetPos=pos;
	pAxis->trajUpdate=1;
//	printf("%lf\n",pTP->p);
//	printf("%lf\n%lf\n%lf\n%lf\n",pTP->v,pTP->a,pTP->j,pTP->d);
//	TPGen_4Order_P2P(pTP, 1,pAxis->sTP_Run);
//	pTP->p = (posDistance + pAxis->dActPos)/1000000;

/*	ExpoParam[0]= pTP->Ts;
	ExpoParam[1]= p_dd;
	ExpoParam[2]= Td;
	ExpoParam[3]= Tj;
	ExpoParam[4]= Ta;
	ExpoParam[5]= Tv;
	ExpoParam[6]= 0;
	ExpoParam[7]= 0;
	ExpoParam[8]= 0;
	ExpoParam[9]= 0;
	GetRunData(&pAxis->sTP_Run,ExpoParam);*/
//	printf("%lf\n",posDistance);
	if(posDistance > 0)
	{
		pAxis->cTrig = 1;
		pAxis->axisCMD = C_Aixs_RunFor;
	}
	else if(posDistance < 0)
	{
		pAxis ->cTrig = 1;
		pAxis->axisCMD = C_Aixs_RunRev;
	}
}

void absolute(double* temp)
{
	if(*temp  > 0 );
	else if(*temp < 0)
		*temp*=(-1);
}
