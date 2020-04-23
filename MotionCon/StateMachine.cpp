#include "StateMachine.h"
#include "rtcontrol.h"
#include "TrajHandler.h"
#include "CMD.h"
#include "SingleTPG.h"

#include "trajplaner.h"
MACHINE Machine;

void AxisStateMachine(Axis* pAxis,int iAxisNum)
{
    int iCurStatus = (int)pAxis->axisStatus;
	if(pAxis->closeStatus == S_Laser_Close)
	{	
		pAxis->dActPos=pAxis->dSolveMethodTwo;
		pAxis->dActPosNotInUse=pAxis->dSolveMethodOne;
	}
	else
	{
		pAxis->dActPos=pAxis->dSolveMethodOne;
		pAxis->dActPosNotInUse=pAxis->dSolveMethodTwo;
	}

	
	pAxis->dActPos+=pAxis->posOffset;
	trajUpdate(pAxis);
	
/*	if(pAxis->axisStatus!=S_Aixs_Open)
	{
		if(pAxis->vel ==0 )
		{
			pAxis->closeFlag=1;
		}
		else if(pAxis->vel<=TP.v)
		{
			pAxis->closeFlag=2;
		}
		else if(pAxis->vel>=TP.v)
		{
			pAxis->closeFlag=2;
		}
	}*/
	switch(iCurStatus)
	{
		case S_Aixs_Open:
			Open(pAxis,iAxisNum);
			CLEAR_MOTOR_MOVING_FLAG(pAxis->flag);
			SET_MOTOR_INPOS_FLAG(pAxis->flag);
			CLEAR_MOTOR_ENABLE_FLAG(pAxis->flag);
			if(pAxis->axisCMD == C_Aixs_Close)
			{
				pAxis->axisStatus = S_Aixs_Close;
			}
			pAxis->axisCMD = C_Nothing;
		break;
		case S_Aixs_Close:
			Close(pAxis,iAxisNum);
			CLEAR_MOTOR_MOVING_FLAG(pAxis->flag);
			SET_MOTOR_INPOS_FLAG(pAxis->flag);
			SET_MOTOR_ENABLE_FLAG(pAxis->flag);
			//命令改变状态
			if(pAxis->axisCMD == C_Aixs_Open)
			{
				pAxis->axisStatus = S_Aixs_Open;
			}
			else if(pAxis->axisCMD == C_Aixs_Step_For)
			{
				StepFor(pAxis,iAxisNum);
				pAxis->axisStatus = S_Aixs_Close;
			}
			else if(pAxis->axisCMD == C_Aixs_Step_Rev)
			{
				StepRev(pAxis,iAxisNum);
				pAxis->axisStatus = S_Aixs_Close;
			}
			else if(pAxis->axisCMD == C_Aixs_Jogfor_Start)
			{
				CLEAR_MOTOR_INPOS_FLAG(pAxis->flag);
				SET_MOTOR_MOVING_FLAG(pAxis->flag);
				pAxis->axisStatus = S_Aixs_Jogfor_Start;
			}
			else if(pAxis->axisCMD == C_Aixs_Jogrev_Start)
			{
				CLEAR_MOTOR_INPOS_FLAG(pAxis->flag);
				SET_MOTOR_MOVING_FLAG(pAxis->flag);
				pAxis->axisStatus = S_Aixs_Jogrev_Start;
			}
			else if(pAxis->axisCMD == C_Aixs_Home)
			{
				pAxis->axisStatus = S_Aixs_Home;
			}
			else if(pAxis->axisCMD == C_Aixs_RunFor)
			{
				pAxis->axisStatus = S_Aixs_RunFor;	
			}
			else if(pAxis->axisCMD == C_Aixs_RunRev)
			{
				pAxis->axisStatus = S_Aixs_RunRev;
			}
			else if(pAxis->axisCMD == C_Aixs_Ident)
			{
				pAxis->axisStatus = S_Aixs_Ident;
			}		
			pAxis->axisCMD = C_Nothing;
		break;		
		case S_Aixs_Jogfor_Start:
			JogFor(pAxis,iAxisNum);
			if(pAxis->axisCMD == C_Aixs_Jog_Stop)
			{
				pAxis->axisStatus = S_Aixs_Close;
			}
			if(pAxis->axisCMD == C_Aixs_Open)
			{
				pAxis->axisStatus = S_Aixs_Open;
			}
		break;
		case S_Aixs_Jogrev_Start:
			JogRev(pAxis,iAxisNum);
			if(pAxis->axisCMD == C_Aixs_Jog_Stop)
			{
				pAxis->axisStatus = S_Aixs_Close;
			}
			if(pAxis->axisCMD == C_Aixs_Open)
			{
				pAxis->axisStatus = S_Aixs_Open;
			}
		break;
		case S_Aixs_Ident:
//			Ident(pAxis,iAxisNum);
//			Close(pAxis,iAxisNum);
		break;
		case S_Aixs_RunFor:
			//不接受除了开环外的任何命令，目前没有急停
            XYRun_Forward(pAxis);
			if(pAxis->axisCMD == C_Aixs_Open)
			{
				pAxis->axisStatus = S_Aixs_Open;
				pAxis->iTrajRealTimeFlag = 0;
				pAxis->dAffData = 0.0;				
			}
		break;
		case S_Aixs_RunRev:	
			//不接受除了开环外的任何命令，目前没有急停
			XYRun_Reverse(pAxis,iAxisNum);
			if(pAxis->axisCMD == C_Aixs_Open)
			{
				pAxis->axisStatus = S_Aixs_Open;
				pAxis->iTrajRealTimeFlag = 0;
				pAxis->dAffData = 0.0;
			}
		break;
		case S_Aixs_Home:
			//不接受除了开环外的任何命令
			Home(pAxis,iAxisNum);
			if(pAxis->axisCMD == C_Aixs_Open)
			{
				pAxis->axisStatus = S_Aixs_Open;
			}	
		break;
	}
}

void StageStateMachine(Stage *pStage)
{
	int iCurStatus = pStage->stageStatus;
	if(pStage->stageType == CoarseStage)
	{
		switch(iCurStatus)
		{
			case S_Stage_ALL_Open:
				if(pStage->arrAxis[CDX].axisStatus == S_Aixs_Close && pStage->arrAxis[CDY].axisStatus == S_Aixs_Close && pStage->arrAxis[XTZ].axisStatus == S_Aixs_Close && pStage->arrAxis[YTZ].axisStatus == S_Aixs_Close)
				{
					pStage->stageStatus= S_Stage_ALL_Close;
				}
				
                pStage->stageCMD = C_ALL_Nothing;
			break;
			case S_Stage_ALL_Close:
				if(pStage->arrAxis[CDX].axisStatus == S_Aixs_Open && pStage->arrAxis[CDY].axisStatus == S_Aixs_Open && pStage->arrAxis[XTZ].axisStatus == S_Aixs_Open && pStage->arrAxis[YTZ].axisStatus == S_Aixs_Open)
				{
					pStage->stageStatus = S_Stage_ALL_Open;
				}
				else if(pStage->stageCMD == C_Stage_Traj_Expo)
				{
					pStage->stageStatus = S_Stage_Traj_Expo;
				}
				else if(pStage->stageCMD == C_Stage_P2P_Traj)
				{
					pStage->stageStatus = S_Stage_P2P_Traj;
				}
				else if(pStage->stageCMD == C_Stage_Circle_Traj)
				{
					pStage->stageStatus = S_Stage_Circle_Traj;
				}
				else if(pStage->stageCMD == C_Stage_Line_Interpolate)
					pStage->stageStatus = S_Stage_Line_Interpolate;
				else if(pStage->stageCMD == C_Stage_Arc_Interpolate)
					pStage->stageStatus = S_Stage_Arc_Interpolate;
				
                    pStage->stageCMD = C_ALL_Nothing;
			break;
			case  S_Stage_Traj_Expo:    
				if(pStage->DDX_Flag==1 && pStage->DDY_Flag==1)
				{
					Traj_Coarse();
				}
				else
					pStage->Traj();

				if(pStage->arrAxis[CDX].axisStatus == S_Aixs_Open && pStage->arrAxis[CDY].axisStatus == S_Aixs_Open && pStage->arrAxis[XTZ].axisStatus == S_Aixs_Open && pStage->arrAxis[YTZ].axisStatus == S_Aixs_Open)
				{	 			
					pStage->stageStatus = S_Stage_ALL_Open;
					Coarse_Count = 0;
                    pStage->stageCMD = C_ALL_Nothing;
					DXPosLast_Coarse = FirstIn;
					DXNegLast_Coarse = FirstIn;
					DYPosLast_Coarse = FirstIn;
					DYNegLast_Coarse = FirstIn;
				}
			break;
			case S_Stage_P2P_Traj:

			break;
			case S_Stage_Circle_Traj:
				Circle(pStage);
				break;
			case S_Stage_Line_Interpolate:
                if(1 == lineCal(lineInterpolate))
                {
					Machine.coarseStage.stageStatus = S_Stage_ALL_Close;
					Machine.coarseStage.arrAxis[0].dSetPoint=lineInterpolate->current.X;
					Machine.coarseStage.arrAxis[1].dSetPoint=lineInterpolate->current.X;
					Machine.coarseStage.arrAxis[2].dSetPoint=lineInterpolate->current.Y;
					Machine.coarseStage.arrAxis[3].dSetPoint=lineInterpolate->current.Y;
					if(pStage->arrAxis[CDX].axisStatus == S_Aixs_Open || pStage->arrAxis[CDY].axisStatus == S_Aixs_Open || pStage->arrAxis[XTZ].axisStatus == S_Aixs_Open || pStage->arrAxis[YTZ].axisStatus == S_Aixs_Open)
					{	 			
						pStage->stageStatus = S_Stage_ALL_Open;
                        pStage->stageCMD = C_ALL_Nothing;
					}
                }
				break;
			case S_Stage_Arc_Interpolate:
				
				if(arcCal(arcInterpolate)==1)
				{
					Machine.coarseStage.stageStatus = S_Stage_ALL_Close;
				}
				Machine.coarseStage.arrAxis[0].dSetPoint=arcInterpolate->startPoint.X;
				Machine.coarseStage.arrAxis[1].dSetPoint=arcInterpolate->startPoint.X;
				Machine.coarseStage.arrAxis[2].dSetPoint=arcInterpolate->startPoint.Y;
				Machine.coarseStage.arrAxis[3].dSetPoint=arcInterpolate->startPoint.Y;
				if(pStage->arrAxis[CDX].axisStatus == S_Aixs_Open || pStage->arrAxis[CDY].axisStatus == S_Aixs_Open || pStage->arrAxis[XTZ].axisStatus == S_Aixs_Open || pStage->arrAxis[YTZ].axisStatus == S_Aixs_Open)
				{	 			
					pStage->stageStatus = S_Stage_ALL_Open;
                    pStage->stageCMD = C_ALL_Nothing;
				}
				break;
		}
	}
	else if(pStage->stageType == FineStage)
	{
		switch(iCurStatus)
		{
			case S_Stage_ALL_Open:
				if(pStage->arrAxis[FDX].axisStatus == S_Aixs_Close && pStage->arrAxis[FDY].axisStatus == S_Aixs_Close && pStage->arrAxis[FTZ].axisStatus == S_Aixs_Close)
				{
					pStage->stageStatus = S_Stage_ALL_Close;
				}
                pStage->stageCMD = C_ALL_Nothing;
			break;
			case S_Stage_ALL_Close:
				if(pStage->arrAxis[FDX].axisStatus == S_Aixs_Open && pStage->arrAxis[FDY].axisStatus == S_Aixs_Open && pStage->arrAxis[FTZ].axisStatus == S_Aixs_Open)
				{
					pStage->stageStatus = S_Stage_ALL_Open;
				}
				if(pStage->stageCMD == C_Stage_Traj_Expo)
				{
					pStage->stageStatus = S_Stage_Traj_Expo;
				}
				if(pStage->stageCMD == C_Stage_P2P_Traj)
				{
					pStage->stageStatus = S_Stage_P2P_Traj;
				}
			break;
			case S_Stage_Traj_Expo:
				pStage->Traj();
				if(pStage->arrAxis[FDX].axisStatus == S_Aixs_Open && pStage->arrAxis[FDY].axisStatus == S_Aixs_Open && pStage->arrAxis[FTZ].axisStatus == S_Aixs_Open)
				{
					pStage->stageStatus = S_Stage_ALL_Open;
					Fine_Count = 0;
                    pStage->stageCMD = C_ALL_Nothing;
					DXPosLast_Fine = FirstIn;
					DXNegLast_Fine = FirstIn;
					DYPosLast_Fine = FirstIn;
					DYNegLast_Fine = FirstIn;
				}
			break;
			case S_Stage_P2P_Traj:

			break;
			case S_Stage_Load_Pos:	
			break;	
		}
	}	
}

void SwitchStateMachine(Axis* pAxis,int iAxisNum)
{
	switch(pAxis->closeStatus)
	{
		case S_Counter_Close:
            CounterClose(pAxis);
			if(pAxis->closeCMD == C_Laser_Close)
			{
				pAxis->closeStatus = S_Laser_Close;
			}
		break;
		case S_Laser_Close:
            LaserSwitch(pAxis);
			if(pAxis->closeCMD == C_Counter_Close)
			{
				pAxis->closeStatus = S_Counter_Close;
			}
		break;
	}
}

