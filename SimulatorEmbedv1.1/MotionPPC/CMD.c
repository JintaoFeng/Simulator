#include "CMD.h"
#include "FileReadWrite.h"
#include "N1225A.h"
#include "StateMachine.h"
#include "EnumDefines.h"
#include "TrajGen_4OrderPoly.h"
#include "TrajHandler.h"
#include "Rtcontrol.h"
#include "Trajplaner.h"
#include "GetData.h"
#include "FileReadWrite.h"
#include "SingleTPG.h"
#include "Main.h"

extern char N1225A_RECORD_FLAG;
extern char open_flag;
extern double ParamData[10];
extern char T_Coarse[1000],T_Fine[1000];


/*
commFun_t commFun[]=
{
	{MOT_ABORT,motionAbort},
	{MOT_ENABLE,motorEnable},
	{MOT_DISABLE,motorDisable},
	{MOT_PAUSE,motionPause},
	{MOT_FORWARD,motionForward},
	{MOT_REVERSE,motionRevrese},
	{MOT_JOGFOR,motionJogFor},
	{MOT_JOGREV,motionJogRev},
	{MOT_STEP,motionStep},
	{MOT_FREE,motionFree},
	{MOT_COORD,motionCooard},
	{MOT_TELEOP,motionTeleop},
	{MOT_SETKP,setKp},
	{MOT_SETKI,setKi},
	{MOT_SETKD,setKd},
	{MOT_SETPOS,setPos},
	{MOT_SETVEL,setVel},
	{MOT_SETACC,setAcc},
	{MOT_SETJERK,setJerk},
	{MOT_SETSNAP,setSnap},
	{MOT_SETACCFEEDCOFF,setFeedForwardCoff},
	{MOT_SETDAOFFSET,setDAOffset},
	{MOT_SETJERK,motionAbort},
};

/*
void setKp(RXData_t *rxdata)
{
	if(rxdata->axis<4)
	{
		Machine.coarseStage.arrAxis[rxdata->axis].sPID.Proportion=rxdata->controller[rxdata->axis].pid.kp;
	}
	else if(rxdata->axis>=4)
	{
		Machine.fineStage.arrAxis[rxdata->axis-4].sPID.Proportion=rxdata->controller[rxdata->axis].pid.kp;
	}
}
void motionAbort(RXData_t* rxdata)
{
	printf("中断\n");
}
void motorEnable(RXData_t* rxdata)
{
	if(rxdata->axis<4)
	{
		Machine.coarseStage.arrAxis[rxdata->axis].axisCMD=C_Aixs_Close;
	}
	else if(rxdata->axis>=4)
	{
		Machine.fineStage.arrAxis[rxdata->axis-4].axisCMD=C_Aixs_Close;
	}
}
void motorDisable(RXData_t* rxdata)
{
	if(rxdata->axis<4)
	{
		Machine.coarseStage.arrAxis[rxdata->axis].axisCMD=C_Aixs_Open;
	}
	else if(rxdata->axis>=4)
	{
		Machine.fineStage.arrAxis[rxdata->axis-4].axisCMD=C_Aixs_Open;
	}
}
void motionPause(RXData_t *rxdata)
{
	
}

void motionForward(RXData_t *rxdata)
{
	
}
void motionRevrese(RXData_t *rxdata)
{
	
}
void motionJogFor(RXData_t *rxdata)
{
	
}
void motionJogRev(RXData_t *rxdata)
{
	
}

void motionStep(RXData_t *rxdata)
{
	
}

void motionFree(RXData_t *rxdata)
{

}
void motionCooard(RXData_t *rxdata)
{

}

void motionTeleop(RXData_t *rxdata)
{
	
}

void setKi(RXData_t *rxdata)
{
	if(rxdata->axis<4)
	{
		Machine.coarseStage.arrAxis[rxdata->axis].sPID.Integral=rxdata->controller[rxdata->axis].pid.ki;
	}
	else if(rxdata->axis>=4)
	{
		Machine.fineStage.arrAxis[rxdata->axis-4].sPID.Proportion=rxdata->controller[rxdata->axis].pid.ki;
	}
}

void setKd(RXData_t *rxdata)
{
	if(rxdata->axis<4)
	{
		Machine.coarseStage.arrAxis[rxdata->axis].sPID.Derivative=rxdata->controller[rxdata->axis].pid.kd;
	}
	else if(rxdata->axis>=4)
	{
		Machine.fineStage.arrAxis[rxdata->axis-4].sPID.Proportion=rxdata->controller[rxdata->axis].pid.kd;
	}
}
void setPos(RXData_t *rxdata)
{
	if(rxdata->axis<4)
	{
		Machine.coarseStage.arrAxis[rxdata->axis].targetPos = rxdata->traj[rxdata->axis].pos;
	}
	else if(rxdata->axis>=4)
	{
		Machine.fineStage.arrAxis[rxdata->axis-4].targetPos = rxdata->traj[rxdata->axis].pos;
	}
	Machine.coarseStage.arrAxis[rxdata->axis].trajUpdate=1;
	printf("pos:%lf",rxdata->traj[rxdata->axis].pos);
}

void setVel(RXData_t *rxdata)
{
	if(rxdata->axis<4)
	{
		Machine.coarseStage.arrAxis[rxdata->axis].targetVel = rxdata->traj[rxdata->axis].vel;
	}
	else if(rxdata->axis>=4)
	{
		Machine.fineStage.arrAxis[rxdata->axis-4].targetVel = rxdata->traj[rxdata->axis].vel;
	}
	Machine.coarseStage.arrAxis[rxdata->axis].trajUpdate=1;
	printf("vel:%lf",rxdata->traj[rxdata->axis].vel);
}

void setAcc(RXData_t *rxdata)
{
	if(rxdata->axis<4)
	{
		Machine.coarseStage.arrAxis[rxdata->axis].targetAcc = rxdata->traj[rxdata->axis].acc;
	}
	else if(rxdata->axis>=4)
	{
		Machine.fineStage.arrAxis[rxdata->axis-4].targetAcc = rxdata->traj[rxdata->axis].acc;
	}
	Machine.coarseStage.arrAxis[rxdata->axis].trajUpdate=1;
	printf("acc:%lf",rxdata->traj[rxdata->axis].acc);
}
void setJerk(RXData_t *rxdata)
{
	if(rxdata->axis<4)
	{
		Machine.coarseStage.arrAxis[rxdata->axis].targetJerk= rxdata->traj[rxdata->axis].jerk;
	}
	else if(rxdata->axis>=4)
	{
		Machine.fineStage.arrAxis[rxdata->axis-4].targetJerk= rxdata->traj[rxdata->axis].jerk;
	}
	Machine.coarseStage.arrAxis[rxdata->axis].trajUpdate=1;
	printf("jerk:%lf",rxdata->traj[rxdata->axis].jerk);
}
void setSnap(RXData_t *rxdata)
{
	if(rxdata->axis<4)
	{
		Machine.coarseStage.arrAxis[rxdata->axis].targetSnap= rxdata->traj[rxdata->axis].snap;
	}
	else if(rxdata->axis>=4)
	{
		Machine.fineStage.arrAxis[rxdata->axis-4].targetSnap= rxdata->traj[rxdata->axis].snap;
	}
	Machine.coarseStage.arrAxis[rxdata->axis].trajUpdate=1;
	printf("snap:%lf",rxdata->traj[rxdata->axis].snap);
}

void setFeedForwardCoff(RXData_t *rxdata)
{
	if(rxdata->axis<4)
	{
		Machine.coarseStage.arrAxis[rxdata->axis].dFeedForwardCoef=rxdata->controller[rxdata->axis].feedForwardCoff;
	}
	else if(rxdata->axis>=4)
	{
		Machine.fineStage.arrAxis[rxdata->axis-4].dFeedForwardCoef=rxdata->controller[rxdata->axis].feedForwardCoff;
	}
}
void setDAOffset(RXData_t *rxdata)
{

}
*/

void GetCMD(int *pCMD)
{
	int i = 0;	
	
	int CAL_MIN_J_AND_D =1;
	double p_dd,p_ddY,p_ddRev;
	double Td, Tj, Ta, Tv;
	double TdY, TjY, TaY, TvY;
	double TdRev, TjRev, TaRev, TvRev;

	switch(*pCMD)
	{
		case RECORD_SELECT:
//			recordMap[recvData.iReserved[0]].func(recordMap[recvData.iReserved[0]].arg1,recvData.iReserved[1]);
	//		commFun[recvData.command].fun(&recvData);
			break;
		case ALL_CLOSE:
			Machine.fineStage.arrAxis[FDX].axisCMD = C_Aixs_Close;
			Machine.fineStage.arrAxis[FDY].axisCMD = C_Aixs_Close;
			Machine.fineStage.arrAxis[FTZ].axisCMD = C_Aixs_Close;
		
			Machine.fineStage.arrAxis[FDZ].axisCMD = C_Aixs_Close;
			Machine.fineStage.arrAxis[FTX].axisCMD = C_Aixs_Close;
			Machine.fineStage.arrAxis[FTY].axisCMD = C_Aixs_Close;

			Machine.coarseStage.arrAxis[CDX].axisCMD = C_Aixs_Close;
			Machine.coarseStage.arrAxis[CDY].axisCMD = C_Aixs_Close;
			Machine.coarseStage.arrAxis[XTZ].axisCMD = C_Aixs_Close;
			Machine.coarseStage.arrAxis[YTZ].axisCMD = C_Aixs_Close;
			DisableIntr1();
			DisableIntr3();
		break;

		case COARSE_ALL_CLOSE:
			Machine.coarseStage.arrAxis[CDX].axisCMD = C_Aixs_Close;
			Machine.coarseStage.arrAxis[CDY].axisCMD = C_Aixs_Close;
			Machine.coarseStage.arrAxis[XTZ].axisCMD = C_Aixs_Close;
			Machine.coarseStage.arrAxis[YTZ].axisCMD = C_Aixs_Close;
		break;
		case COARSE_ALL_OPEN:
			Machine.coarseStage.arrAxis[CDX].axisCMD = C_Aixs_Open;
			Machine.coarseStage.arrAxis[CDY].axisCMD = C_Aixs_Open;
			Machine.coarseStage.arrAxis[XTZ].axisCMD = C_Aixs_Open;
			Machine.coarseStage.arrAxis[YTZ].axisCMD = C_Aixs_Open;
		break;
		case COARSE_ALL_HOME:
			Machine.coarseStage.arrAxis[2].bHomeDone =0.0;
		    Machine.coarseStage.arrAxis[0].bHomeDone =0.0;
			 
			 Machine.coarseStage.arrAxis[1].axisStatus = S_Aixs_Open;

			if(S_Aixs_Close==Machine.coarseStage.arrAxis[2].axisStatus)
			{
				//EnableIntr3();	
				//Machine.coarseStage.arrAxis[2].bIntrEnable=1;	
				//Machine.HomeFlagCorse = 1;
				Machine.coarseStage.arrAxis[2].HomeFlagCorse =  1.0;
			}
			Machine.coarseStage.arrAxis[2].axisCMD = C_Aixs_Home;
			//Machine.coarseStage.arrAxis[3].axisCMD = C_Aixs_Home;

			if(S_Aixs_Close==Machine.coarseStage.arrAxis[0].axisStatus)
			{
				Machine.coarseStage.arrAxis[0].HomeFlagCorse =	1.0;
			}
			Machine.coarseStage.arrAxis[0].axisCMD = C_Aixs_Home;
	
		break;

		//1粗动台闭环与开环
		case COARSE_DX_CLOSE:
			Machine.coarseStage.arrAxis[CDX].axisCMD = C_Aixs_Close;
		break;
		//X2轴闭环
		case COARSE_DY_CLOSE:
			Machine.coarseStage.arrAxis[CDY].axisCMD = C_Aixs_Close;
		break;
		//Y1轴闭环
		case COARSE_XTZ_CLOSE:
			Machine.coarseStage.arrAxis[XTZ].axisCMD = C_Aixs_Close;
		break;
		//Y2轴闭环
		case COARSE_YTZ_CLOSE:
			Machine.coarseStage.arrAxis[YTZ].axisCMD = C_Aixs_Close;
		break;
		//X1轴开环
		case COARSE_DX_OPEN:			
				Machine.coarseStage.arrAxis[CDX].axisCMD = C_Aixs_Open;
		break;
		//X2轴开环
		case COARSE_DY_OPEN:
				Machine.coarseStage.arrAxis[CDY].axisCMD = C_Aixs_Open;
		break;
		//Y1轴开环
		case COARSE_XTZ_OPEN:
				Machine.coarseStage.arrAxis[XTZ].axisCMD = C_Aixs_Open;
		break;
		//Y2轴开环
		case COARSE_YTZ_OPEN:
				Machine.coarseStage.arrAxis[YTZ].axisCMD = C_Aixs_Open;
		break;

		//2粗动台各自由度Step
		case COARSE_DX_STEP_FORWARD:
			Machine.coarseStage.arrAxis[CDX].axisCMD = C_Aixs_Step_For;
		break;
		case COARSE_DY_STEP_FORWARD:
			Machine.coarseStage.arrAxis[CDY].axisCMD = C_Aixs_Step_For;
		break;
		case COARSE_XTZ_STEP_FORWARD:
			Machine.coarseStage.arrAxis[XTZ].axisCMD = C_Aixs_Step_For;
		break;
		case COARSE_YTZ_STEP_FORWARD:
			Machine.coarseStage.arrAxis[YTZ].axisCMD = C_Aixs_Step_For;
		break;
		case COARSE_DX_STEP_REVERSE:
			Machine.coarseStage.arrAxis[CDX].axisCMD = C_Aixs_Step_Rev;
		break;
		case COARSE_DY_STEP_REVERSE:
			Machine.coarseStage.arrAxis[CDY].axisCMD = C_Aixs_Step_Rev;
		break;
		case COARSE_XTZ_STEP_REVERSE:
			Machine.coarseStage.arrAxis[XTZ].axisCMD = C_Aixs_Step_Rev;
		break;
		case COARSE_YTZ_STEP_REVERSE:
			Machine.coarseStage.arrAxis[YTZ].axisCMD = C_Aixs_Step_Rev;
		break;
		//3粗动台走JOG
		case COARSE_DX_JOG_FORWARD:
			Machine.coarseStage.arrAxis[CDX].axisCMD = C_Aixs_Jogfor_Start;
		break;
		case COARSE_DY_JOG_FORWARD:
			Machine.coarseStage.arrAxis[CDY].axisCMD = C_Aixs_Jogfor_Start;
		break;
		case COARSE_XTZ_JOG_FORWARD:
			Machine.coarseStage.arrAxis[XTZ].axisCMD = C_Aixs_Jogfor_Start;
		break;
		case COARSE_YTZ_JOG_FORWARD:
			Machine.coarseStage.arrAxis[YTZ].axisCMD = C_Aixs_Jogfor_Start;
		break;
		case COARSE_DX_JOG_REVERSE:
			Machine.coarseStage.arrAxis[CDX].axisCMD = C_Aixs_Jogrev_Start;
		break;
		case COARSE_DY_JOG_REVERSE:
			Machine.coarseStage.arrAxis[CDY].axisCMD = C_Aixs_Jogrev_Start;
		break;
		case COARSE_XTZ_JOG_REVERSE:
			Machine.coarseStage.arrAxis[XTZ].axisCMD = C_Aixs_Jogrev_Start;
		break;
		case COARSE_YTZ_JOG_REVERSE:
			Machine.coarseStage.arrAxis[YTZ].axisCMD = C_Aixs_Jogrev_Start;
		break;
		case COARSE_DX_JOG_STOP:
			Machine.coarseStage.arrAxis[CDX].axisCMD = C_Aixs_Jog_Stop;
		break;
		case COARSE_DY_JOG_STOP:
			Machine.coarseStage.arrAxis[CDY].axisCMD = C_Aixs_Jog_Stop;
		break;
		case COARSE_XTZ_JOG_STOP:
			Machine.coarseStage.arrAxis[XTZ].axisCMD = C_Aixs_Jog_Stop;
		break;
		case COARSE_YTZ_JOG_STOP:
			Machine.coarseStage.arrAxis[YTZ].axisCMD = C_Aixs_Jog_Stop;
		break;
		//4粗动台辨识
		case COARSE_DX_IDENTIFY:
			Machine.coarseStage.arrAxis[CDX].axisCMD = C_Aixs_Ident;
		break;
		case COARSE_DY_IDENTIFY:
			Machine.coarseStage.arrAxis[CDY].axisCMD = C_Aixs_Ident;
		break;
		case COARSE_XTZ_IDENTIFY:
			//Machine.coarseStage.arrAxis[XTZ].axisCMD = C_Aixs_Ident;
			Machine.coarseStage.arrAxis[XTZ].bIdent = 1;
			recordStruct.iRecordFlag = 1;
		break;
		case COARSE_YTZ_IDENTIFY:
			Machine.coarseStage.arrAxis[YTZ].axisCMD = C_Aixs_Ident;
		break;
		//5粗动台回零
		case COARSE_DX_HOME:

		    Machine.coarseStage.arrAxis[0].bHomeDone=0.0;
			Machine.coarseStage.arrAxis[1].axisCMD= C_Aixs_Open;

			Machine.coarseStage.arrAxis[0].HomeFlagCorse =	1.0;
			Machine.coarseStage.arrAxis[0].axisCMD = C_Aixs_Home;
			Machine.coarseStage.arrAxis[0].stepHome=1;

		break;
		case COARSE_DY_HOME:
			//Machine.coarseStage.arrAxis[CDY].axisCMD = C_Aixs_Home;
		break;
		case COARSE_XTZ_HOME:
			Machine.coarseStage.arrAxis[0].bHomeDone=0.0;
			Machine.coarseStage.arrAxis[3].axisCMD= C_Aixs_Open;
			Machine.coarseStage.arrAxis[2].HomeFlagCorse =	1.0;
			Machine.coarseStage.arrAxis[2].axisCMD = C_Aixs_Home;
			Machine.coarseStage.arrAxis[2].stepHome=1;
		break;
		case COARSE_YTZ_HOME:
			Machine.coarseStage.arrAxis[YTZ].axisCMD = C_Aixs_Home;
		break;
		//6 粗动台轨迹
		case COARSE_DX_RUN_FORWARD:
			Machine.coarseStage.arrAxis[0].cTrig=1;
			Machine.coarseStage.arrAxis[0].axisCMD = C_Aixs_RunFor;			
		break;
		case COARSE_DX_RUN_REVERSE:
			Machine.coarseStage.arrAxis[0].cTrig=1;
			Machine.coarseStage.arrAxis[0].axisCMD = C_Aixs_RunRev;
		break;

		case COARSE_DY_RUN_FORWARD:
			Machine.coarseStage.arrAxis[1].cTrig=1;
			Machine.coarseStage.arrAxis[1].axisCMD = C_Aixs_RunFor;
		break;
		case COARSE_DY_RUN_REVERSE:
			Machine.coarseStage.arrAxis[1].cTrig=1;
			Machine.coarseStage.arrAxis[1].axisCMD = C_Aixs_RunRev;
		break;


		case COARSE_XTZ_RUN_FORWARD:
			Machine.coarseStage.arrAxis[2].cTrig=1;
			Machine.coarseStage.arrAxis[2].axisCMD = C_Aixs_RunFor;			
		break;
		case COARSE_XTZ_RUN_REVERSE:
			Machine.coarseStage.arrAxis[2].cTrig=1;
			Machine.coarseStage.arrAxis[2].axisCMD = C_Aixs_RunRev;
		break;

		case COARSE_YTZ_RUN_FORWARD:
			Machine.coarseStage.arrAxis[3].cTrig=1;
			Machine.coarseStage.arrAxis[3].axisCMD = C_Aixs_RunFor;
		break;
		case COARSE_YTZ_RUN_REVERSE:
			Machine.coarseStage.arrAxis[3].cTrig=1;
			Machine.coarseStage.arrAxis[3].axisCMD = C_Aixs_RunRev;
		break;

		//7粗动台切换
		case COARSE_DX_LASERSWITCH:
			Machine.coarseStage.arrAxis[CDX].closeCMD = C_Laser_Close;
		break;
		case COARSE_DY_LASERSWITCH:
			Machine.coarseStage.arrAxis[CDY].closeCMD = C_Laser_Close;
		break;
		case COARSE_XTZ_LASERSWITCH:
			Machine.coarseStage.arrAxis[XTZ].closeCMD = C_Laser_Close;
		break;
		case COARSE_YTZ_LASERSWITCH:
			Machine.coarseStage.arrAxis[YTZ].closeCMD = C_Laser_Close;
		break;
		case COARSE_DX_EDDYSWITCH:
			Machine.coarseStage.arrAxis[CDX].closeCMD = C_Counter_Close;
		break;
		case COARSE_DY_EDDYSWITCH:
			Machine.coarseStage.arrAxis[CDY].closeCMD = C_Counter_Close;
		break;
		case COARSE_XTZ_EDDYSWITCH:
			Machine.coarseStage.arrAxis[XTZ].closeCMD = C_Counter_Close;
		break;
		case COARSE_YTZ_EDDYSWITCH:
			Machine.coarseStage.arrAxis[YTZ].closeCMD = C_Counter_Close;
		break;
		
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
//1双边粗动台闭环与开环
		case COARSE_DDX_CLOSE:
			Machine.coarseStage.DDX_Flag = 1;
			break;
		case COARSE_DDY_CLOSE:
			Machine.coarseStage.DDY_Flag = 1;
		break;
		case COARSE_DDX_OPEN:
			Machine.coarseStage.DDX_Flag = 0;
		break;
		case COARSE_DDY_OPEN:

			Machine.coarseStage.DDY_Flag = 0;
		break;
//2双边粗动台各自由度Step
		case COARSE_DDX_STEP_FORWARD:
			Machine.coarseStage.arrAxis[0].axisCMD = C_Aixs_Step_For;
			Machine.coarseStage.arrAxis[1].axisCMD = C_Aixs_Step_For;
		break;
		case COARSE_DDY_STEP_FORWARD:
			Machine.coarseStage.arrAxis[2].axisCMD = C_Aixs_Step_For;
			Machine.coarseStage.arrAxis[3].axisCMD = C_Aixs_Step_For;
		break;
		case COARSE_DDX_STEP_REVERSE:
			Machine.coarseStage.arrAxis[0].axisCMD = C_Aixs_Step_Rev;
			Machine.coarseStage.arrAxis[1].axisCMD = C_Aixs_Step_Rev;
		break;
		case COARSE_DDY_STEP_REVERSE:
			Machine.coarseStage.arrAxis[2].axisCMD = C_Aixs_Step_Rev;
			Machine.coarseStage.arrAxis[3].axisCMD = C_Aixs_Step_Rev;
		break;
//3双边粗动台走JOG
		case COARSE_DDX_JOG_FORWARD:
			Machine.coarseStage.arrAxis[0].axisCMD = C_Aixs_Jogfor_Start;
			Machine.coarseStage.arrAxis[1].axisCMD = C_Aixs_Jogfor_Start;
		break;
		case COARSE_DDY_JOG_FORWARD:
			Machine.coarseStage.arrAxis[2].axisCMD = C_Aixs_Jogfor_Start;
			Machine.coarseStage.arrAxis[3].axisCMD = C_Aixs_Jogfor_Start;
		break;
		case COARSE_DDX_JOG_REVERSE:
			Machine.coarseStage.arrAxis[0].axisCMD = C_Aixs_Jogrev_Start;
			Machine.coarseStage.arrAxis[1].axisCMD = C_Aixs_Jogrev_Start;
		break;
		case COARSE_DDY_JOG_REVERSE:
			Machine.coarseStage.arrAxis[2].axisCMD = C_Aixs_Jogrev_Start;
			Machine.coarseStage.arrAxis[3].axisCMD = C_Aixs_Jogrev_Start;
		break;
		case COARSE_DDX_JOG_STOP:
			Machine.coarseStage.arrAxis[0].axisCMD = C_Aixs_Jog_Stop;
			Machine.coarseStage.arrAxis[1].axisCMD = C_Aixs_Jog_Stop;
		break;
		case COARSE_DDY_JOG_STOP:
			Machine.coarseStage.arrAxis[2].axisCMD = C_Aixs_Jog_Stop;
			Machine.coarseStage.arrAxis[3].axisCMD = C_Aixs_Jog_Stop;
		break;

		//4双边粗动台辨识
		/*case COARSE_DDX_IDENTIFY:
			Machine.coarseStage.arrAxis[0].axisCMD = C_Aixs_Ident;
			Machine.coarseStage.arrAxis[1].axisCMD = C_Aixs_Ident;
		break;
		case COARSE_DDY_IDENTIFY:
			Machine.coarseStage.arrAxis[2].axisCMD = C_Aixs_Ident;
			Machine.coarseStage.arrAxis[3].axisCMD = C_Aixs_Ident;
		break;*/
		//5双边粗动台回零
		case COARSE_DDX_HOME:
			Machine.coarseStage.arrAxis[0].bHomeDone =0.0;
			if( 1==Machine.coarseStage.DDX_Flag )
			{
				Machine.coarseStage.arrAxis[0].axisCMD = C_Aixs_Home;
				Machine.coarseStage.arrAxis[0].HomeFlagCorse =  1.0;
				Machine.coarseStage.arrAxis[1].axisStatus = S_Aixs_Close;
				Machine.coarseStage.arrAxis[0].stepHome=1;
			}
	
		break;
		case COARSE_DDY_HOME:

			Machine.coarseStage.arrAxis[2].bHomeDone =0.0;
			if( 1==Machine.coarseStage.DDY_Flag )
			{
				Machine.coarseStage.arrAxis[2].axisCMD = C_Aixs_Home;
				Machine.coarseStage.arrAxis[2].HomeFlagCorse =  1.0;
				Machine.coarseStage.arrAxis[3].axisStatus = S_Aixs_Close;
				Machine.coarseStage.arrAxis[2].stepHome=1;	
			}
		
		break;
		//6双边粗动台切换
		/*case COARSE_DX_LASERSWITCH:
			Machine.coarseStage.arrAxis[CDX].closeCMD = C_Laser_Close;
		break;
		case COARSE_DY_LASERSWITCH:
			Machine.coarseStage.arrAxis[CDY].closeCMD = C_Laser_Close;
		break;
		case COARSE_XTZ_LASERSWITCH:
			Machine.coarseStage.arrAxis[XTZ].closeCMD = C_Laser_Close;
		break;
		case COARSE_YTZ_LASERSWITCH:
			Machine.coarseStage.arrAxis[YTZ].closeCMD = C_Laser_Close;
		break;
		case COARSE_DX_EDDYSWITCH:
			Machine.coarseStage.arrAxis[CDX].closeCMD = C_Counter_Close;
		break;
		case COARSE_DY_EDDYSWITCH:
			Machine.coarseStage.arrAxis[CDY].closeCMD = C_Counter_Close;
		break;
		case COARSE_XTZ_EDDYSWITCH:
			Machine.coarseStage.arrAxis[XTZ].closeCMD = C_Counter_Close;
		break;
		case COARSE_YTZ_EDDYSWITCH:
			Machine.coarseStage.arrAxis[YTZ].closeCMD = C_Counter_Close;
		break;*/


		//7双边轨迹
		case COARSE_DDX_RUN_FORWARD:
			Machine.coarseStage.arrAxis[0].cTrig=1;
			Machine.coarseStage.arrAxis[1].cTrig=1;			
			Machine.coarseStage.arrAxis[0].axisCMD = C_Aixs_RunFor;
			Machine.coarseStage.arrAxis[1].axisCMD = C_Aixs_RunFor;
		break;
		case COARSE_DDX_RUN_REVERSE:
			Machine.coarseStage.arrAxis[0].cTrig=1;
			Machine.coarseStage.arrAxis[1].cTrig=1;			
			Machine.coarseStage.arrAxis[0].axisCMD = C_Aixs_RunRev;
			Machine.coarseStage.arrAxis[1].axisCMD = C_Aixs_RunRev;
		break;

		case COARSE_DDY_RUN_FORWARD:
			Machine.coarseStage.arrAxis[2].cTrig=1;
			Machine.coarseStage.arrAxis[3].cTrig=1;			
			Machine.coarseStage.arrAxis[2].axisCMD = C_Aixs_RunFor;
			Machine.coarseStage.arrAxis[3].axisCMD = C_Aixs_RunFor;
		break;
		case COARSE_DDY_RUN_REVERSE:

			Machine.coarseStage.arrAxis[2].cTrig=1;
			Machine.coarseStage.arrAxis[3].cTrig=1;			
			Machine.coarseStage.arrAxis[2].axisCMD = C_Aixs_RunRev;
			Machine.coarseStage.arrAxis[3].axisCMD = C_Aixs_RunRev;
		break;
		
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
		case FINE_ALL_CLOSE:
			Machine.fineStage.arrAxis[FDX].axisCMD = C_Aixs_Close;
			Machine.fineStage.arrAxis[FDY].axisCMD = C_Aixs_Close;
			Machine.fineStage.arrAxis[FTZ].axisCMD = C_Aixs_Close;

		    Machine.fineStage.arrAxis[FDZ].axisCMD = C_Aixs_Close;
			Machine.fineStage.arrAxis[FTX].axisCMD = C_Aixs_Close;
			Machine.fineStage.arrAxis[FTY].axisCMD = C_Aixs_Close;		
		break;

		case FINE_ALL_OPEN:
			Machine.fineStage.arrAxis[FDX].axisCMD = C_Aixs_Open;
			Machine.fineStage.arrAxis[FDY].axisCMD = C_Aixs_Open;
			Machine.fineStage.arrAxis[FTZ].axisCMD = C_Aixs_Open;
			Machine.fineStage.arrAxis[FDZ].axisCMD = C_Aixs_Open;
			Machine.fineStage.arrAxis[FTX].axisCMD = C_Aixs_Open;
			Machine.fineStage.arrAxis[FTY].axisCMD = C_Aixs_Open;
		break;

		case FINE_ALL_HOME:
			Machine.fineStage.arrAxis[FDX].axisCMD = C_Aixs_Home;
			Machine.fineStage.arrAxis[FDY].axisCMD = C_Aixs_Home;
			Machine.fineStage.arrAxis[FTZ].axisCMD = C_Aixs_Home;
			Machine.fineStage.arrAxis[FDZ].axisCMD = C_Aixs_Home;
			Machine.fineStage.arrAxis[FTX].axisCMD = C_Aixs_Home;
			Machine.fineStage.arrAxis[FTY].axisCMD = C_Aixs_Home;
		break;

		case FINE_LASER_SWITCH:
			Machine.fineStage.arrAxis[FDX].closeCMD = C_Laser_Close;
			Machine.fineStage.arrAxis[FDY].closeCMD = C_Laser_Close;				
			Machine.fineStage.arrAxis[FTZ].closeCMD = C_Laser_Close;
			Machine.fineStage.arrAxis[FDZ].closeCMD = C_Laser_Close;
			Machine.fineStage.arrAxis[FTX].closeCMD = C_Laser_Close;				
			Machine.fineStage.arrAxis[FTY].closeCMD = C_Laser_Close;
		break;

		//1微动台闭环与开环
		case FINE_DX_CLOSE:
			Machine.fineStage.arrAxis[FDX].axisCMD = C_Aixs_Close;
		break;
		case FINE_DY_CLOSE:
			Machine.fineStage.arrAxis[FDY].axisCMD = C_Aixs_Close;
		break;
		case FINE_TZ_CLOSE:
			Machine.fineStage.arrAxis[FTZ].axisCMD = C_Aixs_Close;
		break;

		case FINE_DX_OPEN:
			Machine.fineStage.arrAxis[FDX].axisCMD = C_Aixs_Open;
		break;
		case FINE_DY_OPEN:
			Machine.fineStage.arrAxis[FDY].axisCMD = C_Aixs_Open;
		break;
		case FINE_TZ_OPEN:
			Machine.fineStage.arrAxis[FTZ].axisCMD = C_Aixs_Open;
		break;

		//2微动台STEP
		case FINE_DX_STEP_FORWARD:
			Machine.fineStage.arrAxis[FDX].axisCMD = C_Aixs_Step_For;
		break;
		case FINE_DY_STEP_FORWARD:
			Machine.fineStage.arrAxis[FDY].axisCMD = C_Aixs_Step_For;
		break;
		case FINE_TZ_STEP_FORWARD:
			Machine.fineStage.arrAxis[FTZ].axisCMD = C_Aixs_Step_For;
		break;
		case FINE_DX_STEP_REVERSE:
			Machine.fineStage.arrAxis[FDX].axisCMD = C_Aixs_Step_Rev;
		break;
		case FINE_DY_STEP_REVERSE:
			Machine.fineStage.arrAxis[FDY].axisCMD = C_Aixs_Step_Rev;
		break;
		case FINE_TZ_STEP_REVERSE:
			Machine.fineStage.arrAxis[FTZ].axisCMD = C_Aixs_Step_Rev;
		break;
		case FINE_DX_JOG_FORWARD:
			Machine.fineStage.arrAxis[FDX].axisCMD = C_Aixs_Jogfor_Start;
		break;
		//3微动台走JOG
		case FINE_DY_JOG_FORWARD:
			Machine.fineStage.arrAxis[FDY].axisCMD = C_Aixs_Jogfor_Start;
		break;
		case FINE_TZ_JOG_FORWARD:
			Machine.fineStage.arrAxis[FTZ].axisCMD = C_Aixs_Jogfor_Start;
		break;
		case FINE_DX_JOG_REVERSE:
			Machine.fineStage.arrAxis[FDX].axisCMD = C_Aixs_Jogrev_Start;
		break;
		case FINE_DY_JOG_REVERSE:
			Machine.fineStage.arrAxis[FDY].axisCMD = C_Aixs_Jogrev_Start;
		break;
		case FINE_TZ_JOG_REVERSE:
			Machine.fineStage.arrAxis[FTZ].axisCMD = C_Aixs_Jogrev_Start;
		break;
		case FINE_DX_JOG_STOP:
			Machine.fineStage.arrAxis[FDX].axisCMD = C_Aixs_Jog_Stop;
		break;
		case FINE_DY_JOG_STOP:
			Machine.fineStage.arrAxis[FDY].axisCMD = C_Aixs_Jog_Stop;
		break;
		case FINE_TZ_JOG_STOP:
			Machine.fineStage.arrAxis[FTZ].axisCMD = C_Aixs_Jog_Stop;
		break;
		//4微动台辨识
		case FINE_DX_IDENTIFY:
			//Machine.fineStage.arrAxis[FDX].axisCMD = C_Aixs_Ident;
			Machine.fineStage.arrAxis[FDX].bIdent = 1;
			recordStruct.iRecordFlag = 1;
		break;
		case FINE_DY_IDENTIFY:
			//Machine.fineStage.arrAxis[FDY].axisCMD = C_Aixs_Ident;
			Machine.fineStage.arrAxis[FDY].bIdent = 1;
            recordStruct.iRecordFlag = 1;
		break;
		case FINE_TZ_IDENTIFY:
			//Machine.fineStage.arrAxis[FTZ].axisCMD = C_Aixs_Ident;
			Machine.fineStage.arrAxis[FTZ].bIdent = 1;
			recordStruct.iRecordFlag = 1;

		break;

		case BOTH_DX_IDENTIFY:
			//Machine.fineStage.arrAxis[FDX].axisCMD = C_Aixs_Ident;
			Machine.fineStage.arrAxis[FDZ].bIdent = 1;
			recordStruct.iRecordFlag = 1;
		break;
		case BOTH_DY_IDENTIFY:
			//Machine.fineStage.arrAxis[FDY].axisCMD = C_Aixs_Ident;
			Machine.fineStage.arrAxis[FTX].bIdent = 1;
            recordStruct.iRecordFlag = 1;

		break;
		case BOTH_TZ_IDENTIFY:
			//Machine.fineStage.arrAxis[FTZ].axisCMD = C_Aixs_Ident;
			Machine.fineStage.arrAxis[FTY].bIdent = 1;
			recordStruct.iRecordFlag = 1;
		break;

		//5粗动台回零
		case FINE_DX_HOME:
			Machine.fineStage.arrAxis[FDX].axisCMD = C_Aixs_Home;
		break;
		case FINE_DY_HOME:
			Machine.fineStage.arrAxis[FDY].axisCMD = C_Aixs_Home;
		break;
		case FINE_TZ_HOME:
			Machine.fineStage.arrAxis[FTZ].axisCMD = C_Aixs_Home;
		break;

		//6微动台切换
		case FINE_DX_LASERSWITCH:
			Machine.fineStage.arrAxis[FDX].closeCMD = C_Laser_Close;
		break;
		case FINE_DY_LASERSWITCH:
			Machine.fineStage.arrAxis[FDY].closeCMD = C_Laser_Close;
		break;
		case FINE_TZ_LASERSWITCH:
			Machine.fineStage.arrAxis[FTZ].closeCMD = C_Laser_Close;
		break;
		case FINE_DX_EDDYSWITCH:
			Machine.fineStage.arrAxis[FDX].closeCMD = C_Counter_Close;
		break;
		case FINE_DY_EDDYSWITCH:
			Machine.fineStage.arrAxis[FDY].closeCMD = C_Counter_Close;
		break;
		case FINE_TZ_EDDYSWITCH:
			Machine.fineStage.arrAxis[FTZ].closeCMD = C_Counter_Close;
		break;
		//7微动台轨迹
		case FINE_DX_RUN_FORWARD:
			Machine.fineStage.arrAxis[0].cTrig=1;
			Machine.fineStage.arrAxis[0].axisCMD = C_Aixs_RunFor;
//			Machine.coarseStage.arrAxis[0].cTrig =1;
//			Machine.coarseStage.arrAxis[0].axisCMD = C_Aixs_RunFor;
		break;
		case FINE_DX_RUN_REVERSE:
			Machine.fineStage.arrAxis[0].cTrig=1;
			Machine.fineStage.arrAxis[0].axisCMD = C_Aixs_RunRev;
		break;

		case FINE_DY_RUN_FORWARD:
			Machine.fineStage.arrAxis[1].cTrig=1;
			Machine.fineStage.arrAxis[1].axisCMD = C_Aixs_RunFor;
		break;
		case FINE_DY_RUN_REVERSE:
			Machine.fineStage.arrAxis[1].cTrig=1;
			Machine.fineStage.arrAxis[1].axisCMD = C_Aixs_RunRev;
		break;
		case FINE_TZ_RUN_FORWARD:
			Machine.fineStage.arrAxis[2].cTrig=1;
			Machine.fineStage.arrAxis[2].axisCMD = C_Aixs_RunFor;
		break;
		case FINE_TZ_RUN_REVERSE:
			Machine.fineStage.arrAxis[2].cTrig=1;
			Machine.fineStage.arrAxis[2].axisCMD = C_Aixs_RunRev;
		break;


		case CPARSE_DX_COUNTER_CLEAR:	//X1
			ast_M72_Counter_Clear(0,0);
			break;

		case CPARSE_DY_COUNTER_CLEAR:		//X2
			ast_M72_Counter_Clear(0,1);
			break;
		case CPARSE_XTZ_COUNTER_CLEAR:		//Y1
			ast_M72_Counter_Clear(0,2);
			break;
		case CPARSE_YTZ_COUNTER_CLEAR:		//Y2
			ast_M72_Counter_Clear(0,3);
			break;

		case CPARSE_DDX_COUNTER_CLEAR:
			ast_M72_Counter_Clear(1,0);
			break;
		case CPARSE_DDY_COUNTER_CLEAR:
			ast_M72_Counter_Clear(1,1);
			break;
		//1联动闭环与开环
		/*
		case BOTH_DX_CLOSE:
			Machine.fineStage.arrAxis[FDX].axisCMD = C_Aixs_Close;
			Machine.coarseStage.arrAxis[0].axisCMD = C_Aixs_Close;
			Machine.coarseStage.arrAxis[1].axisCMD = C_Aixs_Close;
		break;
		case BOTH_DY_CLOSE:
			Machine.fineStage.arrAxis[FDY].axisCMD = C_Aixs_Close;
			Machine.coarseStage.arrAxis[2].axisCMD = C_Aixs_Close;
			Machine.coarseStage.arrAxis[3].axisCMD = C_Aixs_Close;
		break;
		case BOTH_TZ_CLOSE:
			Machine.fineStage.arrAxis[FTZ].axisCMD = C_Aixs_Close;
		break;

		case BOTH_DX_OPEN:
			Machine.fineStage.arrAxis[FDX].axisCMD = C_Aixs_Open;
			Machine.coarseStage.arrAxis[0].axisCMD = C_Aixs_Open;
			Machine.coarseStage.arrAxis[1].axisCMD = C_Aixs_Open;	
		break;
		case BOTH_DY_OPEN:
			Machine.fineStage.arrAxis[FDY].axisCMD = C_Aixs_Open;
			Machine.coarseStage.arrAxis[2].axisCMD = C_Aixs_Open;
			Machine.coarseStage.arrAxis[3].axisCMD = C_Aixs_Open;
		break;
		case BOTH_TZ_OPEN:
			Machine.fineStage.arrAxis[FTZ].axisCMD = C_Aixs_Open;
		break;

		//2联动STEP
		case BOTH_DX_STEP_FORWARD:
			Machine.fineStage.arrAxis[FDX].axisCMD = C_Aixs_Step_For;
			Machine.coarseStage.arrAxis[0].axisCMD = C_Aixs_Step_For;
			Machine.coarseStage.arrAxis[1].axisCMD = C_Aixs_Step_For;
		break;
		case BOTH_DY_STEP_FORWARD:
			Machine.fineStage.arrAxis[FDY].axisCMD = C_Aixs_Step_For;
			Machine.coarseStage.arrAxis[2].axisCMD = C_Aixs_Step_For;
			Machine.coarseStage.arrAxis[3].axisCMD = C_Aixs_Step_For;
		break;
		case BOTH_TZ_STEP_FORWARD:
			Machine.fineStage.arrAxis[FTZ].axisCMD = C_Aixs_Step_For;
		break;
		case BOTH_DX_STEP_REVERSE:
			Machine.fineStage.arrAxis[FDX].axisCMD = C_Aixs_Step_Rev;
			Machine.coarseStage.arrAxis[0].axisCMD = C_Aixs_Step_Rev;
			Machine.coarseStage.arrAxis[1].axisCMD = C_Aixs_Step_Rev;
		break;
		case BOTH_DY_STEP_REVERSE:
			Machine.fineStage.arrAxis[FDY].axisCMD = C_Aixs_Step_Rev;
			Machine.coarseStage.arrAxis[2].axisCMD = C_Aixs_Step_Rev;
			Machine.coarseStage.arrAxis[3].axisCMD = C_Aixs_Step_Rev;			
		break;
		case BOTH_TZ_STEP_REVERSE:
			Machine.fineStage.arrAxis[FTZ].axisCMD = C_Aixs_Step_Rev;
		break;
		case BOTH_DX_JOG_FORWARD:
			Machine.fineStage.arrAxis[FDX].axisCMD = C_Aixs_Jogfor_Start;
			Machine.coarseStage.arrAxis[0].axisCMD = C_Aixs_Jogfor_Start;
			Machine.coarseStage.arrAxis[1].axisCMD = C_Aixs_Jogfor_Start;
		break;
		//3联动JOG
		case BOTH_DY_JOG_FORWARD:
			Machine.fineStage.arrAxis[FDY].axisCMD = C_Aixs_Jogfor_Start;
			Machine.coarseStage.arrAxis[2].axisCMD = C_Aixs_Jogfor_Start;
			Machine.coarseStage.arrAxis[3].axisCMD = C_Aixs_Jogfor_Start;
		break;
		case BOTH_TZ_JOG_FORWARD:
			Machine.fineStage.arrAxis[FTZ].axisCMD = C_Aixs_Jogfor_Start;
		break;
		case BOTH_DX_JOG_REVERSE:
			Machine.fineStage.arrAxis[FDX].axisCMD = C_Aixs_Jogrev_Start;
			Machine.coarseStage.arrAxis[0].axisCMD = C_Aixs_Jogrev_Start;
			Machine.coarseStage.arrAxis[1].axisCMD = C_Aixs_Jogrev_Start;
		break;
		case BOTH_DY_JOG_REVERSE:
			Machine.fineStage.arrAxis[FDY].axisCMD = C_Aixs_Jogrev_Start;
			Machine.coarseStage.arrAxis[2].axisCMD = C_Aixs_Jogrev_Start;
			Machine.coarseStage.arrAxis[3].axisCMD = C_Aixs_Jogrev_Start;
		break;
		case BOTH_TZ_JOG_REVERSE:
			Machine.fineStage.arrAxis[FTZ].axisCMD = C_Aixs_Jogrev_Start;
		break;
		case BOTH_DX_JOG_STOP:
			Machine.fineStage.arrAxis[FDX].axisCMD = C_Aixs_Jog_Stop;
			Machine.coarseStage.arrAxis[0].axisCMD = C_Aixs_Jog_Stop;
			Machine.coarseStage.arrAxis[1].axisCMD = C_Aixs_Jog_Stop;
			
		break;
		case BOTH_DY_JOG_STOP:
			Machine.fineStage.arrAxis[FDY].axisCMD = C_Aixs_Jog_Stop;
			Machine.coarseStage.arrAxis[2].axisCMD = C_Aixs_Jog_Stop;
			Machine.coarseStage.arrAxis[3].axisCMD = C_Aixs_Jog_Stop;
		break;
		case BOTH_TZ_JOG_STOP:
			Machine.fineStage.arrAxis[FTZ].axisCMD = C_Aixs_Jog_Stop;
		break;
		//4联动辨识
		case BOTH_DX_IDENTIFY:
			Machine.fineStage.arrAxis[FDX].axisCMD = C_Aixs_Ident;
			Machine.coarseStage.arrAxis[0].axisCMD = C_Aixs_Ident;
			Machine.coarseStage.arrAxis[1].axisCMD = C_Aixs_Ident;
		break;
		case BOTH_DY_IDENTIFY:
			Machine.fineStage.arrAxis[FDY].axisCMD = C_Aixs_Ident;
			Machine.coarseStage.arrAxis[2].axisCMD = C_Aixs_Ident;
			Machine.coarseStage.arrAxis[3].axisCMD = C_Aixs_Ident;			
		break;
		case BOTH_TZ_IDENTIFY:
			Machine.fineStage.arrAxis[FTZ].axisCMD = C_Aixs_Ident;
		break;
		//5联动回零
		case BOTH_DX_HOME:
			Machine.fineStage.arrAxis[FDX].axisCMD = C_Aixs_Home;
		break;
		case BOTH_DY_HOME:
			Machine.fineStage.arrAxis[FDY].axisCMD = C_Aixs_Home;
		break;
		case BOTH_TZ_HOME:
			Machine.fineStage.arrAxis[FTZ].axisCMD = C_Aixs_Home;
		break;

		//6联动切换
		case BOTH_DX_LASERSWITCH:
			Machine.fineStage.arrAxis[FDX].closeCMD = C_Laser_Close;
		break;
		case BOTH_DY_LASERSWITCH:
			Machine.fineStage.arrAxis[FDY].closeCMD = C_Laser_Close;
		break;
		case BOTH_TZ_LASERSWITCH:
			Machine.fineStage.arrAxis[FTZ].closeCMD = C_Laser_Close;
		break;
		case BOTH_DX_EDDYSWITCH:
			Machine.fineStage.arrAxis[FDX].closeCMD = C_Counter_Close;
		break;
		case BOTH_DY_EDDYSWITCH:
			Machine.fineStage.arrAxis[FDY].closeCMD = C_Counter_Close;
		break;
		case BOTH_TZ_EDDYSWITCH:
			Machine.fineStage.arrAxis[FTZ].closeCMD = C_Counter_Close;
		break;
		//7联动轨迹
		case BOTH_DX_RUN_FORWARD:
			Machine.fineStage.arrAxis[0].cTrig=1;
			Machine.coarseStage.arrAxis[0].cTrig=1;
			Machine.coarseStage.arrAxis[1].cTrig=1;			
			Machine.fineStage.arrAxis[0].axisCMD = C_Aixs_RunFor;
			Machine.coarseStage.arrAxis[0].axisCMD = C_Aixs_RunFor;
			Machine.coarseStage.arrAxis[1].axisCMD = C_Aixs_RunFor;
		break;
		case BOTH_DX_RUN_REVERSE:
			Machine.fineStage.arrAxis[0].cTrig=1;
			Machine.coarseStage.arrAxis[0].cTrig=1;
			Machine.coarseStage.arrAxis[1].cTrig=1;
			
			Machine.fineStage.arrAxis[0].axisCMD = C_Aixs_RunRev;
			Machine.coarseStage.arrAxis[0].axisCMD = C_Aixs_RunRev;
			Machine.coarseStage.arrAxis[1].axisCMD = C_Aixs_RunRev;
		break;

		case BOTH_DY_RUN_FORWARD:

			Machine.fineStage.arrAxis[1].cTrig=1;
			Machine.coarseStage.arrAxis[2].cTrig=1;
			Machine.coarseStage.arrAxis[3].cTrig=1;	
			
			Machine.fineStage.arrAxis[1].axisCMD = C_Aixs_RunFor;
			Machine.coarseStage.arrAxis[2].axisCMD = C_Aixs_RunFor;
			Machine.coarseStage.arrAxis[3].axisCMD = C_Aixs_RunFor;
		break;
		case BOTH_DY_RUN_REVERSE:
			Machine.fineStage.arrAxis[1].cTrig=1;
			Machine.coarseStage.arrAxis[2].cTrig=1;
			Machine.coarseStage.arrAxis[3].cTrig=1;
			
			Machine.fineStage.arrAxis[1].axisCMD = C_Aixs_RunRev;
			Machine.coarseStage.arrAxis[2].axisCMD = C_Aixs_RunRev;
			Machine.coarseStage.arrAxis[3].axisCMD = C_Aixs_RunRev;			
		break;
		
		case BOTH_TZ_RUN_FORWARD:
			Machine.fineStage.arrAxis[2].axisCMD = C_Aixs_RunFor;
		break;
		case BOTH_TZ_RUN_REVERSE:
			Machine.fineStage.arrAxis[2].axisCMD = C_Aixs_RunRev;
		break;	
		*/
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//YHT将联动改为新增的三轴
		//1联动闭环与开环
				case BOTH_DX_CLOSE:
					
					Machine.fineStage.arrAxis[FDZ].axisCMD = C_Aixs_Close;
				break;
				case BOTH_DY_CLOSE:
					Machine.fineStage.arrAxis[FTX].axisCMD = C_Aixs_Close;
					
				break;
				case BOTH_TZ_CLOSE:
					Machine.fineStage.arrAxis[FTY].axisCMD = C_Aixs_Close;
				break;
		
				case BOTH_DX_OPEN:
					
					Machine.fineStage.arrAxis[FDZ].axisCMD = C_Aixs_Open;
				break;
				case BOTH_DY_OPEN:
					Machine.fineStage.arrAxis[FTX].axisCMD = C_Aixs_Open;
					
				break;
				case BOTH_TZ_OPEN:
					Machine.fineStage.arrAxis[FTY].axisCMD = C_Aixs_Open;
				break;
		
				//2联动STEP
				case BOTH_DX_STEP_FORWARD:
					
					Machine.fineStage.arrAxis[FDZ].axisCMD = C_Aixs_Step_For;
				break;
				case BOTH_DY_STEP_FORWARD:
					Machine.fineStage.arrAxis[FTX].axisCMD = C_Aixs_Step_For;
					
				break;
				case BOTH_TZ_STEP_FORWARD:
					Machine.fineStage.arrAxis[FTY].axisCMD = C_Aixs_Step_For;
				break;
				case BOTH_DX_STEP_REVERSE:
					Machine.fineStage.arrAxis[FDZ].axisCMD = C_Aixs_Step_Rev;
					
				break;
				case BOTH_DY_STEP_REVERSE:
					Machine.fineStage.arrAxis[FTX].axisCMD = C_Aixs_Step_Rev;
								
				break;
				case BOTH_TZ_STEP_REVERSE:
					Machine.fineStage.arrAxis[FTY].axisCMD = C_Aixs_Step_Rev;
				break;
				case BOTH_DX_JOG_FORWARD:
					Machine.fineStage.arrAxis[FDX].axisCMD = C_Aixs_Jogfor_Start;
					Machine.coarseStage.arrAxis[0].axisCMD = C_Aixs_Jogfor_Start;
					Machine.coarseStage.arrAxis[1].axisCMD = C_Aixs_Jogfor_Start;
				break;
				//3联动JOG
				case BOTH_DY_JOG_FORWARD:
					Machine.fineStage.arrAxis[FDY].axisCMD = C_Aixs_Jogfor_Start;
					Machine.coarseStage.arrAxis[2].axisCMD = C_Aixs_Jogfor_Start;
					Machine.coarseStage.arrAxis[3].axisCMD = C_Aixs_Jogfor_Start;
				break;
				case BOTH_TZ_JOG_FORWARD:
					Machine.fineStage.arrAxis[FTY].axisCMD = C_Aixs_Jogfor_Start;
				break;
				case BOTH_DX_JOG_REVERSE:
					Machine.fineStage.arrAxis[FDX].axisCMD = C_Aixs_Jogrev_Start;
					Machine.coarseStage.arrAxis[0].axisCMD = C_Aixs_Jogrev_Start;
					Machine.coarseStage.arrAxis[1].axisCMD = C_Aixs_Jogrev_Start;
				break;
				case BOTH_DY_JOG_REVERSE:
					Machine.fineStage.arrAxis[FDY].axisCMD = C_Aixs_Jogrev_Start;
					Machine.coarseStage.arrAxis[2].axisCMD = C_Aixs_Jogrev_Start;
					Machine.coarseStage.arrAxis[3].axisCMD = C_Aixs_Jogrev_Start;
					
				break;
				case BOTH_TZ_JOG_REVERSE:
					Machine.fineStage.arrAxis[FTY].axisCMD = C_Aixs_Jogrev_Start;
				break;
				case BOTH_DX_JOG_STOP:
					Machine.fineStage.arrAxis[FDX].axisCMD = C_Aixs_Jog_Stop;
					Machine.coarseStage.arrAxis[0].axisCMD = C_Aixs_Jog_Stop;
					Machine.coarseStage.arrAxis[1].axisCMD = C_Aixs_Jog_Stop;
				break;
				case BOTH_DY_JOG_STOP:
					Machine.fineStage.arrAxis[FDY].axisCMD = C_Aixs_Jog_Stop;
					Machine.coarseStage.arrAxis[2].axisCMD = C_Aixs_Jog_Stop;
					Machine.coarseStage.arrAxis[3].axisCMD = C_Aixs_Jog_Stop;
				break;
				case BOTH_TZ_JOG_STOP:
					Machine.fineStage.arrAxis[FTZ].axisCMD = C_Aixs_Jog_Stop;
				break;
				//4联动辨识
				/*case BOTH_DX_IDENTIFY:
					Machine.fineStage.arrAxis[FDZ].axisCMD = C_Aixs_Ident;
					
				break;
				case BOTH_DY_IDENTIFY:
					Machine.fineStage.arrAxis[FTX].axisCMD = C_Aixs_Ident;
							
				break;
				case BOTH_TZ_IDENTIFY:
					Machine.fineStage.arrAxis[FTY].axisCMD = C_Aixs_Ident;
				break;*/
				//5联动回零
				case BOTH_DX_HOME:
					Machine.fineStage.arrAxis[FDZ].axisCMD = C_Aixs_Home;
				break;
				case BOTH_DY_HOME:
					Machine.fineStage.arrAxis[FTX].axisCMD = C_Aixs_Home;
				break;
				case BOTH_TZ_HOME:
					Machine.fineStage.arrAxis[FTY].axisCMD = C_Aixs_Home;
				break;
		
				//6联动切换
				case BOTH_DX_LASERSWITCH:
					Machine.fineStage.arrAxis[FDZ].closeCMD = C_Laser_Close;
				break;
				case BOTH_DY_LASERSWITCH:
					Machine.fineStage.arrAxis[FTX].closeCMD = C_Laser_Close;
				break;
				case BOTH_TZ_LASERSWITCH:
					Machine.fineStage.arrAxis[FTY].closeCMD = C_Laser_Close;
				break;
				case BOTH_DX_EDDYSWITCH:
					Machine.fineStage.arrAxis[FDZ].closeCMD = C_Counter_Close;
				break;
				case BOTH_DY_EDDYSWITCH:
					Machine.fineStage.arrAxis[FTX].closeCMD = C_Counter_Close;
				break;
				case BOTH_TZ_EDDYSWITCH:
					Machine.fineStage.arrAxis[FTY].closeCMD = C_Counter_Close;
				break;
				//7联动轨迹
				case BOTH_DX_RUN_FORWARD:
	//				Machine.fineStage.arrAxis[FDZ].cTrig=1;
	//		        Machine.fineStage.arrAxis[FDZ].axisCMD = C_Aixs_RunFor;
					Machine.fineStage.arrAxis[0].cTrig=1;
					Machine.coarseStage.arrAxis[0].cTrig=1;
					Machine.coarseStage.arrAxis[1].cTrig=1;			
					Machine.fineStage.arrAxis[0].axisCMD = C_Aixs_RunFor;
					Machine.coarseStage.arrAxis[0].axisCMD = C_Aixs_RunFor;
					Machine.coarseStage.arrAxis[1].axisCMD = C_Aixs_RunFor;
					
				break;
				case BOTH_DX_RUN_REVERSE:
					Machine.fineStage.arrAxis[0].cTrig=1;
					Machine.coarseStage.arrAxis[0].cTrig=1;
					Machine.coarseStage.arrAxis[1].cTrig=1;
					
					Machine.fineStage.arrAxis[0].axisCMD = C_Aixs_RunRev;
					Machine.coarseStage.arrAxis[0].axisCMD = C_Aixs_RunRev;
					Machine.coarseStage.arrAxis[1].axisCMD = C_Aixs_RunRev;				
				break;		
				case BOTH_DY_RUN_FORWARD:
					Machine.fineStage.arrAxis[1].cTrig=1;
					Machine.coarseStage.arrAxis[2].cTrig=1;
					Machine.coarseStage.arrAxis[3].cTrig=1;	
					
					Machine.fineStage.arrAxis[1].axisCMD = C_Aixs_RunFor;
					Machine.coarseStage.arrAxis[2].axisCMD = C_Aixs_RunFor;
					Machine.coarseStage.arrAxis[3].axisCMD = C_Aixs_RunFor;				
				break;
				case BOTH_DY_RUN_REVERSE:
					Machine.fineStage.arrAxis[1].cTrig=1;
					Machine.coarseStage.arrAxis[2].cTrig=1;
					Machine.coarseStage.arrAxis[3].cTrig=1;
					
					Machine.fineStage.arrAxis[1].axisCMD = C_Aixs_RunRev;
					Machine.coarseStage.arrAxis[2].axisCMD = C_Aixs_RunRev;
					Machine.coarseStage.arrAxis[3].axisCMD = C_Aixs_RunRev;							
				break;
				case BOTH_TZ_RUN_FORWARD:
					Machine.fineStage.arrAxis[FTY].cTrig=1;
					Machine.fineStage.arrAxis[FTY].axisCMD = C_Aixs_RunFor;
				break;
				case BOTH_TZ_RUN_REVERSE:
					Machine.fineStage.arrAxis[FTY].cTrig=1;
					Machine.fineStage.arrAxis[FTY].axisCMD = C_Aixs_RunRev;
				break;	
		case ALL_OPEN:
			Machine.coarseStage.arrAxis[CDX].axisCMD = C_Aixs_Open;
			Machine.coarseStage.arrAxis[CDY].axisCMD = C_Aixs_Open;
			Machine.coarseStage.arrAxis[XTZ].axisCMD = C_Aixs_Open;
			Machine.coarseStage.arrAxis[YTZ].axisCMD = C_Aixs_Open;
			Machine.fineStage.arrAxis[FDX].axisCMD = C_Aixs_Open;
			Machine.fineStage.arrAxis[FDY].axisCMD = C_Aixs_Open;
			Machine.fineStage.arrAxis[FTZ].axisCMD = C_Aixs_Open;
			Machine.fineStage.arrAxis[FDZ].axisCMD = C_Aixs_Open;
			Machine.fineStage.arrAxis[FTX].axisCMD = C_Aixs_Open;
			Machine.fineStage.arrAxis[FTY].axisCMD = C_Aixs_Open;
			// Machine.stepHome =  0;
		Machine.coarseStage.arrAxis[2].stepHome =  0;
	  	Machine.coarseStage.arrAxis[0].stepHome =  0;

		Machine.coarseStage.arrAxis[0].HomeFlagCorse =	0;
		Machine.coarseStage.arrAxis[2].HomeFlagCorse =	0;
		break;	
		case STAGE_EXPO:				
			GetRunData(&Machine.coarseStage.arrAxis[0].sTP_Run,XExpoParam);
			GetRunData(&Machine.coarseStage.arrAxis[1].sTP_Run,XExpoParam);
			
			GetRunData(&Machine.coarseStage.arrAxis[2].sTP_Run,YExpoParam);
			GetRunData(&Machine.coarseStage.arrAxis[3].sTP_Run,YExpoParam);
			
			GetRunData(&Machine.fineStage.arrAxis[0].sTP_Run,XExpoParam);
			GetRunData(&Machine.fineStage.arrAxis[1].sTP_Run,YExpoParam);
			
			Machine.coarseStage.stageCMD = C_Stage_Traj_Expo;
			Machine.fineStage.stageCMD = C_Stage_Traj_Expo;
			
		break;

		
		case COARSE_DDX_LASERSWITCH:
			ReadLineParam(lineInterpolate);
			mc_line(lineInterpolate);
			if(Machine.coarseStage.stageStatus==S_Stage_ALL_Close)
				Machine.coarseStage.stageCMD=C_Stage_Line_Interpolate;
			break;
		case COARSE_DDY_LASERSWITCH:
			ReadArcParam(arcInterpolate);
			mc_arc(arcInterpolate);
			if(Machine.coarseStage.stageStatus==S_Stage_ALL_Close)
				Machine.coarseStage.stageCMD=C_Stage_Arc_Interpolate;
			break;

			
		case STAGE_P2P_TRAJ:



			for(i=0;i<10;i++)
			{
				XTrajParam[i]= P2PData[i+1];
				YTrajParam[i]= P2PData[i+1];
			}
			GetRunData(&Machine.coarseStage.arrAxis[0].sTP_Run,XTrajParam);
			GetRunData(&Machine.coarseStage.arrAxis[1].sTP_Run,XTrajParam);
			GetRunData(&Machine.coarseStage.arrAxis[2].sTP_Run,YTrajParam);
			GetRunData(&Machine.coarseStage.arrAxis[3].sTP_Run,YTrajParam);
			GetRunData(&Machine.fineStage.arrAxis[0].sTP_Run,XTrajParam);
			GetRunData(&Machine.fineStage.arrAxis[1].sTP_Run,YTrajParam);
		break;
		case P2PSet:

			for(i=0;i<10;i++)
			{
				XTrajParam[i]=P2PData[i+1];
				YTrajParam[i]=P2PData[i+1];
			}
			GetRunData(&Machine.coarseStage.arrAxis[0].sTP_Run,XTrajParam);
			GetRunData(&Machine.coarseStage.arrAxis[1].sTP_Run,XTrajParam);
			GetRunData(&Machine.coarseStage.arrAxis[2].sTP_Run,YTrajParam);
			GetRunData(&Machine.coarseStage.arrAxis[3].sTP_Run,YTrajParam);
			GetRunData(&Machine.fineStage.arrAxis[0].sTP_Run,XTrajParam);
			GetRunData(&Machine.fineStage.arrAxis[1].sTP_Run,YTrajParam);
			/*GetRunData(&Machine.fineStage.arrAxis[2].sTP_Run,XTrajParam);
			   GetRunData(&Machine.fineStage.arrAxis[3].sTP_Run,YTrajParam);
			   GetRunData(&Machine.fineStage.arrAxis[4].sTP_Run,XTrajParam);
			   GetRunData(&Machine.fineStage.arrAxis[5].sTP_Run,YTrajParam);*/
		
				break;
		case EXPOSet:
			if(Machine.coarseStage.stageStatus == S_Stage_Traj_Expo)
			{
				Machine.coarseStage.stageStatus = S_Stage_ALL_Close;
				GetRunData(&Machine.coarseStage.arrAxis[0].sTP_Run,XExpoParam);
				GetRunData(&Machine.coarseStage.arrAxis[1].sTP_Run,XExpoParam);
				GetRunData(&Machine.coarseStage.arrAxis[2].sTP_Run,YExpoParam);
				GetRunData(&Machine.coarseStage.arrAxis[3].sTP_Run,YExpoParam);
				DXPosLast_Coarse = FirstIn;
				DXNegLast_Coarse = FirstIn;
				DYPosLast_Coarse = FirstIn;
				DYNegLast_Coarse = FirstIn;
				Machine.coarseStage.stageCMD = C_Nothing;
				Coarse_Count = 0;
			}
			else if(Machine.coarseStage.stageStatus == S_Stage_ALL_Close)
			{
				Machine.coarseStage.stageCMD = C_Stage_Traj_Expo;
			}
							     
			/*	ReadExpoTrajFile();
		
				ExpoPlanGen((int)(ExpoData[0]));	
				for(i=0;i<10;i++)
				{
					XExpoParam[i]=ExpoData[i+1];
					YExpoParam[i]=ExpoData[i+1+10];
				}
				//recordStruct.iReadFlag = 1;//处理时记得将此值置零
				
				GetRunData(&Machine.coarseStage.arrAxis[0].sTP_Run,XExpoParam);
				GetRunData(&Machine.coarseStage.arrAxis[1].sTP_Run,XExpoParam);
				
				GetRunData(&Machine.coarseStage.arrAxis[2].sTP_Run,YExpoParam);
				GetRunData(&Machine.coarseStage.arrAxis[3].sTP_Run,YExpoParam);
				
				GetRunData(&Machine.fineStage.arrAxis[0].sTP_Run,XExpoParam);
				GetRunData(&Machine.fineStage.arrAxis[1].sTP_Run,YExpoParam);   */

/*
				 TPGen_4Order_P2P(&TP, &p_dd, &Td, &Tj,&Ta, &Tv, CAL_MIN_J_AND_D);
		
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
			TPGen_4Order_P2P(&TPY, &p_ddY, &TdY, &TjY,&TaY, &TvY, CAL_MIN_J_AND_D);
			
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

		GetRunData(&Machine.coarseStage.arrAxis[0].sTP_Run,XExpoParam);*/
				
			break;	
		case STOPCOLLECT:
			
			break;
		case SET_PARAMETER:
	//		SetParam();
//			commFun[recvData.command].fun(&rxData);
		break;		
		case GET_PARAMETER:
			comData.iCMD=5001;//让发送buff填充特殊帧			
		break;
		case READ_FILE:
			ReadIdentFile();
			TP.p = ParamData[0];
			TP.v = ParamData[1];
			TP.a = ParamData[2];
			TP.j = ParamData[3];
			TP.d = ParamData[4];
			TP.Ts = ParamData[5];
			TP.r = ParamData[6];
			TP.s = ParamData[7];
//			printf("\n");
//			for(i=0;i<8;i++)
//			{
//				printf("%lf\n",ParamData[i]);
//			}
			
			TPY.p = ParamData[8];
			TPY.v = ParamData[9];
			TPY.a = ParamData[10];
			TPY.j = ParamData[11];
			TPY.d = ParamData[12];
			TPY.Ts = ParamData[13];
			TPY.r = ParamData[14];
			TPY.s = ParamData[15];
/*			printf("\n");
			for(i=8;i<16;i++)
			{
				printf("%lf\n",ParamData[i]);
			}
*/
			TPRev.p = ParamData[8] * (ParamData[16]-1);
			TPRev.v = ParamData[9];
			TPRev.a = ParamData[10];
			TPRev.j = ParamData[11];
			TPRev.d = ParamData[12];
			TPRev.Ts = ParamData[13];
			TPRev.r = ParamData[14];
			TPRev.s = ParamData[15];
//			TPGen_4Order_P2P(&TP, &p_dd, &Td, &Tj,&Ta, &Tv, CAL_MIN_J_AND_D);
//			TPGen_4Order_P2P(&TPY, &p_ddY, &TdY, &TjY,&TaY, &TvY, CAL_MIN_J_AND_D);
//			TPGen_4Order_P2P(&TPRev, &p_ddRev, &TdRev, &TjRev,&TaRev, &TvRev, CAL_MIN_J_AND_D);
			memset(T_Coarse,0,sizeof(T_Coarse));
			ExpoPlanGen((int)ParamData[16]);
			memcpy(T_Fine,T_Coarse,sizeof(T_Coarse));
/*			for(i=0;i<(int)ParamData[16]*4;i++)
			{
				printf("%c",T_Fine[i]);
				if((i!=0) && ((i+1)%8==0))
				{
					printf("\n");
				}
			}
*/
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
/*			printf("\n");
			for(i=0;i<10;i++)
			{
				printf("%lf\n",XExpoParam[i]);
			}
	*/
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
/*			printf("\n");
			for(i=0;i<10;i++)
			{
				printf("%lf\n",YExpoParam[i]);
			}
*/
			YExpoRevParam[0]= TPRev.Ts;
			YExpoRevParam[1]= p_ddRev;
			YExpoRevParam[2]= TdRev;
			YExpoRevParam[3]= TjRev;
			YExpoRevParam[4]= TaRev;
			YExpoRevParam[5]= TvRev;
			YExpoRevParam[6]= 0;
			YExpoRevParam[7]= 0;
			YExpoRevParam[8]= 0;
			YExpoRevParam[9]= 0;
			GetRunData(&Machine.coarseStage.arrAxis[0].sTP_Run,XExpoParam);
			GetRunData(&Machine.coarseStage.arrAxis[1].sTP_Run,XExpoParam);
			GetRunData(&Machine.coarseStage.arrAxis[2].sTP_Run,YExpoParam);
			GetRunData(&Machine.coarseStage.arrAxis[3].sTP_Run,YExpoParam);

			GetRunData(&Machine.fineStage.arrAxis[0].sTP_Run,XExpoParam);
			GetRunData(&Machine.fineStage.arrAxis[1].sTP_Run,YExpoParam); 

		break;

		case RECORD_DATA:
			recordStruct.iRecordFlag = 1;
		break;

		case SAVE_FILE:
			recordStruct.iSaveFlag = 1;
			semGive(DataCollect_SemId);
		break;


		/*case READ_EXPO_FILE:
			ReadExpoTrajFile();				
			for(i=0;i<10;i++)
			{
				XExpoParam[i]=ExpoData[i+1];
				YExpoParam[i]=ExpoData[i+1+10];
			}
			//recordStruct.iReadFlag = 1;//处理时记得将此值置零
		break;*/


		case RESET_LASER:
//			BoardReset(1);
//			BoardReset(2);
			N1225A_Init();
			//resetzmi();
/*			if(Machine.coarseStage.stageStatus ==S_Stage_ALL_Close)
			{
				ReadCircleParam();
				circleParam.R*=1000000;
				Machine.coarseStage.stageCMD = C_Stage_Circle_Traj;
				//printf("%d\n%d\n",circleParam.R,circleParam.cirNum);	
			}
			else if(Machine.coarseStage.stageStatus == S_Stage_Circle_Traj)
			{
				Machine.coarseStage.stageCMD = C_Stage_ALL_Close;
			}*/

			
		break;

		case STAGE_LOAD_POS://急停
			
		break;
		case JOG_SPEED://ABSOLUTE_MOVE
			Machine.coarseStage.arrAxis[0].dJogSpd = rxData.dParamData[0];//rxData.jogSpeed[rxData.axis]; // recvData.dParamData[0];
			Machine.coarseStage.arrAxis[1].dJogSpd = rxData.dParamData[0]; // rxData.jogSpeed[rxData.axis];// recvData.dParamData[0];
			Machine.coarseStage.arrAxis[2].dJogSpd = rxData.dParamData[1];//rxData.jogSpeed[rxData.axis];	//recvData.dParamData[1];
			break;
		case ABSOLUTE_MOVE:
			for(i=0;i<2;i++)
			{
				DirParam[i]=DirData[i];
			}

			//ExpoPlanGen((int)(ExpoData[0]));
			for(i=0;i<10;i++)
			{
				XExpoParam[i]=ExpoData[i+1];
				YExpoParam[i]=ExpoData[i+1+10];
			}
		
			GetRunData(&Machine.coarseStage.arrAxis[0].sTP_Run,XExpoParam);
			GetRunData(&Machine.coarseStage.arrAxis[1].sTP_Run,XExpoParam);
			GetRunData(&Machine.coarseStage.arrAxis[2].sTP_Run,YExpoParam);
			GetRunData(&Machine.coarseStage.arrAxis[3].sTP_Run,YExpoParam);
			GetRunData(&Machine.fineStage.arrAxis[0].sTP_Run,XExpoParam);
			GetRunData(&Machine.fineStage.arrAxis[1].sTP_Run,YExpoParam);	
			break;
//		case ABSOLUTE_XMOVE:
			//if(DirParam[0] > 0)
/*			if(DirParam[2] == 1)
			{
	            	Machine.coarseStage.arrAxis[0].cTrig=1;
					Machine.coarseStage.arrAxis[1].cTrig=1;			
					Machine.coarseStage.arrAxis[0].axisCMD = C_Aixs_RunFor;
					Machine.coarseStage.arrAxis[1].axisCMD = C_Aixs_RunFor;
			}
			if(DirParam[2] == 0)
			{
	            	Machine.coarseStage.arrAxis[0].cTrig=1;
					Machine.coarseStage.arrAxis[1].cTrig=1;			
					Machine.coarseStage.arrAxis[0].axisCMD = C_Aixs_RunRev;
					Machine.coarseStage.arrAxis[1].axisCMD = C_Aixs_RunRev;
			}*/
		//	AbsolutMove(&TP,&Machine.coarseStage.arrAxis[0]);
	//	break;
	//	case ABSOLUTE_YMOVE:
/*			if(DirParam[3] == 1)
			{
				Machine.coarseStage.arrAxis[2].cTrig=1;
			    Machine.coarseStage.arrAxis[2].axisCMD = C_Aixs_RunFor;	
			}
			if(DirParam[3] == 0)
			{
				Machine.coarseStage.arrAxis[2].cTrig=1;
			    Machine.coarseStage.arrAxis[2].axisCMD = C_Aixs_RunRev;
			}*/
		//	AbsolutMove(&TPY,&Machine.coarseStage.arrAxis[2]);
	//		break;
		case Coarse_Both_MOVE:
			Machine.coarseStage.stageStatus = 6551;
			/*if(DirParam[0] > 0)
			{
              Machine.coarseStage.arrAxis[0].cTrig=1;
			Machine.coarseStage.arrAxis[1].cTrig=1;			
			Machine.coarseStage.arrAxis[0].axisCMD = C_Aixs_RunFor;
			Machine.coarseStage.arrAxis[1].axisCMD = C_Aixs_RunFor;
			}

			if(DirParam[0] < 0)
			{
              Machine.coarseStage.arrAxis[0].cTrig=1;
			Machine.coarseStage.arrAxis[1].cTrig=1;			
			Machine.coarseStage.arrAxis[0].axisCMD = C_Aixs_RunRev;
			Machine.coarseStage.arrAxis[1].axisCMD = C_Aixs_RunRev;
			}
		    if(DirParam[1] > 0)
			{
				 Machine.coarseStage.arrAxis[2].cTrig=1;
				 Machine.coarseStage.arrAxis[2].axisCMD = C_Aixs_RunFor;	
			}
				
			if(DirParam[1] < 0)
			{
				 Machine.coarseStage.arrAxis[2].cTrig=1;
				Machine.coarseStage.arrAxis[2].axisCMD = C_Aixs_RunRev;
			}*/
		break;
		
		case HOME:
			if(rxData.axis<4)
			{
				Machine.coarseStage.arrAxis[rxData.axis].bHomeDone=0.0;
				Machine.coarseStage.arrAxis[rxData.axis+1].axisCMD= C_Aixs_Open;

				Machine.coarseStage.arrAxis[rxData.axis].HomeFlagCorse =	1.0;
				Machine.coarseStage.arrAxis[rxData.axis].axisCMD = C_Aixs_Home;
				Machine.coarseStage.arrAxis[rxData.axis].stepHome=1;
			}
			
		break;
		case SET_POS:	
			if(rxData.axis<4)
			{
				Machine.coarseStage.arrAxis[rxData.axis].targetPos = rxData.dParamData[0];
			}
			else if(rxData.axis>=4)
			{
				Machine.fineStage.arrAxis[rxData.axis-4].targetPos = rxData.dParamData[0];
			}
			Machine.coarseStage.arrAxis[rxData.axis].trajUpdate=1;
	//		printf("pos:%lf",rxData.dParamData[0]);
			break;
		case SET_Vel:
			if(rxData.axis<4)
			{
				Machine.coarseStage.arrAxis[rxData.axis].targetVel = rxData.dParamData[0];
			}
			else if(rxData.axis>=4)
			{
				Machine.fineStage.arrAxis[rxData.axis-4].targetVel= rxData.dParamData[0];
			}
			Machine.coarseStage.arrAxis[rxData.axis].trajUpdate=1;
	//		printf("vel:%lf",rxData.dParamData[0]);
			break;
		case SET_ACC:
			if(rxData.axis<4)
			{
				Machine.coarseStage.arrAxis[rxData.axis].targetAcc= rxData.dParamData[0];
			}
			else if(rxData.axis>=4)
			{
				Machine.fineStage.arrAxis[rxData.axis-4].targetAcc= rxData.dParamData[0];
			}
			Machine.coarseStage.arrAxis[rxData.axis].trajUpdate=1;
//			printf("Acc:%lf",rxData.dParamData[0]);
		break;
		case SET_JERK:	
			if(rxData.axis<4)
			{
				Machine.coarseStage.arrAxis[rxData.axis].targetJerk= rxData.dParamData[0];
			}
			else if(rxData.axis>=4)
			{
				Machine.fineStage.arrAxis[rxData.axis-4].targetJerk= rxData.dParamData[0];
			}
			Machine.coarseStage.arrAxis[rxData.axis].trajUpdate=1;
	//		printf("Jerk:%lf",rxData.dParamData[0]);
		break;
		case SET_FPOS:
			if(rxData.axis<4)
			{
				Machine.coarseStage.arrAxis[rxData.axis].posOffset = rxData.dParamData[0] - Machine.coarseStage.arrAxis[rxData.axis].dSolveMethodOne;
			}
			else if(rxData.axis>=4)
			{
				Machine.fineStage.arrAxis[rxData.axis].posOffset = rxData.dParamData[0] - Machine.fineStage.arrAxis[rxData.axis].dActPos;
			}
	//		printf("posOffset:%lf",Machine.coarseStage.arrAxis[rxData.axis].posOffset);
		break;
		case CLOSE:
			if(rxData.axis<4)
			{
				Machine.coarseStage.arrAxis[rxData.axis].axisCMD = C_Aixs_Close;
			}
			else if(rxData.axis>=4)
			{
				Machine.fineStage.arrAxis[rxData.axis-4].axisCMD = C_Aixs_Close;
			}
		break;
		case OPEN:
			if(rxData.axis<4)
			{
				Machine.coarseStage.arrAxis[rxData.axis].axisCMD = C_Aixs_Open;
			}
			else if(rxData.axis>=4)
			{
				Machine.fineStage.arrAxis[rxData.axis-4].axisCMD = C_Aixs_Open;
			}
		break;
		case MOVE_ABS:
			AbsolutMove(rxData.dParamData[0],&Machine.coarseStage.arrAxis[rxData.axis]);
		break;
		case MOVE_REL:
			if(rxData.axis<4)
			{
				Machine.coarseStage.arrAxis[rxData.axis].targetPos = fabs(rxData.dParamData[0]);
			}
			else if(rxData.axis>=4)
			{
				Machine.fineStage.arrAxis[rxData.axis-4].targetPos = fabs(rxData.dParamData[0]);
			}
			Machine.coarseStage.arrAxis[rxData.axis].trajUpdate=1;
			if(rxData.dParamData[0]>0)
			{
				if(rxData.axis<4)
				{
					Machine.coarseStage.arrAxis[rxData.axis].cTrig=1;
					Machine.coarseStage.arrAxis[rxData.axis].axisCMD = C_Aixs_RunFor;	
				}	
				else if(rxData.axis>=4)
				{
					Machine.fineStage.arrAxis[rxData.axis-4].cTrig=1;
					Machine.fineStage.arrAxis[rxData.axis-4].axisCMD = C_Aixs_RunFor;	
				}
			}
			else if(rxData.dParamData[0]<0)
			{
				if(rxData.axis<4)
				{
					Machine.coarseStage.arrAxis[rxData.axis].cTrig=1;
					Machine.coarseStage.arrAxis[rxData.axis].axisCMD = C_Aixs_RunRev;	
				}	
				else if(rxData.axis>=4)
				{
					Machine.fineStage.arrAxis[rxData.axis-4].cTrig=1;
					Machine.fineStage.arrAxis[rxData.axis-4].axisCMD = C_Aixs_RunRev;	
				}
			}
		break;
		case JOG:
			if(rxData.dParamData[0]>0)
			{
				if(rxData.axis<4)
				{
					Machine.coarseStage.arrAxis[rxData.axis].axisCMD = C_Aixs_Jogfor_Start;	
				}	
				else if(rxData.axis>=4)
				{
					Machine.fineStage.arrAxis[rxData.axis-4].cTrig=1;
					Machine.fineStage.arrAxis[rxData.axis-4].axisCMD = C_Aixs_Jogfor_Start;	
				}
			}
			else if(rxData.dParamData[0]<0)
			{
				if(rxData.axis<4)
				{
					Machine.coarseStage.arrAxis[rxData.axis].cTrig=1;
					Machine.coarseStage.arrAxis[rxData.axis].axisCMD = C_Aixs_Jogrev_Start;	
				}	
				else if(rxData.axis>=4)
				{
					Machine.fineStage.arrAxis[rxData.axis-4].cTrig=1;
					Machine.fineStage.arrAxis[rxData.axis-4].axisCMD = C_Aixs_Jogrev_Start;	
				}
			}
		break;
		case JOG_STOP:
			if(rxData.axis<4)
			{
				Machine.coarseStage.arrAxis[rxData.axis].axisCMD = C_Aixs_Jog_Stop;
			}
			else if(rxData.axis>=4)
			{
				Machine.fineStage.arrAxis[rxData.axis-4].axisCMD = C_Aixs_Jog_Stop;
			}
		break;
		case STOP:
			for(i=0;i<4;i++)
			{
				Machine.coarseStage.arrAxis[rxData.axis].axisCMD = C_Aixs_Open;
			}
			for(i=0;i<6;i++)
			{
				Machine.fineStage.arrAxis[rxData.axis-4].axisCMD = C_Aixs_Open;
			}
		break;
		case SET_KP:
			for(i=0;i<4;i++)
			{
				Machine.coarseStage.arrAxis[rxData.axis].sPID.Proportion = rxData.dParamData[0];
			}
			for(i=0;i<6;i++)
			{
				Machine.fineStage.arrAxis[rxData.axis-4].sPID.Proportion = rxData.dParamData[0];
			}
			break;
		case SET_KI:
			for(i=0;i<4;i++)
			{
				Machine.coarseStage.arrAxis[rxData.axis].sPID.Integral= rxData.dParamData[0];
			}
			for(i=0;i<6;i++)
			{
				Machine.fineStage.arrAxis[rxData.axis-4].sPID.Integral= rxData.dParamData[0];
			}
			break;
		case SET_KD:
			for(i=0;i<4;i++)
			{
				Machine.coarseStage.arrAxis[rxData.axis].sPID.Derivative= rxData.dParamData[0];
			}
			for(i=0;i<6;i++)
			{
				Machine.fineStage.arrAxis[rxData.axis-4].sPID.Derivative= rxData.dParamData[0];
			}
			break;
		default:
		break;
	}
	*pCMD=0;
}
