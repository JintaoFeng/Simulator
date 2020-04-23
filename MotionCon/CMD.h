#ifndef CMD_H
#define CMD_H

#include "StateMachine.h"
#include "ComWithWin.h"

#define	TRUE		1
#define FALSE		0

#define SET_PARAMETER				5000
#define GET_PARAMETER				5001
#define STAGE_EXPO					5002

#define STAGE_P2P_TRAJ				5003
#define JOG_SPEED					5004
#define ABSOLUTE_MOVE				5007
#define ABSOLUTE_XMOVE				5008
#define ABSOLUTE_YMOVE				5009

#define EXPOSet                     29
#define STOPCOLLECT					30
#define P2PSet                     30029

#define STAGE_LOAD_POS				99990
#define Coarse_Both_MOVE			99992
#define ALL_CLOSE            		99994 

#define READ_FILE					26
#define RECORD_DATA                 27
#define SAVE_FILE					28
#define READ_EXPO_FILE				29

#define RESET_LASER					32

#define RECORD_SELECT				50
#define CLEAR						51

/////////////  Stage Control Command  /////////////////////////////////////
#define ALL_OPEN                        10000  // 粗动台微动台全开环

#define COARSE_ALL_OPEN             	20001  // 粗动台全开环
#define COARSE_ALL_CLOSE            	20002  // 粗动台全闭环
#define COARSE_ALL_HOME					20008  // 粗动台全回零


#define FINE_ALL_OPEN             		10001  // 微动台全开环
#define FINE_ALL_CLOSE            		10002  // 微动台全闭环
#define FINE_ALL_HOME					10008  // 微动台全回零
#define FINE_LASER_SWITCH				10009  // 微动台全切换

// Coarse DX
#define COARSE_DX_OPEN				20101  // 粗DX开环
#define COARSE_DX_CLOSE				20102  // 粗DX闭环
#define COARSE_DX_STEP_FORWARD		20103  // 粗DXstep+
#define COARSE_DX_STEP_REVERSE      20104  // 粗DXstep-		
#define COARSE_DX_JOG_FORWARD    	20105  // 粗DXJog+
#define COARSE_DX_JOG_REVERSE    	20106  // 粗DXJog-
#define COARSE_DX_JOG_STOP     		20107  // 粗DXJog+停
#define COARSE_DX_HOME				20108  // 粗DX回零
#define COARSE_DX_LASERSWITCH		20109  // 粗动台DX切内部
#define COARSE_DX_EDDYSWITCH		20110  // 粗动台DX切回电涡流
#define COARSE_DX_IDENTIFY		    20111  // 粗DX辨识
#define COARSE_DX_RUN_FORWARD		20112	// 粗DX走轨迹+
#define COARSE_DX_RUN_REVERSE		20113  // 粗DX走轨迹-
#define CPARSE_DX_COUNTER_CLEAR		20114	//粗DX计数清零
// Coarse DY
#define COARSE_DY_OPEN				20201  // 粗DY开环
#define COARSE_DY_CLOSE				20202  // 粗DY闭环
#define COARSE_DY_STEP_FORWARD		20203  // 粗DYstep+
#define COARSE_DY_STEP_REVERSE      20204  // 粗DYstep-		
#define COARSE_DY_JOG_FORWARD    	20205  // 粗DYJog+
#define COARSE_DY_JOG_REVERSE    	20206  // 粗DYJog-
#define COARSE_DY_JOG_STOP			20207  // 粗DYJog+停
#define COARSE_DY_HOME				20208  // 粗DY回零
#define COARSE_DY_LASERSWITCH		20209  // 粗动台DY切激光
#define COARSE_DY_EDDYSWITCH		20210  // 粗动台DY切回电涡流
#define COARSE_DY_IDENTIFY		    20211  // 粗DY辨识
#define COARSE_DY_RUN_FORWARD		20212	// 粗DY走轨迹+
#define COARSE_DY_RUN_REVERSE		20213  // 粗DY走轨迹-
#define CPARSE_DY_COUNTER_CLEAR		20214	//粗DY计数清零
// Coarse XTZ
#define COARSE_XTZ_OPEN				20301  // 粗XTZ开环
#define COARSE_XTZ_CLOSE			20302  // 粗XTZ闭环
#define COARSE_XTZ_STEP_FORWARD		20303  // 粗XTZstep+
#define COARSE_XTZ_STEP_REVERSE     20304  // 粗XTZstep-		
#define COARSE_XTZ_JOG_FORWARD    	20305  // 粗XTZJog+
#define COARSE_XTZ_JOG_REVERSE    	20306  // 粗XTZJog-
#define COARSE_XTZ_JOG_STOP	     	20307  // 粗XTZJog+停
#define COARSE_XTZ_HOME				20308  // 粗XTZ回零
#define COARSE_XTZ_LASERSWITCH		20309  // 粗动台XTZ切激光
#define COARSE_XTZ_EDDYSWITCH		20310  // 粗动台XTZ切回电涡流
#define COARSE_XTZ_IDENTIFY		    20311  // 粗XTZ辨识
#define COARSE_XTZ_RUN_FORWARD		20312	// 粗XTZ走轨迹+
#define COARSE_XTZ_RUN_REVERSE		20313  // 粗XTZ走轨迹-
#define CPARSE_XTZ_COUNTER_CLEAR	20314	//粗XTZ计数清零

// Coarse YTZ
#define COARSE_YTZ_OPEN				20401  // 粗YTZ开环
#define COARSE_YTZ_CLOSE			20402  // 粗YTZ闭环
#define COARSE_YTZ_STEP_FORWARD		20403  // 粗YTZstep+
#define COARSE_YTZ_STEP_REVERSE     20404  // 粗YTZstep-		
#define COARSE_YTZ_JOG_FORWARD    	20405  // 粗YTZJog+
#define COARSE_YTZ_JOG_REVERSE    	20406  // 粗YTZJog-
#define COARSE_YTZ_JOG_STOP     	20407  // 粗YTZJog+停
#define COARSE_YTZ_HOME				20408  // 粗YTZ回零
#define COARSE_YTZ_LASERSWITCH		20409  // 粗动台YTZ切激光
#define COARSE_YTZ_EDDYSWITCH		20410  // 粗动台YTZ切回电涡流
#define COARSE_YTZ_IDENTIFY		    20411  // 粗YTZ辨识
#define COARSE_YTZ_RUN_FORWARD		20412	// 粗YTZ走轨迹+
#define COARSE_YTZ_RUN_REVERSE		20413  // 粗YTZ走轨迹-
#define CPARSE_YTZ_COUNTER_CLEAR	20414	//粗YTZ计数清零
//
#define COARSE_DDX_OPEN				20501  // 粗YTZ开环
#define COARSE_DDX_CLOSE			20502  // 粗YTZ闭环
#define COARSE_DDX_STEP_FORWARD		20503  // 粗YTZstep+
#define COARSE_DDX_STEP_REVERSE     20504  // 粗YTZstep-		
#define COARSE_DDX_JOG_FORWARD    	20505  // 粗YTZJog+
#define COARSE_DDX_JOG_REVERSE    	20506  // 粗YTZJog-
#define COARSE_DDX_JOG_STOP     	20507  // 粗YTZJog+停
#define COARSE_DDX_HOME				20508  // 粗YTZ回零
#define COARSE_DDX_LASERSWITCH		20509  // 粗动台YTZ切激光
#define COARSE_DDX_EDDYSWITCH		20510  // 粗动台YTZ切回电涡流
#define COARSE_DDX_IDENTIFY		    20511  // 粗YTZ辨识
#define COARSE_DDX_RUN_FORWARD		20512	// 粗YTZ走轨迹+
#define COARSE_DDX_RUN_REVERSE		20513  // 粗YTZ走轨迹-
#define CPARSE_DDX_COUNTER_CLEAR	20514	//粗DX计数清零

#define COARSE_DDY_OPEN				20601  // 粗YTZ开环
#define COARSE_DDY_CLOSE			20602  // 粗YTZ闭环
#define COARSE_DDY_STEP_FORWARD		20603  // 粗YTZstep+
#define COARSE_DDY_STEP_REVERSE     20604  // 粗YTZstep-		
#define COARSE_DDY_JOG_FORWARD    	20605  // 粗YTZJog+
#define COARSE_DDY_JOG_REVERSE    	20606  // 粗YTZJog-
#define COARSE_DDY_JOG_STOP     	20607  // 粗YTZJog+停
#define COARSE_DDY_HOME				20608  // 粗YTZ回零
#define COARSE_DDY_LASERSWITCH		20609  // 粗动台YTZ切激光
#define COARSE_DDY_EDDYSWITCH		20610  // 粗动台YTZ切回电涡流
#define COARSE_DDY_IDENTIFY		    20611  // 粗YTZ辨识
#define COARSE_DDY_RUN_FORWARD		20612	// 粗YTZ走轨迹+
#define COARSE_DDY_RUN_REVERSE		20613  // 粗YTZ走轨迹-
#define CPARSE_DDY_COUNTER_CLEAR	20614	//粗DDY计数清零


// Fine DX
#define FINE_DX_OPEN				10101  // 微DX开环
#define FINE_DX_CLOSE				10102  // 微DX闭环
#define FINE_DX_STEP_FORWARD		10103  // 微DXstep+
#define FINE_DX_STEP_REVERSE      	10104  // 微DXstep-		
#define FINE_DX_JOG_FORWARD    		10105  // 微DXJog+
#define FINE_DX_JOG_REVERSE    		10106  // 微DXJog-
#define FINE_DX_JOG_STOP	     	10107  // 微DXJog+停
#define FINE_DX_HOME				10108  // 微DX回零
#define FINE_DX_LASERSWITCH			10109  // 微动台DX切激光
#define FINE_DX_EDDYSWITCH			10110  // 微动台DX切回电涡流
#define FINE_DX_IDENTIFY		    10111  // 微DX辨识
#define FINE_DX_RUN_FORWARD			10112	// 微DX走轨迹+
#define FINE_DX_RUN_REVERSE			10113  // 微DX走轨迹-
// Fine DY
#define FINE_DY_OPEN				10201  // 微DY开环
#define FINE_DY_CLOSE				10202  // 微DY闭环
#define FINE_DY_STEP_FORWARD		10203  // 微DYstep+
#define FINE_DY_STEP_REVERSE      	10204  // 微DYstep-		
#define FINE_DY_JOG_FORWARD    		10205  // 微DYJog+
#define FINE_DY_JOG_REVERSE    		10206  // 微DYJog-
#define FINE_DY_JOG_STOP			10207  // 微DYJog+停
#define FINE_DY_HOME				10208  // 微DY回零
#define FINE_DY_LASERSWITCH			10209  // 微动台DY切激光
#define FINE_DY_EDDYSWITCH			10210  // 微动台DY切回电涡流
#define FINE_DY_IDENTIFY		    10211  // 微DY辨识
#define FINE_DY_RUN_FORWARD			10212	// 微DY走轨迹+
#define FINE_DY_RUN_REVERSE			10213  // 微DY走轨迹-

//Fine TZ
#define FINE_TZ_OPEN				10301  // 微TZ开环
#define FINE_TZ_CLOSE				10302  // 微TZ闭环
#define FINE_TZ_STEP_FORWARD		10303  // 微TZstep+
#define FINE_TZ_STEP_REVERSE      	10304  // 微TZstep-		
#define FINE_TZ_JOG_FORWARD    		10305  // 微TZJog+
#define FINE_TZ_JOG_REVERSE    		10306  // 微TZJog-
#define FINE_TZ_JOG_STOP			10307  // 微TZJog+停
#define FINE_TZ_HOME				10308  // 微TZ回零
#define FINE_TZ_LASERSWITCH			10309  // 微动台TZ切激光
#define FINE_TZ_EDDYSWITCH			10310  // 微动台TZ切回电涡流
#define FINE_TZ_IDENTIFY		    10311  // 微TZ辨识
#define FINE_TZ_RUN_FORWARD			10312	// 微TZ走轨迹+
#define FINE_TZ_RUN_REVERSE			10313  // 微TZ走轨迹-

// BOTH DX
#define BOTH_DX_OPEN				10401  // 微DX开环
#define BOTH_DX_CLOSE				10402  // 微DX闭环
#define BOTH_DX_STEP_FORWARD		10403  // 微DXstep+
#define BOTH_DX_STEP_REVERSE      	10404  // 微DXstep-		
#define BOTH_DX_JOG_FORWARD    		10405  // 微DXJog+
#define BOTH_DX_JOG_REVERSE    		10406  // 微DXJog-
#define BOTH_DX_JOG_STOP	     	10407  // 微DXJog+停
#define BOTH_DX_HOME				10408  // 微DX回零
#define BOTH_DX_LASERSWITCH			10409  // 微动台DX切激光
#define BOTH_DX_EDDYSWITCH			10410  // 微动台DX切回电涡流
#define BOTH_DX_IDENTIFY		    10411  // 微DX辨识
#define BOTH_DX_RUN_FORWARD			10412	// 微DX走轨迹+
#define BOTH_DX_RUN_REVERSE			10413  // 微DX走轨迹-
// BOTH DY
#define BOTH_DY_OPEN				10501  // 微DY开环
#define BOTH_DY_CLOSE				10502  // 微DY闭环
#define BOTH_DY_STEP_FORWARD		10503  // 微DYstep+
#define BOTH_DY_STEP_REVERSE      	10504  // 微DYstep-		
#define BOTH_DY_JOG_FORWARD    		10505  // 微DYJog+
#define BOTH_DY_JOG_REVERSE    		10506  // 微DYJog-
#define BOTH_DY_JOG_STOP			10507  // 微DYJog+停
#define BOTH_DY_HOME				10508  // 微DY回零
#define BOTH_DY_LASERSWITCH			10509  // 微动台DY切激光
#define BOTH_DY_EDDYSWITCH			10510  // 微动台DY切回电涡流
#define BOTH_DY_IDENTIFY		    10511  // 微DY辨识
#define BOTH_DY_RUN_FORWARD			10512	// 微DY走轨迹+
#define BOTH_DY_RUN_REVERSE			10513  // 微DY走轨迹-

//BOTH TZ
#define BOTH_TZ_OPEN				10601  // 微TZ开环
#define BOTH_TZ_CLOSE				10602  // 微TZ闭环
#define BOTH_TZ_STEP_FORWARD		10603  // 微TZstep+
#define BOTH_TZ_STEP_REVERSE      	10604  // 微TZstep-		
#define BOTH_TZ_JOG_FORWARD    		10605  // 微TZJog+
#define BOTH_TZ_JOG_REVERSE    		10606  // 微TZJog-
#define BOTH_TZ_JOG_STOP			10607  // 微TZJog+停
#define BOTH_TZ_HOME				10608  // 微TZ回零
#define BOTH_TZ_LASERSWITCH			10609  // 微动台TZ切激光
#define BOTH_TZ_EDDYSWITCH			10610  // 微动台TZ切回电涡流
#define BOTH_TZ_IDENTIFY		    10611  // 微TZ辨识
#define BOTH_TZ_RUN_FORWARD			10612	// 微TZ走轨迹+
#define BOTH_TZ_RUN_REVERSE			10613  // 微TZ走轨迹-

//yht
// Fine DZ
#define FINE_DZ_OPEN				10701  // 微DZ开环
#define FINE_DZ_CLOSE				10702  // 微DZ闭环
#define FINE_DZ_STEP_FORWARD		10703  // 微DZstep+
#define FINE_DZ_STEP_REVERSE      	10704  // 微DZstep-		
#define FINE_DZ_JOG_FORWARD    		10705  // 微DZJog+
#define FINE_DZ_JOG_REVERSE    		10706  // 微DZJog-
#define FINE_DZ_JOG_STOP	     	10707  // 微DZJog+停
#define FINE_DZ_HOME				10708  // 微DZ回零
#define FINE_DZ_LASERSWITCH			10709  // 微动台DX切激光
#define FINE_DZ_EDDYSWITCH			10710  // 微动台DX切回电涡流
#define FINE_DZ_IDENTIFY		    10711  // 微DX辨识
#define FINE_DZ_RUN_FORWARD			10712	// 微DX走轨迹+
#define FINE_DZ_RUN_REVERSE			10713  // 微DX走轨迹-
// Fine TX
#define FINE_TX_OPEN				10801  // 微DY开环
#define FINE_TX_CLOSE				10802  // 微DY闭环
#define FINE_TX_STEP_FORWARD		10803  // 微DYstep+
#define FINE_TX_STEP_REVERSE      	10804  // 微DYstep-		
#define FINE_TX_JOG_FORWARD    		10805  // 微DYJog+
#define FINE_TX_JOG_REVERSE    		10806  // 微DYJog-
#define FINE_TX_JOG_STOP			10807  // 微DYJog+停
#define FINE_TX_HOME				10808  // 微DY回零
#define FINE_TX_LASERSWITCH			10809  // 微动台DY切激光
#define FINE_TX_EDDYSWITCH			10810  // 微动台DY切回电涡流
#define FINE_TX_IDENTIFY		    10811  // 微DY辨识
#define FINE_TX_RUN_FORWARD			10812	// 微DY走轨迹+
#define FINE_TX_RUN_REVERSE			10813  // 微DY走轨迹-

//Fine TY
#define FINE_TY_OPEN				10901  // 微TZ开环
#define FINE_TY_CLOSE				10902  // 微TZ闭环
#define FINE_TY_STEP_FORWARD		10903  // 微TZstep+
#define FINE_TY_STEP_REVERSE      	10904  // 微TZstep-		
#define FINE_TY_JOG_FORWARD    		10905  // 微TZJog+
#define FINE_TY_JOG_REVERSE    		10906  // 微TZJog-
#define FINE_TY_JOG_STOP			10907  // 微TZJog+停
#define FINE_TY_HOME				10908  // 微TZ回零
#define FINE_TY_LASERSWITCH			10909  // 微动台TZ切激光
#define FINE_TY_EDDYSWITCH			10910  // 微动台TZ切回电涡流
#define FINE_TY_IDENTIFY		    10911  // 微TZ辨识
#define FINE_TY_RUN_FORWARD			10912	// 微TZ走轨迹+
#define FINE_TY_RUN_REVERSE			10913  // 微TZ走轨迹-

#define HOME					6000
#define SET_POS					6001
#define SET_Vel					6002
#define SET_ACC					6003
#define SET_JERK				6004
#define SET_FPOS				6005
#define CLOSE					6006
#define OPEN					6007
#define MOVE_ABS				6008
#define MOVE_REL				6009
#define JOG						6010
#define JOG_STOP				6011
#define STOP					6012
#define SET_KP					6013
#define SET_KI					6014
#define SET_KD					6015
#define SET_CONNECT_TYPE		6016









/*
typedef enum
{
    MOT_ABORT,
    MOT_ENABLE,
    MOT_DISABLE,
    MOT_PAUSE,
    MOT_FORWARD,
    MOT_REVERSE,
    MOT_JOGFOR,
    MOT_JOGREV,
    MOT_STEP,
    MOT_FREE,
    MOT_COORD,
    MOT_TELEOP,
    MOT_SETKP,
    MOT_SETKI,
    MOT_SETKD,
    MOT_SETPOS,
    MOT_SETVEL,
    MOT_SETACC,
    MOT_SETJERK,
    MOT_SETSNAP,
    MOT_SETACCFEEDCOFF,
    MOT_SETDAOFFSET
}command_e;
*/

typedef struct
{
	command_e index;
	void (*fun)(RXData_t*);
}commFun_t;

extern commFun_t commFun[];

void motionAbort(RXData_t* rxdata);
void motorEnable(RXData_t* rxdata);
void motorDisable(RXData_t* rxdata);
void motionPause(RXData_t *rxdata);
void motionForward(RXData_t *rxdata);
void motionRevrese(RXData_t *rxdata);
void motionJogFor(RXData_t *rxdata);
void motionJogRev(RXData_t *rxdata);
void motionStep(RXData_t *rxdata);
void motionFree(RXData_t *rxdata);
void motionCooard(RXData_t *rxdata);
void motionTeleop(RXData_t *rxdata);
void setKp(RXData_t *rxdata);
void setKi(RXData_t *rxdata);
void setKd(RXData_t *rxdata);
void setPos(RXData_t *rxdata);
void setVel(RXData_t *rxdata);
void setAcc(RXData_t *rxdata);
void setJerk(RXData_t *rxdata);
void setSnap(RXData_t *rxdata);
void setFeedForwardCoff(RXData_t *rxdata);
void setDAOffset(RXData_t *rxdata);
extern void GetCMD(int *pCMD);
#endif
