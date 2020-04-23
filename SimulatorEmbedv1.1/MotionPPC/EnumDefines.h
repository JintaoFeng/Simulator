#ifndef ENUM_DEFINES_H
#define ENUM_DEFINES_H

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


//自由度枚举
typedef enum tagStageType
{
	FineStage = 1,
	CoarseStage = 2	
	
}StageType;
typedef enum tagAxisType
{
	FineAxis = 1,
	CoarseAxis = 2	
}AxisType;

typedef enum tagAxisIndex
{
	DX = 0, 
	DY = 1, 
	TZ = 2
}AxisIndex;
typedef enum tagAxisFineIndex
{
	FDX = 0, 
	FDY = 1, 
	FTZ = 2,
	FDZ = 3, 
	FTX = 4, 
    FTY = 5, 
	ALLREFRESH_FINE = 98	
}AxisFineIndex;

typedef enum tagAxisCoarseIndex
{
	CDX = 0, 
	CDY = 1, 
	XTZ = 2, 
	YTZ = 3, 
	ALLREFRESH_COARSE = 98
}AxisCoarseIndex;

//轨迹状态
typedef enum tagTrajStatus
{
	Stop = 0, 
	Positive_ACC = 1, 
	Speed = 2,
	Negative_ACC = 3, 
	OT = 4
}TrajStatus;

//微动台与粗动台独立跟随模式
typedef enum tagTrajMode
{
	TrajNothing = 0,
	IsTrajIndependent = 1,
	IsTrajFollow = 2
	
}TrajMode;

//在线轨迹模式（3阶轨迹、四阶轨迹与正弦轨迹）
typedef enum tagTrajOrder
{	
	isTrajSineAccT= 0,
	isTraj3OrderPoly = 1,
	isTraj4OrderPoly= 2
}TrajType;

//台子命令枚举
typedef enum tagStageCMD
{
	C_All_Nothing = 0,
	C_Stage_Traj_Expo = 6051, 
	C_Stage_ALL_Close = 6052, 
	C_Stage_ALL_Open = 6053,
	C_Stage_ALL_Step_For = 6054, 
	C_Stage_ALL_Step_Rev = 6055,
	C_Stage_ALL_Jogfor_Start = 6056, 
	C_Stage_ALL_Jogrev_Start = 6057,
	C_Stage_ALL_Jog_Stop = 6058, 
	C_Stage_P2P_Traj = 6059,
	C_Stage_Circle_Traj = 6060,
	C_Stage_Line_Interpolate = 6061,
	C_Stage_Arc_Interpolate = 6062
}StageCMD;

//自由度命令枚举
typedef enum tagAxisCMD
{
	C_Nothing = 0,
	C_Aixs_Close = 6001, 
	C_Aixs_Open = 6002, 
	C_Aixs_Step_For = 6003, 
	C_Aixs_Step_Rev = 6004,
	C_Aixs_Jogfor_Start = 6005, 
	C_Aixs_Jog_Stop = 6006, 
	C_Aixs_Ident = 6007,
	C_Aixs_Jogrev_Start = 6008, 
	C_Aixs_RunFor = 6009, 
	C_Aixs_RunRev = 6010,
	C_Aixs_Home = 6011, 
	C_Aixs_Center_Switch = 6012, 
	C_Aixs_Switch_Ruler = 6049
}AxisCMD;

//台子状态枚举
typedef enum tagStageStatus
{
	S_Stage_Traj_Expo = 6551, 
	S_Stage_ALL_Close = 6552, 
	S_Stage_ALL_Open = 6553,
	S_Stage_ALL_Jogfor_Start = 6554, 
	S_Stage_ALL_Jogrev_Start = 6555, 
	S_Stage_P2P_Traj = 6556,
	S_Stage_Load_Pos = 6557,
	S_Stage_Circle_Traj = 6558,
	S_Stage_Line_Interpolate = 6559,
	S_Stage_Arc_Interpolate = 6560
}StageStatus;

//自由度状态枚举
typedef enum tagAxisStatus
{
	S_Aixs_Close = 6501, 
	S_Aixs_Open = 6502, 
	S_Aixs_Jogfor_Start = 6503, 
	S_Aixs_Ident = 6504,
	S_Aixs_Jogrev_Start = 6505, 
	S_Aixs_RunFor = 6506, 
	S_Aixs_RunRev = 6507, 
	S_Aixs_Home = 6508,
	S_Aixs_Switch_Ruler = 6549
}AxisStatus;

//闭环命令枚举
typedef enum tagCloseCMD
{
	C_Close_Nothing = 0, 
	C_Counter_Close = 6012, 
	C_Laser_Close = 6013
}CloseCMD;

//闭环状态枚举
typedef enum tagCloseStatus
{
	S_Counter_Close = 6512, 
	S_Laser_Close = 6513
}CloseStatus;

typedef enum tagParamType
{
	PID_INDEPENDENT_TYPE = 11,               //独立模式PID参数
	PID_FOLLOW_TYPE = 12,                   //跟随模式PID参数
	LLC_INDEPENDENT_TYPE = 21,               //独立模式LLC参数
	LLC_FOLLOW_TYPE = 22,                    //跟随模式LLC参数
	LIMIT_TYPE = 31,                        //限幅
	ACC_COEFFICIENT_INDEPENDENT_DXMOVE = 61,   //dx轨迹时，独立模式加速度前馈系数
	ACC_COEFFICIENT_FOLLOW_DXMOVE = 62,       //dx轨迹时，跟随模式加速度前馈系数
	ACC_COEFFICIENT_INDEPENDENT_DYMOVE = 71,   //dy轨迹时，独立模式加速度前馈系数
	ACC_COEFFICIENT_FOLLOW_DYMOVE = 72,        //dy轨迹时，跟随模式加速度前馈系数
}ParamType;

//控制器使能位
typedef enum tagControllerType
{
	PID_flag = 0, 
	LLC_flag = 1
	
}ControllerType;


typedef enum tagTrajLast
{
	FirstIn = 0, 
	RunLast = 1
	
}TrajLast;

typedef enum tagLimitStatus
{
	Underlimit = 0, 
	Overlimit = 1
	
}LimitStatus;
#endif
