#ifndef ZMI4104_H
#define ZMI4104_H

#define ZMI_VME_BASE_ADDR      0x90004000
#define ZMI_VME_ADDR_INCE      0x00010000
#define ZMI_4104_ABS_PERC      1//0.15
#define ZMI_RECORD_NUMBER		   100000

#define ZMI_OPT_AXIS_NUMS      4

#define ZMI_OPT_BOAD_NUMS      2

#define DATA_AGE_ADJUST        0x0030
#define EEPROM_WRITE           0x20AE
#define EEPROM_CONTROL         0x20AC
#define TEST_COMMAND1          0x00A0
#define EEPROM_READ            0x20B0



/////////////////////////////////////////
#define VME_POSITION_MSB       0x0040
#define VME_POSITION_LSB       0x0042
#define VME_POSITION_EXT       0x0044

#define VME_SAMPLE_POSITION_MSB 0x0048
#define VME_SAMPLE_POSITION_LSB 0x004A
#define VME_SAMPLE_POSITION_EXT 0x004A
/////////////////////////////////////////



#define APD_BIAS_DAC           0x00DE
#define APD_GAIN_L2_SET        0x018C
#define APD_OPT_PWR_L2_SET     0x018E
#define APD_SIG_RMS_L2_SET     0x0190
#define SIG_RMS_L2_MIN_LIM     0x0198
#define SIG_RMS_L2_MAX_LIM     0x019A
#define SAMPLE_TIMER           0x2038
#define CONTROL_REGISTER0      0x0010
#define CONTROL_REGISTER1      0x0012
#define CONTROL_REGISTER2      0x0014
#define CONTROL_REGISTER3      0x0016
#define CONTROL_REGISTER4      0x0018
#define CONTROL_REGISTER5      0x001A
#define CONTROL_REGISTER15     0x003A
#define CONTROL_REGISTER16     0x003C
#define CONTROL_REGISTER17     0x003E
#define VME_COMMAND            0x0000
#define SCLK_COMMAND           0x2002

typedef struct EepromStruct
{
	short   date[4];
	short   AssyNum;
	short   config;
	short   revisionOrg;
	short   revisionRew;
	short   SerialNum[5];
	short   NumOfAxes;
	short   OptionalFeature;
}EepromStruct;




typedef struct tagZMI4104C
{
	short   Status0andVMECommand;	//00
	short   Status1andSCLKCommand;	//02
	short   VMEInterruptEnable0;	//04
	short   VMEInterruptEnable1;	//06
	short   VMEErrorStatusClear0;	//08
	short   VMEErrorStatusClear1;	//0A
	short   Reserved0C;		//0C
	short   FirmwareVersion;	//0E
	short 	ControlRegister0;	//10
	short 	ControlRegister1;	//12(Filters)
	short	ControlRegister2;	//14
	short	ControlRegister3;	//16(Init)
	short	ControlRegister4;	//18(P2)
	short	ControlRegister5;	//1A(APD)
	short	VMEInterruptEnable2;	//1C
	short	VMEErrorStatusClear2;	//1E

	short	P2InterruptEnable0;	//20
	short	P2InterruptEnable1;	//22

	short	P2ErrorStatusClear0;	//24
	short	P2ErrorStatusClear1;	//26
	short	VMEAbsolultePhase;	//28

	short	SSIAverage;		//2A
	short	SSIMax;			//2C
	short	PhaseNoiseLimit;	//2E
	short	DataAgeAdjust;		//30
	short	UserExcessVelocity;	//32
	short	SSISquelch;		//34
	short	SigRMSL2;		//36
 	short	SampleTimer;		//38(Axis 3 only)	
	short	ControlRegister15;	//3A(Axis 1 and 3 only)
	short	ControlRegister16;	//3C(Axis 1 and 3 only)
	short	ControlRegister17;	//3E(Axis 1 and 3 only)
	
}ZMI4104C,*pZMI4104C;

extern long RecLaser[ZMI_OPT_AXIS_NUMS*ZMI_OPT_BOAD_NUMS];
//#define 
extern void SetZMIRegisterBit(int iRegAddr,unsigned short lBoadID,unsigned short lAxis,int SetBit,int bitIndex);

int VMEPosRead(unsigned short lBoadID,unsigned short lAxis);
int VMESamPosRead(unsigned short lBoadID,unsigned short lAxis);
void Laser_InitAxis(unsigned short lBoadID,unsigned short lAxis);

#endif