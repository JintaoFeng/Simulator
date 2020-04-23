#include <vxWorks.h>
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
#include "math.h"
#include "ZMI4104.h"

#define LASER_RES_NM              0.154538818359375//mm


void InitLaser()
{

	/*int m = 0;

	for(m=0;m<ZMI_OPT_BOAD_NUMS;m++)
	{
		SetRegister(CONTROL_REGISTER16,m,0,0x0200);
		SetRegister(CONTROL_REGISTER16,m,2,0x0200);
	}*/
	SCLK0OutEnable();

	Laser_InitAxis(0,0);
	Laser_InitAxis(0,1);
	Laser_InitAxis(0,2);
	Laser_InitAxis(0,3);
	Laser_InitAxis(1,0);
	Laser_InitAxis(1,1);
	Laser_InitAxis(1,2);
	Laser_InitAxis(1,3);

}





//-***************************************************************************************************************
//-task
int ZMI_tDisplay=0;

//2013-12-21  记录数据
long RecLaser[ZMI_OPT_AXIS_NUMS*ZMI_OPT_BOAD_NUMS];

pZMI4104C pZMI4104CTest=0x90004000;


int ReadRegister(int iRegAddr,unsigned short lBoadID)
{
	int Val;
	Val = *((volatile unsigned short*)(ZMI_VME_BASE_ADDR+lBoadID*ZMI_VME_ADDR_INCE+iRegAddr));
	return Val;
 }

void WriteRegister(int iRegAddr,unsigned short lBoadID,int iVal)
{
	*((volatile unsigned short*)(ZMI_VME_BASE_ADDR+lBoadID*ZMI_VME_ADDR_INCE+iRegAddr))=iVal;
 }

void SetRegister(int iRegAddr,unsigned short lBoadID,unsigned short lAxis,int SetByte)
{
	unsigned int lADDR_AXIS = lAxis<<12;
	int Val = 0;
	iRegAddr |= lADDR_AXIS;
	Val = ReadRegister(iRegAddr,lBoadID);
	Val |= SetByte;
	WriteRegister(iRegAddr,lBoadID,Val);
}

void SetZMIRegisterBit(int iRegAddr,unsigned short lBoadID,unsigned short lAxis,int SetBit,int bitIndex)
{
	unsigned int lADDR_AXIS = lAxis<<12;
	unsigned int lADDR_BASE = ZMI_VME_BASE_ADDR+ZMI_VME_ADDR_INCE*lBoadID;
	unsigned int lADDR = lADDR_BASE + lADDR_AXIS +iRegAddr;
	unsigned int temp;
	unsigned int* ptr = *((volatile unsigned short*)lADDR);

	temp = *ptr;

	if(SetBit)
	{
		temp |= (1<<bitIndex);
	}
	else
	{
		temp &= ~(1<<bitIndex);
	}
	*ptr = temp;
}

void DataAgeCompensate()
{
	WriteRegister(DATA_AGE_ADJUST,0,16);
	WriteRegister(DATA_AGE_ADJUST,1,11);
	WriteRegister(DATA_AGE_ADJUST,2,6);
	WriteRegister(DATA_AGE_ADJUST,3,0);
}
void WriteEeprom(short address, short value)
{
	short enableWriteAddr = 0x300;
	*((volatile unsigned short*)(ZMI_VME_BASE_ADDR+EEPROM_WRITE)) = value; //write value
	*((volatile unsigned short*)(ZMI_VME_BASE_ADDR+enableWriteAddr)) = 0x00;//write enable
	address |= 0x1000;
	*((volatile unsigned short*)(ZMI_VME_BASE_ADDR+EEPROM_CONTROL)) = address; //write control
}
short ReadEeprom(short address)
{	
	EepromStruct info;
	int eepromStatus1 = 0;
	int eepromStatus2 = 0;
	int mask  = 0x0800;
	short result = 0;
	int test = 0;
	int Val = 0;
	//step 1	
	eepromStatus1 = ReadRegister(TEST_COMMAND1,0);//verify eeprom error(test status register)
	if ((eepromStatus1 & mask) != 0)
	{
		Val = ReadRegister(TEST_COMMAND1,0);
		Val |= 0x0004;
		WriteRegister(TEST_COMMAND1,0,Val);//clear eeprom error(test command register)
	}
	eepromStatus2 = ReadRegister(TEST_COMMAND1,2);
	if ((eepromStatus2 & mask) != 0)
	{
		Val = ReadRegister(TEST_COMMAND1,2);
		Val |= 0x0004;
		WriteRegister(TEST_COMMAND1,2,Val);//clear eeprom error(test command register)
	}
	logMsg("test:%d\n",test,0,0,0,0,0);
	//step 2
	while (1)
	{	
		eepromStatus2 = ReadRegister(TEST_COMMAND1,2);//eeprom busy axis 3 only
		if ((eepromStatus2 & 0x0200) == 0)
		{
			test = 1;
			logMsg("test:%d\n",test,0,0,0,0,0);
			break;
		}
		
	}
	//step 3
	address |= 0x2000;
	*((volatile unsigned short*)(ZMI_VME_BASE_ADDR+EEPROM_CONTROL)) = address;
	//step 4
	while (1)
	{	
		eepromStatus2 = ReadRegister(TEST_COMMAND1,2);//eeprom busy axis 3 only
		if ((eepromStatus2 & 0x0200) == 0)
		{
			test = 2;
			logMsg("test:%d\n",test,0,0,0,0,0);
			break;
		}
		
	}
	//step 5
	result = *((volatile unsigned short*)(ZMI_VME_BASE_ADDR+EEPROM_READ));
	logMsg("result:%d\n",result,0,0,0,0,0);
	return result;	
}

void GetEepromInfo(void)
{
	EepromStruct val;
	short   date[4];
	short   AssyNum;
	short   config;
	short   revisionOrg;
	short   revisionRew;
	short   SerialNum;
	short   NumOfAxes;
	short   OptionalFeature;
	val.date[0] = ReadEeprom(2);
	val.date[1] = ReadEeprom(3);
	val.date[2] = ReadEeprom(4);
	val.AssyNum = ReadEeprom(5);
	val.config = ReadEeprom(7);
	val.revisionOrg = ReadEeprom(8);
	val.revisionRew = ReadEeprom(9);
	val.SerialNum[0] = ReadEeprom(10);
	val.SerialNum[1] = ReadEeprom(12);
	val.SerialNum[2] = ReadEeprom(14);
	val.OptionalFeature = ReadEeprom(15); 
	printf("\n******date:%d %d %d*********\n",val.date[0],val.date[1], val.date[2]);
	printf("\n.......AssyNum:%d......\n", val.AssyNum);
	printf("\nconfig:%d",val.config);
}
//-***************************************************************************************************************
void ZMI_DelayCNT(unsigned int lCOUNT)
{
	unsigned int lDelay = lCOUNT+1;while(lDelay--);
}

//-***************************************************************************************************************
int ZMI_VME_Pos32(unsigned short lBoadID,unsigned short lAxis)
{
	int i = 0;
	unsigned int lADDR_AXIS = lAxis<<12;
	unsigned int lADDR_BASE = ZMI_VME_BASE_ADDR+ZMI_VME_ADDR_INCE*lBoadID;
	
	int    lPos = 0;
	//double lPosDbl = 0;
	int lPosDbl = 0;
	
	int lPosMSB = *((unsigned short*)(lADDR_BASE+lADDR_AXIS+VME_POSITION_MSB));
	int lPosLSB = *((unsigned short*)(lADDR_BASE+lADDR_AXIS+VME_POSITION_LSB));
	//for(i=0;i<10000;i++){}
	
	lPos = (lPosMSB<<16)|(lPosLSB<<0);
	
	lPosDbl = (int)(lPos * ZMI_4104_ABS_PERC);
	
    if((lBoadID==0)&&(lAxis==0))
	{
		//logMsg("\n",0,0,0,0,0,0);
		//logMsg("\n",0,0,0,0,0,0);
		//printf("--------------------[ %d ]------------------------\n",lCOUNT);
	}
    if(lAxis==0)
	{
		//logMsg("\n",0,0,0,0,0,0);
		//logMsg("BoardID%d:",lBoadID,0,0,0,0,0);
	}
	//logMsg("lBoadID:%d\n, lAxis:%d\n, Data:%lf\n",lBoadID,lAxis,lPosDbl,0,0,0);
	//logMsg("lBoadID:%d\n, lAxis:%d\n, Data:%d\n",lBoadID,lAxis,lPosDbl,0,0,0);
	//logMsg("%lf\n",lPosDbl,0,0,0,0,0);
	return lPosDbl;
}
//-***************************************************************************************************************
int ZMI_VME_SAMPLE_Pos32(unsigned short lBoadID,unsigned short lAxis)
{
	int i = 0;
	unsigned int lADDR_AXIS = lAxis<<12;
	unsigned int lADDR_BASE = ZMI_VME_BASE_ADDR+ZMI_VME_ADDR_INCE*lBoadID;
	
	int    lPos = 0;
	//double lPosDbl = 0;
	int lPosDbl = 0;
	
	int lPosMSB = *((unsigned short*)(lADDR_BASE+lADDR_AXIS+VME_SAMPLE_POSITION_MSB));
	int lPosLSB = *((unsigned short*)(lADDR_BASE+lADDR_AXIS+VME_SAMPLE_POSITION_LSB));
	//for(i=0;i<10000;i++){}
	
	lPos = (lPosMSB<<16)|(lPosLSB<<0);
	
	lPosDbl = (int)(lPos * ZMI_4104_ABS_PERC);
	
    if((lBoadID==0)&&(lAxis==0))
	{
		//logMsg("\n",0,0,0,0,0,0);
		//logMsg("\n",0,0,0,0,0,0);
		//printf("--------------------[ %d ]------------------------\n",lCOUNT);
	}
    if(lAxis==0)
	{
		//logMsg("\n",0,0,0,0,0,0);
		//logMsg("BoardID%d:",lBoadID,0,0,0,0,0);
	}
	//logMsg("lBoadID:%d\n, lAxis:%d\n, Data:%lf\n",lBoadID,lAxis,lPosDbl,0,0,0);
	//logMsg("lBoadID:%d\n, lAxis:%d\n, Data:%d\n",lBoadID,lAxis,lPosDbl,0,0,0);
	//logMsg("%lf\n",lPosDbl,0,0,0,0,0);
	return lPosDbl;
}


long ZMI_VME_Pos37(unsigned short lBoadID,unsigned short lAxis)
{
	//--------------------------------------------------------------------------
	unsigned int lADDR_AXIS = lAxis<<12;
	unsigned int lADDR_BASE = ZMI_VME_BASE_ADDR+ZMI_VME_ADDR_INCE*lBoadID;
	//--------------------------------------------------------------------------
	long lPos = 0;
	double lDbl;
	//--------------------------------------------------------------------------
	unsigned int lPosEXT = *((unsigned short*)(lADDR_BASE+lADDR_AXIS+VME_POSITION_EXT));
	unsigned int lPosMSB = *((unsigned short*)(lADDR_BASE+lADDR_AXIS+VME_POSITION_MSB));
	unsigned int lPosLSB = *((unsigned short*)(lADDR_BASE+lADDR_AXIS+VME_POSITION_LSB));
	//lPos = 0x00ff00ff;//(lPosEXT&0x0000);
	//lDbl = 0x00ff00ff;
	lPos = (lPos<<32)|(lPosMSB<<16)|(lPosLSB<<0);
//	lPos = (lPos<<32)|(lPosMSB<<16)|(lPosLSB<<0);
	//--------------------------------------------------------------------------
        if((lBoadID==0)&&(lAxis==0)){
	//logMsg("\n",0,0,0,0,0,0);
	//logMsg("\n",0,0,0,0,0,0);
	}
        if(lAxis==0){
	//logMsg("\n",0,0,0,0,0,0);
	//logMsg("BoardID%d:",lBoadID,0,0,0,0,0);
	}
	//--------------------------------------------------------------------------
	//printf("%d\t",lPos);
	//logMsg("lBoadID:%d\n, lAxis:%d\n, Data:%ld\n",lBoadID,lAxis,lPos,0,0,0);
	//--------------------------------------------------------------------------
	return lPos;
}


void ZMI_LedOnBoard(unsigned char lBoadID)
{
	*((unsigned short*)(ZMI_VME_BASE_ADDR+ZMI_VME_ADDR_INCE*lBoadID+0x203C))|= 0x0002;
}

void ZMI_LedOffBoard(unsigned char lBoadID)
{
	*((unsigned short*)(ZMI_VME_BASE_ADDR+ZMI_VME_ADDR_INCE*lBoadID+0x203C))&= 0xFFFD;
}


//-**************************************************************************************
//-Axis
void ZMI_InitAxis(unsigned short lBoadID,unsigned short lAxis)
{
	//--------------------------------------------------------------------------
	//unsigned int lADDR_AXIS = lAxis<<12;
	//unsigned int lADDR_BASE = ZMI_VME_BASE_ADDR+ZMI_VME_ADDR_INCE*lBoadID;
	//--------------------------------------------------------------------------
	//-set  5200 4450  3585  
	SetRegister(APD_BIAS_DAC,lBoadID,lAxis,1000);       //APD_BIAS_DAC  61.65 MV
	SetRegister(APD_GAIN_L2_SET,lBoadID,lAxis,4450);    //APD_GAIN_L2_SET
	SetRegister(APD_OPT_PWR_L2_SET,lBoadID,lAxis,1500); //APD_OPT_PWR_L2_SET
	SetRegister(APD_SIG_RMS_L2_SET,lBoadID,lAxis,9500); //APD_SIG_RMS_L2_SET
	SetRegister(SIG_RMS_L2_MIN_LIM,lBoadID,lAxis,8000); //SIG_RMS_l2_min_lim
	SetRegister(SIG_RMS_L2_MAX_LIM,lBoadID,lAxis,20000);//SIG_RMS_l2_max_lim
	//--------------------------------------------------------------------------
	//-
	SetRegister(SAMPLE_TIMER,lBoadID,0,20000);
	//--------------------------------------------------------------------------
	//-ctrl reg 
	SetRegister(CONTROL_REGISTER0,lBoadID,lAxis,0x0000);//Control Register 0
	SetRegister(CONTROL_REGISTER1,lBoadID,lAxis,0x0000);//Control Register 1
	SetRegister(CONTROL_REGISTER2,lBoadID,lAxis,0x3f00);//Control Register 2
	SetRegister(CONTROL_REGISTER3,lBoadID,lAxis,0x0000);//Control Register 3 init
	SetRegister(CONTROL_REGISTER4,lBoadID,lAxis,0x0000);//Control Register 4 p2
	SetRegister(CONTROL_REGISTER5,lBoadID,lAxis,0x0001);//Control Register 5 APD
	//--------------------------------------------------------------------------
	//-ctrl reg
	//SetRegister(CONTROL_REGISTER15,lBoadID,lAxis,0x0000);//Control Register 15  32位
	SetRegister(CONTROL_REGISTER15,lBoadID,lAxis,0x0000);//Control Register 15  37位
	SetRegister(CONTROL_REGISTER16,lBoadID,lAxis,0x0000);//Control Register 16
	SetRegister(CONTROL_REGISTER17,lBoadID,lAxis,0x0000);//Control Register 17

	SetRegister(CONTROL_REGISTER15,lBoadID,lAxis,0x0000);//Control Register 15
	SetRegister(CONTROL_REGISTER16,lBoadID,lAxis,0x0000);//Control Register 16
	SetRegister(CONTROL_REGISTER17,lBoadID,lAxis,0x0000);//Control Register 17
	//--------------------------------------------------------------------------
	SetRegister(VME_COMMAND,lBoadID,lAxis,0x0240);  //cmd
	//--------------------------------------------------------------------------

}
void ioemethod(void)
{
	int i = 0;
	resetzmi();
	for (i=0; i<10;i++){;}
	ClockTest0(0);
}
void ioelatch(void)
{
	*((volatile unsigned short*)(0x90004000+0x0018)) |= 0x2000;
	*((volatile unsigned short*)(0x90004000+0x1018)) |= 0x2000;
	*((volatile unsigned short*)(0x90004000+0x2018)) |= 0x2000;
	*((volatile unsigned short*)(0x90004000+0x3018)) |= 0x2000;
}
void testreset(void)
{
	int i = 0;
	resetzmi();
	for (i=0; i<10;i++){;}	
	ClockTest1(3999);
	//ClockTest0(3);
	//SCLK2TestClk0(0);

//	ClockTest0(19);
//	*((unsigned short*)(0x90004000+0x0018)) |= 0x2000;//IOE
//	*((unsigned short*)(0x90004000+0x1018)) |= 0x2000;//IOE
//	*((unsigned short*)(0x90004000+0x2018)) |= 0x2000;//IOE
//	*((unsigned short*)(0x90004000+0x3018)) |= 0x2000;//IOE

//	*((unsigned short*)(0x90004000+0x003C)) |= 0x0010;//
//	*((unsigned short*)(0x90004000+0x103C)) |= 0x0010;//
//	*((unsigned short*)(0x90004000+0x203C)) |= 0x0010;//
//	*((unsigned short*)(0x90004000+0x303C)) |= 0x0010;//


/*	
	*((unsigned short*)(0x90004000+0x010E)) = 0xFFFF;//
	*((unsigned short*)(0x90004000+0x110E)) = 0xFFFF;//
	*((unsigned short*)(0x90004000+0x210E)) = 0xFFFF;//
	*((unsigned short*)(0x90004000+0x310E)) = 0xFFFF;//
*/

//	ioelatch();
}
void resetzmi(void)
{
//Board 1
//	*((volatile unsigned short*)(0x90004000+0x203C)) &= 0xFF7F;//sclk timer disable
	*((unsigned short*)(0x90004000+0x0000)) |= 0x003F;	
	*((unsigned short*)(0x90004000+0x1000)) |= 0x003F;
	*((unsigned short*)(0x90004000+0x2000)) |= 0x003F;
	*((unsigned short*)(0x90004000+0x3000)) |= 0x003F;


//Board 2
//	*((volatile unsigned short*)(0x90014000+0x203C)) &= 0xFF7F;//sclk timer disable
	*((unsigned short*)(0x90014000+0x0000)) |= 0x003F;	
	*((unsigned short*)(0x90014000+0x1000)) |= 0x003F;
	*((unsigned short*)(0x90014000+0x2000)) |= 0x003F;
	*((unsigned short*)(0x90014000+0x3000)) |= 0x003F;	




	*((unsigned short*)(0x90004000+0x0014)) |= 0x0020;	
	*((unsigned short*)(0x90004000+0x1014)) |= 0x0020;
	*((unsigned short*)(0x90004000+0x2014)) |= 0x0020;
	*((unsigned short*)(0x90004000+0x3014)) |= 0x0020;



	*((unsigned short*)(0x90014000+0x0014)) |= 0x0020;	
	*((unsigned short*)(0x90014000+0x1014)) |= 0x0020;
	*((unsigned short*)(0x90014000+0x2014)) |= 0x0020;
	*((unsigned short*)(0x90014000+0x3014)) |= 0x0020;	

	

/*
//Board 3
//	*((volatile unsigned short*)(0x90024000+0x203C)) &= 0xFF7F;//sclk timer disable
	*((unsigned short*)(0x90024000+0x0000)) |= 0x003F;	
	*((unsigned short*)(0x90024000+0x1000)) |= 0x003F;
	*((unsigned short*)(0x90024000+0x2000)) |= 0x003F;
	*((unsigned short*)(0x90024000+0x3000)) |= 0x003F;


//Board 4
//	*((volatile unsigned short*)(0x90034000+0x203C)) &= 0xFF7F;//sclk timer disable


	*((unsigned short*)(0x90034000+0x0000)) |= 0x003F;	
	*((unsigned short*)(0x90034000+0x1000)) |= 0x003F;
	*((unsigned short*)(0x90034000+0x2000)) |= 0x003F;
	*((unsigned short*)(0x90034000+0x3000)) |= 0x003F;
//Board 5
	//	*((volatile unsigned short*)(0x90024000+0x203C)) &= 0xFF7F;//sclk timer disable
		*((unsigned short*)(0x90044000+0x0000)) |= 0x003F;	
		*((unsigned short*)(0x90044000+0x1000)) |= 0x003F;
		*((unsigned short*)(0x90044000+0x2000)) |= 0x003F;
		*((unsigned short*)(0x90044000+0x3000)) |= 0x003F;
	
	
//Board 6
	//	*((volatile unsigned short*)(0x90034000+0x203C)) &= 0xFF7F;//sclk timer disable
	
	
		*((unsigned short*)(0x90054000+0x0000)) |= 0x003F;	
		*((unsigned short*)(0x90054000+0x1000)) |= 0x003F;
		*((unsigned short*)(0x90054000+0x2000)) |= 0x003F;
		*((unsigned short*)(0x90054000+0x3000)) |= 0x003F;
		
//Board 7
	//	*((volatile unsigned short*)(0x90024000+0x203C)) &= 0xFF7F;//sclk timer disable
		*((unsigned short*)(0x90064000+0x0000)) |= 0x003F;	
		*((unsigned short*)(0x90064000+0x1000)) |= 0x003F;
		*((unsigned short*)(0x90064000+0x2000)) |= 0x003F;
		*((unsigned short*)(0x90064000+0x3000)) |= 0x003F;
	
	
//Board 8
	//	*((volatile unsigned short*)(0x90034000+0x203C)) &= 0xFF7F;//sclk timer disable
	
	
		*((unsigned short*)(0x90074000+0x0000)) |= 0x003F;	
		*((unsigned short*)(0x90074000+0x1000)) |= 0x003F;
		*((unsigned short*)(0x90074000+0x2000)) |= 0x003F;
		*((unsigned short*)(0x90074000+0x3000)) |= 0x003F;

*/	
}

void x0()
{
	int i = 0;
	int j = 0;
	*((unsigned short*)(0x90004000+0x203C)) |= 0x0200;//E0=1	
	for (;;)
	{
		*((unsigned short*)(0x90004000+0x2002)) |= 0x0001;//sclk command register x0

		for (j=0; j<500; j++)
			{
				;
			}
	}
}

void CLK00Sample()
{
	int i = 0;
	int j = 0;
	*((unsigned short*)(0x90004000+0x203C)) |= 0x0200;//E0=1

	*((unsigned short*)(0x90004000+0x2002)) |= 0x0001;//sclk command register x0
	
	/*for (;;)
	{
		*((unsigned short*)(0x90004000+0x2002)) |= 0x0001;//sclk command register x0

		for (j=0; j<500; j++)
			{
				;
			}
	}*/
}


void ClockTest0(int i)//SCLLK 0
{
	*((unsigned short*)(0x90004000+0x2038)) = i;//sample timer    
	*((unsigned short*)(0x90004000+0x203C)) |= 0x0280;//E0=1 TE=1    
//	*((unsigned short*)(0x90004000+0x2002)) |= 0x0001;//sclk command register x0	


}

void ClockTest1(int i)//SCLLK 1
{

/*	//第四块卡初始化timer
	*((unsigned short*)(0x90034000+0x2038)) = i;//sample timer    
	*((unsigned short*)(0x90034000+0x203C)) |= 0x0180;//TD = 1 TE = 1    
	*((unsigned short*)(0x90034000+0x203E)) |= 0x0200;//E1 = 1
//	*((unsigned short*)(0x90004000+0x2002)) |= 0x0002;//sclk command register x1*/
/*第2块卡初始化timer
*((unsigned short*)(0x90014000+0x2038)) = i;//sample timer	  
*((unsigned short*)(0x90014000+0x203C)) |= 0x0180;//TD = 1 TE = 1	 
*((unsigned short*)(0x90014000+0x203E)) |= 0x0200;//E1 = 1
*/

		//第一块卡初始化timer
		*((unsigned short*)(0x90004000+0x2038)) = i;//sample timer	  
		*((unsigned short*)(0x90004000+0x203C)) |= 0x0180;//TD = 1 TE = 1	 
		*((unsigned short*)(0x90004000+0x203E)) |= 0x0200;//E1 = 1
	//	*((unsigned short*)(0x90004000+0x2002)) |= 0x0002;//sclk command register x1

}
void SCLK2TestClk0(int i)
{

	*((unsigned short*)(0x90004000+0x203C)) |= 0x0200;//E0
	if (0 == i)
	{
		*((unsigned short*)(0x90004000+0x203E)) |= 0x0080;//SRM=3 	
	}
	else
	{
		*((unsigned short*)(0x90004000+0x203E)) &= 0xFF3F;//SRM=0  FORBIDDEN	
	}
	*((unsigned short*)(0x90004000+0x203E)) &= 0xFEFF;//RD=0
//	*((unsigned short*)(0x90004000+0x203C)) |= 0x0200;//E0
}

void SCLK2TestClk1(int i)
{


//	
	*((unsigned short*)(0x90004000+0x203E)) |= 0x0200;//E1

	if (0 == i)
	{
		*((unsigned short*)(0x90004000+0x203E)) |= 0x0080;//SRM=3 	
	}
	else
	{
		*((unsigned short*)(0x90004000+0x203E)) &= 0xFF3F;//SRM=0  FORBIDDEN	
	}
	*((unsigned short*)(0x90004000+0x203E)) |= 0x0100;//RD=1







	*((unsigned short*)(0x90004000+0x0018)) |= 0x1000;
	*((unsigned short*)(0x90004000+0x1018)) |= 0x1000;
	*((unsigned short*)(0x90004000+0x2018)) |= 0x1000;
	*((unsigned short*)(0x90004000+0x3018)) |= 0x1000;



	*((unsigned short*)(0x90014000+0x0018)) |= 0x1000;
	*((unsigned short*)(0x90014000+0x1018)) |= 0x1000;
	*((unsigned short*)(0x90014000+0x2018)) |= 0x1000;
	*((unsigned short*)(0x90014000+0x3018)) |= 0x1000;


	*((unsigned short*)(0x90024000+0x0018)) |= 0x1000;
	*((unsigned short*)(0x90024000+0x1018)) |= 0x1000;
	*((unsigned short*)(0x90024000+0x2018)) |= 0x1000;
	*((unsigned short*)(0x90024000+0x3018)) |= 0x1000;

}

void LogReg(void)
{
	int regVal,i=0;

	regVal = *((volatile unsigned short*)(0x90004000+0x0000)) ;
	printf("0x0000 Status Register 0:%x\n",regVal);
	regVal = *((volatile unsigned short*)(0x90004000+0x0002)) ;
	printf("0x0002 Status Register 1:%x\n",regVal);

	regVal = *((volatile unsigned short*)(0x90004000+0x0018)) ;
	printf("0x0018 Control Register 4 (P2):%x\n",regVal);

	regVal = *((volatile unsigned short*)(0x90004000+0x003A)) ;
	printf("0x003A Control Register 15(Axis 1 and 3 only):%x\n",regVal);

	regVal = *((volatile unsigned short*)(0x90004000+0x003C)) ;
	printf("0x003C Control Register 16(Axis 1 and 3 only):%x\n",regVal);

	regVal = *((volatile unsigned short*)(0x90004000+0x003E)) ;
	printf("0x003E Control Register 17(Axis 1 and 3 only):%x\n",regVal);



	regVal = *((volatile unsigned short*)(0x90004000+0x010E)) ;
	printf("0x010E:%x\n",regVal);
	regVal = *((volatile unsigned short*)(0x90004000+0x010C)) ;
	printf("0x010C:%x\n",regVal);

	regVal = *((volatile unsigned short*)(0x90004000+0x000E)) ;
	printf("0x000E firmware version:%x\n",regVal);

	regVal = *((volatile unsigned short*)(0x90004000+0x0000)) ;
	printf("0x000 status register 0:%x\n",regVal);

	regVal = *((volatile unsigned short*)(0x90004000+0x00A0)) ;
	printf("BOARD1 0x00A0 Test Status 1 Register:%x\n",regVal);

	regVal = *((volatile unsigned short*)(0x90014000+0x00A0)) ;
	printf("BOARD2 0x00A0 Test Status 1 Register:%x\n",regVal);

	regVal = *((volatile unsigned short*)(0x90024000+0x00A0)) ;
	printf("BOARD3 0x00A0 Test Status 1 Register:%x\n",regVal);

	regVal = *((volatile unsigned short*)(0x90004000+0x003C)) ;
	printf("BOARD1 0x003C control Register 16:%x\n",regVal);

	regVal = *((volatile unsigned short*)(0x90004000+0x203C)) ;
	printf("BOARD1 0x203C control Register 16:%x\n",regVal);

	regVal = *((volatile unsigned short*)(0x90014000+0x003C)) ;
	printf("BOARD2 0x003C Control Register16:%x\n",regVal);

	regVal = *((volatile unsigned short*)(0x90014000+0x203C)) ;
	printf("BOARD2 0x203C Control Register16:%x\n",regVal);

	regVal = *((volatile unsigned short*)(0x90024000+0x003C)) ;
	printf("BOARD3 0x003C Control Register16:%x\n",regVal);

	regVal = *((volatile unsigned short*)(0x90024000+0x203C)) ;
	printf("BOARD3 0x203C Control Register16:%x\n",regVal);

}
void Sclk0elayEnable()
{
	*((volatile unsigned short*)(0x90004000+0x003C)) |= 0x10;
	*((volatile unsigned short*)(0x90004000+0x203C)) |= 0x10;
	*((volatile unsigned short*)(0x90014000+0x003C)) |= 0x10;
	*((volatile unsigned short*)(0x90014000+0x203C)) |= 0x10;
	*((volatile unsigned short*)(0x90024000+0x003C)) |= 0x10;
	*((volatile unsigned short*)(0x90024000+0x203C)) |= 0x10;

}
void DataTest(void)
{
     int regVal,i=0;
//	*((volatile unsigned short*)(0x90004000+0x203E)) |= 0x390;
//    regVal = *((volatile unsigned short*)(0x90004000+0x203E)) ;
	 regVal = *((volatile unsigned short*)(0x90004000+0x0018)) ;
		printf("203e:%x\n",regVal);
//20130115
	*((volatile unsigned short*)(0x90004000+0x203C)) &= 0xFF7F;//sclk timer disable
	*((unsigned short*)(0x90004000+0x0000)) |= 0x003F;	
	*((unsigned short*)(0x90004000+0x1000)) |= 0x003F;
	*((unsigned short*)(0x90004000+0x2000)) |= 0x003F;
	*((unsigned short*)(0x90004000+0x3000)) |= 0x003F;


	
	for(i=0;i<10;i++)
	{;	}
//	*((unsigned short*)(0x90004000+0x2002)) |= 0x0001;//sclk command register	
//	*((unsigned short*)(0x90004000+0x203E)) |= 0x0080;//SRM
//	*((unsigned short*)(0x90004000+0x203C)) |= 0x0200;//E0
}
void ZMI_ClearAxis(unsigned short lBoadID,unsigned short lAxis)
{
	//--------------------------------------------------------------------------
	unsigned int lADDR_AXIS = lAxis<<12;
	unsigned int regVal = 0;
	unsigned int lADDR_BASE = ZMI_VME_BASE_ADDR+ZMI_VME_ADDR_INCE*lBoadID;
	//--------------------------------------------------------------------------
	//20140114
/*	if (0 ==lBoadID && 2 == lAxis)
	{
		*((unsigned short*)(lADDR_BASE+0x203E)) |= 0x0080;//0x0380;
		//*((unsigned short*)(lADDR_BASE+lADDR_AXIS+0x003E)) |= 0x0280;
		regVal = *((volatile unsigned short*)(lADDR_BASE+0x203E)) ;

		printf("203e:%x\n",regVal);
	}
*/
//	*((unsigned short*)(lADDR_BASE+0x003A)) |= 0x1000;
//	*((unsigned short*)(lADDR_BASE+0x203A)) |= 0x1000;
     
/*	*((unsigned short*)(lADDR_BASE+0x0018)) |= 0x1000;
	*((unsigned short*)(lADDR_BASE+0x1018)) |= 0x1000;
	*((unsigned short*)(lADDR_BASE+0x2018)) |= 0x1000;
	*((unsigned short*)(lADDR_BASE+0x3018)) |= 0x1000;
    */	



      //-set
	*((unsigned short*)(lADDR_BASE+lADDR_AXIS+0x0000)) |= 0x003F;
	*((unsigned short*)(lADDR_BASE+lADDR_AXIS+0x0008)) |= 0xFFFF;
	*((unsigned short*)(lADDR_BASE+lADDR_AXIS+0x001e)) |= 0xFFFF;

/*	*((unsigned short*)(lADDR_BASE+lADDR_AXIS+0x0018)) |= 0x1000;
	if(lBoadID==0)
	{
		*((unsigned short*)(lADDR_BASE+lADDR_AXIS+0x0018)) &= 0xEFFF;
	}
*/

	//--------------------------------------------------------------------------

}
void ZMI_StatusAxis(unsigned short lBoadID,unsigned short lAxis)
{
	//--------------------------------------------------------------------------
	unsigned int lADDR_AXIS = lAxis<<12;
	unsigned int lADDR_BASE = ZMI_VME_BASE_ADDR+ZMI_VME_ADDR_INCE*lBoadID;
	//--------------------------------------------------------------------------
	int VME_STATUS0 = *((unsigned short*)(lADDR_BASE+lADDR_AXIS+0x0000));
	int VME_STATUS1 = *((unsigned short*)(lADDR_BASE+lADDR_AXIS+0x0002));
	int APD_ERRORCD = *((unsigned short*)(lADDR_BASE+lADDR_AXIS+0x018A));
	//--------------------------------------------------------------------------
	int GSE_GainTAR = *((unsigned short*)(lADDR_BASE+lADDR_AXIS+0x01BC));//TARGET GAIN
	int GSE_GainACT = *((unsigned short*)(lADDR_BASE+lADDR_AXIS+0x01BE));//ACTUAL GAIN
	int GSE_GainRMS = *((unsigned short*)(lADDR_BASE+lADDR_AXIS+0x01C0));
	int GSE_MeasDCL = *((unsigned short*)(lADDR_BASE+lADDR_AXIS+0x01C2));//&(0x00FF);
	int GSE_MeasDCH = *((unsigned short*)(lADDR_BASE+lADDR_AXIS+0x01C2));//&(0xFF00);
	//--------------------------------------------------------------------------
	int ERR_STATUS0 = *((unsigned short*)(lADDR_BASE+lADDR_AXIS+0x0008));
	int ERR_STATUS1 = *((unsigned short*)(lADDR_BASE+lADDR_AXIS+0x000A));
	int ERR_STATUS2 = *((unsigned short*)(lADDR_BASE+lADDR_AXIS+0x001E));
	//--------------------------------------------------------------------------
        if(lAxis==0){
	printf("\n------------------------------------------------------\n");
	}
	//--------------------------------------------------------------------------
	printf("\n");
	printf("BaseADDR=%x  Axis=%x\n",lADDR_BASE+lADDR_AXIS,lAxis);
	printf("APDv=%x\n",APD_ERRORCD);
	printf("STA0=%x\t STA1=%x\n",VME_STATUS0,VME_STATUS1);
	printf("ERR0=%x\t ERR1=%x\t ERR2=%x\n",ERR_STATUS0,ERR_STATUS1,ERR_STATUS2);
	printf("GTAR=%x\t GACT=%x\t GRMS=%x\t DCVL=%x\n",GSE_GainTAR,GSE_GainACT,GSE_GainRMS,GSE_MeasDCL);
	//--------------------------------------------------------------------------

}
//-**************************************************************************************
//-
int ZMI_GetL2RegValAxis(unsigned short lBoadID,unsigned short lAxis,unsigned short lRegOFFSET)
{
	//--------------------------------------------------------------------------
	unsigned int lADDR_AXIS = lAxis<<12;
	unsigned int lADDR_BASE = ZMI_VME_BASE_ADDR+ZMI_VME_ADDR_INCE*lBoadID;
	//--------------------------------------------------------------------------
	int lL2Reg = *((unsigned short*)(lADDR_BASE+lADDR_AXIS+lRegOFFSET));  // OPT_PWR_L2
	//--------------------------------------------------------------------------
	return(2^(lL2Reg/1024));  
	//--------------------------------------------------------------------------
}

void ZMI_GetL2RegStaAxis(unsigned short lBoadID,unsigned short lAxis)
{
	//--------------------------------------------------------------------------
	unsigned int lADDR_AXIS = lAxis<<12;
	unsigned int lADDR_BASE = ZMI_VME_BASE_ADDR+ZMI_VME_ADDR_INCE*lBoadID;
	//--------------------------------------------------------------------------
	int lOPT_PWR,lOPT_PWR_DC,lAPD_GAIN,lSIG_RMS;
	//--------------------------------------------------------------------------
	lOPT_PWR   = ZMI_GetL2RegValAxis(lBoadID,lAxis,0x0192);  // OPT_PWR
	lOPT_PWR_DC= ZMI_GetL2RegValAxis(lBoadID,lAxis,0x01A4);  // OPT_PWR_DC
	lAPD_GAIN  = ZMI_GetL2RegValAxis(lBoadID,lAxis,0x01A2);  // APD_GAIN
	lSIG_RMS   = ZMI_GetL2RegValAxis(lBoadID,lAxis,0x0036);  // SIG_RMS
	//--------------------------------------------------------------------------
        if(lAxis==0){
	printf("\n------------------------------------------------------\n");
	}
	//--------------------------------------------------------------------------
	printf("\n");
	printf("BaseADDR=%x  Axis=%x\n",lADDR_BASE+lADDR_AXIS,lAxis);
	printf("OPT_PWR= %d\n",lOPT_PWR);
	printf("OPT_PWR_DC= %d\n",lOPT_PWR_DC);
	printf("lAPD_GAIN= %d\n",lAPD_GAIN);
	printf("lSIG_RMS= %d\n",lSIG_RMS);
	//--------------------------------------------------------------------------
}
//-**************************************************************************************
//-

void ZMI_Init()
{
	int lBoad,lAxis = 0;
	for(lBoad = 0;lBoad<ZMI_OPT_BOAD_NUMS;lBoad++){
	for(lAxis = 0;lAxis<ZMI_OPT_AXIS_NUMS;lAxis++){
		ZMI_InitAxis(lBoad,lAxis);
	}}
}
//-
void ZMI_Clear()
{
	int lBoad,lAxis = 0;
	for(lBoad = 0;lBoad<ZMI_OPT_BOAD_NUMS;lBoad++){
	for(lAxis = 0;lAxis<ZMI_OPT_AXIS_NUMS;lAxis++){
		ZMI_ClearAxis(lBoad,lAxis);
	}}
}
//-
void ZMI_STATUS()
{
	int lBoad,lAxis = 0;
	for(lBoad = 0;lBoad<ZMI_OPT_BOAD_NUMS;lBoad++){
	for(lAxis = 0;lAxis<ZMI_OPT_AXIS_NUMS;lAxis++){
		ZMI_StatusAxis(lBoad,lAxis);
	}}
}
//-
void ZMI_L2RegSta()
{
	int lBoad,lAxis = 0;
	for(lBoad = 0;lBoad<ZMI_OPT_BOAD_NUMS;lBoad++){
	for(lAxis = 0;lAxis<ZMI_OPT_AXIS_NUMS;lAxis++){
		ZMI_GetL2RegStaAxis(lBoad,lAxis);
	}}
}

//-
void ZMI_LedOn()
{
	int lBoad;
	for(lBoad = 0;lBoad<ZMI_OPT_BOAD_NUMS;lBoad++){
		ZMI_LedOnBoard(lBoad);ZMI_DelayCNT(1000);
	}
}//-
void ZMI_LedOff()
{
	int lBoad;
	for(lBoad = 0;lBoad<ZMI_OPT_BOAD_NUMS;lBoad++){
		ZMI_LedOffBoard(lBoad);ZMI_DelayCNT(1000);
	}
}

//-
void ZMI_SETUP()
{
	ZMI_Init();  ZMI_DelayCNT(1000);
	ZMI_Clear(); ZMI_DelayCNT(1000);
	ZMI_STATUS();ZMI_DelayCNT(1000);
}

//-**************************************************************************************
//-task

void ZMI_VME_Pos()
{
	double tmp = 0.0;
	int lBoad,lAxis = 0;
	for(lBoad = 0;lBoad<ZMI_OPT_BOAD_NUMS;lBoad++)
	{
		for(lAxis = 0;lAxis<ZMI_OPT_AXIS_NUMS;lAxis++)
		{
           
			//RecLaser[lBoad*4+lAxis] = ZMI_VME_SAMPLE_Pos32(lBoad,lAxis);	
			RecLaser[lBoad*4+lAxis] = ZMI_VME_Pos32(lBoad,lAxis);
		//	ZMI_VME_Pos37(ZMI_VME_BASE_ADDR+ZMI_VME_ADDR_INCE*lBoad,lAxis,lCOUNT);
		}
	}
}
//-**************************************************************************************
//-task

void ZMI_Display(void)
{
	static int lCOUNT = 0;	
	while(1)
	{
		ZMI_VME_Pos(lCOUNT++);

		//printf("%d\t%d\t%d\n",*((unsigned short*)(0xe0b44004)),*((unsigned short*)(0xe0b44006)),lCOUNT++);
			
		taskDelay((int)(2*sysClkRateGet()/2));
	}
}

void ZMI_Start(void)
{
	if(ZMI_tDisplay)
	{
		printf("Display task already started!\n");
		return;
	}
	ZMI_tDisplay=taskSpawn("Display",80,0,2000,(FUNCPTR)ZMI_Display,0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
	printf("Display task started id=%d\n",ZMI_tDisplay);

}

void ZMI_Stop(void)
{
	taskDelete(ZMI_tDisplay);
	ZMI_tDisplay=0;
	printf("Display task stoped\n");

}


void ZMI_Save(void)
{
	FILE* fp = fopen("Laser.txt","w");
	int i = 0;
	int j = 0;
	if (fp != NULL)
	{
		for (i=0; i<ZMI_RECORD_NUMBER;i++)
		{
			for (j=0; j<ZMI_OPT_BOAD_NUMS*4; j++)
			{
				//fprintf(fp,"%.3lf", g_recordData[i][j]);
			}
			fprintf(fp,"\n");
		}
		
	}
	else
	{
		printf("\nFile Open Failed.\n");
		return;
	}
	fclose(fp);
	printf("\nSave Laser Data Over.\n");
}


//------------------------------------------------------------------
void SampleClock0()
{
	//Board 1
	//	*((volatile unsigned short*)(0x90004000+0x203C)) &= 0xFF7F;//sclk timer disable
		*((unsigned short*)(0x90004000+0x0018)) &= 0x7FFF;	
		*((unsigned short*)(0x90004000+0x1018)) &= 0x7FFF;
		*((unsigned short*)(0x90004000+0x2018)) &= 0x7FFF;
		*((unsigned short*)(0x90004000+0x3018)) &= 0x7FFF;
	
	
	//Board 2
	//	*((volatile unsigned short*)(0x90014000+0x203C)) &= 0xFF7F;//sclk timer disable
		*((unsigned short*)(0x90014000+0x0018)) &= 0x7FFF;	
		*((unsigned short*)(0x90014000+0x1018)) &= 0x7FFF;
		*((unsigned short*)(0x90014000+0x2018)) &= 0x7FFF;
		*((unsigned short*)(0x90014000+0x3018)) &= 0x7FFF;	
	
	
	//Board 3
	//	*((volatile unsigned short*)(0x90024000+0x203C)) &= 0xFF7F;//sclk timer disable
		*((unsigned short*)(0x90024000+0x0018)) &= 0x7FFF;	
		*((unsigned short*)(0x90024000+0x1018)) &= 0x7FFF;
		*((unsigned short*)(0x90024000+0x2018)) &= 0x7FFF;
		*((unsigned short*)(0x90024000+0x3018)) &= 0x7FFF;
	
	
	//Board 4
	//	*((volatile unsigned short*)(0x90034000+0x203C)) &= 0xFF7F;//sclk timer disable
	
	
		*((unsigned short*)(0x90034000+0x0018)) &= 0x7FFF;	
		*((unsigned short*)(0x90034000+0x1018)) &= 0x7FFF;
		*((unsigned short*)(0x90034000+0x2018)) &= 0x7FFF;
		*((unsigned short*)(0x90034000+0x3018)) &= 0x7FFF;
	//Board 5
	//	*((volatile unsigned short*)(0x90034000+0x203C)) &= 0xFF7F;//sclk timer disable
	
	
		*((unsigned short*)(0x90044000+0x0018)) &= 0x7FFF;	
		*((unsigned short*)(0x90044000+0x1018)) &= 0x7FFF;
		*((unsigned short*)(0x90044000+0x2018)) &= 0x7FFF;
		*((unsigned short*)(0x90044000+0x3018)) &= 0x7FFF;
	//Board 6
	//	*((volatile unsigned short*)(0x90034000+0x203C)) &= 0xFF7F;//sclk timer disable
	
	
		*((unsigned short*)(0x90054000+0x0018)) &= 0x7FFF;	
		*((unsigned short*)(0x90054000+0x1018)) &= 0x7FFF;
		*((unsigned short*)(0x90054000+0x2018)) &= 0x7FFF;
		*((unsigned short*)(0x90054000+0x3018)) &= 0x7FFF;
	///////////////////////////////////////////////////////////////////
	//Board 7
	//	*((volatile unsigned short*)(0x90004000+0x203C)) &= 0xFF7F;//sclk timer disable
		*((unsigned short*)(0x90064000+0x0018)) &= 0x7FFF;	
		*((unsigned short*)(0x90064000+0x1018)) &= 0x7FFF;
		*((unsigned short*)(0x90064000+0x2018)) &= 0x7FFF;
		*((unsigned short*)(0x90064000+0x3018)) &= 0x7FFF;
	
	
	//Board 8
	//	*((volatile unsigned short*)(0x90014000+0x203C)) &= 0xFF7F;//sclk timer disable
		*((unsigned short*)(0x90074000+0x0018)) &= 0x7FFF;	
		*((unsigned short*)(0x90074000+0x1018)) &= 0x7FFF;
		*((unsigned short*)(0x90074000+0x2018)) &= 0x7FFF;
		*((unsigned short*)(0x90074000+0x3018)) &= 0x7FFF;	
	
	
}



//------------------------------------------------------------------
void SampleClock1()
{
	//Board 1
	//	*((volatile unsigned short*)(0x90004000+0x203C)) &= 0xFF7F;//sclk timer disable
		*((unsigned short*)(0x90004000+0x0018)) |= 0x1000;	
		*((unsigned short*)(0x90004000+0x1018)) |= 0x1000;
		*((unsigned short*)(0x90004000+0x2018)) |= 0x1000;
		*((unsigned short*)(0x90004000+0x3018)) |= 0x1000;
	
	
	//Board 2
	//	*((volatile unsigned short*)(0x90014000+0x203C)) &= 0xFF7F;//sclk timer disable
		*((unsigned short*)(0x90014000+0x0018)) |= 0x1000;	
		*((unsigned short*)(0x90014000+0x1018)) |= 0x1000;
		*((unsigned short*)(0x90014000+0x2018)) |= 0x1000;
		*((unsigned short*)(0x90014000+0x3018)) |= 0x1000;	
	
	
	//Board 3
	//	*((volatile unsigned short*)(0x90024000+0x203C)) &= 0xFF7F;//sclk timer disable
		*((unsigned short*)(0x90024000+0x0018)) |= 0x1000;	
		*((unsigned short*)(0x90024000+0x1018)) |= 0x1000;
		*((unsigned short*)(0x90024000+0x2018)) |= 0x1000;
		*((unsigned short*)(0x90024000+0x3018)) |= 0x1000;
	
	
	//Board 4
	//	*((volatile unsigned short*)(0x90034000+0x203C)) &= 0xFF7F;//sclk timer disable
	
	
		*((unsigned short*)(0x90034000+0x0018)) |= 0x1000;	
		*((unsigned short*)(0x90034000+0x1018)) |= 0x1000;
		*((unsigned short*)(0x90034000+0x2018)) |= 0x1000;
		*((unsigned short*)(0x90034000+0x3018)) |= 0x1000;
	//Board 5
	//	*((volatile unsigned short*)(0x90034000+0x203C)) &= 0xFF7F;//sclk timer disable
	
	
		*((unsigned short*)(0x90044000+0x0018)) |= 0x1000;	
		*((unsigned short*)(0x90044000+0x1018)) |= 0x1000;
		*((unsigned short*)(0x90044000+0x2018)) |= 0x1000;
		*((unsigned short*)(0x90044000+0x3018)) |= 0x1000;
	//Board 6
	//	*((volatile unsigned short*)(0x90034000+0x203C)) &= 0xFF7F;//sclk timer disable
	
	
		*((unsigned short*)(0x90054000+0x0018)) |= 0x1000;	
		*((unsigned short*)(0x90054000+0x1018)) |= 0x1000;
		*((unsigned short*)(0x90054000+0x2018)) |= 0x1000;
		*((unsigned short*)(0x90054000+0x3018)) |= 0x1000;
	///////////////////////////////////////////////////////////////////
	//Board 7
	//	*((volatile unsigned short*)(0x90004000+0x203C)) &= 0xFF7F;//sclk timer disable
		*((unsigned short*)(0x90064000+0x0018)) |= 0x1000;	
		*((unsigned short*)(0x90064000+0x1018)) |= 0x1000;
		*((unsigned short*)(0x90064000+0x2018)) |= 0x1000;
		*((unsigned short*)(0x90064000+0x3018)) |= 0x1000;
	
	
	//Board 8
	//	*((volatile unsigned short*)(0x90014000+0x203C)) &= 0xFF7F;//sclk timer disable
		*((unsigned short*)(0x90074000+0x0018)) |= 0x1000;	
		*((unsigned short*)(0x90074000+0x1018)) |= 0x1000;
		*((unsigned short*)(0x90074000+0x2018)) |= 0x1000;
		*((unsigned short*)(0x90074000+0x3018)) |= 0x1000;	
	
	
}
//---------------------------------------------------------------------------
void SSI_Avg(void)
{
	int lregVal[4] = {0, 0, 0, 0};

	lregVal[0] = *((volatile unsigned short*)(0x90004000+0x002A)) ;
	lregVal[1] = *((volatile unsigned short*)(0x90004000+0x102A)) ;
	lregVal[2] = *((volatile unsigned short*)(0x90004000+0x202A)) ;
	lregVal[3] = *((volatile unsigned short*)(0x90004000+0x302A)) ;
	printf("Board 1#,\nx1=%d\nx2=%d\ny1=%d\ny2=%d\n",lregVal[0],lregVal[1],lregVal[2],lregVal[3]);
	
	lregVal[0] = *((volatile unsigned short*)(0x90014000+0x002A)) ;
	lregVal[1] = *((volatile unsigned short*)(0x90014000+0x102A)) ;
	lregVal[2] = *((volatile unsigned short*)(0x90014000+0x202A)) ;
	lregVal[3] = *((volatile unsigned short*)(0x90014000+0x302A)) ;
	printf("Board 2#,\ny3=%d\nz1=%d\nz2=%d\nz3=%d\n",lregVal[0],lregVal[1],lregVal[2],lregVal[3]);
	
/*	lregVal[0] = *((volatile unsigned short*)(0x90024000+0x002A)) ;
	lregVal[1] = *((volatile unsigned short*)(0x90024000+0x102A)) ;
	lregVal[2] = *((volatile unsigned short*)(0x90024000+0x202A)) ;
	lregVal[3] = *((volatile unsigned short*)(0x90024000+0x302A)) ;
	printf("Board 3#,\nz4=%d\nws1=%d\nws2=%d\nws3=%d\n",lregVal[0],lregVal[1],lregVal[2],lregVal[3]);

	lregVal[0] = *((volatile unsigned short*)(0x90034000+0x002A)) ;
	lregVal[1] = *((volatile unsigned short*)(0x90034000+0x102A)) ;
	lregVal[2] = *((volatile unsigned short*)(0x90034000+0x202A)) ;
	lregVal[3] = *((volatile unsigned short*)(0x90034000+0x302A)) ;
	printf("Board 4#,\nws4=%d\nws5=%d\nws6=%d\nws7=%d\n",lregVal[0],lregVal[1],lregVal[2],lregVal[3]);

	lregVal[0] = *((volatile unsigned short*)(0x90044000+0x002A)) ;
	lregVal[1] = *((volatile unsigned short*)(0x90044000+0x102A)) ;
	lregVal[2] = *((volatile unsigned short*)(0x90044000+0x202A)) ;
	lregVal[3] = *((volatile unsigned short*)(0x90044000+0x302A)) ;
	printf("Board 5#,\nx1=%d\nx2=%d\ny1=%d\ny2=%d\n",lregVal[0],lregVal[1],lregVal[2],lregVal[3]);
	
	lregVal[0] = *((volatile unsigned short*)(0x90054000+0x002A)) ;
	lregVal[1] = *((volatile unsigned short*)(0x90054000+0x102A)) ;
	lregVal[2] = *((volatile unsigned short*)(0x90054000+0x202A)) ;
	lregVal[3] = *((volatile unsigned short*)(0x90054000+0x302A)) ;
	printf("Board 6#,\ny3=%d\nz1=%d\nz2=%d\nz3=%d\n",lregVal[0],lregVal[1],lregVal[2],lregVal[3]);
	
	lregVal[0] = *((volatile unsigned short*)(0x90064000+0x002A)) ;
	lregVal[1] = *((volatile unsigned short*)(0x90064000+0x102A)) ;
	lregVal[2] = *((volatile unsigned short*)(0x90064000+0x202A)) ;
	lregVal[3] = *((volatile unsigned short*)(0x90064000+0x302A)) ;
	printf("Board 7#,\nz4=%d\nws1=%d\nws2=%d\nws3=%d\n",lregVal[0],lregVal[1],lregVal[2],lregVal[3]);

	lregVal[0] = *((volatile unsigned short*)(0x90074000+0x002A)) ;
	lregVal[1] = *((volatile unsigned short*)(0x90074000+0x102A)) ;
	lregVal[2] = *((volatile unsigned short*)(0x90074000+0x202A)) ;
	lregVal[3] = *((volatile unsigned short*)(0x90074000+0x302A)) ;
	printf("Board 8#,\nws4=%d\nws5=%d\nws6=%d\nws7=%d\n",lregVal[0],lregVal[1],lregVal[2],lregVal[3]);*/
	
}
void SSI_Avg10(void)
{
	int lregVal[30] = {0, 0, 0, 0, 0,0,0,0,  0,0, 0,0, 0,0,0,0,0, 0, 0, 0, 0,0,0,0,  0,0, 0,0, 0,0};
	int i = 0;
	int j = 0;
	int k = 0;
	int v = 0;
	int n = 0;
	int time1 = 0;
	int freq = 1;
	int time2 = 0;
	int time = 0;
	FILE* fp = fopen("ssi.txt","w+");
	if (fp==NULL)
	{
		printf("open file failed.");
		return;
	}
	
	
	for (j=0;j<10000;j++)
	{

//		sysTimestampEnable();
	//	freq = sysTimestampFreq();
	//	time1 = sysTimestamp();
	 	lregVal[0] = *((volatile unsigned short*)(0x90004000+0x002A)) ;//x1
		lregVal[1] = *((volatile unsigned short*)(0x90004000+0x102A)) ;//x2
	
		lregVal[2] = *((volatile unsigned short*)(0x90004000+0x202A)) ;//y1
		lregVal[3] = *((volatile unsigned short*)(0x90004000+0x302A)) ;//y2
		lregVal[4] = *((volatile unsigned short*)(0x90014000+0x002A)) ;//y3

		lregVal[5] = *((volatile unsigned short*)(0x90014000+0x102A)) ;//z1
		lregVal[6] = *((volatile unsigned short*)(0x90014000+0x202A)) ;//z2
		lregVal[7] = *((volatile unsigned short*)(0x90014000+0x302A)) ;//z3
		lregVal[8] = *((volatile unsigned short*)(0x90024000+0x002A)) ;//z4
		
		lregVal[9] = *((volatile unsigned short*)(0x90024000+0x102A)) ;//ws1
		lregVal[10] = *((volatile unsigned short*)(0x90024000+0x202A)) ;//ws2
		lregVal[11] = *((volatile unsigned short*)(0x90024000+0x302A)) ;//ws3
		lregVal[12] = *((volatile unsigned short*)(0x90034000+0x002A)) ;//ws4
		
		lregVal[13] = *((volatile unsigned short*)(0x90034000+0x102A)) ;//ws5
		lregVal[14] = *((volatile unsigned short*)(0x90034000+0x202A)) ;//ws6
		lregVal[15] = *((volatile unsigned short*)(0x90034000+0x302A)) ;//ws7
		
		lregVal[16] = *((volatile unsigned short*)(0x90044000+0x002A)) ;//ws4
		lregVal[17] = *((volatile unsigned short*)(0x90044000+0x102A)) ;//ws5
		lregVal[18] = *((volatile unsigned short*)(0x90044000+0x202A)) ;//ws6
		lregVal[19] = *((volatile unsigned short*)(0x90044000+0x302A)) ;//ws7


		lregVal[20] = *((volatile unsigned short*)(0x90054000+0x002A)) ;//ws4
		lregVal[21] = *((volatile unsigned short*)(0x90054000+0x102A)) ;//ws5
		lregVal[22] = *((volatile unsigned short*)(0x90054000+0x202A)) ;//ws6
		lregVal[23] = *((volatile unsigned short*)(0x90054000+0x302A)) ;//ws7

		
		lregVal[24] = *((volatile unsigned short*)(0x90064000+0x002A)) ;//ws4
		lregVal[25] = *((volatile unsigned short*)(0x90064000+0x102A)) ;//ws5
		lregVal[26] = *((volatile unsigned short*)(0x90064000+0x202A)) ;//ws6
		lregVal[27] = *((volatile unsigned short*)(0x90064000+0x302A)) ;//ws7

		lregVal[28] = *((volatile unsigned short*)(0x90074000+0x002A)) ;//ws6
		lregVal[29] = *((volatile unsigned short*)(0x90074000+0x102A)) ;//ws7



		if(j%2==0)
		{
			n = 1000;
		}
		else
		{	
			n=1000;
		}
		for(k=0;k<1000;k++)
			{for(v=0;v<n;v++);}
	//	time2 = sysTimestamp();
	//	time = (time2 - time1)/(freq/1000000);
		fprintf(fp, "%d\t",time);
		for(i=0;i<30;i++)
		{
			fprintf(fp,"%d\t",lregVal[i]);
		}
		//sysTimestampDisable();
		fprintf(fp,"\r\n");
	
	}
	fclose(fp);
	
}
///////////////////////////////////////////////////////////////////////////////////////////////
void resetzmi_RS(void)
{
//Board 1
//	*((volatile unsigned short*)(0x90004000+0x203C)) &= 0xFF7F;//sclk timer disable
	*((unsigned short*)(0x90004000+0x0000)) |= 0x003F;	
	*((unsigned short*)(0x90004000+0x1000)) |= 0x003F;
	*((unsigned short*)(0x90004000+0x2000)) |= 0x003F;
	*((unsigned short*)(0x90004000+0x3000)) |= 0x003F;
//Board 2
//	*((volatile unsigned short*)(0x90014000+0x203C)) &= 0xFF7F;//sclk timer disable
	*((unsigned short*)(0x90014000+0x0000)) |= 0x003F;	
	*((unsigned short*)(0x90014000+0x1000)) |= 0x003F;
	*((unsigned short*)(0x90014000+0x2000)) |= 0x003F;
	*((unsigned short*)(0x90014000+0x3000)) |= 0x003F;	

//Board 3
//	*((volatile unsigned short*)(0x90024000+0x203C)) &= 0xFF7F;//sclk timer disable
	*((unsigned short*)(0x90024000+0x0000)) |= 0x003F;	
	//*((unsigned short*)(0x90024000+0x1000)) |= 0x003F;
	//*((unsigned short*)(0x90024000+0x2000)) |= 0x003F;
	//*((unsigned short*)(0x90024000+0x3000)) |= 0x003F;
	
}

void resetzmi_WSB(void)
{
	//Board 3
	//	*((volatile unsigned short*)(0x90024000+0x203C)) &= 0xFF7F;//sclk timer disable
		//*((unsigned short*)(0x90024000+0x0000)) |= 0x003F;	
		*((unsigned short*)(0x90024000+0x1000)) |= 0x003F;
		*((unsigned short*)(0x90024000+0x2000)) |= 0x003F;
		*((unsigned short*)(0x90024000+0x3000)) |= 0x003F;
	
	
	//Board 4
	//	*((volatile unsigned short*)(0x90034000+0x203C)) &= 0xFF7F;//sclk timer disable
		*((unsigned short*)(0x90034000+0x0000)) |= 0x003F;	
		*((unsigned short*)(0x90034000+0x1000)) |= 0x003F;
		*((unsigned short*)(0x90034000+0x2000)) |= 0x003F;
		*((unsigned short*)(0x90034000+0x3000)) |= 0x003F;
	//Board 5
		//	*((volatile unsigned short*)(0x90024000+0x203C)) &= 0xFF7F;//sclk timer disable
			*((unsigned short*)(0x90044000+0x0000)) |= 0x003F;	
			*((unsigned short*)(0x90044000+0x1000)) |= 0x003F;
			//*((unsigned short*)(0x90044000+0x2000)) |= 0x003F;
			//*((unsigned short*)(0x90044000+0x3000)) |= 0x003F;
	
}

void resetzmi_WSA(void)
{
//Board 5
	//	*((volatile unsigned short*)(0x90024000+0x203C)) &= 0xFF7F;//sclk timer disable
		//*((unsigned short*)(0x90044000+0x0000)) |= 0x003F;	
		//*((unsigned short*)(0x90044000+0x1000)) |= 0x003F;
		*((unsigned short*)(0x90044000+0x2000)) |= 0x003F;
		*((unsigned short*)(0x90044000+0x3000)) |= 0x003F;
//Board 6
	//	*((volatile unsigned short*)(0x90034000+0x203C)) &= 0xFF7F;//sclk timer disable
		*((unsigned short*)(0x90054000+0x0000)) |= 0x003F;	
		*((unsigned short*)(0x90054000+0x1000)) |= 0x003F;
		*((unsigned short*)(0x90054000+0x2000)) |= 0x003F;
		*((unsigned short*)(0x90054000+0x3000)) |= 0x003F;
		
//Board 7
	//	*((volatile unsigned short*)(0x90024000+0x203C)) &= 0xFF7F;//sclk timer disable
		*((unsigned short*)(0x90064000+0x0000)) |= 0x003F;	
		*((unsigned short*)(0x90064000+0x1000)) |= 0x003F;
		*((unsigned short*)(0x90064000+0x2000)) |= 0x003F;
		//*((unsigned short*)(0x90064000+0x3000)) |= 0x003F;
	
}


//-**************************************************************************************

//szx
//long ZMI_VME_Pos37(unsigned short lBoadID,unsigned short lAxis)

int VMEPosRead(unsigned short lBoadID,unsigned short lAxis)
{

	//--------------------------------------------------------------------------
	unsigned int lADDR_AXIS = lAxis<<12;
	unsigned int lADDR_BASE = ZMI_VME_BASE_ADDR+ZMI_VME_ADDR_INCE*lBoadID;
	//--------------------------------------------------------------------------
	long lPos = 0;
	double lDbl;
	//--------------------------------------------------------------------------
	unsigned int lPosEXT = *((unsigned short*)(lADDR_BASE+lADDR_AXIS+VME_POSITION_EXT));
	unsigned int lPosMSB = *((unsigned short*)(lADDR_BASE+lADDR_AXIS+VME_POSITION_MSB));
	unsigned int lPosLSB = *((unsigned short*)(lADDR_BASE+lADDR_AXIS+VME_POSITION_LSB));

	lPos = (lPos<<32)|(lPosMSB<<16)|(lPosLSB<<0);

	return lPos;

}

/*
double VMEPosRead(unsigned short lBoadID,unsigned short lAxis)
{

	//--------------------------------------------------------------------------
	unsigned int lADDR_AXIS = lAxis<<12;
	unsigned int lADDR_BASE = ZMI_VME_BASE_ADDR+ZMI_VME_ADDR_INCE*lBoadID;
	//--------------------------------------------------------------------------
	int iPos = 0;
	double dPos;
	//--------------------------------------------------------------------------
	unsigned int lPosEXT = *((unsigned short*)(lADDR_BASE+lADDR_AXIS+VME_POSITION_EXT))&0xf;
	unsigned int lPosMSB = *((unsigned short*)(lADDR_BASE+lADDR_AXIS+VME_POSITION_MSB));
	unsigned int lPosLSB = *((unsigned short*)(lADDR_BASE+lADDR_AXIS+VME_POSITION_LSB));

	iPos = (lPosMSB<<16)|(lPosLSB<<0);

	if(lPosEXT==1)
	{

		dPos=LASER_RES_NM*iPos+LASER_RES_NM*1073741824*4;


	}
	else if(lPosEXT==0xfe)
	{
		dPos=LASER_RES_NM*iPos-LASER_RES_NM*1073741824*4;
	}
	else
	{
	
		dPos=LASER_RES_NM*iPos;

	}

	

	return dPos;

}

*/
/*
#define LASER_RES_NM              2*0.154538818359375//mm


double VMEPosRead(unsigned short lBoadID,unsigned short lAxis)
{

	//--------------------------------------------------------------------------
	unsigned int lADDR_AXIS = lAxis<<12;
	unsigned int lADDR_BASE = ZMI_VME_BASE_ADDR+ZMI_VME_ADDR_INCE*lBoadID;
	//--------------------------------------------------------------------------
	long lPos = 0;
	double lDbl;
	//--------------------------------------------------------------------------
	unsigned int lPosEXT = *((unsigned short*)(lADDR_BASE+lADDR_AXIS+VME_POSITION_EXT));
	unsigned int lPosMSB = *((unsigned short*)(lADDR_BASE+lADDR_AXIS+VME_POSITION_MSB));
	unsigned int lPosLSB = *((unsigned short*)(lADDR_BASE+lADDR_AXIS+VME_POSITION_LSB));

	lPos = (lPos<<32)|(lPosMSB<<16)|(lPosLSB<<0);


	lDbl=lPos*LASER_RES_NM;

	return lDbl;

}
*/

int VMESamPosRead(unsigned short lBoadID,unsigned short lAxis)
{

	//--------------------------------------------------------------------------
	unsigned int lADDR_AXIS = lAxis<<12;
	unsigned int lADDR_BASE = ZMI_VME_BASE_ADDR+ZMI_VME_ADDR_INCE*lBoadID;
	//--------------------------------------------------------------------------
	long lPos = 0;
	double lDbl;
	//--------------------------------------------------------------------------
	unsigned int lPosEXT = *((unsigned short*)(lADDR_BASE+lADDR_AXIS+VME_SAMPLE_POSITION_EXT));
	unsigned int lPosMSB = *((unsigned short*)(lADDR_BASE+lADDR_AXIS+VME_SAMPLE_POSITION_MSB));
	unsigned int lPosLSB = *((unsigned short*)(lADDR_BASE+lADDR_AXIS+VME_SAMPLE_POSITION_LSB));

	lPos = (lPos<<32)|(lPosMSB<<16)|(lPosLSB<<0);

	return lPos;


}
void SamVMEPosCmd()
{


}




void Laser_InitAxis(unsigned short lBoadID,unsigned short lAxis)
{
	//--------------------------------------------------------------------------
	//unsigned int lADDR_AXIS = lAxis<<12;
	//unsigned int lADDR_BASE = ZMI_VME_BASE_ADDR+ZMI_VME_ADDR_INCE*lBoadID;
	//--------------------------------------------------------------------------
	//-set  5200 4450  3585  
	/*SetRegister(APD_BIAS_DAC,lBoadID,lAxis,1000);       //APD_BIAS_DAC  61.65 MV
	SetRegister(APD_GAIN_L2_SET,lBoadID,lAxis,4450);    //APD_GAIN_L2_SET
	SetRegister(APD_OPT_PWR_L2_SET,lBoadID,lAxis,1500); //APD_OPT_PWR_L2_SET
	SetRegister(APD_SIG_RMS_L2_SET,lBoadID,lAxis,9500); //APD_SIG_RMS_L2_SET
	SetRegister(SIG_RMS_L2_MIN_LIM,lBoadID,lAxis,8000); //SIG_RMS_l2_min_lim
	SetRegister(SIG_RMS_L2_MAX_LIM,lBoadID,lAxis,20000);//SIG_RMS_l2_max_lim
	//--------------------------------------------------------------------------
	//-
	SetRegister(SAMPLE_TIMER,lBoadID,0,20000);
	//--------------------------------------------------------------------------
	//-ctrl reg 
	SetRegister(CONTROL_REGISTER0,lBoadID,lAxis,0x0000);//Control Register 0
	SetRegister(CONTROL_REGISTER1,lBoadID,lAxis,0x0000);//Control Register 1
	SetRegister(CONTROL_REGISTER2,lBoadID,lAxis,0x3f00);//Control Register 2
	SetRegister(CONTROL_REGISTER3,lBoadID,lAxis,0x0000);//Control Register 3 init
	SetRegister(CONTROL_REGISTER4,lBoadID,lAxis,0x0000);//Control Register 4 p2
	SetRegister(CONTROL_REGISTER5,lBoadID,lAxis,0x0001);//Control Register 5 APD
	//--------------------------------------------------------------------------
	//-ctrl reg
	//SetRegister(CONTROL_REGISTER15,lBoadID,lAxis,0x0000);//Control Register 15  32位
	SetRegister(CONTROL_REGISTER15,lBoadID,lAxis,0x0000);//Control Register 15  37位
	SetRegister(CONTROL_REGISTER16,lBoadID,lAxis,0x0000);//Control Register 16
	SetRegister(CONTROL_REGISTER17,lBoadID,lAxis,0x0000);//Control Register 17

	SetRegister(CONTROL_REGISTER15,lBoadID,lAxis,0x0000);//Control Register 15
	SetRegister(CONTROL_REGISTER16,lBoadID,lAxis,0x0000);//Control Register 16
	SetRegister(CONTROL_REGISTER17,lBoadID,lAxis,0x0000);//Control Register 17*/
	//--------------------------------------------------------------------------
	//SetRegister(VME_COMMAND,lBoadID,lAxis,0x0240);  //cmd
	//--------------------------------------------------------------------------

	*((unsigned short*)(0x90004000+0x0014)) |= 0x0020;	
	*((unsigned short*)(0x90004000+0x1014)) |= 0x0020;
	*((unsigned short*)(0x90004000+0x2014)) |= 0x0020;
	*((unsigned short*)(0x90004000+0x3014)) |= 0x0020;



	*((unsigned short*)(0x90014000+0x0014)) |= 0x0020;	
	*((unsigned short*)(0x90014000+0x1014)) |= 0x0020;
	*((unsigned short*)(0x90014000+0x2014)) |= 0x0020;
	*((unsigned short*)(0x90014000+0x3014)) |= 0x0020;	



}

void SCLK0OutEnable()
{

	//SetRegister(CONTROL_REGISTER16,0,2,0x0200);
	*((unsigned short*)(0x90004000+0x2000+CONTROL_REGISTER16)) |= 0x0200;//sclk command register x0

	
}

void ExternalSampleSCLK0()
{
	*((unsigned short*)(0x90004000+0x2002)) |= 0x0001;//sclk command register x0
}

void VMEErrorStatus0()
{
	printf("Display task started id=%d\n",*((unsigned short*)(0x90014000+0x0008)));
	printf("Display task started id=%d\n",*((unsigned short*)(0x90014000+0x1008)));
	printf("Display task started id=%d\n",*((unsigned short*)(0x90014000+0x2008)));
	printf("Display task started id=%d\n",*((unsigned short*)(0x90014000+0x3002)));

}









