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

#include "rtcontrol.h"
#include "posixtimer.h"
#include "ZMI4104.h"
#include "pmc16aiodrv.h"
#include "ComWithWin.h"
#include "StateMachine.h"
#include "ConParamInit.h"
#include "ast_Header.h"
#include "CMD.h"
#include "main.h"
#include "FileReadWrite.h"
#include "ast_M58.h"
#include "GetData.h"
#include "Trajplaner.h"
#include "SingleTPG.h"

#include "N1225A.h"

#define RECORD_NUMBER          100000
RecordStruct recordStruct;

UINT32 intCount ;
double g_dHomePos=0.0;
double g_dHomeSetpoint=0.0;

void EnableIntr1()
{
	M72_Channel_INTConfig1 = 0x0082;
}
void DisableIntr1()
{
	M72_Channel_INTConfig1 = 0x0000;
}

void EnableIntr2()
{
	M72_Channel_INTConfig2 = 0x0082;
}
void DisableIntr2()
{
	M72_Channel_INTConfig2 = 0x0000;
}	

void EnableIntr4()
{
	M72_Channel_INTConfig4 = 0x0082;
}
void DisableIntr4()
{
	M72_Channel_INTConfig4 = 0x0000;
}	

void EnableIntr3()
{
	M72_Channel_INTConfig3 = 0x0082;
}
void DisableIntr3()
{
	M72_Channel_INTConfig3 = 0x0000;
}	
void vmeIrq1Intr()
{
	short value1,value2;   
	sysIntDisable(1);	

	INTConfig = 0x0019;//set VME IRQ Level=001(IRQ1) 
	value1 = *(UINT16 *)(0x901c0480);
	value2 = *(UINT16 *)(0x901c0482);
	*(UINT16 *)(0x901c0480) = value1;	
	*(UINT16 *)(0x901c0482) = value2;

	logMsg("value1=%x,value2=%x\n",value1,value2,0,0,0,0);	

	if(1 == Machine.coarseStage.arrAxis[0].bIntrEnable)
	{
		ast_M72_Counter_Clear(0,0);
		ast_M72_Counter_Clear(0,1);
		Machine.coarseStage.arrAxis[0].axisStatus = S_Aixs_Close;
		Machine.coarseStage.arrAxis[0].dSetPoint = 0;
		Machine.coarseStage.arrAxis[1].dSetPoint = 0;
		Machine.coarseStage.arrAxis[0].HomeFlagCorse = 0;
		Machine.coarseStage.arrAxis[0].stepHome = 2;
		Machine.coarseStage.arrAxis[0].bIntrEnable=0;
		logMsg("value1=%x,value2=%x\n",1,2,0,0,0,0);		
	}

	if(1 == Machine.coarseStage.arrAxis[2].bIntrEnable)
	{
		ast_M72_Counter_Clear(0,2);
		ast_M72_Counter_Clear(0,3);
		Machine.coarseStage.arrAxis[2].axisStatus = S_Aixs_Close;
		Machine.coarseStage.arrAxis[2].dSetPoint = 0;
		Machine.coarseStage.arrAxis[3].dSetPoint = 0;
		Machine.coarseStage.arrAxis[2].HomeFlagCorse = 0;
		Machine.coarseStage.arrAxis[2].stepHome = 2;
		Machine.coarseStage.arrAxis[2].bIntrEnable=0;		
	}
	
	DisableIntr1();
//	DisableIntr2();
	DisableIntr3();
//	DisableIntr4();
	 
	sysIntEnable(1);
}
		
void irqConnect()
{ 

    INTConfig = 0x0019;//set VME IRQ Level=001(IRQ1) for A201S
    //INTConfig = 0x0099;//set VME IRQ Level=001(IRQ1) for A203N
    INTVector = 0x0055;//set VME Vector = 0x55

   
   
	/* Connect IRQ1 interrupt to vector 0x55 */

//	printf("intConnect(0x55,vmeIrq1Intr,0x55);\n");
	if (intConnect(0x55,vmeIrq1Intr,0x55) == ERROR)
	{
		printf("Could not intConnect(0x55,vmeIrq1Intr,0x55)\n\r");
	}
//	printf("sysIntEnable(1);\n\r");
	sysIntEnable(1);		/* Enable IRQ1 interrupt */

}

void InitBuff()
{
	int i;
	for(i=0;i<16;i++)
	{
		recordStruct.pRecordData[i] = (int *)malloc(RECORD_NUMBER*sizeof(int));
	}	
}
 
void DataCollect(void)
{
	node *q = NULL;
	FILE *pFile;
	int i=0;
	int j=0;
	q=head;
	while(1)
	{
		semTake(DataCollect_SemId,WAIT_FOREVER);	
		if(1 == recordStruct.iSaveFlag)
		{
			pFile=fopen("LaserData.txt","w+");
			if(pFile == NULL)
			{
				logMsg("open file failed.",0,0,0,0,0,0);
				return;
			}

			if(memoryStype==1)
			{
				for(i=0;i<RECORD_NUMBER;i++)
				{
					for(j=0;j<13-1;j++)
					{
						fprintf(pFile,"%d\t",*(recordStruct.pRecordData[j]+i));

					}
					fprintf(pFile,"%d\r\n",*(recordStruct.pRecordData[12]+i));

					recordStruct.iSaveCnt++;
				}
			}
			else
			{
				for(i=0;i<RECORD_NUMBER;i++)
				{
					while(q!=NULL)
					{
						fprintf(pFile,"%d\t",*(recordStruct.pRecordData[j]+i));
						q=q->next;
						j++;
					}
					q=head;
					j=0;
					fprintf(pFile,"\r\n");
					recordStruct.iSaveCnt++;
				}
			}
			fclose(pFile);
			if(recordStruct.iSaveCnt>=RECORD_NUMBER)
			{					
				recordStruct.iSaveFlag = 0;
				recordStruct.iSaveCnt = 0;				
				logMsg("Save file Finish.\n",0,0,0,0,0,0);	
			}
		}
	}	
}

double DataToVol( UINT32 data)
{
	double vol;
	data=0xffff&data;
	vol=((long int)data-0x8000)*VOLINOUTRANG/(long)0x8000;
	//return 0.5*vol;
	return vol;
}



void GetFeedBack()
{
	int i = 0;
	//ExternalSampleSCLK0();	
	
	SetRegisterBit(RATE_A, 16, 1);	//Set D16, disable rate generator A
	for(i=0;i<6;i++)
	{
		Machine.fineStage.iADC[i] = AIO_Read_Local32(IN_DATA_BUFF);
		Machine.fineStage.iADC[i] &= 0xFFFF;
	}
	//clear buffer
	SetRegisterBit(IN_DATA_CNTRL, 15, 1);
	
	while(GetRegisterBit(IN_DATA_CNTRL, 15))
	{
	  taskDelay(0);
	}

	SetRegisterBit(RATE_A, 16, 0);	//Clear D16, enable rate generator A	

	Machine.fineStage.dInputVol[0] = DataToVol(Machine.fineStage.iADC[0]);
	Machine.fineStage.dInputVol[1] = DataToVol(Machine.fineStage.iADC[1]);
	Machine.fineStage.dInputVol[2] = DataToVol(Machine.fineStage.iADC[2]);

	Machine.fineStage.dInputVol[3] = DataToVol(Machine.fineStage.iADC[3]);
	Machine.fineStage.dInputVol[4] = DataToVol(Machine.fineStage.iADC[4]);
	Machine.fineStage.dInputVol[5] = DataToVol(Machine.fineStage.iADC[5]);

	ast_M72_Counter_Read(0,0,&Machine.coarseStage.iCounter[0]);
	ast_M72_Counter_Read(0,1,&Machine.coarseStage.iCounter[1]);
	ast_M72_Counter_Read(0,2,&Machine.coarseStage.iCounter[2]);
//	ast_M72_Counter_Read(0,3,&Machine.coarseStage.iCounter[3]);
//	ast_M72_Counter_Read(1,0,&Machine.coarseStage.iCounter[4]);
//	ast_M72_Counter_Read(1,1,&Machine.coarseStage.iCounter[5]);

//	for(i=0;i<4;i++)
//	{
//		Machine.coarseStage.iCounter[i] = (int)(Machine.coarseStage.arrAxis[i].controlObject.pUout[0]);
//	}
	
//	Machine.coarseStage.arrAxis[0].dSolveMethodOne -= LPFilter(Machine.coarseStage.dConDist[0]);
//	Machine.coarseStage.dConDist[0] = 1;
//	Machine.coarseStage.arrAxis[0].dSolveMethodOne += LLCCalculate(&(Machine.coarseStage.arrAxis[0].controlObject),Machine.coarseStage.dConDist[0]);

	/*Machine.fineStage.iLaser[0] = VMEPosRead(0,0);
	Machine.fineStage.iLaser[1] = VMEPosRead(0,1);
	Machine.fineStage.iLaser[2] = VMEPosRead(0,2);
	Machine.fineStage.iLaser[3] = VMEPosRead(0,3);
	Machine.fineStage.iLaser[4] = VMEPosRead(1,0);
	Machine.fineStage.iLaser[5] = VMEPosRead(1,1);
	Machine.fineStage.iLaser[6] = VMEPosRead(1,2);
	Machine.fineStage.iLaser[7] = VMEPosRead(1,3);*/
	
/*	ReadAllPositions(Machine.fineStage.iLaser,4);
	if(Machine.iServoCnt%100==0)
	{
		//读取光强
		readDCPowerLevel(1,1,&Machine.fineStage.laserDCPowerLevel[0]);
		readDCPowerLevel(1,2,&Machine.fineStage.laserDCPowerLevel[1]);
		readDCPowerLevel(1,3,&Machine.fineStage.laserDCPowerLevel[2]);
		readDCPowerLevel(2,1,&Machine.fineStage.laserDCPowerLevel[3]);
		readDCPowerLevel(2,2,&Machine.fineStage.laserDCPowerLevel[4]);
		readDCPowerLevel(2,3,&Machine.fineStage.laserDCPowerLevel[5]);
		readDCPowerLevel(2,4,&Machine.fineStage.laserDCPowerLevel[6]);
		readDCPowerLevel(3,1,&Machine.fineStage.laserDCPowerLevel[7]);
		readDCPowerLevel(3,2,&Machine.fineStage.laserDCPowerLevel[8]);
		readDCPowerLevel(3,3,&Machine.fineStage.laserDCPowerLevel[9]);
		readDCPowerLevel(1,4,&Machine.fineStage.laserDCPowerLevel[10]);

		readACPowerLevel(1,1,&Machine.fineStage.laserACPowerLevel[0]);
		readACPowerLevel(1,2,&Machine.fineStage.laserACPowerLevel[1]);
		readACPowerLevel(1,3,&Machine.fineStage.laserACPowerLevel[2]);
		readACPowerLevel(2,1,&Machine.fineStage.laserACPowerLevel[3]);
		readACPowerLevel(2,2,&Machine.fineStage.laserACPowerLevel[4]);
		readACPowerLevel(2,3,&Machine.fineStage.laserACPowerLevel[5]);
		readACPowerLevel(2,4,&Machine.fineStage.laserACPowerLevel[6]);
		readACPowerLevel(3,1,&Machine.fineStage.laserACPowerLevel[7]);
		readACPowerLevel(3,2,&Machine.fineStage.laserACPowerLevel[8]);
		readACPowerLevel(3,3,&Machine.fineStage.laserACPowerLevel[9]);
		readACPowerLevel(1,4,&Machine.fineStage.laserACPowerLevel[10]);
	}

*/
}

#define SENSER_ORIGIN_S1 1435000
#define SENSER_ORIGIN_S2 1655000
#define SENSER_ORIGIN_S3 -1065000
#define SENSER_ORIGIN_S4 1670000
#define SENSER_ORIGIN_S5 1670000
#define SENSER_ORIGIN_S6 2090000

#define LASER_UNIT_MM              0.000000154538818359375//mm
#define LASER_UNIT_NM              0.154538818359375*2//mm

void PosSolve()
{
	//微动台10V对应3mm，1V对应0.3mm，此步骤是将电压转换成与距离对应的
	//count，和粗动台对应，以100nm为单位。
	
	Machine.fineStage.iCounter[0] = Machine.fineStage.dInputVol[0]*3000 ;
	
	Machine.fineStage.iCounter[1] = Machine.fineStage.dInputVol[1]*3000;

	Machine.fineStage.iCounter[2] = -1 * Machine.fineStage.dInputVol[2]*3000;
	//yht
	Machine.fineStage.iCounter[3] = Machine.fineStage.dInputVol[3]*3000;

		
	Machine.fineStage.iCounter[4] = Machine.fineStage.dInputVol[4]*3000;
		
	Machine.fineStage.iCounter[5] = Machine.fineStage.dInputVol[5]*3000;
	
	//DX
	Machine.fineStage.arrAxis[FDX].dSolveMethodOne = (100 * Machine.fineStage.iCounter[0] - SENSER_ORIGIN_S1) * Machine.fineStage.arrAxis[0].iDir; 
	//DY
	Machine.fineStage.arrAxis[FDY].dSolveMethodOne = ((100 * Machine.fineStage.iCounter[1] - SENSER_ORIGIN_S2)+(100*Machine.fineStage.iCounter[2] - SENSER_ORIGIN_S3))/2.0 * Machine.fineStage.arrAxis[1].iDir;
	//Machine.fineStage.arrAxis[1].dSolveMethodOne = (100 * Machine.fineStage.iCounter[1] - SENSER_ORIGIN_S2) * Machine.fineStage.arrAxis[1].iDir;
	//TZ
	Machine.fineStage.arrAxis[FTZ].dSolveMethodOne = ((100 * Machine.fineStage.iCounter[1] - SENSER_ORIGIN_S2)-(100*Machine.fineStage.iCounter[2] - SENSER_ORIGIN_S3))/2.0 * Machine.fineStage.arrAxis[2].iDir;
//	Machine.fineStage.arrAxis[FTZ].dSolveMethodOne = atan2(Machine.fineStage.arrAxis[FTZ].dSolveMethodOne,45500000)*1000000000;
	//yht ZDZ
	Machine.fineStage.arrAxis[FDZ].dSolveMethodOne = ((100*Machine.fineStage.iCounter[3] - SENSER_ORIGIN_S4) + (100*Machine.fineStage.iCounter[5] - SENSER_ORIGIN_S6))/2.0 * Machine.fineStage.arrAxis[3].iDir;
	//TX
	Machine.fineStage.arrAxis[FTX].dSolveMethodOne = ((100 * Machine.fineStage.iCounter[5] - SENSER_ORIGIN_S6) - (100 * Machine.fineStage.iCounter[4] - SENSER_ORIGIN_S5))/2 ;	
//	Machine.fineStage.arrAxis[FTX].dSolveMethodOne = atan2(Machine.fineStage.arrAxis[FTX].dSolveMethodOne,181000000)*1000000000;
	//TY
	Machine.fineStage.arrAxis[FTY].dSolveMethodOne = ((100 *  Machine.fineStage.iCounter[3] - SENSER_ORIGIN_S4) - (100 * Machine.fineStage.iCounter[4] - SENSER_ORIGIN_S5))/2;
//	Machine.fineStage.arrAxis[FTY].dSolveMethodOne = atan2(Machine.fineStage.arrAxis[FTY].dSolveMethodOne,85500000)*1000000000;
	//Basic
	Machine.fineStage.arrAxis[FDX].dSolveMethodTwo = (Machine.fineStage.iLaser[2]*LASER_UNIT_NM+Machine.fineStage.iLaser[3]*LASER_UNIT_NM)/2 ;

	Machine.fineStage.arrAxis[FDY].dSolveMethodTwo = (Machine.fineStage.iLaser[6]*LASER_UNIT_NM+Machine.fineStage.iLaser[7]*LASER_UNIT_NM)/2 ;
		
//	Machine.fineStage.arrAxis[FTZ].dSolveMethodTwo = Machine.fineStage.arrAxis[FDX].dSolveMethodTwo-Machine.fineStage.arrAxis[FDY].dSolveMethodTwo; //-(Machine.fineStage.iLaser[2]*LASER_UNIT_NM+Machine.fineStage.iLaser[4]*LASER_UNIT_NM) /2+(Machine.fineStage.iLaser[3]*LASER_UNIT_NM+Machine.fineStage.iLaser[5]*LASER_UNIT_NM) /2;
	
	Machine.fineStage.arrAxis[FTZ].dSolveMethodTwo = -(Machine.fineStage.iLaser[2]*LASER_UNIT_NM + Machine.fineStage.iLaser[4]*LASER_UNIT_NM) /2 + (Machine.fineStage.iLaser[3]*LASER_UNIT_NM + Machine.fineStage.iLaser[5]*LASER_UNIT_NM) /2;
	//yht
	Machine.fineStage.arrAxis[FDZ].dSolveMethodTwo = -(Machine.fineStage.iLaser[0]*LASER_UNIT_NM+Machine.fineStage.iLaser[1]*LASER_UNIT_NM)/2 ;
	
	Machine.fineStage.arrAxis[FTX].dSolveMethodTwo = (Machine.fineStage.iLaser[6]*LASER_UNIT_NM+Machine.fineStage.iLaser[7]*LASER_UNIT_NM)/2 - (Machine.fineStage.iLaser[8]*LASER_UNIT_NM+Machine.fineStage.iLaser[9]*LASER_UNIT_NM)/2;
			
	Machine.fineStage.arrAxis[FTY].dSolveMethodTwo = -(Machine.fineStage.iLaser[2]*LASER_UNIT_NM-Machine.fineStage.iLaser[3]*LASER_UNIT_NM) /2+(Machine.fineStage.iLaser[4]*LASER_UNIT_NM+Machine.fineStage.iLaser[5]*LASER_UNIT_NM) /2;

	//粗动台	
	if(Machine.coarseStage.DDX_Flag == 1)
	{
		Machine.coarseStage.arrAxis[0].dSolveMethodOne = (CoarseResolution*Machine.coarseStage.iCounter[0]+CoarseResolution*Machine.coarseStage.iCounter[1])/2;
		Machine.coarseStage.arrAxis[1].dSolveMethodOne = (CoarseResolution*Machine.coarseStage.iCounter[0]-CoarseResolution*Machine.coarseStage.iCounter[1]);
	}
	else
	{
		Machine.coarseStage.arrAxis[0].dSolveMethodOne=CoarseResolution*Machine.coarseStage.iCounter[0];
		Machine	.coarseStage.arrAxis[1].dSolveMethodOne=CoarseResolution*Machine.coarseStage.iCounter[1];
	}
	if(Machine.coarseStage.DDY_Flag == 1)
	{
		Machine.coarseStage.arrAxis[2].dSolveMethodOne = (CoarseResolution*Machine.coarseStage.iCounter[2]+CoarseResolution*Machine.coarseStage.iCounter[3])/2;
		Machine.coarseStage.arrAxis[3].dSolveMethodOne = (CoarseResolution*Machine.coarseStage.iCounter[2]-CoarseResolution*Machine.coarseStage.iCounter[3]);
	}
	else
	{
		Machine.coarseStage.arrAxis[2].dSolveMethodOne=CoarseResolution*Machine.coarseStage.iCounter[2];
		Machine.coarseStage.arrAxis[3].dSolveMethodOne=CoarseResolution*Machine.coarseStage.iCounter[3];
	}
		
	Machine.coarseStage.arrAxis[4].dActPos=CoarseResolution*Machine.coarseStage.iCounter[4];
	Machine.coarseStage.arrAxis[5].dActPos=CoarseResolution*Machine.coarseStage.iCounter[5];
}

void ConCalc()
{
	int i = 0;

	for(i=0;i<4;i++)
	{
		Machine.coarseStage.arrAxis[i].dConCalc= Machine.coarseStage.arrAxis[i].OutCalc();
		calcSpeed(&Machine.coarseStage.arrAxis[i]);
		calcAcc(&Machine.coarseStage.arrAxis[i]);
	}
	for(i=0;i<6;i++)
	{
		Machine.fineStage.arrAxis[i].dConCalc = Machine.fineStage.arrAxis[i].OutCalc();
	}

	
	if(Machine.coarseStage.DDX_Flag == 1)
	{
		if(Machine.coarseStage.arrAxis[1].dConCalc > 1.5)
			Machine.coarseStage.arrAxis[1].dConCalc = 1.5;
		else if(Machine.coarseStage.arrAxis[1].dConCalc < -1.5)
			Machine.coarseStage.arrAxis[1].dConCalc = -1.5;
	}
	if(Machine.coarseStage.DDY_Flag == 1)
	{
		if(Machine.coarseStage.arrAxis[3].dConCalc > 1.5)
			Machine.coarseStage.arrAxis[3].dConCalc = 1.5;
		else if(Machine.coarseStage.arrAxis[3].dConCalc < -1.5)
			Machine.coarseStage.arrAxis[3].dConCalc = -1.5;
	}
}


//
//将ConCalc计算值分配给ConDist
//
void ConDist()
{
	if(Machine.coarseStage.DDX_Flag == 1 )
	{
		Machine.coarseStage.dConDist[0] = Machine.coarseStage.arrAxis[0].dConCalc;
		Machine.coarseStage.dConDist[1] = Machine.coarseStage.arrAxis[0].dConCalc;
		Machine.coarseStage.dConDist[0] = Machine.coarseStage.dConDist[0] + Machine.coarseStage.arrAxis[1].dConCalc;
		Machine.coarseStage.dConDist[1] = Machine.coarseStage.dConDist[1] - Machine.coarseStage.arrAxis[1].dConCalc;
	}
	else
	{
		Machine.coarseStage.dConDist[0] = Machine.coarseStage.arrAxis[0].dConCalc + Machine.coarseStage.DAOffset[0];
		Machine.coarseStage.dConDist[1] = Machine.coarseStage.arrAxis[1].dConCalc + Machine.coarseStage.DAOffset[1];
	}
	if(Machine.coarseStage.DDY_Flag == 1)
	{
		Machine.coarseStage.dConDist[2] = Machine.coarseStage.arrAxis[2].dConCalc;
		Machine.coarseStage.dConDist[3] = Machine.coarseStage.arrAxis[2].dConCalc;
		Machine.coarseStage.dConDist[2] = Machine.coarseStage.dConDist[2] + Machine.coarseStage.arrAxis[3].dConCalc;
		Machine.coarseStage.dConDist[3] = Machine.coarseStage.dConDist[3] - Machine.coarseStage.arrAxis[3].dConCalc;
	}
	else
	{
		Machine.coarseStage.dConDist[2] = Machine.coarseStage.arrAxis[2].dConCalc + Machine.coarseStage.DAOffset[2];
		Machine.coarseStage.dConDist[3] = Machine.coarseStage.arrAxis[2].dConCalc + Machine.coarseStage.DAOffset[2];
	}
	//X向
	Machine.fineStage.dConDist[0] = 0.5*Machine.fineStage.arrAxis[0].dConCalc - 0.5 * Machine.fineStage.arrAxis[2].dConCalc + Machine.fineStage.DAOffset[0];
	Machine.fineStage.dConDist[1] = 0.5*Machine.fineStage.arrAxis[0].dConCalc + 0.5 * Machine.fineStage.arrAxis[2].dConCalc + Machine.fineStage.DAOffset[1];
	//Y向

	Machine.fineStage.dConDist[2] = 0.5*Machine.fineStage.arrAxis[1].dConCalc+0.5*Machine.fineStage.arrAxis[2].dConCalc + Machine.fineStage.DAOffset[2];
	Machine.fineStage.dConDist[3] = 0.5*Machine.fineStage.arrAxis[1].dConCalc-0.5*Machine.fineStage.arrAxis[2].dConCalc + Machine.fineStage.DAOffset[3];
	
	//  Z X向
	Machine.fineStage.dConDist[4] = 0.25*Machine.fineStage.arrAxis[3].dConCalc-Machine.fineStage.arrAxis[5].dConCalc + Machine.fineStage.DAOffset[4];
	Machine.fineStage.dConDist[5] = 0.25*Machine.fineStage.arrAxis[3].dConCalc-Machine.fineStage.arrAxis[4].dConCalc + Machine.fineStage.DAOffset[5];
	//Z Y向
	Machine.fineStage.dConDist[6] = 0.25*Machine.fineStage.arrAxis[3].dConCalc+Machine.fineStage.arrAxis[5].dConCalc + Machine.fineStage.DAOffset[6];
	
	Machine.fineStage.dConDist[7] = 0.25*Machine.fineStage.arrAxis[3].dConCalc+Machine.fineStage.arrAxis[4].dConCalc + Machine.fineStage.DAOffset[7];

/*
//X向
Machine.fineStage.dConDist[0] = 0.5*Machine.fineStage.arrAxis[0].dConCalc + 0.5 * Machine.fineStage.arrAxis[2].dConCalc + Machine.fineStage.DAOffset[0];
Machine.fineStage.dConDist[1] = 0.5*Machine.fineStage.arrAxis[0].dConCalc - 0.5 * Machine.fineStage.arrAxis[2].dConCalc + Machine.fineStage.DAOffset[1];
//Y向

Machine.fineStage.dConDist[2] = 0.5*Machine.fineStage.arrAxis[1].dConCalc+0.5*Machine.fineStage.arrAxis[2].dConCalc + Machine.fineStage.DAOffset[2];
Machine.fineStage.dConDist[3] = 0.5*Machine.fineStage.arrAxis[1].dConCalc-0.5*Machine.fineStage.arrAxis[2].dConCalc + Machine.fineStage.DAOffset[3];

Machine.fineStage.dConDist[2] = -1 * Machine.fineStage.dConDist[2];
Machine.fineStage.dConDist[3] = -1 * Machine.fineStage.dConDist[3];



//	Z X向
Machine.fineStage.dConDist[4] = 0.25*Machine.fineStage.arrAxis[3].dConCalc+Machine.fineStage.arrAxis[5].dConCalc + Machine.fineStage.DAOffset[4];
Machine.fineStage.dConDist[5] = 0.25*Machine.fineStage.arrAxis[3].dConCalc-Machine.fineStage.arrAxis[4].dConCalc + Machine.fineStage.DAOffset[5];
//Z Y向
Machine.fineStage.dConDist[6] = 0.25*Machine.fineStage.arrAxis[3].dConCalc-Machine.fineStage.arrAxis[5].dConCalc + Machine.fineStage.DAOffset[6];

Machine.fineStage.dConDist[7] = 0.25*Machine.fineStage.arrAxis[3].dConCalc+Machine.fineStage.arrAxis[4].dConCalc + Machine.fineStage.DAOffset[7];
*/
//Machine.fineStage.dConDist[4] = -1 * Machine.fineStage.dConDist[4] ;
//Machine.fineStage.dConDist[5] = -1 * Machine.fineStage.dConDist[5] ;
//Machine.fineStage.dConDist[6] = -1 * Machine.fineStage.dConDist[6] ;
//Machine.fineStage.dConDist[7] = -1 * Machine.fineStage.dConDist[7] ;

Machine.fineStage.dConDist[0] = Machine.fineStage.arrAxis[0].dConCalc;
Machine.fineStage.dConDist[1] = Machine.fineStage.arrAxis[1].dConCalc;

Machine.fineStage.dConDist[2] = Machine.fineStage.arrAxis[2].dConCalc;
	
}

#define VMAX_OUTPUT_Counter 4.0
#define VMIN_OUTPUT_Counter -4.0


#define VMAX_OUTPUT_Fine 0.30
#define VMIN_OUTPUT_Fine -0.30

void ErrCheck()
{
	int i;


	//限幅
	//限幅策略有以下几种
	//0、最为严格，有超出最大值就全部开环，输出清零，此方案最安全，但最不方便控制调试
	//1、以粗动台，微动台为单位，当前超过最大值当前台子开环，输出清零	
	//2、以粗动台，微动台为单位，当前超过最大值清零
	//3、以轴为单位，当前超过最大值清零
	

	//如果不想必须开环才复位，请在此赋值
	Machine.coarseStage.iStageErrCheck=0;
	Machine.fineStage.iStageErrCheck=0;


	/*for(i=0;i<4;i++)
	{
		if(Machine.coarseStage.dConDist[i]>VMAX_OUTPUT_Counter)
		{
			Machine.coarseStage.iStageErrCheck = 1;
		}
		if(Machine.coarseStage.dConDist[i]<VMIN_OUTPUT_Counter)
		{
			Machine.coarseStage.iStageErrCheck = 1;
		}
		
	}	
	
	for(i=0;i<4;i++)
	{
		if(1==Machine.coarseStage.iStageErrCheck)
		{
			Machine.coarseStage.dConDist[i]=0;
		}
	}*/


	for(i=0;i<4;i++)
		{
			if(Machine.coarseStage.dConDist[i]>VMAX_OUTPUT_Counter)
			{
				Machine.coarseStage.dConDist[i ]= 0;
//				Machine.coarseStage.arrAxis[i].axisStatus = S_Aixs_Open;
			}
			if(Machine.coarseStage.dConDist[i]<VMIN_OUTPUT_Counter)
			{
				Machine.coarseStage.dConDist[i] = 0;
//				Machine.coarseStage.arrAxis[i].axisStatus = S_Aixs_Open;
			}			
		}	

	
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
	/*for(i=0;i<8;i++)
	{
		if(Machine.fineStage.dConDist[i]>VMAX_OUTPUT_Fine)
		{
			Machine.fineStage.iStageErrCheck= 1;
		}
		if(Machine.fineStage.dConDist[i]<VMIN_OUTPUT_Fine)
		{
			Machine.fineStage.iStageErrCheck = 1;
		}
		
	}	

	for(i=0;i<8;i++)
	{
		if(1==Machine.fineStage.iStageErrCheck)
		{
			Machine.fineStage.dConDist[i]=0;
		}
	}*/


    for(i=0;i<8;i++)
	{
		if(Machine.fineStage.dConDist[i]>VMAX_OUTPUT_Fine)
		{
			Machine.fineStage.dConDist[i]=VMAX_OUTPUT_Fine;
		}
		if(Machine.fineStage.dConDist[i]<VMIN_OUTPUT_Fine)
		{
			Machine.fineStage.dConDist[i]=VMIN_OUTPUT_Fine;
		}
		
	}






}
void Output()
{	

	int i = 0;
   /*Machine.fineStage.iDAC[4] = 0.8/20*0x8000+0x8000;
	
	Machine.fineStage.iDAC[5] = 0.4/20*0x8000+0x8000;
	Machine.fineStage.iDAC[6] = 0.5/20*0x8000+0x8000;
	Machine.fineStage.iDAC[7] = 0.5/20*0x8000+0x8000;
	

	
	Machine.fineStage.iDAC[0] = 7.0/20*0x8000+0x8000;
	//Machine.fineStage.iDAC[1] = 0.7/20*0x8000+0x8000;
	Machine.fineStage.iDAC[2] = 7.0/20*0x8000+0x8000;
	//Machine.fineStage.iDAC[3] = 1.1/20*0x8000+0x8000;*/	

	Machine.fineStage.iDAC[0] = Machine.fineStage.dConDist[0]/10*0x8000+0x8000;
	Machine.fineStage.iDAC[1] = Machine.fineStage.dConDist[1]/10*0x8000+0x8000;
	Machine.fineStage.iDAC[2] = Machine.fineStage.dConDist[2]/10*0x8000+0x8000;
   Machine.fineStage.iDAC[3] = Machine.fineStage.dConDist[3]/10*0x8000+0x8000;
	//yht
//    Machine.fineStage.iDAC[4] = Machine.fineStage.dConDist[4]/10*0x8000+0x8000;//设置跳线后，输出电流减少4倍
//	Machine.fineStage.iDAC[5] = Machine.fineStage.dConDist[5]/10*0x8000+0x8000;
//	Machine.fineStage.iDAC[6] = Machine.fineStage.dConDist[6]/10*0x8000+0x8000;
//	Machine.fineStage.iDAC[7] = Machine.fineStage.dConDist[7]/10*0x8000+0x8000;

	Machine.coarseStage.iDAC[0] = Machine.coarseStage.dConDist[0]/10*0x8000+0x8000;
	Machine.coarseStage.iDAC[1] = Machine.coarseStage.dConDist[1]/10*0x8000+0x8000;
	Machine.coarseStage.iDAC[2] = Machine.coarseStage.dConDist[2]/10*0x8000+0x8000;
	Machine.coarseStage.iDAC[3] = Machine.coarseStage.dConDist[3]/10*0x8000+0x8000;
//	Machine.coarseStage.iDAC[4] = Machine.coarseStage.dConDist[4]/10*0x8000+0x8000;
//	Machine.coarseStage.iDAC[5] = Machine.coarseStage.dConDist[5]/10*0x8000+0x8000;
//	Machine.coarseStage.iDAC[6] = Machine.coarseStage.dConDist[6]/10*0x8000+0x8000;
//	Machine.coarseStage.iDAC[7] = Machine.coarseStage.dConDist[7]/10*0x8000+0x8000;

	for(i=0;i<4;i++)
	{
		Machine.coarseStage.iDAC[i] = (Machine.coarseStage.iDAC[i])|(0x00000+i*0x10000);
		AIO_Write_Local3202(OUT_DATA_BUFF, Machine.coarseStage.iDAC[i]);
	}

	for(i=0;i<8;i++)
	{	
		Machine.fineStage.iDAC[i] = (Machine.fineStage.iDAC[i])|(0x00000+i*0x10000);
		AIO_Write_Local32(OUT_DATA_BUFF, Machine.fineStage.iDAC[i]);
	}

	SetRegisterBit(BCR, 11, 1);		
//	SetRegisterBit02(BCR, 11, 1);	
}



void WriteDA()
{
	int samplenum = 16;
	setupaio();
	PMCModeSet();
	SetRegisterBit(IN_DATA_CNTRL, 15, 1);
	AIO_Write_Local32(IN_DATA_CNTRL, 0x8000|(int)samplenum);
	AIO_Write_Local32(OUT_DATA_BUFF, 40000);
	SetRegisterBit(BCR, 11, 1);	
}

void RecordData()
{
	int i = 0;
	node* q=NULL;
	q=head;
	if(1==recordStruct.iRecordFlag)
	{
	if(memoryStype==1)
	{
		*(recordStruct.pRecordData[0]+recordStruct.iRecordCnt) = (int)Machine.coarseStage.arrAxis[0].dSolveMethodOne;
		*(recordStruct.pRecordData[1]+recordStruct.iRecordCnt) = (int)Machine.coarseStage.arrAxis[0].fVel;
		*(recordStruct.pRecordData[2]+recordStruct.iRecordCnt) = (int)Machine.coarseStage.arrAxis[0].dSetPoint;

		*(recordStruct.pRecordData[3]+recordStruct.iRecordCnt) = (int)Machine.coarseStage.arrAxis[0].vel;
		*(recordStruct.pRecordData[4]+recordStruct.iRecordCnt) = (int)Machine.coarseStage.arrAxis[1].dActPos ;
		*(recordStruct.pRecordData[5]+recordStruct.iRecordCnt) = (int)Machine.coarseStage.arrAxis[1].fVel;


		*(recordStruct.pRecordData[6]+recordStruct.iRecordCnt)=(int)Machine.coarseStage.arrAxis[1].dSetPoint;
		*(recordStruct.pRecordData[7]+recordStruct.iRecordCnt)=(int)Machine.coarseStage.arrAxis[1].vel;
		*(recordStruct.pRecordData[8]+recordStruct.iRecordCnt)=(int)(Machine.coarseStage.dConDist[0]);
		*(recordStruct.pRecordData[9]+recordStruct.iRecordCnt)=(int)(10000*Machine.fineStage.arrAxis[3].dConCalc);
		*(recordStruct.pRecordData[10]+recordStruct.iRecordCnt)=(int)(10000*Machine.fineStage.arrAxis[4].dConCalc);
		*(recordStruct.pRecordData[11]+recordStruct.iRecordCnt)=(int)(10000*Machine.fineStage.arrAxis[5].dConCalc);

		*(recordStruct.pRecordData[12]+recordStruct.iRecordCnt)=(int)Machine.dIdentData;


		//*(recordStruct.pRecordData[7]+recordStruct.iRecordCnt)=(int)Machine.fineStage.arrAxis[1].dError;			
		//*(recordStruct.pRecordData[8]+recordStruct.iRecordCnt)=(int)Machine.fineStage.arrAxis[2].dError;

		*(recordStruct.pRecordData[14]+recordStruct.iRecordCnt)=(int)Machine.fineStage.arrAxis[3].dError;	
		*(recordStruct.pRecordData[13]+recordStruct.iRecordCnt)=(int)Machine.fineStage.arrAxis[4].dError;			
		*(recordStruct.pRecordData[15]+recordStruct.iRecordCnt)=(int)Machine.fineStage.arrAxis[5].dError;
	}
	else
	{
		while(q!=NULL)
		{
			if(q->data == &Machine.fineStage.arrAxis[0].dConCalc || q->data == &Machine.fineStage.arrAxis[1].dConCalc || q->data == &Machine.fineStage.arrAxis[2].dConCalc|| q->data == &Machine.fineStage.arrAxis[3].dConCalc|| q->data == &Machine.fineStage.arrAxis[4].dConCalc|| q->data == &Machine.fineStage.arrAxis[5].dConCalc)
				*(recordStruct.pRecordData[i] + recordStruct.iRecordCnt) = (int)((*(q->data))*10000);
			else
				*(recordStruct.pRecordData[i] + recordStruct.iRecordCnt) = (int)(*(q->data));
			q=q->next;
			i++;
		}
	}
/*		
		//控制器输出
		*(recordStruct.pRecordData[9]+recordStruct.iRecordCnt)=	Machine.fineStage.iLaser[0];
		*(recordStruct.pRecordData[10]+recordStruct.iRecordCnt)=Machine.fineStage.iLaser[1];
		*(recordStruct.pRecordData[11]+recordStruct.iRecordCnt)=Machine.fineStage.iLaser[2];
		//*(recordStruct.pRecordData[12]+recordStruct.iRecordCnt)=Machine.fineStage.iLaserAfterFilter[1];
		*(recordStruct.pRecordData[12]+recordStruct.iRecordCnt)=(Machine.fineStage.iLaser[0]+Machine.fineStage.iLaser[2])/2- Machine.fineStage.iLaser[1];
		*(recordStruct.pRecordData[13]+recordStruct.iRecordCnt)=Machine.fineStage.iLaser[4];
		*(recordStruct.pRecordData[14]+recordStruct.iRecordCnt)=Machine.fineStage.iLaser[5];
		*(recordStruct.pRecordData[15]+recordStruct.iRecordCnt)=Machine.fineStage.iLaser[6];*/

		
		recordStruct.iRecordCnt++;


		if (recordStruct.iRecordCnt >= RECORD_NUMBER)
		{	
			recordStruct.iRecordFlag = 0;
			recordStruct.iRecordCnt = 0;
			printf("Record finish!!");
		}
	}
}



void TmpDataFillIn()
{
	comData.axisStatus[0]=Machine.coarseStage.arrAxis[0].axisStatus;
	comData.axisStatus[1]=Machine.coarseStage.arrAxis[1].axisStatus;
	comData.axisStatus[2]=Machine.coarseStage.arrAxis[2].axisStatus;
	comData.axisStatus[3]=Machine.coarseStage.arrAxis[3].axisStatus;
	comData.axisStatus[4]=Machine.fineStage.arrAxis[0].axisStatus;
	comData.axisStatus[5]=Machine.fineStage.arrAxis[1].axisStatus;
	comData.axisStatus[6]=Machine.fineStage.arrAxis[2].axisStatus;
	comData.axisStatus[7]=Machine.fineStage.arrAxis[3].axisStatus;
	comData.axisStatus[8]=Machine.fineStage.arrAxis[4].axisStatus;
	comData.axisStatus[9]=Machine.fineStage.arrAxis[5].axisStatus;

	comData.conDist[0]=Machine.coarseStage.dConDist[0];
	comData.conDist[1]=Machine.coarseStage.dConDist[1];
	comData.conDist[2]=Machine.coarseStage.dConDist[2];
	comData.conDist[3]=Machine.coarseStage.dConDist[3];
	comData.conDist[4]=Machine.fineStage.dConDist[0];
	comData.conDist[5]=Machine.fineStage.dConDist[1];
	comData.conDist[6]=Machine.fineStage.dConDist[2];
	comData.conDist[7]=Machine.fineStage.dConDist[3];
	comData.conDist[8]=Machine.fineStage.dConDist[4];
	comData.conDist[9]=Machine.fineStage.dConDist[5];

	comData.Error[0]=Machine.coarseStage.arrAxis[0].dError;
	comData.Error[1]=Machine.coarseStage.arrAxis[1].dError;
	comData.Error[2]=Machine.coarseStage.arrAxis[2].dError;
	comData.Error[3]=Machine.coarseStage.arrAxis[3].dError;
	comData.Error[4]=Machine.fineStage.arrAxis[4].dError;
	comData.Error[5]=Machine.fineStage.arrAxis[5].dError;
	comData.Error[6]=Machine.fineStage.arrAxis[6].dError;
	comData.Error[7]=Machine.fineStage.arrAxis[7].dError;
	comData.Error[8]=Machine.fineStage.arrAxis[8].dError;
	comData.Error[9]=Machine.fineStage.arrAxis[9].dError;

	comData.feedPosition[0]=Machine.coarseStage.arrAxis[0].dActPos;
	comData.feedPosition[1]=Machine.coarseStage.arrAxis[1].dActPos;
	comData.feedPosition[2]=Machine.coarseStage.arrAxis[2].dActPos;
	comData.feedPosition[3]=Machine.coarseStage.arrAxis[3].dActPos;
	comData.feedPosition[4]=Machine.fineStage.arrAxis[0].dActPos;
	comData.feedPosition[5]=Machine.fineStage.arrAxis[1].dActPos;
	comData.feedPosition[6]=Machine.fineStage.arrAxis[2].dActPos;
	comData.feedPosition[7]=Machine.fineStage.arrAxis[3].dActPos;
	comData.feedPosition[8]=Machine.fineStage.arrAxis[4].dActPos;
	comData.feedPosition[9]=Machine.fineStage.arrAxis[5].dActPos;

	comData.feedVel[0]=Machine.coarseStage.arrAxis[0].fVel;
	comData.feedVel[1]=Machine.coarseStage.arrAxis[1].fVel;
	comData.feedVel[2]=Machine.coarseStage.arrAxis[2].fVel;
	comData.feedVel[3]=Machine.coarseStage.arrAxis[3].fVel;
	comData.feedVel[4]=Machine.fineStage.arrAxis[0].fVel;
	comData.feedVel[5]=Machine.fineStage.arrAxis[1].fVel;
	comData.feedVel[6]=Machine.fineStage.arrAxis[2].fVel;
	comData.feedVel[7]=Machine.fineStage.arrAxis[3].fVel;
	comData.feedVel[8]=Machine.fineStage.arrAxis[4].fVel;
	comData.feedVel[9]=Machine.fineStage.arrAxis[5].fVel;

	comData.refPosition[0]=Machine.coarseStage.arrAxis[0].dSetPoint;
	comData.refPosition[1]=Machine.coarseStage.arrAxis[1].dSetPoint;
	comData.refPosition[2]=Machine.coarseStage.arrAxis[2].dSetPoint;
	comData.refPosition[3]=Machine.coarseStage.arrAxis[3].dSetPoint;
	comData.refPosition[4]=Machine.fineStage.arrAxis[0].dSetPoint;
	comData.refPosition[5]=Machine.fineStage.arrAxis[1].dSetPoint;
	comData.refPosition[6]=Machine.fineStage.arrAxis[2].dSetPoint;
	comData.refPosition[7]=Machine.fineStage.arrAxis[3].dSetPoint;
	comData.refPosition[8]=Machine.fineStage.arrAxis[4].dSetPoint;
	comData.refPosition[9]=Machine.fineStage.arrAxis[5].dSetPoint;

	comData.refVel[0]=Machine.coarseStage.arrAxis[0].vel;
	comData.refVel[1]=Machine.coarseStage.arrAxis[1].vel;
	comData.refVel[2]=Machine.coarseStage.arrAxis[2].vel;
	comData.refVel[3]=Machine.coarseStage.arrAxis[3].vel;
	comData.refVel[4]=Machine.fineStage.arrAxis[0].vel;
	comData.refVel[5]=Machine.fineStage.arrAxis[1].vel;
	comData.refVel[6]=Machine.fineStage.arrAxis[2].vel;
	comData.refVel[7]=Machine.fineStage.arrAxis[3].vel;
	comData.refVel[8]=Machine.fineStage.arrAxis[4].vel;
	comData.refVel[9]=Machine.fineStage.arrAxis[5].vel;

	comData.refAcc[0]=Machine.coarseStage.arrAxis[0].acc;
	comData.refAcc[1]=Machine.coarseStage.arrAxis[1].acc;
	comData.refAcc[2]=Machine.coarseStage.arrAxis[2].acc;
	comData.refAcc[3]=Machine.coarseStage.arrAxis[3].acc;
	comData.refAcc[4]=Machine.fineStage.arrAxis[0].acc;
	comData.refAcc[5]=Machine.fineStage.arrAxis[1].acc;
	comData.refAcc[6]=Machine.fineStage.arrAxis[2].acc;
	comData.refAcc[7]=Machine.fineStage.arrAxis[3].acc;
	comData.refAcc[8]=Machine.fineStage.arrAxis[4].acc;
	comData.refAcc[9]=Machine.fineStage.arrAxis[5].acc;

	comData.motorState[0]=Machine.coarseStage.arrAxis[0].flag;
	comData.motorState[1]=Machine.coarseStage.arrAxis[1].flag;
	comData.motorState[2]=Machine.coarseStage.arrAxis[2].flag;
	comData.motorState[3]=Machine.coarseStage.arrAxis[3].flag;

	
	//界面显示从50加起
	comData.iCMD=1;
	comData.iFineData[0]=2;
	comData.iCoarseData[0]=Machine.iServoCnt*2;

	comData.iCoarseData[5] = Machine.iExecTime;
	comData.iCoarseData[6] = recordStruct.iSaveCnt;	
	comData.iCoarseData[7] = recordStruct.iSaveFlag;

	comData.iCoarseData[8] = recordStruct.iRecordCnt;
	comData.iCoarseData[9] = recordStruct.iRecordFlag;

	comData.iCoarseData[10] = Machine.coarseStage.iCounter[0];
	comData.iCoarseData[11] = Machine.coarseStage.iCounter[1];
	comData.iCoarseData[12] = Machine.coarseStage.iCounter[2];
	comData.iCoarseData[13] = Machine.coarseStage.iCounter[3];

	comData.iCoarseData[14] = Machine.coarseStage.arrAxis[0].controllerType;
	comData.iCoarseData[15] = Machine.coarseStage.arrAxis[1].controllerType;
	comData.iCoarseData[16] = Machine.coarseStage.arrAxis[2].controllerType;
	comData.iCoarseData[17] = Machine.coarseStage.arrAxis[3].controllerType;

	comData.iCoarseData[18] = Machine.coarseStage.arrAxis[0].bHome;
	comData.iCoarseData[19] = Machine.coarseStage.arrAxis[1].bHome;
	comData.iCoarseData[20] = Machine.coarseStage.arrAxis[2].bHome;
	comData.iCoarseData[21] = Machine.coarseStage.arrAxis[3].bHome;

//yht
	//粗轴状态
	comData.iCoarseData[26] = Machine.coarseStage.arrAxis[0].axisStatus;	
	comData.iCoarseData[27] = Machine.coarseStage.arrAxis[1].axisStatus;	
	comData.iCoarseData[28] = Machine.coarseStage.arrAxis[2].axisStatus;	
	comData.iCoarseData[29] = Machine.coarseStage.arrAxis[3].axisStatus;	
	comData.iCoarseData[30] = Machine.coarseStage.DDX_Flag;
	comData.iCoarseData[31] = Machine.coarseStage.DDY_Flag;
		//粗计数
	comData.iCoarseData[34] = Machine.coarseStage.iCounter[0];	
	comData.iCoarseData[35] = Machine.coarseStage.iCounter[1];	
	comData.iCoarseData[36] = Machine.coarseStage.iCounter[2];
	comData.iCoarseData[37] = Machine.coarseStage.iCounter[3];	
	comData.iCoarseData[38] = Machine.coarseStage.stageStatus;	
	comData.iCoarseData[39] = Machine.fineStage.stageStatus;
	
	comData.iCoarseData[40] = Machine.coarseStage.arrAxis[0].closeFlag;
	comData.iCoarseData[41] = Machine.coarseStage.arrAxis[1].closeFlag;
	comData.iCoarseData[42] = Machine.coarseStage.arrAxis[2].closeFlag;
	comData.iCoarseData[43] = Machine.coarseStage.arrAxis[3].closeFlag;
		//交流激光光强
    comData.iFineData[4] = Machine.fineStage.laserACPowerLevel[0];
	comData.iFineData[5] = Machine.fineStage.laserACPowerLevel[1];
	comData.iFineData[6] = Machine.fineStage.laserACPowerLevel[2];
	comData.iFineData[7] = Machine.fineStage.laserACPowerLevel[3];
	comData.iFineData[8] = Machine.fineStage.laserACPowerLevel[4];
	comData.iFineData[9] = Machine.fineStage.laserACPowerLevel[5];
	comData.iFineData[10] = Machine.fineStage.laserACPowerLevel[6];
	comData.iFineData[11] = Machine.fineStage.laserACPowerLevel[7];
	comData.iFineData[12] = Machine.fineStage.laserACPowerLevel[8];
	comData.iFineData[13] = Machine.fineStage.laserACPowerLevel[9];
	comData.iFineData[14] = Machine.fineStage.laserACPowerLevel[10];

	//直流激光光强
    comData.iFineData[15] = Machine.fineStage.laserDCPowerLevel[0];
	comData.iFineData[16] = Machine.fineStage.laserDCPowerLevel[1];
	comData.iFineData[17] = Machine.fineStage.laserDCPowerLevel[2];
	comData.iFineData[18] = Machine.fineStage.laserDCPowerLevel[3];
	comData.iFineData[19] = Machine.fineStage.laserDCPowerLevel[4];
	comData.iFineData[20] = Machine.fineStage.laserDCPowerLevel[5];
	comData.iFineData[21] = Machine.fineStage.laserDCPowerLevel[6];
	comData.iFineData[22] = Machine.fineStage.laserDCPowerLevel[7];
	comData.iFineData[23] = Machine.fineStage.laserDCPowerLevel[8];
	comData.iFineData[24] = Machine.fineStage.laserDCPowerLevel[9];
	comData.iFineData[25] = Machine.fineStage.laserDCPowerLevel[10];
	
	//激光读数
	comData.iFineData[27] = Machine.fineStage.iLaser[0];
	comData.iFineData[28] = Machine.fineStage.iLaser[1];
	comData.iFineData[29] = Machine.fineStage.iLaser[2];
	comData.iFineData[30] = Machine.fineStage.iLaser[3];
	comData.iFineData[31] = Machine.fineStage.iLaser[4];
	comData.iFineData[32] = Machine.fineStage.iLaser[5];
	comData.iFineData[33] = Machine.fineStage.iLaser[6];
	comData.iFineData[34] = Machine.fineStage.iLaser[7];
	comData.iFineData[35] = Machine.fineStage.iLaser[8];
	comData.iFineData[36] = Machine.fineStage.iLaser[9];
//	comData.iFineData[37] = Machine.fineStage.iLaser[10];
//	comData.iFineData[38] = Machine.fineStage.iLaser[11];
//	comData.iFineData[39] = Machine.fineStage.iLaser[12];


	comData.dFineData[0] = (int)Machine.fineStage.arrAxis[0].dActPos;
	comData.dFineData[3] = (int)Machine.fineStage.arrAxis[0].dSetPoint;
	comData.dFineData[6] = (int)Machine.fineStage.arrAxis[0].dError;


	comData.dFineData[1] = (int)Machine.fineStage.arrAxis[1].dActPos;
	comData.dFineData[4] = (int)Machine.fineStage.arrAxis[1].dSetPoint;
	comData.dFineData[7] = (int)Machine.fineStage.arrAxis[1].dError;

	comData.dFineData[2] = (int)Machine.fineStage.arrAxis[2].dActPos;
	comData.dFineData[5] = (int)Machine.fineStage.arrAxis[2].dSetPoint;
	comData.dFineData[8] = (int)Machine.fineStage.arrAxis[2].dError;

	comData.dFineData[38] = (int)Machine.fineStage.arrAxis[3].dActPos;
	comData.dFineData[41] = (int)Machine.fineStage.arrAxis[3].dSetPoint;
	comData.dFineData[44] = (int)Machine.fineStage.arrAxis[3].dError;

	comData.dFineData[39] = (int)Machine.fineStage.arrAxis[4].dActPos;
	comData.dFineData[42] = (int)Machine.fineStage.arrAxis[4].dSetPoint;
	comData.dFineData[45] = (int)Machine.fineStage.arrAxis[4].dError;

	comData.dFineData[40] = (int)Machine.fineStage.arrAxis[5].dActPos;
	comData.dFineData[43] = (int)Machine.fineStage.arrAxis[5].dSetPoint;
	comData.dFineData[46] = (int)Machine.fineStage.arrAxis[5].dError;

	//微控制器
	comData.dFineData[9] = Machine.fineStage.arrAxis[0].dConCalc;
	comData.dFineData[10] = Machine.fineStage.arrAxis[1].dConCalc;	
	comData.dFineData[11] = Machine.fineStage.arrAxis[2].dConCalc;
	comData.dFineData[47] = Machine.fineStage.arrAxis[3].dConCalc;
	comData.dFineData[48] = Machine.fineStage.arrAxis[4].dConCalc;	
	comData.dFineData[49] = Machine.fineStage.arrAxis[5].dConCalc;


	//微轴状态
	comData.dFineData[12] = Machine.fineStage.arrAxis[0].axisStatus;
	comData.dFineData[13] = Machine.fineStage.arrAxis[1].axisStatus;	
	comData.dFineData[14] = Machine.fineStage.arrAxis[2].axisStatus;
	comData.dFineData[50] = Machine.fineStage.arrAxis[3].axisStatus;
	comData.dFineData[51] = Machine.fineStage.arrAxis[4].axisStatus;	
	comData.dFineData[52] = Machine.fineStage.arrAxis[5].axisStatus;


	//微DA
	comData.dFineData[15] = Machine.fineStage.dConDist[0];	
	comData.dFineData[16] = Machine.fineStage.dConDist[1];	
	comData.dFineData[17] = Machine.fineStage.dConDist[2];	
	comData.dFineData[18] = Machine.fineStage.dConDist[3];	

	comData.dFineData[70] = Machine.fineStage.dConDist[4];	//yht
	comData.dFineData[71] = Machine.fineStage.dConDist[5];//yht
	comData.dFineData[72] = Machine.fineStage.dConDist[6];	//yht
	comData.dFineData[73] = Machine.fineStage.dConDist[7];//yht

	//微计数
	comData.dFineData[19] = Machine.fineStage.iCounter[0];	
	comData.dFineData[20] = Machine.fineStage.iCounter[1];	
	comData.dFineData[21] = Machine.fineStage.iCounter[2];	
	comData.dFineData[22] = Machine.fineStage.iCounter[3];	
	comData.dFineData[68] = Machine.fineStage.iCounter[4];	//yht
	comData.dFineData[69] = Machine.fineStage.iCounter[5];	//yht
	//加速度前馈
	comData.dFineData[23] = Machine.fineStage.arrAxis[0].dAffData;	
	comData.dFineData[24] = Machine.fineStage.arrAxis[1].dAffData;		
	comData.dFineData[25] = Machine.fineStage.arrAxis[2].dAffData;	
	comData.dFineData[53] = Machine.fineStage.arrAxis[3].dAffData;	
	comData.dFineData[54] = Machine.fineStage.arrAxis[4].dAffData;		
	comData.dFineData[55] = Machine.fineStage.arrAxis[5].dAffData;	

	//切换反馈
	
	comData.dFineData[26] = (int)Machine.fineStage.arrAxis[0].dActPosNotInUse;	
	comData.dFineData[27] = (int)Machine.fineStage.arrAxis[1].dActPosNotInUse;		
	comData.dFineData[28] = (int)Machine.fineStage.arrAxis[2].dActPosNotInUse;
	comData.dFineData[56] = (int)Machine.fineStage.arrAxis[3].dActPosNotInUse;	
	comData.dFineData[57] = (int)Machine.fineStage.arrAxis[4].dActPosNotInUse;		
	comData.dFineData[58] = (int)Machine.fineStage.arrAxis[5].dActPosNotInUse;

	comData.dFineData[29] = (int)Machine.fineStage.arrAxis[0].posInfor.dActPos;	
	comData.dFineData[30] = (int)Machine.fineStage.arrAxis[1].posInfor.dActPos;		
	comData.dFineData[31] = (int)Machine.fineStage.arrAxis[2].posInfor.dActPos;	
	comData.dFineData[59] = (int)Machine.fineStage.arrAxis[3].posInfor.dActPos;	
	comData.dFineData[60] = (int)Machine.fineStage.arrAxis[4].posInfor.dActPos;		
	comData.dFineData[61] = (int)Machine.fineStage.arrAxis[5].posInfor.dActPos;	

	comData.dFineData[32] = (int)Machine.fineStage.arrAxis[0].posInfor.dSetPoint; 
	comData.dFineData[33] = (int)Machine.fineStage.arrAxis[1].posInfor.dSetPoint; 	
	comData.dFineData[34] = (int)Machine.fineStage.arrAxis[2].posInfor.dSetPoint; 
	comData.dFineData[62] = (int)Machine.fineStage.arrAxis[3].posInfor.dSetPoint; 
	comData.dFineData[63] = (int)Machine.fineStage.arrAxis[4].posInfor.dSetPoint; 	
	comData.dFineData[64] = (int)Machine.fineStage.arrAxis[5].posInfor.dSetPoint;

	comData.dFineData[35] = (int)Machine.fineStage.arrAxis[0].posInfor.dError; 
	comData.dFineData[36] = (int)Machine.fineStage.arrAxis[1].posInfor.dError; 	
	comData.dFineData[37] = (int)Machine.fineStage.arrAxis[2].posInfor.dError; 
	comData.dFineData[65] = (int)Machine.fineStage.arrAxis[3].posInfor.dError; 
	comData.dFineData[66] = (int)Machine.fineStage.arrAxis[4].posInfor.dError; 	
	comData.dFineData[67] = (int)Machine.fineStage.arrAxis[5].posInfor.dError; 

	comData.dFineData[74] = Machine.fineStage.arrAxis[0].acc;
	comData.dFineData[75] = Machine.fineStage.arrAxis[1].acc;
	comData.dFineData[76] = Machine.fineStage.arrAxis[2].acc;
	comData.dFineData[77] = Machine.fineStage.arrAxis[3].acc;
	comData.dFineData[78] = Machine.fineStage.arrAxis[4].acc;
	comData.dFineData[79] = Machine.fineStage.arrAxis[5].acc;

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////

	
	//粗反馈   界面显示从180加起
	comData.dCoarseData[0] = Machine.coarseStage.arrAxis[0].dActPos;//(int)head->data;	
	comData.dCoarseData[1] = Machine.coarseStage.arrAxis[1].dActPos;//*(head->data);	
	comData.dCoarseData[2] = Machine.coarseStage.arrAxis[2].dActPos;//(int) tail->data;	
	comData.dCoarseData[3] = Machine.coarseStage.arrAxis[3].dActPos;//*(tail->data);	
	comData.dCoarseData[50] = Machine.coarseStage.arrAxis[4].dActPos;
	comData.dCoarseData[51] = Machine.coarseStage.arrAxis[5].dActPos;
	//粗指令	
	comData.dCoarseData[4] = Machine.coarseStage.arrAxis[0].dSetPoint;	
	comData.dCoarseData[5] = Machine.coarseStage.arrAxis[1].dSetPoint;	
	comData.dCoarseData[6] = Machine.coarseStage.arrAxis[2].dSetPoint;	
	comData.dCoarseData[7] = Machine.coarseStage.arrAxis[3].dSetPoint;

	//粗误差	
	comData.dCoarseData[8] = Machine.coarseStage.arrAxis[0].dError;	
	comData.dCoarseData[9] = Machine.coarseStage.arrAxis[1].dError;	
	comData.dCoarseData[10] = Machine.coarseStage.arrAxis[2].dError;	
	comData.dCoarseData[11] = Machine.coarseStage.arrAxis[3].dError;	

	//粗控制器
	comData.dCoarseData[12] = Machine.coarseStage.arrAxis[0].dConCalc;	
	comData.dCoarseData[13] = Machine.coarseStage.arrAxis[1].dConCalc;	
	comData.dCoarseData[14] = Machine.coarseStage.arrAxis[2].dConCalc;	
	comData.dCoarseData[15] = Machine.coarseStage.arrAxis[3].dConCalc;	

	//粗DA
	comData.dCoarseData[16] = Machine.coarseStage.dConDist[0];	
	comData.dCoarseData[17] = Machine.coarseStage.dConDist[1];	
	comData.dCoarseData[18] = Machine.coarseStage.dConDist[2];	
	comData.dCoarseData[19] = Machine.coarseStage.dConDist[3];	

	comData.dCoarseData[20] = Machine.coarseStage.arrAxis[0].vel/1000000;
	comData.dCoarseData[21] = Machine.coarseStage.arrAxis[1].vel/1000000;
	comData.dCoarseData[22] = Machine.coarseStage.arrAxis[2].vel/1000000;
	comData.dCoarseData[23] = Machine.coarseStage.arrAxis[3].vel/1000000;
	
	comData.dCoarseData[24] = Machine.coarseStage.arrAxis[0].fVel/1000000;
	comData.dCoarseData[25] = Machine.coarseStage.arrAxis[1].fVel/1000000;
	comData.dCoarseData[26] = Machine.coarseStage.arrAxis[2].fVel/1000000;
	comData.dCoarseData[27] = Machine.coarseStage.arrAxis[3].fVel/1000000;
  
	comData.dCoarseData[28] = Machine.coarseStage.arrAxis[0].fAcc/1000000;
	comData.dCoarseData[29] = Machine.coarseStage.arrAxis[1].fAcc/1000000;
	comData.dCoarseData[30] = Machine.coarseStage.arrAxis[2].fAcc/1000000;
	comData.dCoarseData[31] = Machine.coarseStage.arrAxis[3].fAcc/1000000;

	comData.dCoarseData[32] = Machine.coarseStage.arrAxis[0].stepHome;
	comData.dCoarseData[33] = Machine.coarseStage.arrAxis[2].stepHome;

	comData.dCoarseData[34] = Machine.coarseStage.arrAxis[0].bIntrEnable ;	//底轴1 Q限位采样home
	comData.dCoarseData[35] = Machine.coarseStage.arrAxis[2].bIntrEnable;	//底轴1 P限位采样

	comData.dCoarseData[36] =  Machine.stepFlagYQ;		//YQ限位采样home
	comData.dCoarseData[37] = Machine.stepFlagYP;		//Y諴限位采样

	comData.dCoarseData[38] =  Machine.stepFlagXQ;		//YQ限位采样home
	comData.dCoarseData[39] = Machine.stepFlagXP;		//Y諴限位采样
}

void CountForNetSend()
{
	static int s_iCountForNetSend=0;
	s_iCountForNetSend++;
	if(s_iCountForNetSend>999)
	{
		TmpDataFillIn();
		semGive(TransDataToWin_SemId);
		s_iCountForNetSend=0;
	}
}



/*void SysIdent(int iCmd )
{
	static int s_iTemp[6]={0,0,0,0,0,0};

	if(s_iTemp[iCmd]<100000)
	{

		Machine.dIdentData = dIdentBuff[s_iTemp[iCmd]];
		Machine.coarseStage.arrAxis[iCmd].dConCalc=Machine.coarseStage.arrAxis[iCmd].dConCalc + Machine.coarseStage.arrAxis[iCmd].dIdentGain * Machine.dIdentData;
		s_iTemp[iCmd]++;
		//logMsg("value1=%x,value2=%x\n",6,6,0,0,0,0);
	}
	else
	{
		Machine.coarseStage.arrAxis[iCmd].bIdent=0;
		Machine.dIdentData=0;
		s_iTemp[iCmd]=0;
		logMsg("value1=%x,value2=%x\n",7,7,0,0,0,0);
	}
}*/

void SysIdent(int iCmd )
{
	static int s_iTemp[6]={0,0,0,0,0,0};

	if(s_iTemp[iCmd]<100000)
	{
		Machine.dIdentData = dIdentBuff[s_iTemp[iCmd]];
		Machine.fineStage.arrAxis[iCmd].dConCalc = Machine.fineStage.arrAxis[iCmd].dConCalc + Machine.fineStage.arrAxis[iCmd].dIdentGain * Machine.dIdentData;
		s_iTemp[iCmd]++;
	}
	else
	{
		Machine.fineStage.arrAxis[iCmd].bIdent=0;
		Machine.dIdentData=0;
		s_iTemp[iCmd]=0;
		logMsg("value1=%x,value2=%x\n",7,7,0,0,0,0);
	}
}

void trajUpdate(Axis *pAxis)
{
	
	s_TrajParam_4_Order_Poly *tpRun;
	if(pAxis->trajUpdate == 1)
	{
		TP.p = pAxis->targetPos;
		TP.v = pAxis->targetVel;
		TP.a = pAxis->targetAcc;
		TP.j = pAxis->targetJerk;
		TP.d = pAxis->targetSnap;
		TP.Ts = 0.0002;
		TP.r =15;
		TP.s = 1e-9;
		if(GET_MOTOR_MOVING_FLAG(pAxis->flag) == 0)
		{
			TPGen_4Order_P2P(&TP,&(pAxis->sTP_Run));		
		}
		pAxis->trajUpdate = 0;
	}	
}

void ServoCycle()
{
	int i=0;
	while(1)
	{
		semTake(servotimeSemId,WAIT_FOREVER);

		sysTimestampEnable();
	   	Machine.iTime1= sysTimestamp();
	    Machine.iFreq = sysTimestampFreq();
		
		GetFeedBack();
		PosSolve();
		for(i=0;i<4;i++)
		{
			AxisStateMachine(&Machine.coarseStage.arrAxis[i],i);
		}
		for(i=0;i<6;i++)
		{
			AxisStateMachine(&Machine.fineStage.arrAxis[i],i);
		}

		StageStateMachine(&Machine.coarseStage);
		StageStateMachine(&Machine.fineStage);
		ConCalc();


		for(i=0;i<6;i++)
		{
			if(Machine.fineStage.arrAxis[i].bIdent==1)
			{
				SysIdent(i);
			}
		}
		ConDist();
//		ErrCheck();
//		Output();
//		for(i=0;i<4;i++)
//		{
//			Machine.coarseStage.arrAxis[i].dActPos = LLCCalculate(&(Machine.coarseStage.arrAxis[i].controlObject),Machine.coarseStage.dConDist[i]);
//		}

		Machine.iServoCnt++;
		if(Machine.iServoCnt>50000)
		{
			Machine.iServoCnt = 0;
//			printf("%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",Machine.coarseStage.arrAxis[0].controlObject.pA[1],
//			Machine.coarseStage.arrAxis[0].controlObject.pA[2],Machine.coarseStage.arrAxis[0].controlObject.pA[3],Machine.coarseStage.arrAxis[0].controlObject.pA[4],
//			Machine.coarseStage.arrAxis[0].controlObject.pA[5],Machine.coarseStage.arrAxis[0].controlObject.pB[1],
//			Machine.coarseStage.arrAxis[0].controlObject.pB[2],Machine.coarseStage.arrAxis[0].controlObject.pB[3],Machine.coarseStage.arrAxis[0].controlObject.pB[4],
//			Machine.coarseStage.arrAxis[0].controlObject.pB[5]);
			printf("%lf,%lf,%lf,%lf\n",Machine.coarseStage.arrAxis[0].controlObject.pA[1],Machine.coarseStage.arrAxis[0].controlObject.pA[2],
			Machine.coarseStage.arrAxis[0].controlObject.pB[1],Machine.coarseStage.arrAxis[0].controlObject.pB[2]);
		}
		CountForNetSend();
		RecordData();

//		ast_M58_DIO();
//		ast_M58_Protect();
//		ast_M58_SoftProtect();

		Machine.iTime2= sysTimestamp();
   	 	sysTimestampDisable();
   	 	Machine.iExecTime=(Machine.iTime2-Machine.iTime1)/(Machine.iFreq/1000000);
	}
}

