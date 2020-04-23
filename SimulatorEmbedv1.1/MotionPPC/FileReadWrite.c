#include "FileReadWrite.h"
#include "stdio.h"
#include "StateMachine.h"
#include "TrajHandler.h"
#include "GetData.h"
#include "Trajplaner.h"

double dIdentBuff[BUFLENGTH]={0.0};
double ParamData[20]={0};
double PIDParam[20]={0};
double P2PData[12];
double  DirData[12]; 
double ExpoData[24];

void ReadIdentFile()
{
	
	FILE * pFile;
	int iLineLen[10]={0};
	pFile=fopen("IdentU_Dx5.txt","r");
	while(!feof(pFile))
	{
		fscanf(pFile,"%lf\n",(dIdentBuff+iLineLen[0]));		
		iLineLen[0]++;	
	}
	fclose(pFile);	
	logMsg("value1=%x,value2=%x\n",13,9,0,0,0,0);
}

void ReadParam()
{
	FILE* CoarseParamFile,FineParamFile;
	int i=0;
	CoarseParamFile = fopen("CoarseParamData.txt","r");
	if(CoarseParamFile==NULL)
	{
		printf("ParamFile open error\n");
		return;
	}
	while(!feof(CoarseParamFile))
	{
		fscanf(CoarseParamFile,"%lf\n",(ParamData+i));
		i++;
	}
	fclose(CoarseParamFile);
			
}

void ReadDAOffset()
{
	FILE* DAOffsetFile;
	int i=0;
	DAOffsetFile = fopen("DAOffset.txt","r");
	if(DAOffsetFile==NULL)
	{
		printf("DAOffsetFile open error\n");
		return;
	}
	while(!feof(DAOffsetFile))
	{
		if(i < 4)
		{
			fscanf(DAOffsetFile,"%f\n",(Machine.coarseStage.DAOffset+i));
		}
		else
		{
			fscanf(DAOffsetFile,"%f\n",(Machine.fineStage.DAOffset+i-4));
		}
		
		i++;
	}
	fclose(DAOffsetFile);
			
}
//extern CIRCLE circleParam;
void ReadCircleParam()
{
	FILE* CircleParamFile;
	int i=0;
	CircleParamFile = fopen("CircleParamData.txt","r");
	if(CircleParamFile==NULL)
	{
		printf("ParamFile open error\n");
		return;
	}
	while(!feof(CircleParamFile))
	{
		fscanf(CircleParamFile,"%d\n",(&circleParam.R + i));
		i++;
	}
	fclose(CircleParamFile);
//	printf("value=%lf\n,value1=%lf\n,value2=%lf\n,value3=%lf\n,value4=%lf\n,value5=%lf\n,value6=%lf\n,value7=%lf\n,
//		value8=%lf\n,value9=%lf\n,value10=%lf\n,value11=%lf\n,value12=%lf\n,value13=%lf\n,value14=%lf\n,value15=%lf\n",
//		ParamData[0],ParamData[1],ParamData[2],ParamData[3],ParamData[4],ParamData[5],ParamData[6],ParamData[7],
//		ParamData[8],ParamData[9],ParamData[10],ParamData[11],ParamData[12],ParamData[13],ParamData[14],ParamData[15]);
//		printf("%d\n%d\n",circleParam.R,circleParam.cirNum);	
}

void ReadDirParam()
{
	FILE * DirFile;
	int i=0;
	DirFile=fopen("Dir.txt","r");
	if (DirFile == NULL)
	{
		logMsg("Open Dir file failed \n",0,0,0,0,0,0);
		return ;
	}
	while(!feof(DirFile))
	{
		if(i < 6)
		{
			fscanf(DirFile,"%d\n",&(Machine.fineStage.arrAxis[i].iDir));
	//		printf("%d\n",Machine.fineStage.arrAxis[i].Dir);
		}
		else
		{
			fscanf(DirFile,"%d\n",&(Machine.fineStage.arrAxis[i].iLaserDir));
		}
		i++; 
	}
	fclose(DirFile);
}

void ReadRecordIndex()
{
	FILE * RecordIndexFile;
	int i=0;
	double value;
	RecordIndexFile=fopen("RecordIndex.txt","r");
	if (RecordIndexFile == NULL)
	{
		logMsg("Open RecordIndex file failed \n",0,0,0,0,0,0);
		return ;
	}
	while(!feof(RecordIndexFile))
	{
		if(i < 3)
		{
//			fscanf(RecordIndexFile,"%d\n",&(recvData.iReserved[i]));
//			printf("%d\n",recvData.iReserved[i]);
		}
		i++; 
	}
//	fclose(RecordIndexFile);
//	if(recvData.iReserved[1] == 50)
//	{
//		memoryStype=0;
//		recordMap[recvData.iReserved[1]].arg2 = recvData.iReserved[2];
	//	value = &Machine.coarseStage.arrAxis[0].dActPos;
	//	printf("%d\n",&Machine.coarseStage.arrAxis[0].dActPos);
	//	printf("%d\n",recordMap[recvData.iReserved[1]].arg1);
//		recordMap[recvData.iReserved[1]].func(recordMap[recvData.iReserved[1]].arg1,recordMap[recvData.iReserved[1]].arg2);
//	}
//	else
//	{
//		memoryStype=1;
//	}
}


void ReadLineParam(lineInterpolate_t *line)
{
	FILE* lineParamFile;
	int i=0;
	lineParamFile=fopen("lineParam.txt","r");
	
	if(lineParamFile==NULL)
	{
		logMsg("Open lineParamFile is failed\n",0,0,0,0,0,0);
		return;
	}
	while(!feof(lineParamFile))
	{
		fscanf(lineParamFile,"%lf\n",ParamData+i);
		i++;
	}
	fclose(lineParamFile);
	line->startPoint.X=Machine.coarseStage.arrAxis[0].dSetPoint;
	line->startPoint.Y=Machine.coarseStage.arrAxis[2].dSetPoint;
	line->endPoint.X=ParamData[0];
	line->endPoint.Y=ParamData[1];
	line->Ts=0.0002;
	line->vel=ParamData[2];
	printf("%lf\n""%lf\n""%lf\n",line->endPoint.X,line->endPoint.Y,line->vel);
}

void ReadArcParam(arcInterpolate_t *arc)
{
	FILE* arcParamFile;
	int i=0;
	arcParamFile=fopen("arcParam.txt","r");

	if(arcParamFile==NULL)
	{
		logMsg("Open arcParamFile is failed\n",0,0,0,0,0,0);
		return;
	}
	while(!feof(arcParamFile))
	{
		fscanf(arcParamFile,"%lf\n",ParamData+i);
		i++;
	}
	fclose(arcParamFile);
	arc->startPoint.X=Machine.coarseStage.arrAxis[0].dSetPoint;
	arc->startPoint.Y=Machine.coarseStage.arrAxis[2].dSetPoint;
	arc->endPoint.X=ParamData[0];
	arc->endPoint.Y=ParamData[1];
	arc->centerPoint.X=ParamData[2];
	arc->centerPoint.Y=ParamData[3];
	arc->Ts=0.0002;
	arc->vel=ParamData[4];
	arc->radius=ParamData[5];
	arc->dir=ParamData[6];
	printf("%lf\n""%lf\n""%lf\n""%lf\n""%lf\n""%lf\n""%d\n",arc->endPoint.X,arc->endPoint.Y,arc->centerPoint.X,arc->centerPoint.Y,arc->vel,arc->radius,arc->dir);
}


