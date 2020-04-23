
#ifndef RTCONTROL_H
#define RTCONTROL_H

#define CoarseResolution 100
#define FineResolution 20

extern void TangceFrom();

extern void ServoCycle();
extern void DataCollect(void);

typedef struct tagRecordStruct
{
	int *pRecordData[16];
	
	int iRecordFlag;
	int iRecordCnt;
	int iSaveFlag;
	int iSaveCnt;
	int iReadFlag;
	
}RecordStruct;


RecordStruct recordStruct;






#endif

