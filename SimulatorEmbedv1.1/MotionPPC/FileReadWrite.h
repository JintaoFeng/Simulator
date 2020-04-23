#ifndef FILEREADWRITE_H
#define FILEREADWRITE_H
#define BUFLENGTH 100000

int g_iDataLen[10];
extern double dIdentBuff[BUFLENGTH];

extern double P2PData[12];
extern double ExpoData[24];
extern double  DirData[12]; 
extern double PIDParam[20];

void ReadParam();


#endif
