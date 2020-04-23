
#ifndef PMC16AIODRV_H
#define	PMC16AIODRV_H

//包含所有的函数，需要进一步删节

#include "vxWorks.h"
#include "stdio.h"
#include "math.h"

/*==============================================================================
 Board Register Addresses
==============================================================================*/
#define       BCR                0x00
#define       ICR                0x04
#define       IN_DATA_BUFF       0x08
#define       IN_DATA_CNTRL      0x0C
#define       RATE_A             0x10
#define       RATE_B             0x14
#define       OUT_DATA_BUFF      0x18
#define       OUT_DATA_CNTRL     0x1C
#define       SCAN_CNTRL         0x20
#define       DIO                0x24
#define       FW_REV             0x28
#define       AUTOCAL            0x2C

#define       DMA			     1

#define		  MINUS_ONE_LONG     0xFFFFFFFFL

#define 	  AIO16_BASE_ADDR    (0xa1000100)
#define 	  AIO16_BASE2_ADDR    (0xa1000300)

#define       PLX_BASE_ADDR      (0xE0240400)


//============================================================================================
#define	      VOLINOUTRANG	10.0	//input & output range
//============================================================================================


UINT32 AIO_Read_Local32(UINT32 Reg);
UINT32 AIO_Read_Local3202(UINT32 Reg);


void AIO_Write_Local32(UINT32 Reg,UINT32 val);
void SetRegisterBit(UINT32 Reg,int bitIndex,int val);
UINT32 GetRegisterBit(UINT32 Reg,int bitIndex);


void aio16();
void aio16_out();

void reset();
void setupaio();
void PMCModeSet(void);
void AO(double value, UINT32 ch);
void AI(double val);
//double DataToVol( UINT32 data);
UINT32 VolToData( double vol);
void StatusDisplay(void);
void adtest(void);
void IOset(char low, char high);
void IOW( int lowbyte, int highbyte);
void readinput(UINT32 size);
void ADVsIO();
void iolow();
UINT32 ADsinglechan(int chanel);
void inputscan(void);
void adread(UINT32 *result);
void tmanuout(double vol0, double vol1, double vol2);

#endif



