//update Jan, 15, 09 10am
//single input scan and out put burst by set BCR bits, test OK!!!!!
//Test on computers
// setupaio() 中删除了autocalibration的部分

#include "pmc16aiodrv.h"

INT32* dataBuf = (INT32*)0x08000000;

//make display normal
UINT32 Xchange(UINT32 val)
{
	UINT32 temp;
	UINT32 temp1,temp2,temp3,temp4;
	
	temp1=val&0xff;
	temp2=val>>8&0xff;
	temp3=val>>16&0xff;
	temp4=val>>24&0xff;

	temp1=temp1<<24;
	temp2=temp2<<16;
	temp3=temp3<<8;
	temp=temp1+temp2+temp3+temp4;
	//printf("%x",temp);
	return temp;
}

UINT32 AIO_Read_Local32(UINT32 Reg)
{
	UINT32 val;
	val = *(UINT32*)(AIO16_BASE_ADDR+Reg);
	val=Xchange(val);     //new line
    return val;
}

void AIO_Write_Local32(UINT32 Reg,UINT32 val)
{
    val=Xchange(val);     //new line
	*(UINT32*)(AIO16_BASE_ADDR+Reg) = val;
}

//yht
void AIO_Write_Local3202(UINT32 Reg,UINT32 val)
{
    val=Xchange(val);     //new line
	*(UINT32*)(AIO16_BASE2_ADDR+Reg) = val;
}

void SetRegisterBit(UINT32 Reg,int bitIndex,int val)
{
	UINT32 temp;
	UINT32* ptr = (UINT32*)(AIO16_BASE_ADDR+Reg);

	temp = *ptr;
	temp=Xchange(temp);//new line
	
	if(val)
	{
		temp = temp | (1<<bitIndex);		
	}
	else
	{
		temp = temp & ~(1<<bitIndex);
	}
	temp=Xchange(temp);//new line
	
	*ptr = temp;
}

void SetRegisterBit02(UINT32 Reg,int bitIndex,int val)
{
	UINT32 temp;
	UINT32* ptr = (UINT32*)(AIO16_BASE2_ADDR+Reg);

	temp = *ptr;
	temp=Xchange(temp);//new line
	
	if(val)
	{
		temp = temp | (1<<bitIndex);		
	}
	else
	{
		temp = temp & ~(1<<bitIndex);
	}
	temp=Xchange(temp);//new line
	
	*ptr = temp;
}

UINT32 GetRegisterBit(UINT32 Reg,int bitIndex)
{
	UINT32 temp;
	UINT32* ptr = (UINT32*)(AIO16_BASE_ADDR+Reg);
	
	temp = *ptr;
	temp=Xchange(temp);//new line
	return (temp>>bitIndex & 0x1);
}
UINT32 GetRegisterBit02(UINT32 Reg,int bitIndex)
{
	UINT32 temp;
	UINT32* ptr = (UINT32*)(AIO16_BASE2_ADDR+Reg);
	
	temp = *ptr;
	temp=Xchange(temp);//new line
	return (temp>>bitIndex & 0x1);
}

/* this funstion read back ADC buffer and store to databuf */
//Eng Huang

void test1()
{
	int i;int ru;

	for(i=0;i<32;i++)
	{
		SetRegisterBit(DIO,i,1);

	}
	for(i=0;i<4;i++)
	{
		unsigned int val=0;
		(*(unsigned int *)(AIO16_BASE_ADDR+DIO)) = 0x05;
		val=(*(unsigned int *)(AIO16_BASE_ADDR+DIO));
		printf("%x\r\n",val);
	}

}

void test2()
{
	int i;
	//for(i=0;i<4;i++){
	//char val=0;
	//val=(*(char *)(AIO16_BASE_ADDR+DIO+i));
	//printf("%x",val);
	//}
	int r;
	for(i=0;i<32;i++)
	{
		r=GetRegisterBit(DIO,i);
		printf("%x",r);
	}

}

void test3()
{
	SetRegisterBit(RATE_B,16,1);
}

void test5()
{
	int a;int i;
	for(i=0;i<32;i++)
	{
		a=GetRegisterBit(BCR,i);
		printf("%x",a);
	}
	printf("\n");
	for(i=0;i<32;i++)
	{
		a=GetRegisterBit02(BCR,i);
		printf("%x",a);
	}
	printf("\n");
}

void test6(int i)
{
	//*(UINT32*)(AIO16_BASE_ADDR+0x24) = 0x60000;
	if(i==0)
	{
		AIO_Write_Local32(DIO,0x000d0010);
	}
	else
	{
		*(UINT32*)(AIO16_BASE_ADDR+0x24) = 0xffffffff;
	}
}

void test7()
{
	*(UINT32*)(AIO16_BASE_ADDR+0x10) = 0x12345678;
}
void test8()
{
	*(UINT32*)(AIO16_BASE_ADDR+0x18) = 0x12345678;
}

void test9()
{
	*(UINT32*)(AIO16_BASE_ADDR+0x18) = 0xffff0000;
	*(UINT32*)(AIO16_BASE_ADDR+0x18) = 0x00000100;
	*(UINT32*)(AIO16_BASE_ADDR+0x18) = 0x12340200;
	*(UINT32*)(AIO16_BASE_ADDR+0x18) = 0x43210f00;
	SetRegisterBit(BCR, 11, 1);	
}

void test10()
{
	*(UINT32*)(AIO16_BASE_ADDR+0x18) = 0x00000000;
	*(UINT32*)(AIO16_BASE_ADDR+0x18) = 0xffff0100;
	*(UINT32*)(AIO16_BASE_ADDR+0x18) = 0x43210200;
	*(UINT32*)(AIO16_BASE_ADDR+0x18) = 0x12340f00;
	SetRegisterBit(BCR, 11, 1);	
}
void init()
{
	int samplenum=4*4;

	setupaio();	
	PMCModeSet();	
	SetRegisterBit(IN_DATA_CNTRL, 15, 1);
	AIO_Write_Local32(IN_DATA_CNTRL, 0x8000|(UINT32)samplenum);	//Or with 0x8000, clear the buffer
	AIO_Write_Local3202(IN_DATA_CNTRL, 0x8000|(UINT32)samplenum);
}

void test11()
{
	int i;
	int data1[4],data2[4],data3[4];

	UINT32 temp;
	static long count=0;

	SetRegisterBit(RATE_A, 16, 1);	//Set D16, disable rate generator A

	for(i=0;i<1;i++)
	{
		temp=AIO_Read_Local32(IN_DATA_BUFF);//0
		temp=AIO_Read_Local32(IN_DATA_BUFF);
		temp=AIO_Read_Local32(IN_DATA_BUFF);
		temp=AIO_Read_Local32(IN_DATA_BUFF);
		temp=AIO_Read_Local32(IN_DATA_BUFF);
		temp=AIO_Read_Local32(IN_DATA_BUFF);
		temp=AIO_Read_Local32(IN_DATA_BUFF);
		temp=AIO_Read_Local32(IN_DATA_BUFF);
		temp=AIO_Read_Local32(IN_DATA_BUFF);
		temp=AIO_Read_Local32(IN_DATA_BUFF);
		temp=AIO_Read_Local32(IN_DATA_BUFF);//10
		temp=AIO_Read_Local32(IN_DATA_BUFF);
		temp=AIO_Read_Local32(IN_DATA_BUFF);
		temp=AIO_Read_Local32(IN_DATA_BUFF);
		temp=AIO_Read_Local32(IN_DATA_BUFF);
		temp=AIO_Read_Local32(IN_DATA_BUFF);
		temp=AIO_Read_Local32(IN_DATA_BUFF);
		temp=AIO_Read_Local32(IN_DATA_BUFF);
		temp=AIO_Read_Local32(IN_DATA_BUFF);
		temp=AIO_Read_Local32(IN_DATA_BUFF);
		temp=AIO_Read_Local32(IN_DATA_BUFF);//20
		temp=AIO_Read_Local32(IN_DATA_BUFF);
		temp=AIO_Read_Local32(IN_DATA_BUFF);
		temp=AIO_Read_Local32(IN_DATA_BUFF);
		data1[i]=AIO_Read_Local32(IN_DATA_BUFF);//24
		temp=AIO_Read_Local32(IN_DATA_BUFF);
		data2[i]=AIO_Read_Local32(IN_DATA_BUFF);//26
		temp=AIO_Read_Local32(IN_DATA_BUFF);
		data3[i]=AIO_Read_Local32(IN_DATA_BUFF);//28
		temp=AIO_Read_Local32(IN_DATA_BUFF);
		temp=AIO_Read_Local32(IN_DATA_BUFF);//30
		temp=AIO_Read_Local32(IN_DATA_BUFF);
		//four channels per scan
	}


//	IOW( 0, 0xff);
	
	//clear buffer
	SetRegisterBit(IN_DATA_CNTRL, 15, 1);
	
	while(GetRegisterBit(IN_DATA_CNTRL, 15))
	{
	  taskDelay(0);
	}

	SetRegisterBit(RATE_A, 16, 0);	//Clear D16, enable rate generator A	
//	IOW( 0, 0x00);

printf("%d\t%d\t%d\n",data1[0],data2[0],data3[0]);
}

void aio16()
{
	int chan_num=16;
	UINT32 indata;
	UINT32 RATEWORD;
	int i;
	int sample_size;
//	sysClkRateSet(1000);				//Set the ticks per second

	sample_size = 256*chan_num;			//Buf size equals the number of chanels multiply its FIFO size
	
	SetRegisterBit(BCR,15,1);
	taskDelay(sysClkRateGet()/2);          //Delay 0.5s, sysClkRateGet() returns the number of ticks per second

	while(GetRegisterBit(BCR,15))			//Wait if the initializion is not completed	
	{	
		taskDelay(10);
	}//while

	AIO_Write_Local32(RATE_A, 0x50);		//Set the frequency of rate generator A, 300 000 Hz
	AIO_Write_Local3202(RATE_A, 0x50);		//Set the frequency of rate generator A, 300 000 Hz

	/*Disable Clock*/
	indata = AIO_Read_Local32(RATE_A);
	AIO_Write_Local32(RATE_A, indata | 0x10000);	//Disable rate generator A by set D16 high
	/* Clear Buffer and write threshold value = sample_size*/
	AIO_Write_Local32(IN_DATA_CNTRL, 0x8000|(UINT32)sample_size);	//Clear the buffer

	/*Enable clock*/
	AIO_Write_Local32(RATE_A, indata & 0xFFFEFFFF);		//Clear D16, enable rate generator A


	while (!GetRegisterBit(IN_DATA_CNTRL,16))		//Wait for the overflow of input buffer!
		taskDelay(1);

	
	for(i=0;i<sample_size;i++){
		
		*(dataBuf+i) = AIO_Read_Local32(IN_DATA_BUFF);

		/* print out the any 1 channel of data,which shoulde be input by an external signal */
//		if( (i%32)==0) 
		printf("ch %d:   %08x\n",i%16,*(dataBuf+i) );
	}
	printf("Done!\n");
}


/* this funstion write a sine wave to DAC buffer, all channel should output sine signal  */
//need to test the frequency by change the Rate_B
//Eng Huang
void aio16_out()
{
	UINT32 indata;
	int i,j;
	double flt;
	int sample_size;

//	sysClkRateSet(1000);

	sample_size = 256*1024;
	SetRegisterBit(BCR,15,1);
	SetRegisterBit(BCR,06,0);
	SetRegisterBit(BCR,10,1);
	AIO_Write_Local32(OUT_DATA_CNTRL, 0x8000|(UINT32)sample_size);
	AIO_Write_Local3202(OUT_DATA_CNTRL, 0x8000|(UINT32)sample_size);

	indata=24000000/512;
	indata=0x00ffff&indata;
	printf("RATE_B =%x\n",indata);

	AIO_Write_Local32(RATE_B, indata); /* EN output clk */

	for(i=0;i<sample_size;i++)
	{
		flt = 32000.0*sin(2.0*3.1415926*i/64.0);
		*(dataBuf+i) = (short)flt;
	}	
/*
	while (GetRegisterBit(OUT_DATA_CNTRL,16))
		taskDelay(1);
*/
	for(i=0;i<sample_size;i++)
	{	
		 AIO_Write_Local32(OUT_DATA_BUFF, *(dataBuf+i)&0x00ffff|0x00000);//DA 00
		 AIO_Write_Local32(OUT_DATA_BUFF, *(dataBuf+i)&0x00ffff|0x10000);//DA 01
		 AIO_Write_Local32(OUT_DATA_BUFF, *(dataBuf+i)&0x00ffff|0x30000);
		 AIO_Write_Local32(OUT_DATA_BUFF, *(dataBuf+i)&0x00ffff|0x40000);
	}

	printf("Done!\n");	
}

/****************************************************************
*
*	reset the PMC-AIO CARD
	如果进行Autocal操作，则DA 输出口有方波输出
*
****************************************************************/
void reset()
{
	//Reset all the register to its default by set the bit15 in BCR
	SetRegisterBit(BCR,15,1);
	taskDelay(sysClkRateGet()/2);
	while(GetRegisterBit(BCR,15))			//Wait if the initializion is not completed	
	{	
		taskDelay(10);
	}//while
}
/*
3.4 Analog Input/Output Parameters
	3.4.1 Analog Voltage Range
	The analog inputs and outputs share a common voltage range that is selected by BCR control
bit field D04-D05, as shown in Table 3.4-1.
	Table 3.4-1. Analog Voltage Range Selection (BCR field D04-D05)
		RANGE[1:0] ANALOG INPUT RANGE
		0 ±2.5 Volts
		1 ±5 Volts
		2 ±10 Volts
		3 ±10 Volts
3.5 Analog Input Control

	3.5.1 Input Data Organization
	Conversion data from the analog-to-digital converter (ADC) flows through a 256-word transfer
FIFO into the analog input data buffer, and from the data buffer to the PCI bus as analog input
data. The data buffer appears to the PCI bus as a single read-only register.

	3.5.1.1 Input Data Buffer
	Analog input data is read from the Analog Input Data Buffer in longword-serial format, as shown
in Table 3.5-1. Each value is right-justified to the LSB, and occupies bit positions D00 through
D15. D17-D31 are always returned as zero's. The capacity of the input data buffer is
32K-samples.
	D16 in the input data buffer is set HIGH when the associated data field D00-D15 contains
Channel 00 data. D16 is LOW for all channels other than Channel 00.
	Table 3.5-1. Input Data Buffer
	Offset: 0008h Default: N/A
	DATA BIT MODE* DESIGNATION DESCRIPTION
	D00 RO DATA00 Least significant data bit
	D01-D14 RO DATA01 - DATA14 Intermediate data bits
	D15 RO DATA15 Most significant data bit
	D16 RO CHANNEL 00 TAG Indicates Channel 00.
	D17-D31 RO (Inactive) ---
	* RO indicates read-only access. Write-data is ignored.


	3.5.1.2 Data Coding Format
	Analog input and analog output data is arranged as 16 active right-justified data bits with the
	coding conventions shown in Table 3.5-2. The default format is offset binary. Two's
	complement format is selected by clearing the Offset Binary control bit LOW in the BCR.
	Note: Unless indicated otherwise, offset binary coding is assumed throughout this document.
	Table 3.5-2. Input/Output Data Coding; 16-Bit Data
	ANAOG OUTPUT LEVEL DIGITAL VALUE (Hex)
	OFFSET BINARY TWO'S COMPLEMENT
	Positive Full Scale minus 1 LSB XXXX FFFF XXXX 7FFF
	Zero plus 1 LSB XXXX 8001 XXXX 0001
	Zero XXXX 8000 XXXX 0000
	Zero minus 1 LSB XXXX 7FFF XXXX FFFF
	Negative Full Scale plus 1 LSB XXXX 0001 XXXX 8001
	Negative Full Scale XXXX 0000 XXXX 8000

	3.5.2 Input Data Buffer Control
	The Input Data Buffer control register shown in Table 3.5-3 controls and monitors the flow of
data through the analog input data buffer. Asserting the Clear Buffer control bit HIGH clears, or
empties, the buffer, and also aborts any input scan that might be in progress. The Threshold
Flag is HIGH when the number of values in the input data buffer and the 256-Word input
transfer FIFO exceeds the input threshold value defined by bits D00-D14, and is LOW if the
number is equal to or less than the threshold value. An interrupt (Section 3.9) can be
programmed to occur on either the rising or falling edge of the threshold flag.
	Note: The threshold flag does not respond to samples queued in the transfer FIFO.
	Table 3.5-3. Input Data Buffer Control Register
	Offset: 000Ch Default: 0000 7FFEh
	DATA BIT MODE* DESIGNATION DEF DESCRIPTION
	D00-14 R/W THRESHOLD VALUE 7FFEh Input buffer threshold value.
	D15 R/W CLEAR BUFFER * 0 Clears (empties) the input buffer when asserted HIGH.
	Aborts current input scan.
	D16 RO THRESHOLD FLAG 0 Asserted HIGH when the number of values in the input
	buffer exceeds the THRESHOLD VALUE.
	D17-D31 RO (Inactive) 0 ---
	*Clears automatically when operation is completed

	3.5.3 Analog Input Function Modes
	BCR control bits D00-D03 (AIM0-AIM3) control the analog input configuration, and provide
selftest modes for monitoring the integrity of the analog input and output networks. Table 3.5-4
summarizes the input scanning modes.
	Table 3.5-4. Analog Input Function Selection (BCR field D00-D03)
		AIM[3:0] FUNCTION OR MODE
		0 Differential analog input mode (Default mode).
		1 Single-Ended analog input mode.
		2 ZERO test. Internal ground reference is connected to all analog input channels.
		3 +VREF test. Internal voltage reference is connected to all analog input channels.
		4 Monitor Output Channel 00
		5 Monitor Output Channel 01
		6 Monitor Output Channel 02
		7 Monitor Output Channel 03
		8 Monitor Output Channel 04
		9 Monitor Output Channel 05
		10 Monitor Output Channel 06
		11 Monitor Output Channel 07
		12-15 (Reserved)
	3.5.3.1 Differential Inputs
	The analog inputs default to the differential configuration when power is applied, or after
initialization. In this mode, the 16 analog input lines are arranged as eight differential pairs, with
each pair representing a single input channel. Differential channels are even-numbered from 00
through 14; as 00, 02,...12, 14.
	3.5.3.2 Single-ended Inputs
	With the single-ended input mode selected, each of the 16 analog input lines is measured in
reference to a common Input Return, and represents an individual input channel. Single-ended
input channels are numbered from 00 through 15; as 00, 01,...14, 15.
	3.5.3.3 Selftest Modes
	In each of the six selftest modes, the analog input lines from the system I/O connector are
ignored and have no effect on the selftest results. Specified board accuracy applies to all
selftest measurements, and for critical measurements the average value of multiple readings
should be used.
	The ZERO selftest measures a dead-zero reference signal and should produce a midscale
reading of 0000 8000h. For the +VREF test, a precision reference voltage equal to 96.15% of
fullscale is applied as an analog input, and should produce a reading of 0000 FB12h.
	Each of the eight analog outputs also can be monitored, and should provide a reading equal to
the value written to the associated output DAC (within specified board accuracy).
	3.5.4 Input Scan Timing
	For each analog input scan clock, all selected analog inputs are scanned once at the maximum
conversion rate, with one conversion performed per channel. The number of channels included
in each scan is controlled from 2-16 channels by the Scan Size control bit field in the Scan and
Sync control register, or any single channel can be selected. Each scan commences with
Channel 00 and proceeds upward through successive input channels until the selected number
of channels has been digitized.
	3.5.4.1 Conversion Rate
	During each input scan, the selected channels are scanned and digitized at a fixed conversion
rate that is slightly higher than 300,000 conversions per second. The analog input scan
clocking rate has no effect on the conversion rate.
	3.5.4.2 Scan Rate
	To ensure that all scan clocks are acknowledged, the analog input clock frequency Finput
should not exceed:
	Finput-max (Hz) = 300,000 / Nchan,
	where Nchan is the number of channels in a scan. For example, an 8-channel scan should not
be clocked at a frequency higher than 300,000/8 = 37,500Hz. At higher clock frequencies the
duration of each scan exceeds the clocking period, and some clocks will be ignored.
	3.5.4.3 Scan Clocking Source
	The Scan and Sync control register (Section 3.4.2) provides four sources for analog input scan
clocks. The clock can be provided by (a) either of the two rate generators on the board, (b) the
External Sync hardware input line, or (c) the Input Sync control bit in the BCR.
	If the BCR Input Sync bit is selected as the analog input clock source, an input scan occurs
each time the control bit is set HIGH. The Input Sync bit remains HIGH until the scan is
completed, after which the bit is cleared automatically.
	If the External Sync input line is the analog input clock source, each HIGH-to-LOW transition of
the input line initiates an input scan.
	3.5.5 Scanning Modes
	The analog inputs can be scanned in groups of 4, 8 or 16 channels, or any single channel can
be selected for digitizing. If the INPUT SCANNING MODE control bit in the Scan and Sync
control register is LOW, the multiple-channel mode is selected, and the number of channels in a
scan is selected by the SCAN SIZE control field. An input scan begins with Channel-00, and
proceeds upward through successive channels until the selected number of channels has been
digitized and stored in the input data buffer.
	A 2-channel scan mode overrides all other input mode selections, and digitizes input
Channels 00 and 01.
	If the INPUT SCANNING MODE control bit is HIGH, the single-channel mode is selected, and
the channel to be digitized is selected by the SINGLE_CHANNEL SELECT control field.

/**********************************************************************************
*	此函数设置了BCR寄存器的值，设定了板卡的AD输入模式，电压输入输出范围
*	由于每次设定RANGE之后应该进行Autocal操作，过程中DA输出RANGE电压的方波
*	因此，此函数应该在板上电或者重启后进行，且过程中一定得先关闭驱动的输出使能
*
*********************************************************************************
*	便于调试时的安全，在此不进行Autocal操作，影响不太明显
*********************************************************************************
*	RANGE -5~5
*	INPUT MOD single end
*
*	Debug mod ,no Autocal
**********************************************************************************/
void setupaio()
{
	UINT32	AIM;	//Analog input mode bit 0~3
	UINT32	RANGE;	//Analog input/out range, bit 4~5;
	UINT32	temp;

	UINT32 SCANSIZE;	// Bit 0~1
	UINT32 INPUTSCANMOD;	//bit 11;

	UINT32 channel;

	//Rate Generator
	unsigned int inputfreq;		//input frequency
	unsigned int outputfreq;
	UINT32	RATEWORD;			//Corresponding to the frequency

	//////////////////Step 1 Reset///////////////////
	//Reset all the register to its default by set the bit15 in BCR
	SetRegisterBit(BCR,15,1);
	taskDelay(sysClkRateGet()/2);
	while(GetRegisterBit(BCR,15))			//Wait if the initializion is not completed	
	{	
		taskDelay(10);
	}
	////////////////////////////////////////////////
	
	/////////////////Step 2  ////////////////////////
	AIM=0;		//single end input
	//RANGE=0X01;	//-5~5V   (if not -10~10, need do autocalibration)
	RANGE=0X02; //-10~10V 
	//
/*
Table 3.2-1. Board Control Register (BCR)
Offset: 0000h Default: 0000 4060h

DATA BIT	MODE	DESIGNATION		DEF		DESCRIPTION
D00-D03		 R/W AIM0 0 Analog input mode. Selects input configuration or selftest mode. Defaults to differential input mode
D04			 R/W RANGE0 0 Analog input/output range. Defaults to ±10V range.
D05			 R/W RANGE1 1
D06 			 R/W OFFSET BINARY 1 Selects offset-binary analog I/O data format when asserted HIGH, or two's complement when LOW.
D07 			 R/W (Reserved) 0 ---
D08 			 R/W SIMULTANEOUS OUTPUTS 0 Selects simultaneous or channel-sequential output mode.
Defaults LOW to channel-sequential output mode.
D09 R/W ENABLE OUTPUT BURST 0 Enables output bursting (one-shot) mode when HIGH.
D10 R/W ENABLE OUTPUT LOOPING 0 Enables output function looping when HIGH.
D11 R/W *OUTPUT SYNC 0 Initiates a single output burst, when enabled by ENABLE
OUTPUT BURST. Clears automatically upon burst
completion,
D12 R/W *INPUT SYNC 0 Initiates a single input scan, when selected in the Scan
and Sync Control Register. Clears automatically upon
scan completion,
D13 R/W *AUTOCAL 0 Initiates an autocalibration operation when asserted.
Clears automatically upon autocal completion,
D14 RO AUTOCAL PASS 1 Set HIGH at reset or autocal initialization. A HIGH state
after autocal confirms a successful calibration.
D15 R/W *INITIALIZE 0 Initializes the board when set HIGH. Sets defaults for all
registers.
D16-D31 RO (Inactive) 0 ---
*/
	AIO_Write_Local32(BCR, 0x00004060);
	AIO_Write_Local3202(BCR, 0x00004060);

	temp=AIO_Read_Local32(BCR);
	temp=(temp&0xFFFFFFC0)|(AIM|((RANGE&0X0F)<<4));
	AIO_Write_Local32(BCR, temp);
	AIO_Write_Local3202(BCR, temp);

	//First set every register to its default value
	//BCR not done here, so be careful
	//SetRegisterBit(RATE_A,16,1);
	//SetRegisterBit(RATE_B,16,1);
/*
	Table 3.4-2. Scan and Sync Control Register
	Offset: 0020h Default: 0000 02D1h
	DATA
	BIT
	MODE DESIGNATION DEF DESCRIPTION
	D00-D01 R/W ANALOG INPUT SCAN SIZE 1 Number of input channels per scan when operating in the
	Multiple-Channel scanning mode.
	0 => 4 channels per scan
	1 => 8 channels per scan
	2 => 16 channels per scan (single-ended mode only)
	3 => (Reserved)
	Ignored in the Single-Channel and Two-Channel
	scanning modes described below:
	D02-D03 R/W ANALOG INPUTS SCAN CLOCK 0 Selects the analog input scan clocking source:
	0 => Internal Rate-A generator output
	1 => Internal Rate-B generator output
	2 => External Sync input line
	3 => BCR Input Sync control bit.
	D04-D05 R/W ANALOG OUTPUTS CLOCK 1 Selects the analog output channel clocking source.
	0 => Internal Rate-A generator output
	1 => Internal Rate-B generator output
	2 => External Sync Input line
	3 => Disabled
	D06-D07 R/W ANALOG OUTPUTS SYNC 3 Selects the burst sync source for the analog outputs.
	0 => Internal Rate-A generator output
	1 => Internal Rate-B generator output
	2 => External Sync Input line
	3 => BCR Output Sync control bit .
	D08-D09 R/W EXT SYNC OUTPUT SOURCE 2 Selects the signal source for the External Sync output
	line:
	0 => Analog Inputs Scan Clock
	1 => Analog Outputs Sync
	2 => External Sync Input line (passthru mode)
	3 => Disabled
	D10 R/W RATE-B CLOCK SOURCE 0 Selects the clock input source for the Rate-B generator:
	0 => Master clock
	1 => Rate-A generator output.
	D11 R/W INPUT SCANNING MODE 0 Selects the input scanning mode. Ignored if Two-
	Channel scanning is selected (See TWO-CHANNEL
	SCAN below).
	0 => Multiple-Channel Mode
	1 => Single-Channel Mode
	D12-16 R/W SINGLE-CHANNEL SELECT 0 Selects the input channel number when operating in the
	Single-Channel scanning mode. Ignored in the Multiple-
	Channel and Two-Channel scanning modes.
	D17 R/W TWO-CHANNEL SCAN 0 Invokes a 2-Channel scan size when HIGH. Overides
	the selected Input Scanning Mode
	D18-D31 RO (Inactive) 0 ---
	R/W = Read/Write, RO = Read-Only.
	NOTE: The terms "sync" and "trigger" are used interchangeably in this text.
*/

	AIO_Write_Local32(SCAN_CNTRL, 0x000002D1);

#define	IN_CLK_RATEA
#ifdef	IN_CLK_RATEA

	inputfreq =	30000;//60000;					//采样周期
	RATEWORD=30000000/inputfreq;
	AIO_Write_Local32(RATE_A,0x00010000|RATEWORD);	//D16 is set High to disable Rate_A

	printf("use Rate_A as input clk\n");
#else
	//INPUT SYNC  BCR bit 12 as input sync
	temp=AIO_Read_Local32(SCAN_CNTRL);
	temp=(temp&0x7f3)|0x0c;		//D2~3 11 for BCR Input Sync ,7f3 set scan mod(clear D11);
	AIO_Write_Local32(SCAN_CNTRL,temp);
#endif

/*
	//不进行AutoCal操作
	//Auto calibration by set bit 13
	SetRegisterBit(BCR,13,1);
	taskDelay(sysClkRateGet()*3);			//AutoCal need about 3s to finish

	while(GetRegisterBit(BCR,13))			//Wait if the initializion is not completed	
	{	
		taskDelay(10);
	}

	//Check the AUTO CALI PASS status
	if(GetRegisterBit(BCR,14))
	{
		printf(" The Auto calibration is compeleted successfully!\n");
	}
	else
	{
		printf(" The auto calibration failed, please check you hardware!\n");
	}
*/

}

/*************************************************************************************
*
*	set the control registers
*
*	注意，此不可作为通用的设置，因为BCR的设置在此没有复原，原来在BCR中设定的
*	特定的输入输出模式不会被改变，setupaio()中有设置BCR的部分
*	通过设置宏，选择工作模式

******************************************************************************************/
void PMCModeSet(void)
{
	//no bcr part

	//Scan and Sync Control Reg
	UINT32 SCANSIZE;	// Bit 0~1
	UINT32 INPUTSCANMOD;	//bit 11;
	UINT32 temp;	

	UINT32 channel;

	//Rate Generator
	unsigned int inputfreq;		//input frequency
	unsigned int outputfreq;
	UINT32	RATEWORD;			//Corresponding to the frequency

//First set every register to its default value
	//BCR not done here, so be careful
	SetRegisterBit(RATE_A,16,1);
	SetRegisterBit(RATE_B,16,1);

	AIO_Write_Local32(SCAN_CNTRL, 0x000002D1);
	AIO_Write_Local3202(SCAN_CNTRL, 0x000002D1);

//Then set the wanted workmod


	//Set Scan and Sync Control Register default val 0x0000 02d2
	// 16 chanels per scan, Rate A input clk, Rate B output clk
#undef	SINGLEINPUT 			//单通道输入
#ifdef	SINGLEINPUT
	channel=0;									//通道号
	temp=AIO_Read_Local32(SCAN_CNTRL);
	channel=channel&0x1f;
	temp=(temp&0x000000ffc)|channel<<12;		//clear D0~1, choose 4 channel per scan, select channel 0 D12~16;
	AIO_Write_Local32(SCAN_CNTRL, temp);	
	SetRegisterBit(SCAN_CNTRL,11,1);	//single channel mod
#else
	SCANSIZE=1;
        //SCANSIZE=3;                          
		//set numbers of channels per scan
		//0~4 channels,1~8 channels, ..., 3~32 channels
	temp=AIO_Read_Local32(SCAN_CNTRL);
	temp=(temp&0xfffffff0)|(0x0f&SCANSIZE);
	AIO_Write_Local32(SCAN_CNTRL, temp);
#endif
		
#define	IN_CLK_RATEA
#ifdef	IN_CLK_RATEA		

	inputfreq =	30000;//60000;					//采样周期
	RATEWORD=30000000/inputfreq;
	AIO_Write_Local32(RATE_A,0x00010000|RATEWORD);	//D16 is set High to disable Rate_A

	printf("use Rate_A as input clk\n");
#else
	//INPUT SYNC  BCR bit 12 as input sync
	temp=AIO_Read_Local32(SCAN_CNTRL);
	temp=(temp&0x7f3)|0x0c;		//D2~3 11 for BCR Input Sync ,7f3 set scan mod(clear D11);
	AIO_Write_Local32(SCAN_CNTRL,temp);
//	printf("use BCR Input Sync bit as input clk\n");
#endif

#define		SIMULTANEOUSCLK
#define		BURSTMOD
#ifdef		SIMULTANEOUSCLK
	//synchronous mod, one burst 
	temp=AIO_Read_Local32(SCAN_CNTRL);
	temp=(temp&0xffffffcf)|0x30;		//D4~5 11, disable output clock
	AIO_Write_Local32(SCAN_CNTRL, temp);

	SetRegisterBit(BCR, 8, 1);		//simultaneous out clk mod
	#ifdef	BURSTMOD
		SetRegisterBit(BCR, 9, 1);		//Burst mod
	#else
		SetRegisterBit(BCR, 9, 0);	
	#endif
	
	temp=AIO_Read_Local32(SCAN_CNTRL);
	temp=(temp&0xffffff0f)|0xd0;	// D4~7 RATEB for output clk
	AIO_Write_Local32(SCAN_CNTRL, temp);

	outputfreq = 20000;//300000;
	RATEWORD = 30000000/outputfreq;
	AIO_Write_Local32(RATE_B,0x00010000|RATEWORD);	//D16 is set High to disable Rate_B
	SetRegisterBit(RATE_B, 16, 0);	//Enable RATEB
#endif

	//Clear input and output buffer
	SetRegisterBit(IN_DATA_CNTRL, 15, 1);	
	while(GetRegisterBit(IN_DATA_CNTRL, 15))
	{
		taskDelay(1);
	}
	SetRegisterBit(OUT_DATA_CNTRL, 15, 1);	
	while(GetRegisterBit(OUT_DATA_CNTRL, 15))
	{
		taskDelay(1);
	}
//	StatusDisplay();

}	


/***************************************************************
ao, ai, do, di
****************************************************************/
void AO(double value, UINT32 ch)
{
	int temp1;
	UINT32 temp2;
	temp1=(int)((float)((float)(value+10)/20)*0xffff);
	temp2=0x30000&(ch<<16);
	AIO_Write_Local32(OUT_DATA_BUFF, temp1|temp2);
	AIO_Write_Local3202(OUT_DATA_BUFF, temp1|temp2);
	printf("The output voltage is %f, chanel %d\n", value, ch);
	printf("Output val in digital : %x\n",temp1|temp2);
}

/****************************************************************************
* Inherite from function aio16()
* 
****************************************************************************/
void AI(double val)
{
	UINT32 indata;
	int i;
	int sample_size;

	sample_size = 256;			//Only a chanel is used

	SetRegisterBit(BCR,15,1);
	taskDelay(sysClkRateGet()/2);          //Delay 0.5s, sysClkRateGet() returns the number of ticks per second

	while(GetRegisterBit(BCR,15))			//Wait if the initializion is not completed	
	{	
		taskDelay(10);
	}//while
	
	//output in chanel 00
	AO(val,0);

	indata=AIO_Read_Local32(BCR);
	indata=indata|0x0040;		//AIM=4 for monitor chanel 00;
	AIO_Write_Local32(BCR,indata); 
	AIO_Write_Local3202(BCR,indata); 

	AIO_Write_Local32(RATE_A, 0x50);		//Set the frequency of rate generator A, 300 000 Hz
	AIO_Write_Local3202(RATE_A, 0x50);

	/*Disable Clock*/
	indata = AIO_Read_Local32(RATE_A);
	AIO_Write_Local32(RATE_A, indata | 0x10000);	//Disable rate generator A by set D16 high
	AIO_Write_Local3202(RATE_A, indata | 0x10000);	
	/* Clear Buffer and write threshold value = sample_size*/
	AIO_Write_Local32(IN_DATA_CNTRL, 0x8000|(UINT32)sample_size);	//Or with 0x8000, clear the buffer
	AIO_Write_Local3202(IN_DATA_CNTRL, 0x8000|(UINT32)sample_size);

	/*Enable clock*/
	AIO_Write_Local32(RATE_A, indata & 0xFFFEFFFF);		//Clear D16, enable rate generator A
	AIO_Write_Local3202(RATE_A, indata & 0xFFFEFFFF);

	while (!GetRegisterBit(IN_DATA_CNTRL,16))		//Wait for the overflow of input buffer!
		taskDelay(1);

	for(i=0;i<sample_size;i++){
		
		*(dataBuf+i) = AIO_Read_Local32(IN_DATA_BUFF);

		/* print out the any 1 channel of data,which shoulde be input by an external signal */
		if( (i%16)==0) printf("ch %d:   %08x\n",i,*(dataBuf+i) );
	}
	printf("Done!\n");

}

void DO(int value, UINT32 ch) 
{
	logMsg("t",0,0,0,0,0,0);
}

int DI(UINT32 ch)
{

}

/**************************************************************************
*
*	Change ADC output to voltage 
* 	Range defined by VOLINOUTRANG
*	Only range is -10~10, and Date Coding format is OFFSET BINARY
*	0x0000 will be converted to -10v
*	Test Ok Dec 06
***************************************************************************/
/*
double DataToVol( UINT32 data)
{
	double vol;
	data=0xffff&data;
//	vol=((long int)data-0x8000)*10/(double)0x8000;
	vol=((long int)data-0x8000)*VOLINOUTRANG/(double)0x8000;
//	printf("The equivelent vol is in DataToVol : %f\n",vol);
	return 0.5*vol;
}
*/
/**************************************************************************
*
*
*	Change voltage to DAC input for convert
*	Range defined by VOLINOUTRANG and Date Coding format is OFFSET BINARY
*	Test Ok Dec 06
***************************************************************************/
UINT32 VolToData( double vol)
{
	UINT32 data;
	data=vol/VOLINOUTRANG*0x8000+0x8000;
//	printf("The equivelent data for the voltage in VolToData:0X%X\n", data);
	return data;
}
/******************************************************************************
*
*	Display the value of registers 
*
********************************************************************************/
void StatusDisplay(void)
{
	printf("BCR	=	%x\n",AIO_Read_Local32(BCR));	
	printf("ICR	=	%x\n",AIO_Read_Local32(ICR));	
	printf("IN_DATA_CNTRL=	%x\n",AIO_Read_Local32(IN_DATA_CNTRL));	
	printf("RATE_A	=	%x\n",AIO_Read_Local32(RATE_A));	
	printf("RATE_B 	=	%x\n",AIO_Read_Local32(RATE_B ));	
	printf("OUT_DATA_CNTRL=	%x\n",AIO_Read_Local32(OUT_DATA_CNTRL));	
	printf("SCAN_CNTRL=	%x\n",AIO_Read_Local32(SCAN_CNTRL));
	printf("DIO	=	%x\n",AIO_Read_Local32(DIO));	
	printf("OUT_DATA_BUFF	=	%x\n",AIO_Read_Local32(OUT_DATA_BUFF));	

}
/*****************************************************************************
* Test the the sensor input signals
* Call PMCModeSet to set work mod first
*
*	此函数采三个通道的传感器输入值
******************************************************************************/
void adtest(void)
{
	int sample_size;
	int chan_num=3;
	UINT32 indata;
	UINT32 temp;
	int i,j;
//	sysClkRateSet(1000);				//Set the ticks per second

	sample_size = 3;			// 每次扫描的通道数，如果大于4，则需要改动PMCModeSet中的值

	indata=AIO_Read_Local32(BCR);

	for(j=0;j<100;j++)
	{
		//Disable Clock
		indata = AIO_Read_Local32(RATE_A);
		AIO_Write_Local32(RATE_A, indata | 0x10000);	//Disable rate generator A by set D16 high
		//Clear Buffer and write threshold value = sample_size
		AIO_Write_Local32(IN_DATA_CNTRL, 0x8000|(UINT32)sample_size);	//Or with 0x8000, clear the buffer
		SetRegisterBit(IN_DATA_CNTRL, 15, 1);	
		while(GetRegisterBit(IN_DATA_CNTRL, 15))
		{
			taskDelay(1);
		}

		//Enable clock/
		SetRegisterBit(RATE_A, 16, 0);	//Clear D16, enable rate generator A


		while (!GetRegisterBit(IN_DATA_CNTRL,16))		//Wait for the overflow of input buffer!
			taskDelay(1);
		SetRegisterBit(RATE_A, 16, 1);		//Disable the Rate generator A
		
	//	printf("convert result:\n");
		for(i=0;i<sample_size;i++)	
		{

	//		temp=AIO_Read_Local32(IN_DATA_BUFF);
	//		temp=AIO_Read_Local32(IN_DATA_BUFF);
			temp=AIO_Read_Local32(IN_DATA_BUFF);
	//		printf("%d, ",j);
	//		printf("ch %d, %08x, ",i%chan_num,temp);
//			printf("%x,",temp);
			printf("%f,", DataToVol(temp));
		}
		printf("\n");
		taskDelay(200);
	}//for


}

/***********************************************************************************************
*
*	0 for input, 1 for ouput
*
***********************************************************************************************/
void IOset(char low, char high)
{

	SetRegisterBit(DIO, 18, low);
	SetRegisterBit(DIO, 19, high);
//SetRegisterBit(DIO, 1, low);
//SetRegisterBit(DIO, 2, high);

}
		
/**********************************************************************************************
*	IO test only
	write the IO Port
*	set IO port first
***********************************************************************************************/
void IOW( int lowbyte, int highbyte)
{
	UINT32 temp;
	temp=AIO_Read_Local32(DIO);
	temp=(temp&0xffff0000)|((0x00ff&highbyte)<<8)|(0x00ff&lowbyte);
	AIO_Write_Local32(DIO,temp);
}

void readinput(UINT32 size)
{
	long int i;
	for(i=0;i<size;i++)
	{
		printf("NO %d:\t%x\n",i,AIO_Read_Local32(IN_DATA_BUFF));
	}
}

/*********************************************************************
*  Messure the convert time of D/A 
*  
*  
*
**********************************************************************/   
void wADVsIO()
{

	IOset(1,1);

	IOW(0XFF,0XFF);
	tmanuout(2.0,2.0,2.0);
	
}

void iolow()
{
	UINT32 temp;
	UINT32 highbyte,lowbyte;
	highbyte=0x00;
	lowbyte=0x00;
	temp=AIO_Read_Local32(DIO);
	temp=(temp&0xffff0000)|((0x00ff&highbyte)<<8)|(0x00ff&lowbyte);
	printf("temp=%x\n",temp);
	AIO_Write_Local32(DIO,temp);
}

UINT32 ADsinglechan(int chanel)
{
	UINT32 temp;
	//stop Rate A
	SetRegisterBit(RATE_A, 16,1);

	temp=0x001f&chanel;						//D0~5 of chanel is used to identify chanel 00~31
	temp=temp<<12;							//D12~16
	temp=temp|0x000002DE;					//0x000002D2 is default value of Scan and Sync
	AIO_Write_Local32(SCAN_CNTRL,temp);		//
	SetRegisterBit(SCAN_CNTRL, 11, 1);		//set scan mod single chanel

	SetRegisterBit(IN_DATA_CNTRL, 15, 1);	//Clear the buffer,

	while(GetRegisterBit(IN_DATA_CNTRL, 15))	
	{
		taskDelay(1);
	}
	
	SetRegisterBit(BCR, 12, 1);
	taskDelay(10); 
	while(GetRegisterBit(BCR,12))	//Dec 11, remain high.
	{
		taskDelay(1);
	}

	printf("Input = %x\n", AIO_Read_Local32(IN_DATA_BUFF));
//	return(AIO_Read_Local32(IN_DATA_BUFF));
	
}
/***********************************************************************
*	Initiate a single input scan by setting bit 12 in BCR
*	Call PMCModeSet first
*   Test OK Dec 26th 
************************************************************************/
void winputscan(void)
{
	UINT32 temp;
	int i;
	//stop Rate A
//	SetRegisterBit(RATE_A, 16,1);
	
	SetRegisterBit(IN_DATA_CNTRL, 15, 1);	//Clear the buffer,

	while(GetRegisterBit(IN_DATA_CNTRL, 15))	
	{
		taskDelay(1);
	}

	SetRegisterBit(BCR, 12, 1);
	taskDelay(10); 
	while(GetRegisterBit(BCR,12))	//Dec 11, remain high.
	{
		taskDelay(1);
	}
	for(i=0;i<4;i++)
	{
		printf("Input = %x\n", AIO_Read_Local32(IN_DATA_BUFF));
	}
	
}



/*****************************************************************************
* Test the the sensor input signals
* Call PMCModeSet to set work mod first
******************************************************************************/
void adread(UINT32 *result)
{
	UINT32 indata;
	UINT32 temp;
	int i,j,size;
	size = 3*1;			// samples per chanel

	indata=AIO_Read_Local32(BCR);

	//Disable Clock
	indata = AIO_Read_Local32(RATE_A);
	AIO_Write_Local32(RATE_A, indata | 0x10000);	//Disable rate generator A by set D16 high
	//Clear Buffer and write threshold value = sample_size
//do in other place
	AIO_Write_Local32(IN_DATA_CNTRL, 0x8000|(UINT32)size);	//Or with 0x8000, clear the buffer
	SetRegisterBit(IN_DATA_CNTRL, 15, 1);	
	while(GetRegisterBit(IN_DATA_CNTRL, 15))
	{
		taskDelay(1);
	}

		//Enable clock/
	SetRegisterBit(RATE_A, 16, 0);	//Clear D16, enable rate generator A

	while (!GetRegisterBit(IN_DATA_CNTRL,16))		//Wait for the overflow of input buffer!
		taskDelay(1);
	SetRegisterBit(RATE_A, 16, 1);		//Disable the Rate generator A
	for(i=0;i<size;i++)
	{
		*(result+i)=AIO_Read_Local32(IN_DATA_BUFF);
	}
	//clear buffer	
	AIO_Write_Local32(IN_DATA_CNTRL, 0x8000|(UINT32)size);	//Or with 0x8000, clear the buffer
}

/******************************************************************************************************
* Initiate a output burst by setting BCR output bit
* Test func
*
*******************************************************************************************************/
void tmanuout(double vol0, double vol1, double vol2)
{
	UINT32 value=0x8000;

//	printf("Vol0=%f\tVol1=%f\tVol2=%f\n",vol0,vol1,vol2);	
	value=VolToData(vol0)|0X00000;
	AIO_Write_Local32(OUT_DATA_BUFF, value);
	value=VolToData(vol1)|0x10000;
	AIO_Write_Local32(OUT_DATA_BUFF, value);
	value=VolToData(vol2)|0x20000|0x40000|0x80000;	//D16~17 channel tag, D18 group end flag,D19 burst end flag
	AIO_Write_Local32(OUT_DATA_BUFF, value);
	SetRegisterBit(BCR, 11, 1);
				
	while(GetRegisterBit(BCR, 11))	
	{
		taskDelay(1);
	}

//	printf("output finish\n");

}

void inputscanstart(void)
{
	int sample_size=40;

//	AIO_Write_Local32(IN_DATA_CNTRL, 0x8000|(UINT32)sample_size);	//Or with 0x8000, clear the buffer
/*	SetRegisterBit(IN_DATA_CNTRL, 15, 1);	
	while(GetRegisterBit(IN_DATA_CNTRL, 15))
	{
		;
	}*/

	//Enable clock/
//	SetRegisterBit(RATE_A, 16, 0);	//Clear D16, enable rate generator A


	while (!GetRegisterBit(IN_DATA_CNTRL,16))		//Wait for the overflow of input buffer!
	{;}

	SetRegisterBit(RATE_A, 16, 1);		//Disable the Rate generator A
//	printf("DATA sample finished!\n");
}
