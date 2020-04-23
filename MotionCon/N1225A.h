#ifndef N1225A_H
#define N1225A_H

#define Board_SUM  3
#define Axis_SUM   10

#define N1225A_VME_BASE_ADDR			0X90180000
#define N1225A_Board_ADDR_INCE			0X10000
#define N1225A_Axis_ADDR_INCE			0x200


#define CalBoard_Addr(nBoard)			(N1225A_VME_BASE_ADDR + (nBoard - 1) * N1225A_Board_ADDR_INCE)
#define CalBoardAxis_Addr(nBoard,nAxis)	(N1225A_VME_BASE_ADDR + (nBoard - 1) * N1225A_Board_ADDR_INCE + (nAxis - 1) * N1225A_Axis_ADDR_INCE)
#define CalAxis_Addr(nAxis)				((nAxis - 1) * N1225A_Axis_ADDR_INCE)
#define CalBoardAxisPosition_Addr(nBoard,nAxis,nPos)		\
										(N1225A_VME_BASE_ADDR + (nBoard - 1) * N1225A_Board_ADDR_INCE + (nAxis - 1) * N1225A_Axis_ADDR_INCE + (nPos-1) * 8)

#define Swap_16(word)					(unsigned short)((((unsigned short)(word) & 0x00ff) << 8) | (((unsigned short)(word) & 0xff00) >> 8) )
				
#define Swap_32(longWord)				(unsigned int)((((unsigned int)(Swap_16(longWord)) & 0x0000ffff) << 16) | ((unsigned int)(Swap_16(longWord>>16)) & 0x0000ffff))

#define MACReg_H						0x0040
#define MACReg_M						0x0042
#define MACReg_L						0x0044
#define MACReg_MSW						0x0040
#define MACReg_LSW						0x0044

#define ControlStatusReg_Axis1			0x0002
#define ControlStatusReg_Axis2			0x0202
#define ControlStatusReg_Axis3			0x0402
#define ControlStatusReg_Axis4			0x0602

#define CommandReg						0x000C

#define OutputControlStatusReg			0x0008

#define PowerLevelReg					0x0014

#define LaserSourceControlReg			0x0004

#define SampleDelayReg					0x009A

#define SamplModeMaskReg				0x009C

#define SampleStatusReg					0x00A0

#define SetupReg						0x0026

#define AutoSamplingPosReg				0x0144

#define AutoSamplingPosExtReg			0x0140

#define StandardSamplingPosReg			0x0104
#define StandardSamplingPosExtReg		0x0100

#define PosPresetReg_H					0x0080
#define PosPresetReg_L					0x0084

#define ComparatorConfigurReg			0x007C
#define LowerPosComparator_H			0x0074
#define LowerPosComparatorReg_L			0x0078

#define HighPosComparatorReg_H			0x006C
#define HighPosComparatorReg_L			0x0070

#define HighVelComparatorReg			0x0064
#define LowVelComparatorReg				0x0068

#define AutoSamplingVelReg				0x0170
#define StandardSamplingVelReg			0x0130

#define OutputHoldRateControlReg		0x00B4

#define ErrorStatusResetReg				0x0028
#define IRQErrorMaskReg					0x00A4

#define GainSquelchSettingsReg			0x001C

unsigned int ReadReg(unsigned int Reg);//, unsigned char BoadID);
void WriteReg(unsigned int Reg, unsigned int Val);
void SetRegBit(unsigned char BoardID, unsigned int Reg, unsigned char index);
void ClearRegBit(unsigned char BoardID, unsigned int Reg, unsigned char index);
void readMAC(unsigned char BoadID,long long *MAC);
void communicationTest(unsigned char BoadID);
long long AutoReadPos(unsigned char BoardID, unsigned char Axis, unsigned char Pos);
void BoardReset(unsigned char BoardID);
void LaserSourceInit(unsigned int BoardID, unsigned char Axis);
void SetupLSB(unsigned int BoardID, unsigned char Axis, unsigned short LSB);
void SampleDelay(unsigned int BoardID, unsigned char Axis, unsigned short time);
void OutputControlInit(unsigned int BoardID, unsigned char Axis);
void ReadAllPositions(int  *position, unsigned char PosReg);
void readPowerLevel(char nBoard,char nAxis,int* powerLevel);
void readACPowerLevel(char nBoard,char nAxis,int* powerLevel);
#endif