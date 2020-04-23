#include "N1225A.h"

/*
 *	void readMAC(uint8_t BoadID,long *MAC)
 *	@brief 读取激光板的MAC地址，一共6个字节，排列方式为MAC[5]:MAC[4]:MAC[3]:
 *			MAC[2]:MAC[1]:MAC[0];
 *	@param BoadID 激光板序号，1-∞;
 *	@param *MAC 用来返回读到的地址
 *	@retval None
 */
void readMAC(unsigned char BoadID,long long* MAC)
{
	unsigned int BoadAddr = CalBoard_Addr(BoadID);

	unsigned short MAC_H, MAC_M, MAC_L;
	MAC_H = *((unsigned int*)(BoadAddr + MACReg_H));
	MAC_M = *((unsigned int*)(BoadAddr + MACReg_M));
	MAC_L = *((unsigned int*)(BoadAddr + MACReg_L));
	printf("mac = %x,mac = %x,mac = %x,%d\r\n",MAC_H,MAC_M,MAC_L,sizeof(MAC_H));

	
/*	
	unsigned int MAC_MSW, MAC_LSW;
//	long long mac;
	MAC_MSW = *((unsigned int *)(BoadAddr + MACReg_MSW));
	MAC_LSW = *((unsigned int *)(BoadAddr + MACReg_MSW));
	MAC_MSW = Swap_32(MAC_MSW);
	MAC_LSW = Swap_32(MAC_LSW);
//	mac = MAC_MSW;  
//	logMsg("mac=%x,%x\n",MAC_MSW,MAC_LSW,0,0,0,0);
	printf("mac_h = %x,mac_l=%x,%d\r\n",MAC_MSW,MAC_LSW,sizeof(MAC_MSW));
//	*MAC = (mac << 32) | (MAC_LSW << 0);
*/
}

/*
 * void communicationTest(uint8_t BoadID)
 * @brief 用来测试板卡和单板计算机是否通信正常，若正常，轴1状态灯开始闪烁，一段时间后恢复绿色常量
 * @param BoadID 激光板序号，1-∞;
 * @retval None
 */
void communicationTest(unsigned char BoadID)
{
	unsigned int Addr = N1225A_VME_BASE_ADDR + (BoadID - 1) * N1225A_Board_ADDR_INCE;
	unsigned int i = 0;
	*((unsigned int *)(Addr + ControlStatusReg_Axis1)) = 0x0080;
	for ( i = 0; i < 10000; i++);
	*((unsigned int *)(Addr + ControlStatusReg_Axis1)) = 0x0000;
}

/*
 * unsigned int ReadReg(unsigned int Reg, unsigned char BoadID)
 * @brief 读取寄存器的值
 * @param BoadID 激光板序号，1-∞;
 * @param Reg 寄存器地址；
 * @retval val 返回当前寄存器的值
 */
unsigned int ReadReg(unsigned int Reg)//, unsigned char BoadID)
{
	unsigned int Val;
	Val = *((unsigned int*)Reg);
	return Val;
}

/*
 * void WriteReg(unsigned int Reg, unsigned char BoadID, unsigned short Val)
 * @brief 写入寄存器
 * @param BoadID 激光板序号，1-∞;
 * @param Reg 寄存器地址；
 * @retval None
 */

void WriteReg(unsigned int Reg, unsigned int Val)
{
//	unsigned int RegAddr = CalBoard_Addr(BoardID) + Reg;
	*((unsigned int*)Reg) = Val;
}
/*
 * void SetRegBit(unsigned char BoardID, unsigned int Reg,unsigned char index)
 * @brief 将寄存器中的某一位置位
 * @param BoadID 激光板序号，1-∞;
 * @param Reg 寄存器地址；
 * @param index 需要置位的bit
 * @retval None
 */
void SetRegBit(unsigned char BoardID, unsigned int Reg,unsigned char index)
{
	unsigned int RegAddr = CalBoard_Addr(BoardID) + Reg;
	unsigned int temp;
	temp = *((unsigned int*)RegAddr);
	temp |= (0x01 << index);
	*((unsigned int*)RegAddr) = temp;
}

/*
 * void ClearRegBit(unsigned char BoardID, unsigned int Reg, unsigned char index)
 * @brief 将寄存器中的某一位置0
 * @param BoadID 激光板序号，1-∞;
 * @param Reg 寄存器地址；
 * @param index 需要置0的bit
 * @retval None
 */
void ClearRegBit(unsigned char BoardID, unsigned int Reg, unsigned char index)
{
	unsigned int RegAddr = CalBoard_Addr(BoardID) + Reg;
	unsigned int temp;
	temp = *((unsigned int*)RegAddr);
	temp &= ~(0x01 << index);
	*((unsigned int*)RegAddr) = temp;
}
/*
 * unsigned int AutoReadPos(unsigned char BoardID, unsigned char Axis,unsigned char Pos)
 * @brief 自动读取轴的位置寄存器
 * @param BoadID 激光板序号，1-∞;
 * @param Axis 轴的序号 1-4；
 * @param Pos 位置寄存器 1-6
 * @retval Position 返回当前位置寄存器的值
 */
long long AutoReadPos(unsigned char BoardID, unsigned char Axis,unsigned char Pos)
{
	unsigned int PosAddr = CalBoardAxisPosition_Addr(BoardID, Axis, Pos) + AutoSamplingPosReg;
	unsigned int ExtPosAddr = CalBoardAxisPosition_Addr(BoardID, Axis, Pos) + AutoSamplingPosExtReg;
	long long pos_value = 0;
	pos_value = *((unsigned int*)ExtPosAddr);
	pos_value = (long long)(pos_value << 32) + *((unsigned int*)PosAddr) ;
	return pos_value;
}


/*
 * void BoardReset(unsigned char BoardID)
 * @brief 同时复位一块板上的所有轴的位置或者重信加载重装寄存器的值
 * @param BoadID 激光板序号，1-∞;
 * @retval None
 */
void BoardReset(unsigned char BoardID)
{
	unsigned int RegAddr = CalBoard_Addr(BoardID) + CommandReg;	
	*((unsigned int*)RegAddr) |= (0x01 << 14);
}

/*
 * void AxisReset(unsigned int BoardID, unsigned char Axis)
 * @brief 如果轴一般控制和状态寄存器中的预置启用位为0，则轴复位为0。
 *        如果预设的启用位为1，则将轴位置设置为轴位置偏移寄存器中的值。
 * @param BoadID 激光板序号，1-∞;
 * @retval None
 */
void AxisReset(unsigned int BoardID, unsigned char Axis)
{
	unsigned int RegAddr = (Axis - 1) * N1225A_Axis_ADDR_INCE + CommandReg;
//	*((unsigned int*)RegAddr) |= (0x01<<8);
	SetRegBit(BoardID, RegAddr, 8);
}

/*
 * void N1225A_Init(unsigned int nBoard)
 * @brief 初始化
 * @retval None
 */
void N1225A_Init(void)
{
	unsigned char board = 0,axis = 0;
	for (board = 1; board <= Board_SUM; board++)
	{
		unsigned int BoardAddr = CalBoard_Addr(board);
		for (axis = 1; axis <= 4; axis++)
		{
			unsigned int AxisAddr = CalBoardAxis_Addr(board,axis);
			LaserSourceInit(board, axis);
			SetupLSB(board, axis, 0);
			OutputControlInit(board, axis);
			SampleDelay(board, axis, 0x00FF);
//			AxisReset(board, axis);
			WriteReg((AxisAddr + SamplModeMaskReg), 0x00000000);		//每次采样操作后都更新速度和位置寄存器
			WriteReg((AxisAddr + GainSquelchSettingsReg), 0x00000000);		//turn on AGC, turn off squelch
		}
		BoardReset(board);
		WriteReg((BoardAddr + ErrorStatusResetReg) , 0xFFFFFFFF);			//clear any errors
		WriteReg((BoardAddr + IRQErrorMaskReg), 0x00000000);			//disable any interrupts
		WriteReg((BoardAddr + OutputHoldRateControlReg), 0x00000000);			//Do NOT drive output HOLD line
	}
	
//	with the clock divider output
}

/*
 * void LaserSourceInit(unsigned int BoardID,unsigned char Axis)
 * @brief 对N1225A板上的轴进行初始化
 * @param BoadID 激光板序号，1-∞;
 * @param Axis 轴的序号 1-4；
 * @retval None
 */
void LaserSourceInit(unsigned int BoardID,unsigned char Axis)
{
	unsigned int BoardAddr = CalBoard_Addr(BoardID);
	unsigned int AxisAddr = CalBoardAxis_Addr(BoardID, Axis);
	unsigned short MeasA = (Axis - 1) << 4;
	if (BoardID == 1)
	{
		unsigned int MeasB_1 = 3;
		unsigned int value_1 = (MeasA & 0X00F0) | (MeasB_1 & 0x000f);
		*((unsigned int *)(AxisAddr + LaserSourceControlReg)) = value_1;		//设置第一块板的第四轴为参考基准
//		printf("value_1 = %x\r\n",value_1);
	}
	else
	{
		unsigned short MeasB_2 = 4;
		unsigned short value_2 = (MeasA & 0X00F0) | (MeasB_2 & 0x000F);
		*((unsigned int *)(AxisAddr + LaserSourceControlReg)) = value_2;		//其他板的参考基准全部设为第一块板的第四周
//		printf("value_2 = %x\r\n", value_2);
	}
}

/*
 * void SetupLSB(unsigned int BoardID, unsigned char Axis,unsigned short LSB)
 * @brief 设置采样精度
 * @param BoadID 激光板序号，1-∞;
 * @param Axis 轴的序号 1-4；
 * @param LSB 采样精度，可选择的值包括：
				0.000: 0.15 nm resolution
				1.001: 0.3 nm resolution
				2.010: 0.6 nm resolution
				3.011: 1.2 nm resolution
				4.100: 2.4 nm resolution
				5.101: 4.8 nm resolution
				6.110: 4.8 nm resolution
				7.111: 4.8 nm resolution
 * @retval None
 */
void SetupLSB(unsigned int BoardID, unsigned char Axis,unsigned short LSB)
{
	unsigned int AxisAddr = CalBoardAxis_Addr(BoardID, Axis);
	*((unsigned int *)(AxisAddr + SetupReg)) = (0X0007 & LSB);	
}

/*
 * void SampleDelay(unsigned int BoardID, unsigned char Axis,)
 * @brief 设置读取位置寄存器和速度寄存器的延迟时间
 * @param BoadID 激光板序号，1-∞;
 * @param Axis 轴的序号 1-4；
 * @param time 如果想要自动采样，则寄存器的值必须全为1，即整个寄存器的值为0xFF,
 * @retval None
 */
void SampleDelay(unsigned int BoardID, unsigned char Axis,unsigned short time)
{
	unsigned int AxisAddr = CalBoardAxis_Addr(BoardID, Axis);
	*((unsigned int *)(AxisAddr + SampleDelayReg)) = time;
}

/*
 * void OutputControlInit(unsigned int BoardID, unsigned char Axis)
 * @brief 对N1225A进行输出控制初始化，初始化位,设置第一块板为异步模式1，时钟驱动，其余的板设置为异步模式0，
 *        并且每个轴的P2接口地址不一样
 * @param BoadID 激光板序号，1-∞;
 * @param Axis 轴的序号 1-4；
 * @retval None
 */
void OutputControlInit(unsigned int BoardID, unsigned char Axis)
{
	unsigned int RegAddr = CalBoardAxis_Addr(BoardID,Axis) + OutputControlStatusReg;
	unsigned short AxisAddr_P2 = (BoardID - 1) + (Axis - 1);
	if ( Axis == 1)
	{
		if(BoardID == 1)
		{
			WriteReg(RegAddr,(0xD000| AxisAddr_P2));
		}
		else
		{
			WriteReg(RegAddr,(0x4000 | AxisAddr_P2));
		}		
	}
	else
	{
		WriteReg(RegAddr,(0x0000 | AxisAddr_P2));
	}
}

/*
 * void ReadAllPositions(unsigned int *position,unsigned char PosReg)
 * @brief 对所有的轴进行采样并进行读数
 * @param *position，需要填入的数组;
 * @param PosReg 需要采样的寄存器，1-4；
 * @retval None
 */
void ReadAllPositions(int *position,unsigned char PosReg)
{
	unsigned short Drivesample = 0x01 << (PosReg + 8);
	unsigned char board = 0, axis = 0;
	unsigned int ComRegAddr = CalBoard_Addr(1) + CommandReg;
	unsigned int Value = ReadReg(ComRegAddr);
	unsigned char i = 0;
//	long long Temp;
	Value |= Drivesample;
	WriteReg(ComRegAddr, Value);
	for (board = 1; board <= Board_SUM; board++)
	{
		for (axis = 1; axis <= 4; axis++)
		{
			if ((board == 1 && axis == 4) || (board == 3 && axis == 4))
				continue;
			else
			{
				unsigned int PosReg_Addr = CalBoardAxisPosition_Addr(board, axis, PosReg) + StandardSamplingPosReg;
				*(position+i) = ReadReg(PosReg_Addr);
				if (i == Axis_SUM)
					return;
				i++;
			}
		}
	}
}

void readDCPowerLevel(char nBoard,char nAxis,int* powerLevel)
{
	unsigned int regAddr = CalBoardAxis_Addr(nBoard, nAxis) + PowerLevelReg;
	*powerLevel = *((unsigned int *)regAddr) & 0x0000ffff;
//	printf("Data = %x\r\n",Data);
}

void readACPowerLevel(char nBoard,char nAxis,int* powerLevel)
{
	unsigned int regAddr = CalBoardAxis_Addr(nBoard, nAxis) + PowerLevelReg;
	*powerLevel = *((unsigned int *)regAddr)>>16;

}