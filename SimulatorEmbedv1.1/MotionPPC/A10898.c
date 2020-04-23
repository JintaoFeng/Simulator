#include "stdio.h"
#include "math.h"
#include "A10898.H"			/*定义10898A的寄存器地址*/

//#define	BOARDSYN			//P2 同步(~Sample1)
#define TEST
#define LOG

#define Lambda      632.991354e-6	
#define Comp  		0.999728766	
#define Fold  		4	
#define ResX  		256	
#define Convert  	Comp * Lambda / (Fold * ResX)
#define ConvertVel 	Lambda/(Fold*0.4194304)

A10898* *pBase;

void setBit(short *address,char bit)
{
	short temp;
	temp=*((short*)address);
	temp|=0x01<<bit;
	*((short*)address)=temp;

}
void clearBit(short *address,char bit)
{
	short temp;
	temp=*((int *)address);
	temp&=~(0x01<<bit);
	*((int *)address)=temp;
}

void A10898_Init(char board,int address)
{
	address+=0x90000000;
	pBase[board]=(A10898*)address;
	printf("The %x laser board address is %x\n",board,address);
}

void A10898_Start(char board,A10898_Axis axis)
{
	pBase[board][axis].Control = 0x0;	
	
	#ifdef TEST
		pBase[board][axis].TestRateA |= A9X_TF_ON|A9X_TR_DIV(20000000);
		pBase[board][axis].TestRateB |= A9X_TF_ON|A9X_TR_DIV(10000000);
		pBase[board][axis].LsrSource |= (A9X_MEAS_A(A9X_MS_TEST)|A9X_MEAS_B(A9X_MS_TEST));
	#else
		pBase[board][axis].LsrSource = (A9X_MEAS_A(A9X_MS_MEAS_1)|A9X_MEAS_B(A9X_MS_COM1));
		if(board==0 && axis==X)
			pBase[board][axis].LsrSource |= A9X_COM_DRV(1,A9X_CD_MEAS2);
	#endif
	
	pBase[board][axis].OutControl = 0x0;
	pBase[board][axis].SampleDelay = 0xFF;
	pBase[board][axis].SampleModeMask = 0x0;
	pBase[board][axis].IRQerrMask = 0x0;
	pBase[board][axis].OvflLevelOut = 0;
	pBase[board][axis].FilterControl = 0x10;
	taskDelay((int)(sysClkRateGet()/1000)*2);
	A10898_Reset(board,axis);
	
}

void A10898_Reset(char board,A10898_Axis axis)
{
	pBase[board][axis].ErrStatusReset = 0x7f;
	pBase[board][axis].ErrStatusReset = 0x0;
	pBase[board][axis].Command |= A9X_RESET_POSITION;
}

int A10898_ReadPos(char board,A10898_Axis axis,char Channel)
{
	#ifdef BOARDSYN
		if(board==0 && axis==X)
			 pBase[board][axis].Command = A9X_SAMPLE_DRV98(Channel);
	#else
		pBase[board][axis].Command = A9X_SAMPLE_POS(Channel);
	#endif
	#ifdef LOG
		printf("%x\n",pBase[board][axis].PVnorm.Pos[Channel].l);
	#endif
	return pBase[board][axis].PVnorm.Pos[Channel].l;
}

int A10898_ReadVel(char board,A10898_Axis axis,char Channel)
{
/*	#ifdef BOARDSYN
		if(board==0 && axis==X)
			 pBase[board][axis].Command = A9X_SAMPLE_DRV98(Channel);
	#else*/
		pBase[board][axis].Command = A9X_SAMPLE_VEL(Channel);
		
	#ifdef LOG
		printf("%x\n",pBase[board][axis].PVnorm.Vel[Channel].l);
	#endif
	return pBase[board][axis].PVnorm.Vel[Channel].l;
}

int A10898_Auto_ReadPos(char board,A10898_Axis axis,char Channel)
{
	#ifdef	LOG
		printf("%x\n",pBase[board][axis].PVauto.Pos[Channel].l);
	#endif
	return pBase[board][axis].PVauto.Pos[Channel].l;
}

int A10898_Auto_ReadVel(char board,A10898_Axis axis,char Channel)
{
	#ifdef	LOG
		printf("%x\n",pBase[board][axis].PVauto.Vel[Channel].l);
	#endif
	return pBase[board][axis].PVauto.Vel[Channel].l;
}
void test10898()
{
	A10898_Init(0,0x400);
	A10898_Start(0,X);
	A10898_Start(0,Y);
	
}

