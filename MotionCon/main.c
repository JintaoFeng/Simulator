#include "ConParamInit.h"
#include "rtcontrol.h"


#include "ComWithWin.h"
#include "ZMI4104.h"
#include "ast_M58.h"
#include "N1225A.h"

#define STACK_SIZE 	12000//8000  //The size of task stack

typedef union
{
	char a[4];
	int i;
}union_t;

void text()
{
	union_t var;
	var.a[0]=0x01;
	var.a[1]=0x02;
	var.a[2]=0x03;
	var.a[3]=0x04;
	printf("var sizeof=%x\n",sizeof(var));
	printf("var.i=%x\n",var.i);
	printf("%d\n",sizeof(comData));
	printf("%d\n",sizeof(TransConParam));
	printf("RXData_t=%d\n",sizeof(RXData_t));
}

void text1()
{
	union_t var_1;
	var_1.i=0x01020304;
	printf("var_1 sizeof=%x\n",sizeof(var_1));
	printf("");
	printf("var_1.a[0]=%x\n var_1.a[1]=%x\n var_1.a[2]=%x\n var.a[3]=%x\n",var_1.a[0],var_1.a[1],var_1.a[2],var_1.a[3]);
}

void InitCounter()
{
//	ast_M72_Init(0,0x1C0000,1);	
	ast_M72_Init(0,0x1C0000,2);
	
	ast_M72_Counter_Start(0,0,4);
	ast_M72_Counter_Start(0,1,4);
	ast_M72_Counter_Start(0,2,4);
	ast_M72_Counter_Start(0,3,4);
	
//	ast_M72_Counter_Start(1,0,4);
//	ast_M72_Counter_Start(1,1,4);
//	ast_M72_Counter_Start(1,2,4);
//	ast_M72_Counter_Start(1,3,4);
}

