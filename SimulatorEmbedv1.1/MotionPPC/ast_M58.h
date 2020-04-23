/****************************************************************************************
* Filename :            ast_Header.h                                                                
* Version:              1.0                                                                      
* Programmer(s):        AST
* Created :             2016-03-08                                                                     
* Description :         vxworks driver for M72                                
* Modification History:   
*****************************************************************************************/


#include "vxWorks.h"

#define M58_Channel_baseadrs  0x901c0000 /*A203N BaseAdress */


#define Port_B_A   *(short*) (M58_Channel_baseadrs + 0x02)
#define Port_D_C   *(short*) (M58_Channel_baseadrs + 0x00)
#define M58_Channel_ConReg0    *(short*) (M58_Channel_baseadrs + 0x80)
#define M58_Channel_ConReg1    *(short*) (M58_Channel_baseadrs + 0x82)
#define M58_Channel_ConReg2     *(short*) (M58_Channel_baseadrs + 0x84)
#define M58_Channel_ConReg3     *(short*) (M58_Channel_baseadrs + 0x86)


void ast_M58_Init();
void ast_M58_DIO();

void ast_M58_Protect();
void ast_M58_SoftProtect();
void HomeAll();







