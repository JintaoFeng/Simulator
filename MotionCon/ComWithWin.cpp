#include "ComWithWin.h"	//
#include "CMD.h"		//GetCMD
#include "StateMachine.h"	//
#include "TrajHandler.h"	//
#include "ConParamHandler.h"	//
#include "main.h"
#include "CMD.h"
//#include <QTcpServer>
//#include <QTcpSocket>

#define LOCAL_SERVER_PORT 1100 /* 监听的端口号 ,隐含问题，值太小*/

ComData comData;
RecvData recvData;
RXData_t rxData;

void RXData_t::doubleByteConvert(double *src)
{
    char* p=(char*)src;
    char temp[4];
    temp[0]=p[0];
    temp[1]=p[1];
    temp[2]=p[2];
    temp[3]=p[3];
    p[0]=p[7];
    p[1]=p[6];
    p[2]=p[5];
    p[3]=p[4];
    p[4]=temp[3];
    p[5]=temp[2];
    p[6]=temp[1];
    p[7]=temp[0];
}

void RXData_t::intByteConvert(int *src)
{
    char* p=(char*)src;
    char temp[2];
    temp[0]=p[0];
    temp[1]=p[1];
    p[0]=p[3];
    p[1]=p[2];
    p[2]=temp[1];
    p[3]=temp[0];
}
