#ifndef COM_WITH_WIN_H
#define COM_WITH_WIN_H

#include "StateMachine.h"

void RecvFromWin();
void SendToWin();


class RXData_t
{
public:
    RXData_t()
    {

    }
    void doubleByteConvert(double *src);
    void intByteConvert(int *src);
//private:
public:
    int iCMD;
    int axis;
    int iReserved[8];
    double dParamData[30];
};

//extern RXData_t rxData;
#endif

