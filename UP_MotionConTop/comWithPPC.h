#ifndef COMDATA_T_H
#define COMDATA_T_H

#include "control.h"
#include "command.h"
#include <QTcpSocket>
#include <QAbstractSocket>
#include <QHostAddress>
#include <QMessageBox>
#include <QObject>
//#include "ui_UPMotionCon.h"


#define INT_FINE_NUM         49
#define INT_COARSE_NUM       50
#define DOUBLE_FINE_NUM         80
#define DOUBLE_COARSE_NUM       200
#define SWAP_32(x) (((x)&0xff000000)>>24)|(((x)&0x00ff0000)>>8)|(((x)&0x0000ff00)<<8)|(((x)&0x000000ff)<<24)
typedef enum tagAxisStatus
{
    S_Aixs_Close = 6501,
    S_Aixs_Open = 6502,
    S_Aixs_Jogfor_Start = 6503,
    S_Aixs_Ident = 6504,
    S_Aixs_Jogrev_Start = 6505,
    S_Aixs_RunFor = 6506,
    S_Aixs_RunRev = 6507,
    S_Aixs_Home = 6508,
    S_Aixs_Switch_Ruler = 6549
}axisStatus;

class RXData_t
{
public:
    RXData_t();
    double getRVelocity(int axis);
    double getFPosition(int axis);
    double getFVelocity(int axis);
    double getRPosition(int axis);
    double getRAcc(int axis);
    double getFAcc(int axis);
    double getRJerk(int axis);
    double getPosError(int axis);
    axisStatus getaxisStatus(int axis);
    double getConDist(int axis);
    void doubleByteConvert(double *src);
    void intByteConvert(int *src);
//private:
public:
    int iCMD;
    int iFineData[INT_FINE_NUM];
    int iCoarseData[INT_COARSE_NUM];
    double dFineData[DOUBLE_FINE_NUM];
    double dCoarseData[DOUBLE_COARSE_NUM];
    char abc[792];
    double refPosition[10];
    double refVel[10];
    double refAcc[10];
    double refJerk[10];
    double refSnap[10];
    double feedPosition[10];
    double feedVel[10];
    double feedAcc[10];
    double feedJerk[10];
    double feedSnap[10];
    double posError[10];
    double conDist[10];
    float DAOffset[10];
    int axisStatus[10];
    int stageStatus[10];
    int dir[10];
    int laserACPowerLevel[10];
    int laserDCPowerLevel[10];
    int motorState[10];
    controller_t controler[10];
};


class client_t :public QTcpSocket
{
    Q_OBJECT
public:
    client_t();
    QString getSocketError();
    void socketClose();
public slots:

signals:
    void display(QString a);
    void updateStates();
private:
    QTcpSocket *clientSock;
    QString addr;
    quint16 port;
    QByteArray data;
    RXData_t rxData;
};

class TXData_t
{
public:

private:
    int iCMD;
    int stageType;
    int iDof;
    int  paramType;
    int iOrder;
    int iReserved[5];
    double dParamData[24];
    char abc[792];
};

class laser_t
{
public:
private:
    int ACPowerLevel;
    int DCPowerLevel;
    int readCount;

};





#endif // COMDATA_T_H
