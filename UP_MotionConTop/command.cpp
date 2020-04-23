#include "command.h"

TXData_two::TXData_two()
{
//    qDebug()<<sizeof(TXData_two);
    memset(this,0,sizeof(TXData_two));
}
void TXData_two::setPID_Kp(int axis, float kp)
{
    this->setSeq(5001);
    this->setAxis(axis);
    this->floatByteConvert(&kp);
    this->setCommand(MOT_SETKP);
    controller[axis].setKp(kp);
}
void TXData_two::setPID_Ki(int axis, float ki)
{
    this->setSeq(5001);
    this->setCommand(MOT_SETKI);
    this->setAxis(axis);
     this->floatByteConvert(&ki);
    controller[axis].setKi(ki);
}

void TXData_two::setPID_Kd(int axis, float kd)
{
    this->setSeq(5001);
    this->setCommand(MOT_SETKD);
    this->setAxis(axis);
    this->floatByteConvert(&kd);
    controller[axis].setKd(kd);
}
void TXData_two::setSeq(int seq)
{
    this->intByteConvert(&seq);
    this->seq=seq;
}
void TXData_two::setAxis(int axis)
{
    this->intByteConvert(&axis);
    this->axis=axis;
}
void TXData_two::setCommand(command_e command)
{
    int a=command;
    this->intByteConvert(&(a));
    this->commend=(command_e)a;
}
void TXData_two::doubleByteConvert(double *src)
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
void TXData_two::intByteConvert(int *src)
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
void TXData_two::floatByteConvert(float *src)
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

void TXData_two::setTrajPos(int axis, double pos)
{
    this->setSeq(5001);
    this->setCommand(MOT_SETPOS);
    this->setAxis(axis);
    this->doubleByteConvert(&pos);
    traj[axis].setPos(pos);
}

void TXData_two::setTrajVel(int axis, double vel)
{
    this->setSeq(5001);
    this->setCommand(MOT_SETVEL);
    this->setAxis(axis);
    this->doubleByteConvert(&vel);
    traj[axis].setVel(vel);
}
void TXData_two::setTrajAcc(int axis, double acc)
{
    this->setSeq(5001);
    this->setCommand(MOT_SETACC);
    this->setAxis(axis);
    this->doubleByteConvert(&acc);
    traj[axis].setAcc(acc);
}
void TXData_two::setTrajJerk(int axis, double jerk)
{
    this->setSeq(5001);
    this->setCommand(MOT_SETJERK);
    this->setAxis(axis);
    this->doubleByteConvert(&jerk);
    traj[axis].setJerk(jerk);
}

void TXData_two::setTrajSnap(int axis, double snap)
{
    this->setSeq(5001);
    this->setCommand(MOT_SETSNAP);
    this->setAxis(axis);
    this->doubleByteConvert(&snap);
    traj[axis].setSnap(snap);
}

void traj_t::setAcc(double acc)
{
    this->acc=acc;
}
void traj_t::setPos(double pos)
{
    this->pos=pos;
}
void traj_t::setVel(double vel)
{
    this->vel=vel;
}
void traj_t::setJerk(double jerk)
{
    this->jerk=jerk;
}
void traj_t::setSnap(double snap)
{
    this->snap=snap;
}


comData_t::comData_t()
{
    memset(this,0,sizeof(comData_t));
}
void comData_t::setAxis(int axis)
{
    this->intByteConvert(&axis);
    this->axis=axis;
}
void comData_t::jog(int axis, double vel)
{
    setAxis(axis);
    setCMD(6010);
    this->doubleByteConvert(&vel);
    dParamData[0]=vel;
}
void comData_t::close(int axis)
{
    setAxis(axis);
    setCMD(6006);
}
void comData_t::setCMD(int cmd)
{
    intByteConvert(&cmd);
    CMD=cmd;
}
void comData_t::doubleByteConvert(double *src)
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
void comData_t::intByteConvert(int *src)
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
void comData_t::open(int axis)
{
    setAxis(axis);
    setCMD(6007);
}
void comData_t::jogStop(int axis)
{
    setAxis(axis);
    setCMD(6007);
}

void comData_t::moveRel(int axis, double pos)
{
    setAxis(axis);
    setCMD(6009);
    doubleByteConvert(&pos);
    dParamData[0]=pos;
}
void comData_t::moveAbs(int axis, double pos)
{
    setAxis(axis);
    setCMD(6008);
    doubleByteConvert(&pos);
    dParamData[0]=pos;
}
void comData_t::setPos(int axis, double pos)
{
    setAxis(axis);
    setCMD(6001);
    doubleByteConvert(&pos);
    dParamData[0]=pos;
}
void comData_t::setVel(int axis, double vel)
{
    setAxis(axis);
    setCMD(6002);
    doubleByteConvert(&vel);
    dParamData[0]=vel;
}
void comData_t::setAcc(int axis, double acc)
{
    setAxis(axis);
    setCMD(6003);
    doubleByteConvert(&acc);
    dParamData[0]=acc;
}

void comData_t::setJerk(int axis, double jerk)
{
    setAxis(axis);
    setCMD(6004);
    doubleByteConvert(&jerk);
    dParamData[0]=jerk;
}
void comData_t::setFPos(int axis, double fPos)
{
    setAxis(axis);
    setCMD(6005);
    doubleByteConvert(&fPos);
    dParamData[0]=fPos;
}

void comData_t::setKp(int axis,double kp)
{
    setAxis(axis);
    setCMD(6013);
    doubleByteConvert(&kp);
    dParamData[0]=kp;
}
void comData_t::setKi(int axis,double ki)
{
    setAxis(axis);
    setCMD(6014);
    doubleByteConvert(&ki);
    dParamData[0]=ki;
}
void comData_t::setKd(int axis,double kd)
{
    setAxis(axis);
    setCMD(6015);
    doubleByteConvert(&kd);
    dParamData[0]=kd;
}

void comData_t::setConnectType(ConnectType type)
{
    setCMD(6016);
    intByteConvert((int *)(&type));
    iParamData[0] = type;
//    qDebug()<<type;
}
