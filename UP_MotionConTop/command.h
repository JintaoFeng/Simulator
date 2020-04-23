#ifndef COMMAND_T_H
#define COMMAND_T_H
#include "control.h"
#include "comWithPPC.h"
typedef enum
{
    MOT_ABORT,
    MOT_ENABLE,
    MOT_DISABLE,
    MOT_PAUSE,
    MOT_FORWARD,
    MOT_REVERSE,
    MOT_JOGFOR,
    MOT_JOGREV,
    MOT_STEP,
    MOT_FREE,
    MOT_COORD,
    MOT_TELEOP,
    MOT_SETKP,
    MOT_SETKI,
    MOT_SETKD,
    MOT_SETPOS,
    MOT_SETVEL,
    MOT_SETACC,
    MOT_SETJERK,
    MOT_SETSNAP,
    MOT_SETFEEDCOFF,
    MOT_SETDAOFFSET
}command_e;

class traj_t
{
public:
 //   traj_t();
    void setPos(double pos);
    void setVel(double vel);
    void setAcc(double acc);
    void setJerk(double jerk);
    void setSnap(double snap);
private:
    double pos;
    double vel;
    double acc;
    double jerk;
    double snap;
};


class TXData_two
{
public:
    TXData_two();
    void setPID_Kp(int axis,float kp);
    void setPID_Ki(int axis,float ki);
    void setPID_Kd(int axis,float kd);
    void setFeedForwardCoff(int axis,float value);
    void setTrajPos(int axis,double pos);
    void setTrajVel(int axis,double vel);
    void setTrajAcc(int axis, double acc);
    void setTrajJerk(int axis,double jerk);
    void setTrajSnap(int axis,double snap);
    void setAxis(int axis);
    void setSeq(int seq);
    void setCommand(command_e command);
    void doubleByteConvert(double *src);
    void intByteConvert(int *src);
    void floatByteConvert(float *src);
private:
    int seq;
    int axis;
    command_e commend;
    controller_t controller[10];
    traj_t traj[10];
    double jogSpeed[10];
};
typedef enum
{
    TCP=0,
    simulater
}ConnectType;

class comData_t
{
public:
    comData_t();
    void home();
    void setPos(int axis,double pos);
    void setVel(int axis,double vel);
    void setAcc(int axis,double acc);
    void setJerk(int axis,double jerk);
    void setFPos(int axis,double fPos);
    void moveAbs(int axis,double pos);
    void moveRel(int axis,double pos);
    void jog(int axis,double vel);
    void jogStop(int axis);
    void open(int axis);
    void close(int axis);
    void setCMD(int cmd);
    void setAxis(int axis);
    void doubleByteConvert(double *src);
    void intByteConvert(int *src);
    void floatByteConvert(float *src);
    void setKp(int axis,double kp);
    void setKi(int axis,double ki);
    void setKd(int axis,double kd);
    void setConnectType(ConnectType type);
private:
    int CMD;
    int axis;
    int iParamData[8];
    double dParamData[30];
};

#endif // COMMAND_T_H
