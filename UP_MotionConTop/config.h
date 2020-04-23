#ifndef CONFIG_H
#define CONFIG_H


typedef enum
{
    AXIS_0 = 0,
    AXIS_1,
    AXIS_2,
    AXIS_3,
    AXIS_4,
    AXIS_5,
    AXIS_6,
    AXIS_7,
    AXIS_8,
    AXIS_9
}AXIS_ENUM;

class axis_t
{
public:

private:

};
typedef unsigned short MOTOR_FLAG;
typedef unsigned short MOTION_FLAG;


#define MOTION_ENABLE   0X0001
#define MOTION_INPOS    0X0002
#define MOTION_COORD    0X0004
#define MOTION_ERROR    0X0008
#define MOTION_TELEOP   0X0010

#define MOTOR_ENABLE    0X0001
#define MOTOR_ACTIVE    0X0002
#define MOTOR_INPOS     0X0004
#define MOTOR_ERROR     0X0008
#define MOTOR_RIGHT_LIMIT   0X0010
#define MOTOR_LEFT_LIMIT    0X0020
#define MOTOR_HOMED     0X0040


class motor_t
{
public:

private:
    int type;
    double maxPosLimit;
    double minPosLimit;
    double velLimit;
    double accLimit;
    double minError;
    double maxError;
    MOTOR_FLAG flag;
    double posCmd;
    double velCmd;
    double accCmd;
    double posFeed;
    double velFeed;

};
typedef enum
{

}axisState;

typedef enum
{

}motorState;

typedef enum
{
    motionDisable=0,
    motionFree,
    motionTeleop,
    motionCoord,
    motionMaster,
    motionSlave
}motionState;

#endif // CONFIG_H
