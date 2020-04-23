#ifndef PID_T_H
#define PID_T_H

class PID_t
{
public:
 //   PID_t();
 //   ~pid_t();
    void setKp(float kp);
    void setKi(float ki);
    void setKd(float kd);
    float getKp();
    float getKi();
    float getKd();
private:
    float kp;
    float ki;
    float kd;
};
typedef enum
{
    PID,
    LLC
}controlerType;

class controller_t : public PID_t
{
public:
    void setFeedForwardCoff(float coff);
    float getFeedForwardCoff();
private:

    float feedForwardCoff;
};

#endif // PID_T_H
