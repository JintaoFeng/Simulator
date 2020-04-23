#include "control.h"

void PID_t::setKp(float kp)
{
    this->kp=kp;
}
void PID_t::setKi(float ki)
{
    this->ki=ki;
}
void PID_t::setKd(float kd)
{
    this->kd=kd;
}
