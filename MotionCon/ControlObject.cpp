#include "ControlObject.h"
#include <memory.h>
ControlObject::ControlObject()
{
    memset(this,0,sizeof(ControlObject));
}
ControlObject::ControlObject(double *a,double *b)
{
    memcpy(this->a,a,sizeof(double)*3);
    memcpy(this->b,b,sizeof(double)*3);
}

void ControlObject::ControlProcess(double in)
{
    this->in[0]=in;
    for(int i=1;i<3;i++)
    {
        this->out[0] += this->a[i]*this->in[i];
    }
    for(int i=1;i<3;i++)
    {
        this->out[0] += this->b[i]*this->out[i];
    }
    for(int i=2;i>0;i--)
    {
        out[i]=out[i-1];
        this->in[i]=this->in[i-1];
    }
}
