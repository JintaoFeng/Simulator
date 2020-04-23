#ifndef CONTROLOBJECT_H
#define CONTROLOBJECT_H


class ControlObject
{
public:
    ControlObject();
    ControlObject(double *a,double *b);
    void ControlProcess(double in);
private:
    double in[3];
    double out[3];
    double a[3];
    double b[3];
};

#endif // CONTROLOBJECT_H
