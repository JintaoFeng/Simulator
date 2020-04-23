#ifndef SINGLETPG_H
#define SINGLETPG_H

#include "math.h"
#include "TrajGen_4OrderPoly.h"

#define GRAVITATIONAL_ACCELERATION (10.0)
#define MAX_FILENAME_LENGTH (255)

#define DEFAULT_ACCURACY (1E-9)
#define DEFAULT_FREQ (5000.0)
#define DEFAULT_TIMEINTERVAL (1.0 / DEFAULT_FREQ)
#define DEFAULT_DISPLACEMENT (0.1)
#define DEFAULT_VELOCITY (0.4)
#define DEFAULT_ACCELERATION (1.6 * GRAVITATIONAL_ACCELERATION)
#define DEFAULT_JERK (700.0)
#define DEFAULT_SNAP (120000.0)
#define DEFAULT_WAITTIME (0.1)
#define DEFAULT_FILENAMEPREFIX "SingleTPG_"

#define MAX_DISPLACEMENT (1.0)
#define MAX_VELOCITY (1.5)
#define MAX_ACCELERATION (5.0 * GRAVITATIONAL_ACCELERATION)
#define MAX_JERK (20000.0)
#define MAX_SNAP (2000000.0)

#define WARNING_DISPLACEMENT (0.4)

#define TURE 1
#define FALSE 0

//#define FLOATEQU(O1,O2) (fabs(O1 - O2) < DEFAULT_ACCURACY) ? TURE : FALSE

//const double tolerance = 2.220446049250313e-016;

#define tolerance   2.220446049250313e-016 


typedef enum {e_SUCCESS, e_ERRINPUT}Status;
//#define  e_SUCCESS 1
//#define  e_ERRINPUT 0



//typedef struct s_TParam s_TParam;
typedef struct {
	double p;
	double v;
	double a;
	double j;
	double d;
	double Ts;
	double r;
    double s;
}s_TParam;

//extern int TPGen_4Order_P2P(const s_TParam *TP, double *dd, double *Td, double *Tj, double *Ta, double *Tv, int cal_d_j );
int TPGen_4Order_P2P(const s_TParam *TP,s_TrajParam_4_Order_Poly *time);

double SIGN(double x);
#endif // SINGLETPG_H

