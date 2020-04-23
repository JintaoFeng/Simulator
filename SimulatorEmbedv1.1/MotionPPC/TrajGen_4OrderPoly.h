#ifndef TRAJGEN_4ORDERPOLY_H
#define TRAJGEN_4ORDERPOLY_H

//#define CPM_FOR_TRAJ 1E7
#define CPM_FOR_TRAJ 1E6   //change on 2014-07-26
#define NUM_DELAY_PERIOD 2
#define FLOATEQU(O1,O2) ((fabs(O1 - O2) < 1E-9) ? 1 : 0)
#define PI    3.141592653

typedef struct{
	double Ts;
	double d;
	double Td;
	double Tj;
	double Ta;
	double Tv;
	double Tw;
	double a;
	double b;
	double c;
}s_TrajParam_4_Order_Poly;

typedef struct
{
	double amax;//目标加速度
	double vmax;//目标速度
	double smax;//目标行程
	int WaitCyc;
}s_TrajParam_Sine_Poly;

extern int TrajGen_4_Order_Poly(s_TrajParam_4_Order_Poly *TP, double *pd, double *pj, double *pa, double *pv, double *ps, int *c);
extern int X_TrajGen_4_Order_Poly(s_TrajParam_4_Order_Poly *TP, double *pd, double *pj, double *pa, double *pv, double *ps, int *c);
extern int Y_TrajGen_4_Order_Poly(s_TrajParam_4_Order_Poly *TP, double *pd, double *pj, double *pa, double *pv, double *ps, int *c);
extern int TrajGen_4_Order_Poly_Reciprocation(s_TrajParam_4_Order_Poly *TP, double *pd, double *pj, double *pa, double *pv, double *ps, int *c, long *pcount);
extern int TrajSine_Poly_Coarse(s_TrajParam_Sine_Poly *TP,double *a,double *v,double *s);
extern int TrajSine_Poly_Fine(s_TrajParam_Sine_Poly *TP,double *a,double *v,double *s);






extern int TrajGen4OrderX1(s_TrajParam_4_Order_Poly *TP, double *pd, double *pj, double *pa, double *pv, double *ps, int *c);

extern int TrajGen4OrderX2(s_TrajParam_4_Order_Poly *TP, double *pd, double *pj, double *pa, double *pv, double *ps, int *c);

extern int TrajGen4OrderY1(s_TrajParam_4_Order_Poly *TP, double *pd, double *pj, double *pa, double *pv, double *ps, int *c);

extern int TrajGen4OrderY2(s_TrajParam_4_Order_Poly *TP, double *pd, double *pj, double *pa, double *pv, double *ps, int *c);

extern int TrajGen4OrderFineX(s_TrajParam_4_Order_Poly *TP, double *pd, double *pj, double *pa, double *pv, double *ps, int *c);

extern int TrajGen4OrderFineY(s_TrajParam_4_Order_Poly *TP, double *pd, double *pj, double *pa, double *pv, double *ps, int *c);

extern int TrajGen4OrderFineTz(s_TrajParam_4_Order_Poly *TP, double *pd, double *pj, double *pa, double *pv, double *ps, int *c);





#endif
