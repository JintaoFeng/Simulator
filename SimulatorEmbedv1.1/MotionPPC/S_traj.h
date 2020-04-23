#ifndef S_TRAJ_H
#define S_TRAJ_H

#define MIN(a,b) a>b?a:b
typedef struct
{
	double pos;
	double vel;
	double acc;
	double jerk;
	
	double vlim;
	double v0;
	double v1;
	double alima;
	double alimd;
//	double 
	
	double Tj1;
	double Tj2;
	double Ta;
	double Td;
	double Tv;
	double La;
	double Ld;
	double Lv;
	double sigma;
}S_Traj_t;

void paramInit(S_Traj_t *traj);
int sign(double pos);
void trajProfile(S_Traj_t *traj,double shift,double vel,double acc,double jerk);
void calcTrajParam(S_Traj_t *traj);

#endif
