#include "TrajGen_4OrderPoly.h"
#include "math.h"
#include "stdlib.h"


int TrajGen_4_Order_Poly(s_TrajParam_4_Order_Poly *TP, double *pd, double *pj, double *pa, double *pv, double *ps, int *c)
{
    static double d[NUM_DELAY_PERIOD] = {0.0}, j[NUM_DELAY_PERIOD] = {0.0}, a[NUM_DELAY_PERIOD] = {0.0}, v[NUM_DELAY_PERIOD] = {0.0}, s[NUM_DELAY_PERIOD] = {0.0};
    //d[0]是当前周期值，d[1]是上周期值，类推
    static double t = -1.0;
    static int i = 0;
	static int Status = 0;
	static int shift = 0;
    static double ti[16] = {0,0,0,0,0,0};	//time intervals; t[0] = TotalTime
	if(2 == *c)
    {
	    t = -1.0;
	    for(i = 0; i < NUM_DELAY_PERIOD; i++)
        {
            d[i] = 0.0;
            j[i] = 0.0;
            a[i] = 0.0;
            v[i] = 0.0;
            s[i] = 0.0;
        }
	    d[0] = TP->d;
		Status = 0;
		*c = 0;
	}
	if(1 == *c)     //Start & Initialize
	{
	    t = TP->Ts / 2.0;
		*c = 0;
	    for(i = 0; i < NUM_DELAY_PERIOD; i++)
        {
            d[i] = 0.0;
            j[i] = 0.0;
            a[i] = 0.0;
            v[i] = 0.0;
            s[i] = 0.0;
        }
	    d[0] = TP->d;
		Status = 0;
        ti[1] = (TP->Td);
        ti[2] = (TP->Td) + (TP->Tj);
        ti[3] = 2.0 * (TP->Td) + (TP->Tj);
        ti[4] = 2.0 * (TP->Td) + (TP->Tj) + (TP->Ta);
        ti[5] = 3.0 * (TP->Td) + (TP->Tj) + (TP->Ta);
        ti[6] = 3.0 * (TP->Td) + 2.0 *(TP->Tj) + (TP->Ta);
        ti[7] = 4.0 * (TP->Td) + 2.0 *(TP->Tj) + (TP->Ta);
        ti[8] = 4.0 * (TP->Td) + 2.0 *(TP->Tj) + (TP->Ta) + (TP->Tv);
        ti[9] = 5.0 * (TP->Td) + 2.0 *(TP->Tj) + (TP->Ta) + (TP->Tv);
        ti[10] = 5.0 * (TP->Td) + 3.0 *(TP->Tj) + (TP->Ta) + (TP->Tv);
        ti[11] = 6.0 * (TP->Td) + 3.0 *(TP->Tj) + (TP->Ta) + (TP->Tv);
        ti[12] = 6.0 * (TP->Td) + 3.0 *(TP->Tj) + 2.0 * (TP->Ta) + (TP->Tv);
        ti[13] = 7.0 * (TP->Td) + 3.0 *(TP->Tj) + 2.0 * (TP->Ta) + (TP->Tv);
        ti[14] = 7.0 * (TP->Td) + 4.0 * (TP->Tj) + 2.0 * (TP->Ta) + (TP->Tv);
        ti[15] = 8.0 * (TP->Td) + 4.0 * (TP->Tj) + 2.0 * (TP->Ta) + (TP->Tv);
        if(FLOATEQU(TP->Tw,0.0))    
			ti[0] = ti[15] + TP->Ts;
        else                      
			ti[0] = ti[15] + TP->Tw;
	}
	if(0.0 <= t && t <= ti[0])
    {
        //Phase 1
        if(0.0 <= t && t <= ti[1])
        {
            d[0] = TP->d;
            Status = 1;
        }
        //Phase 2
        else if(ti[1] < t && t <= ti[2])
        {
            d[0] = 0.0;
            Status = 1;
        }
        //Phase 3
        else if(ti[2] < t && t <= ti[3])
        {
            d[0] = -TP->d;
            Status = 1;
        }
        //Phase 4
        else if(ti[3] < t && t <= ti[4])
        {
            d[0] = 0.0;
            Status = 1;
        }
        //Phase 5
        else if(ti[4] < t && t <= ti[5])
        {
            d[0] = -TP->d;
            Status = 1;
        }
        //Phase 6
        else if(ti[5] < t && t <= ti[6])
        {
            d[0] = 0.0;
            Status = 1;
        }
        //Phase 7
        else if(ti[6] < t && t <= ti[7])
        {
            d[0] = TP->d;
            Status = 1;
        }
        //Phase 8
        else if(ti[7] < t && t <= ti[8])
        {
            d[0] = 0.0;
            Status = 2;
        }
        //Phase 9
        else if(ti[8] < t && t <= ti[9])
        {
            d[0] = -TP->d;
            Status = 3;
        }
        //Phase 10
        else if(ti[9] < t && t <= ti[10])
        {
            d[0] = 0.0;
            Status = 3;
        }
        //Phase 11
        else if(ti[10] < t && t <= ti[11])
        {
            d[0] = TP->d;
            Status = 3;
        }
        //Phase 12
        else if(ti[11] < t && t <= ti[12])
        {
            d[0] = 0.0;
            Status = 3;
        }
        //Phase 13
        else if(ti[12] < t && t <= ti[13])
        {
            d[0] = TP->d;
            Status = 3;
        }
        //Phase 14
        else if(ti[13] < t && t <= ti[14])
        {
            d[0] = 0.0;
            Status = 3;
        }
        //Phase 15
        else if(ti[14] < t && t <= ti[15])
        {
            d[0] = -TP->d;
            Status = 3;
        }
        //Phase 16
        else if(ti[15] < t && t <= ti[0])
        {
            d[0] = 0;
            Status = 4;
        } 
        j[0] = j[1] + d[1] * TP->Ts;
        a[0] = a[1] + j[1] * TP->Ts;
        v[0] = v[1] + a[1] * TP->Ts;
        s[0] = s[1] + v[1] * TP->Ts;
        for(shift = NUM_DELAY_PERIOD - 1; shift > 0; shift --)
        {
            d[shift] = d[shift - 1];
            j[shift] = j[shift - 1];
            a[shift] = a[shift - 1];
            v[shift] = v[shift - 1];
            s[shift] = s[shift - 1];
        } 
        t += TP->Ts;  
    }
	else
    {
	    for(i = 0; i < NUM_DELAY_PERIOD; i++)
        {
            d[i] = 0.0;
            j[i] = 0.0;
            a[i] = 0.0;
            v[i] = 0.0;
            s[i] = 0.0;
        }
		Status = 0;
	} 

	*pd = CPM_FOR_TRAJ * d[0];
	*pj = CPM_FOR_TRAJ * j[0];
	*pa = CPM_FOR_TRAJ * a[0];
	*pv = CPM_FOR_TRAJ * v[0];
	*ps = CPM_FOR_TRAJ * s[0];

	*pd = floor(CPM_FOR_TRAJ * d[0]+0.5);
	*pj = floor(CPM_FOR_TRAJ * j[0]+0.5);
	*pa = floor(CPM_FOR_TRAJ * a[0]/100+0.5);
	*pv = floor(CPM_FOR_TRAJ * v[0]+0.5);
	*ps = floor(CPM_FOR_TRAJ * s[0]+0.5);

    return Status;
}


int TrajGen4OrderX1(s_TrajParam_4_Order_Poly *TP, double *pd, double *pj, double *pa, double *pv, double *ps, int *c)
{
    static double d[NUM_DELAY_PERIOD] = {0.0}, j[NUM_DELAY_PERIOD] = {0.0}, a[NUM_DELAY_PERIOD] = {0.0}, v[NUM_DELAY_PERIOD] = {0.0}, s[NUM_DELAY_PERIOD] = {0.0};
    //d[0]是当前周期值，d[1]是上周期值，类推
    static double t = -1.0;
    static int i = 0;
	static int Status = 0;
	static int shift = 0;
    static double ti[16] = {0,0,0,0,0,0};
	if(2 == *c)
    {
	    t = -1.0;
	    for(i = 0; i < NUM_DELAY_PERIOD; i++)
        {
            d[i] = 0.0;
            j[i] = 0.0;
            a[i] = 0.0;
            v[i] = 0.0;
            s[i] = 0.0;
        }
	    d[0] = TP->d;
		Status = 0;
		*c = 0;
	}
	if(1 == *c)     //Start & Initialize
	{
	    t = TP->Ts / 2.0;
		*c = 0;
	    for(i = 0; i < NUM_DELAY_PERIOD; i++)
        {
            d[i] = 0.0;
            j[i] = 0.0;
            a[i] = 0.0;
            v[i] = 0.0;
            s[i] = 0.0;
        }
	    d[0] = TP->d;
		Status = 0;
        ti[1] = (TP->Td);
        ti[2] = (TP->Td) + (TP->Tj);
        ti[3] = 2.0 * (TP->Td) + (TP->Tj);
        ti[4] = 2.0 * (TP->Td) + (TP->Tj) + (TP->Ta);
        ti[5] = 3.0 * (TP->Td) + (TP->Tj) + (TP->Ta);
        ti[6] = 3.0 * (TP->Td) + 2.0 *(TP->Tj) + (TP->Ta);
        ti[7] = 4.0 * (TP->Td) + 2.0 *(TP->Tj) + (TP->Ta);
        ti[8] = 4.0 * (TP->Td) + 2.0 *(TP->Tj) + (TP->Ta) + (TP->Tv);
        ti[9] = 5.0 * (TP->Td) + 2.0 *(TP->Tj) + (TP->Ta) + (TP->Tv);
        ti[10] = 5.0 * (TP->Td) + 3.0 *(TP->Tj) + (TP->Ta) + (TP->Tv);
        ti[11] = 6.0 * (TP->Td) + 3.0 *(TP->Tj) + (TP->Ta) + (TP->Tv);
        ti[12] = 6.0 * (TP->Td) + 3.0 *(TP->Tj) + 2.0 * (TP->Ta) + (TP->Tv);
        ti[13] = 7.0 * (TP->Td) + 3.0 *(TP->Tj) + 2.0 * (TP->Ta) + (TP->Tv);
        ti[14] = 7.0 * (TP->Td) + 4.0 * (TP->Tj) + 2.0 * (TP->Ta) + (TP->Tv);
        ti[15] = 8.0 * (TP->Td) + 4.0 * (TP->Tj) + 2.0 * (TP->Ta) + (TP->Tv);
        if(FLOATEQU(TP->Tw,0.0))    ti[0] = ti[15] + TP->Ts;
        else                        ti[0] = ti[15] + TP->Tw;
	}
	if(0.0 <= t && t <= ti[0])
    {
        //Phase 1
        if(0.0 <= t && t <= ti[1])
        {
            d[0] = TP->d;
            Status = 1;
        }
        //Phase 2
        else if(ti[1] < t && t <= ti[2])
        {
            d[0] = 0.0;
            Status = 1;
        }
        //Phase 3
        else if(ti[2] < t && t <= ti[3])
        {
            d[0] = -TP->d;
            Status = 1;
        }
        //Phase 4
        else if(ti[3] < t && t <= ti[4])
        {
            d[0] = 0.0;
            Status = 1;
        }
        //Phase 5
        else if(ti[4] < t && t <= ti[5])
        {
            d[0] = -TP->d;
            Status = 1;
        }
        //Phase 6
        else if(ti[5] < t && t <= ti[6])
        {
            d[0] = 0.0;
            Status = 1;
        }
        //Phase 7
        else if(ti[6] < t && t <= ti[7])
        {
            d[0] = TP->d;
            Status = 1;
        }
        //Phase 8
        else if(ti[7] < t && t <= ti[8])
        {
            d[0] = 0.0;
            Status = 2;
        }
        //Phase 9
        else if(ti[8] < t && t <= ti[9])
        {
            d[0] = -TP->d;
            Status = 3;
        }
        //Phase 10
        else if(ti[9] < t && t <= ti[10])
        {
            d[0] = 0.0;
            Status = 3;
        }
        //Phase 11
        else if(ti[10] < t && t <= ti[11])
        {
            d[0] = TP->d;
            Status = 3;
        }
        //Phase 12
        else if(ti[11] < t && t <= ti[12])
        {
            d[0] = 0.0;
            Status = 3;
        }
        //Phase 13
        else if(ti[12] < t && t <= ti[13])
        {
            d[0] = TP->d;
            Status = 3;
        }
        //Phase 14
        else if(ti[13] < t && t <= ti[14])
        {
            d[0] = 0.0;
            Status = 3;
        }
        //Phase 15
        else if(ti[14] < t && t <= ti[15])
        {
            d[0] = -TP->d;
            Status = 3;
        }
        //Phase 16
        else if(ti[15] < t && t <= ti[0])
        {
            d[0] = 0;
            Status = 4;
        } 
        j[0] = j[1] + d[1] * TP->Ts;
        a[0] = a[1] + j[1] * TP->Ts;
        v[0] = v[1] + a[1] * TP->Ts;
        s[0] = s[1] + v[1] * TP->Ts;
        for(shift = NUM_DELAY_PERIOD - 1; shift > 0; shift --)
        {
            d[shift] = d[shift - 1];
            j[shift] = j[shift - 1];
            a[shift] = a[shift - 1];
            v[shift] = v[shift - 1];
            s[shift] = s[shift - 1];
        } 
        t += TP->Ts;  
    }
	else
    {
	    for(i = 0; i < NUM_DELAY_PERIOD; i++)
        {
            d[i] = 0.0;
            j[i] = 0.0;
            a[i] = 0.0;
            v[i] = 0.0;
            s[i] = 0.0;
        }
		Status = 0;
	} 

    *pd = floor(CPM_FOR_TRAJ * d[0]+0.5);
    *pj = floor(CPM_FOR_TRAJ * j[0]+0.5);
    *pa = floor(CPM_FOR_TRAJ * a[0]+0.5);
    *pv = floor(CPM_FOR_TRAJ * v[0]+0.5);
    *ps = floor(CPM_FOR_TRAJ * s[0]+0.5);

    return Status;
}


int TrajGen4OrderX2(s_TrajParam_4_Order_Poly *TP, double *pd, double *pj, double *pa, double *pv, double *ps, int *c)
{
    static double d[NUM_DELAY_PERIOD] = {0.0}, j[NUM_DELAY_PERIOD] = {0.0}, a[NUM_DELAY_PERIOD] = {0.0}, v[NUM_DELAY_PERIOD] = {0.0}, s[NUM_DELAY_PERIOD] = {0.0};
    //d[0]是当前周期值，d[1]是上周期值，类推
    static double t = -1.0;
    static int i = 0;
	static int Status = 0;
	static int shift = 0;
    static double ti[16] = {0,0,0,0,0,0};	//time intervals; t[0] = TotalTime
	if(2 == *c)
    {
	    t = -1.0;
	    for(i = 0; i < NUM_DELAY_PERIOD; i++)
        {
            d[i] = 0.0;
            j[i] = 0.0;
            a[i] = 0.0;
            v[i] = 0.0;
            s[i] = 0.0;
        }
	    d[0] = TP->d;
		Status = 0;
		*c = 0;
	}
	if(1 == *c)     //Start & Initialize
	{
	    t = TP->Ts / 2.0;
		*c = 0;
	    for(i = 0; i < NUM_DELAY_PERIOD; i++)
        {
            d[i] = 0.0;
            j[i] = 0.0;
            a[i] = 0.0;
            v[i] = 0.0;
            s[i] = 0.0;
        }
	    d[0] = TP->d;
		Status = 0;
        ti[1] = (TP->Td);
        ti[2] = (TP->Td) + (TP->Tj);
        ti[3] = 2.0 * (TP->Td) + (TP->Tj);
        ti[4] = 2.0 * (TP->Td) + (TP->Tj) + (TP->Ta);
        ti[5] = 3.0 * (TP->Td) + (TP->Tj) + (TP->Ta);
        ti[6] = 3.0 * (TP->Td) + 2.0 *(TP->Tj) + (TP->Ta);
        ti[7] = 4.0 * (TP->Td) + 2.0 *(TP->Tj) + (TP->Ta);
        ti[8] = 4.0 * (TP->Td) + 2.0 *(TP->Tj) + (TP->Ta) + (TP->Tv);
        ti[9] = 5.0 * (TP->Td) + 2.0 *(TP->Tj) + (TP->Ta) + (TP->Tv);
        ti[10] = 5.0 * (TP->Td) + 3.0 *(TP->Tj) + (TP->Ta) + (TP->Tv);
        ti[11] = 6.0 * (TP->Td) + 3.0 *(TP->Tj) + (TP->Ta) + (TP->Tv);
        ti[12] = 6.0 * (TP->Td) + 3.0 *(TP->Tj) + 2.0 * (TP->Ta) + (TP->Tv);
        ti[13] = 7.0 * (TP->Td) + 3.0 *(TP->Tj) + 2.0 * (TP->Ta) + (TP->Tv);
        ti[14] = 7.0 * (TP->Td) + 4.0 * (TP->Tj) + 2.0 * (TP->Ta) + (TP->Tv);
        ti[15] = 8.0 * (TP->Td) + 4.0 * (TP->Tj) + 2.0 * (TP->Ta) + (TP->Tv);

		
        if(FLOATEQU(TP->Tw,0.0))    ti[0] = ti[15] + TP->Ts;
        else                        ti[0] = ti[15] + TP->Tw;
	}
	if(0.0 <= t && t <= ti[0])
    {
        //Phase 1
        if(0.0 <= t && t <= ti[1])
        {
            d[0] = TP->d;
            Status = 1;
        }
        //Phase 2
        else if(ti[1] < t && t <= ti[2])
        {
            d[0] = 0.0;
            Status = 1;
        }
        //Phase 3
        else if(ti[2] < t && t <= ti[3])
        {
            d[0] = -TP->d;
            Status = 1;
        }
        //Phase 4
        else if(ti[3] < t && t <= ti[4])
        {
            d[0] = 0.0;
            Status = 1;
        }
        //Phase 5
        else if(ti[4] < t && t <= ti[5])
        {
            d[0] = -TP->d;
            Status = 1;
        }
        //Phase 6
        else if(ti[5] < t && t <= ti[6])
        {
            d[0] = 0.0;
            Status = 1;
        }
        //Phase 7
        else if(ti[6] < t && t <= ti[7])
        {
            d[0] = TP->d;
            Status = 1;
        }
        //Phase 8
        else if(ti[7] < t && t <= ti[8])
        {
            d[0] = 0.0;
            Status = 2;
        }
        //Phase 9
        else if(ti[8] < t && t <= ti[9])
        {
            d[0] = -TP->d;
            Status = 3;
        }
        //Phase 10
        else if(ti[9] < t && t <= ti[10])
        {
            d[0] = 0.0;
            Status = 3;
        }
        //Phase 11
        else if(ti[10] < t && t <= ti[11])
        {
            d[0] = TP->d;
            Status = 3;
        }
        //Phase 12
        else if(ti[11] < t && t <= ti[12])
        {
            d[0] = 0.0;
            Status = 3;
        }
        //Phase 13
        else if(ti[12] < t && t <= ti[13])
        {
            d[0] = TP->d;
            Status = 3;
        }
        //Phase 14
        else if(ti[13] < t && t <= ti[14])
        {
            d[0] = 0.0;
            Status = 3;
        }
        //Phase 15
        else if(ti[14] < t && t <= ti[15])
        {
            d[0] = -TP->d;
            Status = 3;
        }
        //Phase 16
        else if(ti[15] < t && t <= ti[0])
        {
            d[0] = 0;
            Status = 4;
        } 
        j[0] = j[1] + d[1] * TP->Ts;
        a[0] = a[1] + j[1] * TP->Ts;
        v[0] = v[1] + a[1] * TP->Ts;
        s[0] = s[1] + v[1] * TP->Ts;
        for(shift = NUM_DELAY_PERIOD - 1; shift > 0; shift --)
        {
            d[shift] = d[shift - 1];
            j[shift] = j[shift - 1];
            a[shift] = a[shift - 1];
            v[shift] = v[shift - 1];
            s[shift] = s[shift - 1];
        } 
        t += TP->Ts;  
    }
	else
    {
	    for(i = 0; i < NUM_DELAY_PERIOD; i++)
        {
            d[i] = 0.0;
            j[i] = 0.0;
            a[i] = 0.0;
            v[i] = 0.0;
            s[i] = 0.0;
        }
		Status = 0;
	} 

	// *pd = CPM_FOR_TRAJ * d[0];
	// *pj = CPM_FOR_TRAJ * j[0];
	// *pa = CPM_FOR_TRAJ * a[0];
	// *pv = CPM_FOR_TRAJ * v[0];
	// *ps = CPM_FOR_TRAJ * s[0];

    *pd = floor(CPM_FOR_TRAJ * d[0]+0.5);
    *pj = floor(CPM_FOR_TRAJ * j[0]+0.5);
    *pa = floor(CPM_FOR_TRAJ * a[0]+0.5);
    *pv = floor(CPM_FOR_TRAJ * v[0]+0.5);
    *ps = floor(CPM_FOR_TRAJ * s[0]+0.5);

	// *pd = _round(CPM_FOR_TRAJ * d[0]);
	// *pj = _round(CPM_FOR_TRAJ * j[0]);
	// *pa = _round(CPM_FOR_TRAJ * a[0]);
	// *pv = _round(CPM_FOR_TRAJ * v[0]);
	// *ps = _round(CPM_FOR_TRAJ * s[0]);

    return Status;
}




int TrajGen4OrderY1(s_TrajParam_4_Order_Poly *TP, double *pd, double *pj, double *pa, double *pv, double *ps, int *c)
{
    static double d[NUM_DELAY_PERIOD] = {0.0}, j[NUM_DELAY_PERIOD] = {0.0}, a[NUM_DELAY_PERIOD] = {0.0}, v[NUM_DELAY_PERIOD] = {0.0}, s[NUM_DELAY_PERIOD] = {0.0};
    //d[0]是当前周期值，d[1]是上周期值，类推
    static double t = -1.0;
    static int i = 0;
	static int Status = 0;
	static int shift = 0;
    static double ti[16] = {0,0,0,0,0,0};	//time intervals; t[0] = TotalTime
	if(2 == *c)
    {
	    t = -1.0;
	    for(i = 0; i < NUM_DELAY_PERIOD; i++)
        {
            d[i] = 0.0;
            j[i] = 0.0;
            a[i] = 0.0;
            v[i] = 0.0;
            s[i] = 0.0;
        }
	    d[0] = TP->d;
		Status = 0;
		*c = 0;
	}
	if(1 == *c)     //Start & Initialize
	{
	    t = TP->Ts / 2.0;
		*c = 0;
	    for(i = 0; i < NUM_DELAY_PERIOD; i++)
        {
            d[i] = 0.0;
            j[i] = 0.0;
            a[i] = 0.0;
            v[i] = 0.0;
            s[i] = 0.0;
        }
	    d[0] = TP->d;
		Status = 0;
        ti[1] = (TP->Td);
        ti[2] = (TP->Td) + (TP->Tj);
        ti[3] = 2.0 * (TP->Td) + (TP->Tj);
        ti[4] = 2.0 * (TP->Td) + (TP->Tj) + (TP->Ta);
        ti[5] = 3.0 * (TP->Td) + (TP->Tj) + (TP->Ta);
        ti[6] = 3.0 * (TP->Td) + 2.0 *(TP->Tj) + (TP->Ta);
        ti[7] = 4.0 * (TP->Td) + 2.0 *(TP->Tj) + (TP->Ta);
        ti[8] = 4.0 * (TP->Td) + 2.0 *(TP->Tj) + (TP->Ta) + (TP->Tv);
        ti[9] = 5.0 * (TP->Td) + 2.0 *(TP->Tj) + (TP->Ta) + (TP->Tv);
        ti[10] = 5.0 * (TP->Td) + 3.0 *(TP->Tj) + (TP->Ta) + (TP->Tv);
        ti[11] = 6.0 * (TP->Td) + 3.0 *(TP->Tj) + (TP->Ta) + (TP->Tv);
        ti[12] = 6.0 * (TP->Td) + 3.0 *(TP->Tj) + 2.0 * (TP->Ta) + (TP->Tv);
        ti[13] = 7.0 * (TP->Td) + 3.0 *(TP->Tj) + 2.0 * (TP->Ta) + (TP->Tv);
        ti[14] = 7.0 * (TP->Td) + 4.0 * (TP->Tj) + 2.0 * (TP->Ta) + (TP->Tv);
        ti[15] = 8.0 * (TP->Td) + 4.0 * (TP->Tj) + 2.0 * (TP->Ta) + (TP->Tv);
        if(FLOATEQU(TP->Tw,0.0))    ti[0] = ti[15] + TP->Ts;
        else                        ti[0] = ti[15] + TP->Tw;
	}
	if(0.0 <= t && t <= ti[0])
    {
        //Phase 1
        if(0.0 <= t && t <= ti[1])
        {
            d[0] = TP->d;
            Status = 1;
        }
        //Phase 2
        else if(ti[1] < t && t <= ti[2])
        {
            d[0] = 0.0;
            Status = 1;
        }
        //Phase 3
        else if(ti[2] < t && t <= ti[3])
        {
            d[0] = -TP->d;
            Status = 1;
        }
        //Phase 4
        else if(ti[3] < t && t <= ti[4])
        {
            d[0] = 0.0;
            Status = 1;
        }
        //Phase 5
        else if(ti[4] < t && t <= ti[5])
        {
            d[0] = -TP->d;
            Status = 1;
        }
        //Phase 6
        else if(ti[5] < t && t <= ti[6])
        {
            d[0] = 0.0;
            Status = 1;
        }
        //Phase 7
        else if(ti[6] < t && t <= ti[7])
        {
            d[0] = TP->d;
            Status = 1;
        }
        //Phase 8
        else if(ti[7] < t && t <= ti[8])
        {
            d[0] = 0.0;
            Status = 2;
        }
        //Phase 9
        else if(ti[8] < t && t <= ti[9])
        {
            d[0] = -TP->d;
            Status = 3;
        }
        //Phase 10
        else if(ti[9] < t && t <= ti[10])
        {
            d[0] = 0.0;
            Status = 3;
        }
        //Phase 11
        else if(ti[10] < t && t <= ti[11])
        {
            d[0] = TP->d;
            Status = 3;
        }
        //Phase 12
        else if(ti[11] < t && t <= ti[12])
        {
            d[0] = 0.0;
            Status = 3;
        }
        //Phase 13
        else if(ti[12] < t && t <= ti[13])
        {
            d[0] = TP->d;
            Status = 3;
        }
        //Phase 14
        else if(ti[13] < t && t <= ti[14])
        {
            d[0] = 0.0;
            Status = 3;
        }
        //Phase 15
        else if(ti[14] < t && t <= ti[15])
        {
            d[0] = -TP->d;
            Status = 3;
        }
        //Phase 16
        else if(ti[15] < t && t <= ti[0])
        {
            d[0] = 0;
            Status = 4;
        } 
        j[0] = j[1] + d[1] * TP->Ts;
        a[0] = a[1] + j[1] * TP->Ts;
        v[0] = v[1] + a[1] * TP->Ts;
        s[0] = s[1] + v[1] * TP->Ts;
        for(shift = NUM_DELAY_PERIOD - 1; shift > 0; shift --)
        {
            d[shift] = d[shift - 1];
            j[shift] = j[shift - 1];
            a[shift] = a[shift - 1];
            v[shift] = v[shift - 1];
            s[shift] = s[shift - 1];
        } 
        t += TP->Ts;  
    }
	else
    {
	    for(i = 0; i < NUM_DELAY_PERIOD; i++)
        {
            d[i] = 0.0;
            j[i] = 0.0;
            a[i] = 0.0;
            v[i] = 0.0;
            s[i] = 0.0;
        }
		Status = 0;
	} 

	// *pd = CPM_FOR_TRAJ * d[0];
	// *pj = CPM_FOR_TRAJ * j[0];
	// *pa = CPM_FOR_TRAJ * a[0];
	// *pv = CPM_FOR_TRAJ * v[0];
	// *ps = CPM_FOR_TRAJ * s[0];

    *pd = floor(CPM_FOR_TRAJ * d[0]+0.5);
    *pj = floor(CPM_FOR_TRAJ * j[0]+0.5);
    *pa = floor(CPM_FOR_TRAJ * a[0]+0.5);
    *pv = floor(CPM_FOR_TRAJ * v[0]+0.5);
    *ps = floor(CPM_FOR_TRAJ * s[0]+0.5);

	// *pd = _round(CPM_FOR_TRAJ * d[0]);
	// *pj = _round(CPM_FOR_TRAJ * j[0]);
	// *pa = _round(CPM_FOR_TRAJ * a[0]);
	// *pv = _round(CPM_FOR_TRAJ * v[0]);
	// *ps = _round(CPM_FOR_TRAJ * s[0]);

    return Status;
}




int TrajGen4OrderY2(s_TrajParam_4_Order_Poly *TP, double *pd, double *pj, double *pa, double *pv, double *ps, int *c)
{
    static double d[NUM_DELAY_PERIOD] = {0.0}, j[NUM_DELAY_PERIOD] = {0.0}, a[NUM_DELAY_PERIOD] = {0.0}, v[NUM_DELAY_PERIOD] = {0.0}, s[NUM_DELAY_PERIOD] = {0.0};
    //d[0]是当前周期值，d[1]是上周期值，类推
    static double t = -1.0;
    static int i = 0;
	static int Status = 0;
	static int shift = 0;
    static double ti[16] = {0,0,0,0,0,0};	//time intervals; t[0] = TotalTime
	if(2 == *c)
    {
	    t = -1.0;
	    for(i = 0; i < NUM_DELAY_PERIOD; i++)
        {
            d[i] = 0.0;
            j[i] = 0.0;
            a[i] = 0.0;
            v[i] = 0.0;
            s[i] = 0.0;
        }
	    d[0] = TP->d;
		Status = 0;
		*c = 0;
	}
	if(1 == *c)     //Start & Initialize
	{
	    t = TP->Ts / 2.0;
		*c = 0;
	    for(i = 0; i < NUM_DELAY_PERIOD; i++)
        {
            d[i] = 0.0;
            j[i] = 0.0;
            a[i] = 0.0;
            v[i] = 0.0;
            s[i] = 0.0;
        }
	    d[0] = TP->d;
		Status = 0;
        ti[1] = (TP->Td);
        ti[2] = (TP->Td) + (TP->Tj);
        ti[3] = 2.0 * (TP->Td) + (TP->Tj);
        ti[4] = 2.0 * (TP->Td) + (TP->Tj) + (TP->Ta);
        ti[5] = 3.0 * (TP->Td) + (TP->Tj) + (TP->Ta);
        ti[6] = 3.0 * (TP->Td) + 2.0 *(TP->Tj) + (TP->Ta);
        ti[7] = 4.0 * (TP->Td) + 2.0 *(TP->Tj) + (TP->Ta);
        ti[8] = 4.0 * (TP->Td) + 2.0 *(TP->Tj) + (TP->Ta) + (TP->Tv);
        ti[9] = 5.0 * (TP->Td) + 2.0 *(TP->Tj) + (TP->Ta) + (TP->Tv);
        ti[10] = 5.0 * (TP->Td) + 3.0 *(TP->Tj) + (TP->Ta) + (TP->Tv);
        ti[11] = 6.0 * (TP->Td) + 3.0 *(TP->Tj) + (TP->Ta) + (TP->Tv);
        ti[12] = 6.0 * (TP->Td) + 3.0 *(TP->Tj) + 2.0 * (TP->Ta) + (TP->Tv);
        ti[13] = 7.0 * (TP->Td) + 3.0 *(TP->Tj) + 2.0 * (TP->Ta) + (TP->Tv);
        ti[14] = 7.0 * (TP->Td) + 4.0 * (TP->Tj) + 2.0 * (TP->Ta) + (TP->Tv);
        ti[15] = 8.0 * (TP->Td) + 4.0 * (TP->Tj) + 2.0 * (TP->Ta) + (TP->Tv);
        if(FLOATEQU(TP->Tw,0.0))    ti[0] = ti[15] + TP->Ts;
        else                        ti[0] = ti[15] + TP->Tw;
	}
	if(0.0 <= t && t <= ti[0])
    {
        //Phase 1
        if(0.0 <= t && t <= ti[1])
        {
            d[0] = TP->d;
            Status = 1;
        }
        //Phase 2
        else if(ti[1] < t && t <= ti[2])
        {
            d[0] = 0.0;
            Status = 1;
        }
        //Phase 3
        else if(ti[2] < t && t <= ti[3])
        {
            d[0] = -TP->d;
            Status = 1;
        }
        //Phase 4
        else if(ti[3] < t && t <= ti[4])
        {
            d[0] = 0.0;
            Status = 1;
        }
        //Phase 5
        else if(ti[4] < t && t <= ti[5])
        {
            d[0] = -TP->d;
            Status = 1;
        }
        //Phase 6
        else if(ti[5] < t && t <= ti[6])
        {
            d[0] = 0.0;
            Status = 1;
        }
        //Phase 7
        else if(ti[6] < t && t <= ti[7])
        {
            d[0] = TP->d;
            Status = 1;
        }
        //Phase 8
        else if(ti[7] < t && t <= ti[8])
        {
            d[0] = 0.0;
            Status = 2;
        }
        //Phase 9
        else if(ti[8] < t && t <= ti[9])
        {
            d[0] = -TP->d;
            Status = 3;
        }
        //Phase 10
        else if(ti[9] < t && t <= ti[10])
        {
            d[0] = 0.0;
            Status = 3;
        }
        //Phase 11
        else if(ti[10] < t && t <= ti[11])
        {
            d[0] = TP->d;
            Status = 3;
        }
        //Phase 12
        else if(ti[11] < t && t <= ti[12])
        {
            d[0] = 0.0;
            Status = 3;
        }
        //Phase 13
        else if(ti[12] < t && t <= ti[13])
        {
            d[0] = TP->d;
            Status = 3;
        }
        //Phase 14
        else if(ti[13] < t && t <= ti[14])
        {
            d[0] = 0.0;
            Status = 3;
        }
        //Phase 15
        else if(ti[14] < t && t <= ti[15])
        {
            d[0] = -TP->d;
            Status = 3;
        }
        //Phase 16
        else if(ti[15] < t && t <= ti[0])
        {
            d[0] = 0;
            Status = 4;
        } 
        j[0] = j[1] + d[1] * TP->Ts;
        a[0] = a[1] + j[1] * TP->Ts;
        v[0] = v[1] + a[1] * TP->Ts;
        s[0] = s[1] + v[1] * TP->Ts;
        for(shift = NUM_DELAY_PERIOD - 1; shift > 0; shift --)
        {
            d[shift] = d[shift - 1];
            j[shift] = j[shift - 1];
            a[shift] = a[shift - 1];
            v[shift] = v[shift - 1];
            s[shift] = s[shift - 1];
        } 
        t += TP->Ts;  
    }
	else
    {
	    for(i = 0; i < NUM_DELAY_PERIOD; i++)
        {
            d[i] = 0.0;
            j[i] = 0.0;
            a[i] = 0.0;
            v[i] = 0.0;
            s[i] = 0.0;
        }
		Status = 0;
	} 

	*pd = CPM_FOR_TRAJ * d[0];
	*pj = CPM_FOR_TRAJ * j[0];
	*pa = CPM_FOR_TRAJ * a[0];
	*pv = CPM_FOR_TRAJ * v[0];
	*ps = CPM_FOR_TRAJ * s[0];

	*pd = floor(CPM_FOR_TRAJ * d[0]+0.5);
	*pj = floor(CPM_FOR_TRAJ * j[0]+0.5);
	*pa = floor(CPM_FOR_TRAJ * a[0]/100+0.5);
	*pv = floor(CPM_FOR_TRAJ * v[0]+0.5);
	*ps = floor(CPM_FOR_TRAJ * s[0]+0.5);
    return Status;
}




int TrajGen4OrderFineX(s_TrajParam_4_Order_Poly *TP, double *pd, double *pj, double *pa, double *pv, double *ps, int *c)
{
    static double d[NUM_DELAY_PERIOD] = {0.0}, j[NUM_DELAY_PERIOD] = {0.0}, a[NUM_DELAY_PERIOD] = {0.0}, v[NUM_DELAY_PERIOD] = {0.0}, s[NUM_DELAY_PERIOD] = {0.0};
    //d[0]是当前周期值，d[1]是上周期值，类推
    static double t = -1.0;
    static int i = 0;
	static int Status = 0;
	static int shift = 0;
    static double ti[16] = {0,0,0,0,0,0};	//time intervals; t[0] = TotalTime
	if(2 == *c)
    {
	    t = -1.0;
	    for(i = 0; i < NUM_DELAY_PERIOD; i++)
        {
            d[i] = 0.0;
            j[i] = 0.0;
            a[i] = 0.0;
            v[i] = 0.0;
            s[i] = 0.0;
        }
	    d[0] = TP->d;
		Status = 0;
		*c = 0;
	}
	if(1 == *c)     //Start & Initialize
	{
	    t = TP->Ts / 2.0;
		*c = 0;
	    for(i = 0; i < NUM_DELAY_PERIOD; i++)
        {
            d[i] = 0.0;
            j[i] = 0.0;
            a[i] = 0.0;
            v[i] = 0.0;
            s[i] = 0.0;
        }
	    d[0] = TP->d;
		Status = 0;
        ti[1] = (TP->Td);
        ti[2] = (TP->Td) + (TP->Tj);
        ti[3] = 2.0 * (TP->Td) + (TP->Tj);
        ti[4] = 2.0 * (TP->Td) + (TP->Tj) + (TP->Ta);
        ti[5] = 3.0 * (TP->Td) + (TP->Tj) + (TP->Ta);
        ti[6] = 3.0 * (TP->Td) + 2.0 *(TP->Tj) + (TP->Ta);
        ti[7] = 4.0 * (TP->Td) + 2.0 *(TP->Tj) + (TP->Ta);
        ti[8] = 4.0 * (TP->Td) + 2.0 *(TP->Tj) + (TP->Ta) + (TP->Tv);
        ti[9] = 5.0 * (TP->Td) + 2.0 *(TP->Tj) + (TP->Ta) + (TP->Tv);
        ti[10] = 5.0 * (TP->Td) + 3.0 *(TP->Tj) + (TP->Ta) + (TP->Tv);
        ti[11] = 6.0 * (TP->Td) + 3.0 *(TP->Tj) + (TP->Ta) + (TP->Tv);
        ti[12] = 6.0 * (TP->Td) + 3.0 *(TP->Tj) + 2.0 * (TP->Ta) + (TP->Tv);
        ti[13] = 7.0 * (TP->Td) + 3.0 *(TP->Tj) + 2.0 * (TP->Ta) + (TP->Tv);
        ti[14] = 7.0 * (TP->Td) + 4.0 * (TP->Tj) + 2.0 * (TP->Ta) + (TP->Tv);
        ti[15] = 8.0 * (TP->Td) + 4.0 * (TP->Tj) + 2.0 * (TP->Ta) + (TP->Tv);
        if(FLOATEQU(TP->Tw,0.0))  
			ti[0] = ti[15] + TP->Ts;
        else                      
			ti[0] = ti[15] + TP->Tw;
	}
	if(0.0 <= t && t <= ti[0])
    {
        //Phase 1
        if(0.0 <= t && t <= ti[1])
        {
            d[0] = TP->d;
            Status = 1;
        }
        //Phase 2
        else if(ti[1] < t && t <= ti[2])
        {
            d[0] = 0.0;
            Status = 1;
        }
        //Phase 3
        else if(ti[2] < t && t <= ti[3])
        {
            d[0] = -TP->d;
            Status = 1;
        }
        //Phase 4
        else if(ti[3] < t && t <= ti[4])
        {
            d[0] = 0.0;
            Status = 1;
        }
        //Phase 5
        else if(ti[4] < t && t <= ti[5])
        {
            d[0] = -TP->d;
            Status = 1;
        }
        //Phase 6
        else if(ti[5] < t && t <= ti[6])
        {
            d[0] = 0.0;
            Status = 1;
        }
        //Phase 7
        else if(ti[6] < t && t <= ti[7])
        {
            d[0] = TP->d;
            Status = 1;
        }
        //Phase 8
        else if(ti[7] < t && t <= ti[8])
        {
            d[0] = 0.0;
            Status = 2;
        }
        //Phase 9
        else if(ti[8] < t && t <= ti[9])
        {
            d[0] = -TP->d;
            Status = 3;
        }
        //Phase 10
        else if(ti[9] < t && t <= ti[10])
        {
            d[0] = 0.0;
            Status = 3;
        }
        //Phase 11
        else if(ti[10] < t && t <= ti[11])
        {
            d[0] = TP->d;
            Status = 3;
        }
        //Phase 12
        else if(ti[11] < t && t <= ti[12])
        {
            d[0] = 0.0;
            Status = 3;
        }
        //Phase 13
        else if(ti[12] < t && t <= ti[13])
        {
            d[0] = TP->d;
            Status = 3;
        }
        //Phase 14
        else if(ti[13] < t && t <= ti[14])
        {
            d[0] = 0.0;
            Status = 3;
        }
        //Phase 15
        else if(ti[14] < t && t <= ti[15])
        {
            d[0] = -TP->d;
            Status = 3;
        }
        //Phase 16
        else if(ti[15] < t && t <= ti[0])
        {
            d[0] = 0;
            Status = 4;
        } 
        j[0] = j[1] + d[1] * TP->Ts;
        a[0] = a[1] + j[1] * TP->Ts;
        v[0] = v[1] + a[1] * TP->Ts;
        s[0] = s[1] + v[1] * TP->Ts;
        for(shift = NUM_DELAY_PERIOD - 1; shift > 0; shift --)
        {
            d[shift] = d[shift - 1];
            j[shift] = j[shift - 1];
            a[shift] = a[shift - 1];
            v[shift] = v[shift - 1];
            s[shift] = s[shift - 1];
        } 
        t += TP->Ts;  
    }
	else
    {
	    for(i = 0; i < NUM_DELAY_PERIOD; i++)
        {
            d[i] = 0.0;
            j[i] = 0.0;
            a[i] = 0.0;
            v[i] = 0.0;
            s[i] = 0.0;
        }
		Status = 0;
	} 

	// *pd = CPM_FOR_TRAJ * d[0];
	// *pj = CPM_FOR_TRAJ * j[0];
	// *pa = CPM_FOR_TRAJ * a[0];
	// *pv = CPM_FOR_TRAJ * v[0];
	// *ps = CPM_FOR_TRAJ * s[0];

    *pd = floor(CPM_FOR_TRAJ * d[0]+0.5);
    *pj = floor(CPM_FOR_TRAJ * j[0]+0.5);
    *pa = floor(CPM_FOR_TRAJ * a[0]+0.5);
    *pv = floor(CPM_FOR_TRAJ * v[0]+0.5);
    *ps = floor(CPM_FOR_TRAJ * s[0]+0.5);

	// *pd = _round(CPM_FOR_TRAJ * d[0]);
	// *pj = _round(CPM_FOR_TRAJ * j[0]);
	// *pa = _round(CPM_FOR_TRAJ * a[0]);
	// *pv = _round(CPM_FOR_TRAJ * v[0]);
	// *ps = _round(CPM_FOR_TRAJ * s[0]);

    return Status;
}




int TrajGen4OrderFineY(s_TrajParam_4_Order_Poly *TP, double *pd, double *pj, double *pa, double *pv, double *ps, int *c)
{
    static double d[NUM_DELAY_PERIOD] = {0.0}, j[NUM_DELAY_PERIOD] = {0.0}, a[NUM_DELAY_PERIOD] = {0.0}, v[NUM_DELAY_PERIOD] = {0.0}, s[NUM_DELAY_PERIOD] = {0.0};
    //d[0]是当前周期值，d[1]是上周期值，类推
    static double t = -1.0;
    static int i = 0;
	static int Status = 0;
	static int shift = 0;
    static double ti[16] = {0,0,0,0,0,0};	//time intervals; t[0] = TotalTime
	if(2 == *c)
    {
	    t = -1.0;
	    for(i = 0; i < NUM_DELAY_PERIOD; i++)
        {
            d[i] = 0.0;
            j[i] = 0.0;
            a[i] = 0.0;
            v[i] = 0.0;
            s[i] = 0.0;
        }
	    d[0] = TP->d;
		Status = 0;
		*c = 0;
	}
	if(1 == *c)     //Start & Initialize
	{
	    t = TP->Ts / 2.0;
		*c = 0;
	    for(i = 0; i < NUM_DELAY_PERIOD; i++)
        {
            d[i] = 0.0;
            j[i] = 0.0;
            a[i] = 0.0;
            v[i] = 0.0;
            s[i] = 0.0;
        }
	    d[0] = TP->d;
		Status = 0;
        ti[1] = (TP->Td);
        ti[2] = (TP->Td) + (TP->Tj);
        ti[3] = 2.0 * (TP->Td) + (TP->Tj);
        ti[4] = 2.0 * (TP->Td) + (TP->Tj) + (TP->Ta);
        ti[5] = 3.0 * (TP->Td) + (TP->Tj) + (TP->Ta);
        ti[6] = 3.0 * (TP->Td) + 2.0 *(TP->Tj) + (TP->Ta);
        ti[7] = 4.0 * (TP->Td) + 2.0 *(TP->Tj) + (TP->Ta);
        ti[8] = 4.0 * (TP->Td) + 2.0 *(TP->Tj) + (TP->Ta) + (TP->Tv);
        ti[9] = 5.0 * (TP->Td) + 2.0 *(TP->Tj) + (TP->Ta) + (TP->Tv);
        ti[10] = 5.0 * (TP->Td) + 3.0 *(TP->Tj) + (TP->Ta) + (TP->Tv);
        ti[11] = 6.0 * (TP->Td) + 3.0 *(TP->Tj) + (TP->Ta) + (TP->Tv);
        ti[12] = 6.0 * (TP->Td) + 3.0 *(TP->Tj) + 2.0 * (TP->Ta) + (TP->Tv);
        ti[13] = 7.0 * (TP->Td) + 3.0 *(TP->Tj) + 2.0 * (TP->Ta) + (TP->Tv);
        ti[14] = 7.0 * (TP->Td) + 4.0 * (TP->Tj) + 2.0 * (TP->Ta) + (TP->Tv);
        ti[15] = 8.0 * (TP->Td) + 4.0 * (TP->Tj) + 2.0 * (TP->Ta) + (TP->Tv);
        if(FLOATEQU(TP->Tw,0.0))    ti[0] = ti[15] + TP->Ts;
        else                        ti[0] = ti[15] + TP->Tw;
	}
	if(0.0 <= t && t <= ti[0])
    {
        //Phase 1
        if(0.0 <= t && t <= ti[1])
        {
            d[0] = TP->d;
            Status = 1;
        }
        //Phase 2
        else if(ti[1] < t && t <= ti[2])
        {
            d[0] = 0.0;
            Status = 1;
        }
        //Phase 3
        else if(ti[2] < t && t <= ti[3])
        {
            d[0] = -TP->d;
            Status = 1;
        }
        //Phase 4
        else if(ti[3] < t && t <= ti[4])
        {
            d[0] = 0.0;
            Status = 1;
        }
        //Phase 5
        else if(ti[4] < t && t <= ti[5])
        {
            d[0] = -TP->d;
            Status = 1;
        }
        //Phase 6
        else if(ti[5] < t && t <= ti[6])
        {
            d[0] = 0.0;
            Status = 1;
        }
        //Phase 7
        else if(ti[6] < t && t <= ti[7])
        {
            d[0] = TP->d;
            Status = 1;
        }
        //Phase 8
        else if(ti[7] < t && t <= ti[8])
        {
            d[0] = 0.0;
            Status = 2;
        }
        //Phase 9
        else if(ti[8] < t && t <= ti[9])
        {
            d[0] = -TP->d;
            Status = 3;
        }
        //Phase 10
        else if(ti[9] < t && t <= ti[10])
        {
            d[0] = 0.0;
            Status = 3;
        }
        //Phase 11
        else if(ti[10] < t && t <= ti[11])
        {
            d[0] = TP->d;
            Status = 3;
        }
        //Phase 12
        else if(ti[11] < t && t <= ti[12])
        {
            d[0] = 0.0;
            Status = 3;
        }
        //Phase 13
        else if(ti[12] < t && t <= ti[13])
        {
            d[0] = TP->d;
            Status = 3;
        }
        //Phase 14
        else if(ti[13] < t && t <= ti[14])
        {
            d[0] = 0.0;
            Status = 3;
        }
        //Phase 15
        else if(ti[14] < t && t <= ti[15])
        {
            d[0] = -TP->d;
            Status = 3;
        }
        //Phase 16
        else if(ti[15] < t && t <= ti[0])
        {
            d[0] = 0;
            Status = 4;
        } 
        j[0] = j[1] + d[1] * TP->Ts;
        a[0] = a[1] + j[1] * TP->Ts;
        v[0] = v[1] + a[1] * TP->Ts;
        s[0] = s[1] + v[1] * TP->Ts;
        for(shift = NUM_DELAY_PERIOD - 1; shift > 0; shift --)
        {
            d[shift] = d[shift - 1];
            j[shift] = j[shift - 1];
            a[shift] = a[shift - 1];
            v[shift] = v[shift - 1];
            s[shift] = s[shift - 1];
        } 
        t += TP->Ts;  
    }
	else
    {
	    for(i = 0; i < NUM_DELAY_PERIOD; i++)
        {
            d[i] = 0.0;
            j[i] = 0.0;
            a[i] = 0.0;
            v[i] = 0.0;
            s[i] = 0.0;
        }
		Status = 0;
	} 

	// *pd = CPM_FOR_TRAJ * d[0];
	// *pj = CPM_FOR_TRAJ * j[0];
	// *pa = CPM_FOR_TRAJ * a[0];
	// *pv = CPM_FOR_TRAJ * v[0];
	// *ps = CPM_FOR_TRAJ * s[0];

    *pd = floor(CPM_FOR_TRAJ * d[0]+0.5);
    *pj = floor(CPM_FOR_TRAJ * j[0]+0.5);
    *pa = floor(CPM_FOR_TRAJ * a[0]+0.5);
    *pv = floor(CPM_FOR_TRAJ * v[0]+0.5);
    *ps = floor(CPM_FOR_TRAJ * s[0]+0.5);

	// *pd = _round(CPM_FOR_TRAJ * d[0]);
	// *pj = _round(CPM_FOR_TRAJ * j[0]);
	// *pa = _round(CPM_FOR_TRAJ * a[0]);
	// *pv = _round(CPM_FOR_TRAJ * v[0]);
	// *ps = _round(CPM_FOR_TRAJ * s[0]);

    return Status;
}

int TrajGen4OrderFineTz(s_TrajParam_4_Order_Poly *TP, double *pd, double *pj, double *pa, double *pv, double *ps, int *c)
{
    static double d[NUM_DELAY_PERIOD] = {0.0}, j[NUM_DELAY_PERIOD] = {0.0}, a[NUM_DELAY_PERIOD] = {0.0}, v[NUM_DELAY_PERIOD] = {0.0}, s[NUM_DELAY_PERIOD] = {0.0};
    //d[0]是当前周期值，d[1]是上周期值，类推
    static double t = -1.0;
    static int i = 0;
	static int Status = 0;
	static int shift = 0;
    static double ti[16] = {0,0,0,0,0,0};	//time intervals; t[0] = TotalTime
	if(2 == *c)
    {
	    t = -1.0;
	    for(i = 0; i < NUM_DELAY_PERIOD; i++)
        {
            d[i] = 0.0;
            j[i] = 0.0;
            a[i] = 0.0;
            v[i] = 0.0;
            s[i] = 0.0;
        }
	    d[0] = TP->d;
		Status = 0;
		*c = 0;
	}
	if(1 == *c)     //Start & Initialize
	{
	    t = TP->Ts / 2.0;
		*c = 0;
	    for(i = 0; i < NUM_DELAY_PERIOD; i++)
        {
            d[i] = 0.0;
            j[i] = 0.0;
            a[i] = 0.0;
            v[i] = 0.0;
            s[i] = 0.0;
        }
	    d[0] = TP->d;
		Status = 0;
        ti[1] = (TP->Td);
        ti[2] = (TP->Td) + (TP->Tj);
        ti[3] = 2.0 * (TP->Td) + (TP->Tj);
        ti[4] = 2.0 * (TP->Td) + (TP->Tj) + (TP->Ta);
        ti[5] = 3.0 * (TP->Td) + (TP->Tj) + (TP->Ta);
        ti[6] = 3.0 * (TP->Td) + 2.0 *(TP->Tj) + (TP->Ta);
        ti[7] = 4.0 * (TP->Td) + 2.0 *(TP->Tj) + (TP->Ta);
        ti[8] = 4.0 * (TP->Td) + 2.0 *(TP->Tj) + (TP->Ta) + (TP->Tv);
        ti[9] = 5.0 * (TP->Td) + 2.0 *(TP->Tj) + (TP->Ta) + (TP->Tv);
        ti[10] = 5.0 * (TP->Td) + 3.0 *(TP->Tj) + (TP->Ta) + (TP->Tv);
        ti[11] = 6.0 * (TP->Td) + 3.0 *(TP->Tj) + (TP->Ta) + (TP->Tv);
        ti[12] = 6.0 * (TP->Td) + 3.0 *(TP->Tj) + 2.0 * (TP->Ta) + (TP->Tv);
        ti[13] = 7.0 * (TP->Td) + 3.0 *(TP->Tj) + 2.0 * (TP->Ta) + (TP->Tv);
        ti[14] = 7.0 * (TP->Td) + 4.0 * (TP->Tj) + 2.0 * (TP->Ta) + (TP->Tv);
        ti[15] = 8.0 * (TP->Td) + 4.0 * (TP->Tj) + 2.0 * (TP->Ta) + (TP->Tv);
        if(FLOATEQU(TP->Tw,0.0))    ti[0] = ti[15] + TP->Ts;
        else                        ti[0] = ti[15] + TP->Tw;
	}
	if(0.0 <= t && t <= ti[0])
    {
        //Phase 1
        if(0.0 <= t && t <= ti[1])
        {
            d[0] = TP->d;
            Status = 1;
        }
        //Phase 2
        else if(ti[1] < t && t <= ti[2])
        {
            d[0] = 0.0;
            Status = 1;
        }
        //Phase 3
        else if(ti[2] < t && t <= ti[3])
        {
            d[0] = -TP->d;
            Status = 1;
        }
        //Phase 4
        else if(ti[3] < t && t <= ti[4])
        {
            d[0] = 0.0;
            Status = 1;
        }
        //Phase 5
        else if(ti[4] < t && t <= ti[5])
        {
            d[0] = -TP->d;
            Status = 1;
        }
        //Phase 6
        else if(ti[5] < t && t <= ti[6])
        {
            d[0] = 0.0;
            Status = 1;
        }
        //Phase 7
        else if(ti[6] < t && t <= ti[7])
        {
            d[0] = TP->d;
            Status = 1;
        }
        //Phase 8
        else if(ti[7] < t && t <= ti[8])
        {
            d[0] = 0.0;
            Status = 2;
        }
        //Phase 9
        else if(ti[8] < t && t <= ti[9])
        {
            d[0] = -TP->d;
            Status = 3;
        }
        //Phase 10
        else if(ti[9] < t && t <= ti[10])
        {
            d[0] = 0.0;
            Status = 3;
        }
        //Phase 11
        else if(ti[10] < t && t <= ti[11])
        {
            d[0] = TP->d;
            Status = 3;
        }
        //Phase 12
        else if(ti[11] < t && t <= ti[12])
        {
            d[0] = 0.0;
            Status = 3;
        }
        //Phase 13
        else if(ti[12] < t && t <= ti[13])
        {
            d[0] = TP->d;
            Status = 3;
        }
        //Phase 14
        else if(ti[13] < t && t <= ti[14])
        {
            d[0] = 0.0;
            Status = 3;
        }
        //Phase 15
        else if(ti[14] < t && t <= ti[15])
        {
            d[0] = -TP->d;
            Status = 3;
        }
        //Phase 16
        else if(ti[15] < t && t <= ti[0])
        {
            d[0] = 0;
            Status = 4;
        } 
        j[0] = j[1] + d[1] * TP->Ts;
        a[0] = a[1] + j[1] * TP->Ts;
        v[0] = v[1] + a[1] * TP->Ts;
        s[0] = s[1] + v[1] * TP->Ts;
        for(shift = NUM_DELAY_PERIOD - 1; shift > 0; shift --)
        {
            d[shift] = d[shift - 1];
            j[shift] = j[shift - 1];
            a[shift] = a[shift - 1];
            v[shift] = v[shift - 1];
            s[shift] = s[shift - 1];
        } 
        t += TP->Ts;  
    }
	else
    {
	    for(i = 0; i < NUM_DELAY_PERIOD; i++)
        {
            d[i] = 0.0;
            j[i] = 0.0;
            a[i] = 0.0;
            v[i] = 0.0;
            s[i] = 0.0;
        }
		Status = 0;
	} 

	// *pd = CPM_FOR_TRAJ * d[0];
	// *pj = CPM_FOR_TRAJ * j[0];
	// *pa = CPM_FOR_TRAJ * a[0];
	// *pv = CPM_FOR_TRAJ * v[0];
	// *ps = CPM_FOR_TRAJ * s[0];

    *pd = floor(CPM_FOR_TRAJ * d[0]+0.5);
    *pj = floor(CPM_FOR_TRAJ * j[0]+0.5);
    *pa = floor(CPM_FOR_TRAJ * a[0]+0.5);
    *pv = floor(CPM_FOR_TRAJ * v[0]+0.5);
    *ps = floor(CPM_FOR_TRAJ * s[0]+0.5);

	// *pd = _round(CPM_FOR_TRAJ * d[0]);
	// *pj = _round(CPM_FOR_TRAJ * j[0]);
	// *pa = _round(CPM_FOR_TRAJ * a[0]);
	// *pv = _round(CPM_FOR_TRAJ * v[0]);
	// *ps = _round(CPM_FOR_TRAJ * s[0]);

    return Status;
}










int X_TrajGen_4_Order_Poly(s_TrajParam_4_Order_Poly *TP, double *pd, double *pj, double *pa, double *pv, double *ps, int *c)
{
    static double d[NUM_DELAY_PERIOD] = {0.0}, j[NUM_DELAY_PERIOD] = {0.0}, a[NUM_DELAY_PERIOD] = {0.0}, v[NUM_DELAY_PERIOD] = {0.0}, s[NUM_DELAY_PERIOD] = {0.0};
    //d[0]是当前周期值，d[1]是上周期值，类推
    static double t = -1.0;
    static int i = 0;
	static int Status = 0;
	static int shift = 0;
    static double ti[16] = {0,0,0,0,0,0};	//time intervals; t[0] = TotalTime
	if(2 == *c)
    {
	    t = -1.0;
	    for(i = 0; i < NUM_DELAY_PERIOD; i++)
        {
            d[i] = 0.0;
            j[i] = 0.0;
            a[i] = 0.0;
            v[i] = 0.0;
            s[i] = 0.0;
        }
	    d[0] = TP->d;
		Status = 0;
		*c = 0;
	}
	if(1 == *c)     //Start & Initialize
	{
	    t = TP->Ts / 2.0;
		*c = 0;
	    for(i = 0; i < NUM_DELAY_PERIOD; i++)
        {
            d[i] = 0.0;
            j[i] = 0.0;
            a[i] = 0.0;
            v[i] = 0.0;
            s[i] = 0.0;
        }
	    d[0] = TP->d;
		Status = 0;
        ti[1] = (TP->Td);
        ti[2] = (TP->Td) + (TP->Tj);
        ti[3] = 2.0 * (TP->Td) + (TP->Tj);
        ti[4] = 2.0 * (TP->Td) + (TP->Tj) + (TP->Ta);
        ti[5] = 3.0 * (TP->Td) + (TP->Tj) + (TP->Ta);
        ti[6] = 3.0 * (TP->Td) + 2.0 *(TP->Tj) + (TP->Ta);
        ti[7] = 4.0 * (TP->Td) + 2.0 *(TP->Tj) + (TP->Ta);
        ti[8] = 4.0 * (TP->Td) + 2.0 *(TP->Tj) + (TP->Ta) + (TP->Tv);
        ti[9] = 5.0 * (TP->Td) + 2.0 *(TP->Tj) + (TP->Ta) + (TP->Tv);
        ti[10] = 5.0 * (TP->Td) + 3.0 *(TP->Tj) + (TP->Ta) + (TP->Tv);
        ti[11] = 6.0 * (TP->Td) + 3.0 *(TP->Tj) + (TP->Ta) + (TP->Tv);
        ti[12] = 6.0 * (TP->Td) + 3.0 *(TP->Tj) + 2.0 * (TP->Ta) + (TP->Tv);
        ti[13] = 7.0 * (TP->Td) + 3.0 *(TP->Tj) + 2.0 * (TP->Ta) + (TP->Tv);
        ti[14] = 7.0 * (TP->Td) + 4.0 * (TP->Tj) + 2.0 * (TP->Ta) + (TP->Tv);
        ti[15] = 8.0 * (TP->Td) + 4.0 * (TP->Tj) + 2.0 * (TP->Ta) + (TP->Tv);
        if(FLOATEQU(TP->Tw,0.0))    ti[0] = ti[15] + TP->Ts;
        else                        ti[0] = ti[15] + TP->Tw;
	}
	if(0.0 <= t && t <= ti[0])
    {
        //Phase 1
        if(0.0 <= t && t <= ti[1])
        {
            d[0] = TP->d;
            Status = 1;
        }
        //Phase 2
        else if(ti[1] < t && t <= ti[2])
        {
            d[0] = 0.0;
            Status = 1;
        }
        //Phase 3
        else if(ti[2] < t && t <= ti[3])
        {
            d[0] = -TP->d;
            Status = 1;
        }
        //Phase 4
        else if(ti[3] < t && t <= ti[4])
        {
            d[0] = 0.0;
            Status = 1;
        }
        //Phase 5
        else if(ti[4] < t && t <= ti[5])
        {
            d[0] = -TP->d;
            Status = 1;
        }
        //Phase 6
        else if(ti[5] < t && t <= ti[6])
        {
            d[0] = 0.0;
            Status = 1;
        }
        //Phase 7
        else if(ti[6] < t && t <= ti[7])
        {
            d[0] = TP->d;
            Status = 1;
        }
        //Phase 8
        else if(ti[7] < t && t <= ti[8])
        {
            d[0] = 0.0;
            Status = 2;
        }
        //Phase 9
        else if(ti[8] < t && t <= ti[9])
        {
            d[0] = -TP->d;
            Status = 3;
        }
        //Phase 10
        else if(ti[9] < t && t <= ti[10])
        {
            d[0] = 0.0;
            Status = 3;
        }
        //Phase 11
        else if(ti[10] < t && t <= ti[11])
        {
            d[0] = TP->d;
            Status = 3;
        }
        //Phase 12
        else if(ti[11] < t && t <= ti[12])
        {
            d[0] = 0.0;
            Status = 3;
        }
        //Phase 13
        else if(ti[12] < t && t <= ti[13])
        {
            d[0] = TP->d;
            Status = 3;
        }
        //Phase 14
        else if(ti[13] < t && t <= ti[14])
        {
            d[0] = 0.0;
            Status = 3;
        }
        //Phase 15
        else if(ti[14] < t && t <= ti[15])
        {
            d[0] = -TP->d;
            Status = 3;
        }
        //Phase 16
        else if(ti[15] < t && t <= ti[0])
        {
            d[0] = 0;
            Status = 4;
        } 
        j[0] = j[1] + d[1] * TP->Ts;
        a[0] = a[1] + j[1] * TP->Ts;
        v[0] = v[1] + a[1] * TP->Ts;
        s[0] = s[1] + v[1] * TP->Ts;
        for(shift = NUM_DELAY_PERIOD - 1; shift > 0; shift --)
        {
            d[shift] = d[shift - 1];
            j[shift] = j[shift - 1];
            a[shift] = a[shift - 1];
            v[shift] = v[shift - 1];
            s[shift] = s[shift - 1];
        } 
        t += TP->Ts;  
    }
	else
    {
	    for(i = 0; i < NUM_DELAY_PERIOD; i++)
        {
            d[i] = 0.0;
            j[i] = 0.0;
            a[i] = 0.0;
            v[i] = 0.0;
            s[i] = 0.0;
        }
		Status = 0;
	} 

	*pd = CPM_FOR_TRAJ * d[0];
	*pj = CPM_FOR_TRAJ * j[0];
	*pa = CPM_FOR_TRAJ * a[0];
	*pv = CPM_FOR_TRAJ * v[0];
	*ps = CPM_FOR_TRAJ * s[0];

	*pd = floor(CPM_FOR_TRAJ * d[0]+0.5);
	*pj = floor(CPM_FOR_TRAJ * j[0]+0.5);
	*pa = floor(CPM_FOR_TRAJ * a[0]/100+0.5);
	*pv = floor(CPM_FOR_TRAJ * v[0]+0.5);
	*ps = floor(CPM_FOR_TRAJ * s[0]+0.5);
    return Status;
}

int Y_TrajGen_4_Order_Poly(s_TrajParam_4_Order_Poly *TP, double *pd, double *pj, double *pa, double *pv, double *ps, int *c)
{
    static double d[NUM_DELAY_PERIOD] = {0.0}, j[NUM_DELAY_PERIOD] = {0.0}, a[NUM_DELAY_PERIOD] = {0.0}, v[NUM_DELAY_PERIOD] = {0.0}, s[NUM_DELAY_PERIOD] = {0.0};
    //d[0]是当前周期值，d[1]是上周期值，类推
    static double t = -1.0;
    static int i = 0;
	static int Status = 0;
	static int shift = 0;
    static double ti[16] = {0,0,0,0,0,0};	//time intervals; t[0] = TotalTime
	if(2 == *c)
    {
	    t = -1.0;
	    for(i = 0; i < NUM_DELAY_PERIOD; i++)
        {
            d[i] = 0.0;
            j[i] = 0.0;
            a[i] = 0.0;
            v[i] = 0.0;
            s[i] = 0.0;
        }
	    d[0] = TP->d;
		Status = 0;
		*c = 0;
	}
	if(1 == *c)     //Start & Initialize
	{
	    t = TP->Ts / 2.0;
		*c = 0;
	    for(i = 0; i < NUM_DELAY_PERIOD; i++)
        {
            d[i] = 0.0;
            j[i] = 0.0;
            a[i] = 0.0;
            v[i] = 0.0;
            s[i] = 0.0;
        }
	    d[0] = TP->d;
		Status = 0;
        ti[1] = (TP->Td);
        ti[2] = (TP->Td) + (TP->Tj);
        ti[3] = 2.0 * (TP->Td) + (TP->Tj);
        ti[4] = 2.0 * (TP->Td) + (TP->Tj) + (TP->Ta);
        ti[5] = 3.0 * (TP->Td) + (TP->Tj) + (TP->Ta);
        ti[6] = 3.0 * (TP->Td) + 2.0 *(TP->Tj) + (TP->Ta);
        ti[7] = 4.0 * (TP->Td) + 2.0 *(TP->Tj) + (TP->Ta);
        ti[8] = 4.0 * (TP->Td) + 2.0 *(TP->Tj) + (TP->Ta) + (TP->Tv);
        ti[9] = 5.0 * (TP->Td) + 2.0 *(TP->Tj) + (TP->Ta) + (TP->Tv);
        ti[10] = 5.0 * (TP->Td) + 3.0 *(TP->Tj) + (TP->Ta) + (TP->Tv);
        ti[11] = 6.0 * (TP->Td) + 3.0 *(TP->Tj) + (TP->Ta) + (TP->Tv);
        ti[12] = 6.0 * (TP->Td) + 3.0 *(TP->Tj) + 2.0 * (TP->Ta) + (TP->Tv);
        ti[13] = 7.0 * (TP->Td) + 3.0 *(TP->Tj) + 2.0 * (TP->Ta) + (TP->Tv);
        ti[14] = 7.0 * (TP->Td) + 4.0 * (TP->Tj) + 2.0 * (TP->Ta) + (TP->Tv);
        ti[15] = 8.0 * (TP->Td) + 4.0 * (TP->Tj) + 2.0 * (TP->Ta) + (TP->Tv);
        if(FLOATEQU(TP->Tw,0.0))    ti[0] = ti[15] + TP->Ts;
        else                        ti[0] = ti[15] + TP->Tw;
	}
	if(0.0 <= t && t <= ti[0])
    {
        //Phase 1
        if(0.0 <= t && t <= ti[1])
        {
            d[0] = TP->d;
            Status = 1;
        }
        //Phase 2
        else if(ti[1] < t && t <= ti[2])
        {
            d[0] = 0.0;
            Status = 1;
        }
        //Phase 3
        else if(ti[2] < t && t <= ti[3])
        {
            d[0] = -TP->d;
            Status = 1;
        }
        //Phase 4
        else if(ti[3] < t && t <= ti[4])
        {
            d[0] = 0.0;
            Status = 1;
        }
        //Phase 5
        else if(ti[4] < t && t <= ti[5])
        {
            d[0] = -TP->d;
            Status = 1;
        }
        //Phase 6
        else if(ti[5] < t && t <= ti[6])
        {
            d[0] = 0.0;
            Status = 1;
        }
        //Phase 7
        else if(ti[6] < t && t <= ti[7])
        {
            d[0] = TP->d;
            Status = 1;
        }
        //Phase 8
        else if(ti[7] < t && t <= ti[8])
        {
            d[0] = 0.0;
            Status = 2;
        }
        //Phase 9
        else if(ti[8] < t && t <= ti[9])
        {
            d[0] = -TP->d;
            Status = 3;
        }
        //Phase 10
        else if(ti[9] < t && t <= ti[10])
        {
            d[0] = 0.0;
            Status = 3;
        }
        //Phase 11
        else if(ti[10] < t && t <= ti[11])
        {
            d[0] = TP->d;
            Status = 3;
        }
        //Phase 12
        else if(ti[11] < t && t <= ti[12])
        {
            d[0] = 0.0;
            Status = 3;
        }
        //Phase 13
        else if(ti[12] < t && t <= ti[13])
        {
            d[0] = TP->d;
            Status = 3;
        }
        //Phase 14
        else if(ti[13] < t && t <= ti[14])
        {
            d[0] = 0.0;
            Status = 3;
        }
        //Phase 15
        else if(ti[14] < t && t <= ti[15])
        {
            d[0] = -TP->d;
            Status = 3;
        }
        //Phase 16
        else if(ti[15] < t && t <= ti[0])
        {
            d[0] = 0;
            Status = 4;
        } 
        j[0] = j[1] + d[1] * TP->Ts;
        a[0] = a[1] + j[1] * TP->Ts;
        v[0] = v[1] + a[1] * TP->Ts;
        s[0] = s[1] + v[1] * TP->Ts;
        for(shift = NUM_DELAY_PERIOD - 1; shift > 0; shift --)
        {
            d[shift] = d[shift - 1];
            j[shift] = j[shift - 1];
            a[shift] = a[shift - 1];
            v[shift] = v[shift - 1];
            s[shift] = s[shift - 1];
        } 
        t += TP->Ts;  
    }
	else
    {
	    for(i = 0; i < NUM_DELAY_PERIOD; i++)
        {
            d[i] = 0.0;
            j[i] = 0.0;
            a[i] = 0.0;
            v[i] = 0.0;
            s[i] = 0.0;
        }
		Status = 0;
	} 

	*pd = CPM_FOR_TRAJ * d[0];
	*pj = CPM_FOR_TRAJ * j[0];
	*pa = CPM_FOR_TRAJ * a[0];
	*pv = CPM_FOR_TRAJ * v[0];
	*ps = CPM_FOR_TRAJ * s[0];

	*pd = floor(CPM_FOR_TRAJ * d[0]+0.5);
	*pj = floor(CPM_FOR_TRAJ * j[0]+0.5);
	*pa = floor(CPM_FOR_TRAJ * a[0]/100+0.5);
	*pv = floor(CPM_FOR_TRAJ * v[0]+0.5);
	*ps = floor(CPM_FOR_TRAJ * s[0]+0.5);

    return Status;
}

int TrajGen_4_Order_Poly_Reciprocation(s_TrajParam_4_Order_Poly *TP, double *pd, double *pj, double *pa, double *pv, double *ps, int *c, long *pcount)
{
    static double d[NUM_DELAY_PERIOD] = {0.0}, j[NUM_DELAY_PERIOD] = {0.0}, a[NUM_DELAY_PERIOD] = {0.0}, v[NUM_DELAY_PERIOD] = {0.0}, s[NUM_DELAY_PERIOD] = {0.0};
    //d[0]是当前周期值，d[1]是上周期值，类推
    static double t = -1.0;
    static int i = 0;
	static int Status = 0;
	static int shift = 0;
	static int count = 0;
    static double ti[32] = {0};	//time intervals; t[0] = TotalTime
	if(2 == *c)
    {
	    t = -1.0;
	    for(i = 0; i < NUM_DELAY_PERIOD; i++)
        {
            d[i] = 0.0;
            j[i] = 0.0;
            a[i] = 0.0;
            v[i] = 0.0;
            s[i] = 0.0;
        }
	    d[0] = TP->d;
		Status = 0;
		*c = 0;
	}
	if(1 == *c)     //Start & Initialize
	{
	    t = TP->Ts / 2.0;
		*c = 0;
	    for(i = 0; i < NUM_DELAY_PERIOD; i++)
        {
            d[i] = 0.0;
            j[i] = 0.0;
            a[i] = 0.0;
            v[i] = 0.0;
            s[i] = 0.0;
        }
	    d[0] = TP->d;
		Status = 0;
        ti[1] = (TP->Td);
        ti[2] = (TP->Td) + (TP->Tj);
        ti[3] = 2.0 * (TP->Td) + (TP->Tj);
        ti[4] = 2.0 * (TP->Td) + (TP->Tj) + (TP->Ta);
        ti[5] = 3.0 * (TP->Td) + (TP->Tj) + (TP->Ta);
        ti[6] = 3.0 * (TP->Td) + 2.0 *(TP->Tj) + (TP->Ta);
        ti[7] = 4.0 * (TP->Td) + 2.0 *(TP->Tj) + (TP->Ta);
        ti[8] = 4.0 * (TP->Td) + 2.0 *(TP->Tj) + (TP->Ta) + (TP->Tv);
        ti[9] = 5.0 * (TP->Td) + 2.0 *(TP->Tj) + (TP->Ta) + (TP->Tv);
        ti[10] = 5.0 * (TP->Td) + 3.0 *(TP->Tj) + (TP->Ta) + (TP->Tv);
        ti[11] = 6.0 * (TP->Td) + 3.0 *(TP->Tj) + (TP->Ta) + (TP->Tv);
        ti[12] = 6.0 * (TP->Td) + 3.0 *(TP->Tj) + 2.0 * (TP->Ta) + (TP->Tv);
        ti[13] = 7.0 * (TP->Td) + 3.0 *(TP->Tj) + 2.0 * (TP->Ta) + (TP->Tv);
        ti[14] = 7.0 * (TP->Td) + 4.0 * (TP->Tj) + 2.0 * (TP->Ta) + (TP->Tv);
        ti[15] = 8.0 * (TP->Td) + 4.0 * (TP->Tj) + 2.0 * (TP->Ta) + (TP->Tv);
		
        if(FLOATEQU(TP->Tw,0.0))    ti[0] = ti[15] + TP->Ts;
        else                        ti[0] = ti[15] + TP->Tw;
		
		ti[16] = ti[0];
		ti[17] = ti[0] + ti[1];
		ti[18] = ti[0] + ti[2];
		ti[19] = ti[0] + ti[3];
		ti[20] = ti[0] + ti[4];
		ti[21] = ti[0] + ti[5];
		ti[22] = ti[0] + ti[6];
		ti[23] = ti[0] + ti[7];
		ti[24] = ti[0] + ti[8];
		ti[25] = ti[0] + ti[9];
		ti[26] = ti[0] + ti[10];
		ti[27] = ti[0] + ti[11];
		ti[28] = ti[0] + ti[12];
		ti[29] = ti[0] + ti[13];
		ti[30] = ti[0] + ti[14];
		ti[31] = ti[0] + ti[15];

		if(FLOATEQU(TP->Tw,0.0))    ti[0] = ti[31] + TP->Ts;
        else                        ti[0] = ti[31] + TP->Tw;
	}
	if(0.0 <= t && t <= ti[0])
    {
		count++;
        //Phase 1
        if(0.0 <= t && t <= ti[1])
        {
            d[0] = TP->d;
            Status = 1;
        }
        //Phase 2
        else if(ti[1] < t && t <= ti[2])
        {
            d[0] = 0.0;
            Status = 1;
        }
        //Phase 3
        else if(ti[2] < t && t <= ti[3])
        {
            d[0] = -TP->d;
            Status = 1;
        }
        //Phase 4
        else if(ti[3] < t && t <= ti[4])
        {
            d[0] = 0.0;
            Status = 1;
        }
        //Phase 5
        else if(ti[4] < t && t <= ti[5])
        {
            d[0] = -TP->d;
            Status = 1;
        }
        //Phase 6
        else if(ti[5] < t && t <= ti[6])
        {
            d[0] = 0.0;
            Status = 1;
        }
        //Phase 7
        else if(ti[6] < t && t <= ti[7])
        {
            d[0] = TP->d;
            Status = 1;
        }
        //Phase 8
        else if(ti[7] < t && t <= ti[8])
        {
            d[0] = 0.0;
            Status = 2;
        }
        //Phase 9
        else if(ti[8] < t && t <= ti[9])
        {
            d[0] = -TP->d;
            Status = 3;
        }
        //Phase 10
        else if(ti[9] < t && t <= ti[10])
        {
            d[0] = 0.0;
            Status = 3;
        }
        //Phase 11
        else if(ti[10] < t && t <= ti[11])
        {
            d[0] = TP->d;
            Status = 3;
        }
        //Phase 12
        else if(ti[11] < t && t <= ti[12])
        {
            d[0] = 0.0;
            Status = 3;
        }
        //Phase 13
        else if(ti[12] < t && t <= ti[13])
        {
            d[0] = TP->d;
            Status = 3;
        }
        //Phase 14
        else if(ti[13] < t && t <= ti[14])
        {
            d[0] = 0.0;
            Status = 3;
        }
        //Phase 15
        else if(ti[14] < t && t <= ti[15])
        {
            d[0] = -TP->d;
            Status = 3;
        }
        //Phase 16
        else if(ti[15] < t && t <= ti[16])
        {
            d[0] = 0;
            Status = 4;
        }
		//Phase 17
        else if(ti[16] < t && t <= ti[17])
        {
            d[0] = -TP->d;
            Status = 5;
        }
		//Phase 18
        else if(ti[17] < t && t <= ti[18])
        {
            d[0] = 0.0;
            Status = 5;
        }
        //Phase 19
        else if(ti[18] < t && t <= ti[19])
        {
            d[0] = TP->d;
            Status = 5;
        }
        //Phase 20
        else if(ti[19] < t && t <= ti[20])
        {
            d[0] = 0.0;
            Status = 5;
        }
        //Phase 21
        else if(ti[20] < t && t <= ti[21])
        {
            d[0] = TP->d;
            Status = 5;
        }
        //Phase 22
        else if(ti[21] < t && t <= ti[22])
        {
            d[0] = 0.0;
            Status = 5;
        }
        //Phase 23
        else if(ti[22] < t && t <= ti[23])
        {
            d[0] = -TP->d;
            Status = 5;
        }
        //Phase 24
        else if(ti[23] < t && t <= ti[24])
        {
            d[0] = 0.0;
            Status = 6;
        }
        //Phase 25
        else if(ti[24] < t && t <= ti[25])
        {
            d[0] = TP->d;
            Status = 7;
        }
        //Phase 26
        else if(ti[25] < t && t <= ti[26])
        {
            d[0] = 0.0;
            Status = 7;
        }
        //Phase 27
        else if(ti[26] < t && t <= ti[27])
        {
            d[0] = -TP->d;
            Status = 7;
        }
        //Phase 28
        else if(ti[27] < t && t <= ti[28])
        {
            d[0] = 0.0;
            Status = 7;
        }
        //Phase 29
        else if(ti[28] < t && t <= ti[29])
        {
            d[0] = -TP->d;
            Status = 7;
        }
        //Phase 30
        else if(ti[29] < t && t <= ti[30])
        {
            d[0] = 0.0;
            Status = 7;
        }
        //Phase 31
        else if(ti[30] < t && t <= ti[31])
        {
            d[0] = TP->d;
            Status = 7;
        }
        //Phase 32
        else if(ti[31] < t && t <= ti[0])
        {
            d[0] = 0;
            Status = 8;
        }
        j[0] = j[1] + d[1] * TP->Ts;
        a[0] = a[1] + j[1] * TP->Ts;
        v[0] = v[1] + a[1] * TP->Ts;
        s[0] = s[1] + v[1] * TP->Ts;
        for(shift = NUM_DELAY_PERIOD - 1; shift > 0; shift --)
        {
            d[shift] = d[shift - 1];
            j[shift] = j[shift - 1];
            a[shift] = a[shift - 1];
            v[shift] = v[shift - 1];
            s[shift] = s[shift - 1];
        }
        t += TP->Ts;
    }
	else
    {
	    for(i = 0; i < NUM_DELAY_PERIOD; i++)
        {
            d[i] = 0.0;
            j[i] = 0.0;
            a[i] = 0.0;
            v[i] = 0.0;
            s[i] = 0.0;
        }

		Status = 0;
		count = 0;
	}

	*pd = floor(CPM_FOR_TRAJ * d[0]+0.5);
	*pj = floor(CPM_FOR_TRAJ * j[0]+0.5);
	*pa = floor(CPM_FOR_TRAJ * a[0]/100+0.5);
	*pv = floor(CPM_FOR_TRAJ * v[0]+0.5);
	*ps = floor(CPM_FOR_TRAJ * s[0]+0.5);
	*pcount = count;
    return Status;
}
int TrajSine_Poly_Coarse(s_TrajParam_Sine_Poly *TP,double *a,double *v,double *s)
{
	static int temp_Coarse = 0;
	static int Status = 0;
	static int Coarse_Wait = 0;
	static double A = 0;
	static double w = 0;
	static double t1,t2,ts;

	A = TP->amax;
	w = 2*A/TP->vmax;

	t1 = PI*TP->vmax/2/TP->amax;
	t2 = (TP->smax - 2*A*t1/w)/TP->vmax + t1;

	ts = temp_Coarse*0.0002;

	if(ts<(t1+t2))
	{
		if(ts<=t1)
		{
			*a = A*sin(w*ts);
			*v = -A/w*cos(w*ts) + A/w;
			Status = 1;
			//logMsg("Traj is Postive!\n",0,0,0,0,0,0);
		}
		if(t1<=ts && ts<=t2)
		{
			*a = 0;
			*v = TP->vmax;
			Status = 2;
			//logMsg("Traj is Speed!\n",0,0,0,0,0,0);
		}
		if(t2<=ts && ts<=(t2+t1))
		{
			*a = -A*sin(w*(ts-t2));
			*v = A/w*cos(w*(ts-t2)) + TP->vmax - A/w;
			Status = 3;
			//logMsg("Traj is negtive!\n",0,0,0,0,0,0);
		}
		*s = *s + *v*0.0002;;
		temp_Coarse = temp_Coarse + 1;
		//logMsg("The temp is %d\n",temp,0,0,0,0,0);	
	}
	else if(TP->WaitCyc > 0)
	{
		if(Coarse_Wait >= TP->WaitCyc)
		{
			Status = 0;
			temp_Coarse = 0;
			Coarse_Wait = 0;	
			logMsg("The traj's waiting is finished!\n",0,0,0,0,0,0);
		}
		else
		{
			//logMsg("Traj is wait for singol!\n",0,0,0,0,0,0);
			Coarse_Wait++;
			Status = 4;
		}
	}
	else
	{
		Status = 0;
		temp_Coarse = 0;
		Coarse_Wait = 0;
	}
	return Status; 
}

int TrajSine_Poly_Fine(s_TrajParam_Sine_Poly *TP,double *a,double *v,double *s)
{
	static int temp_Fine = 0;
	static int Status = 0;
	static int Fine_Wait = 0;
	static double A = 0;
	static double w = 0;
	static double t1,t2,ts;

	A = TP->amax;
	w = 2*A/TP->vmax;

	t1 = PI*TP->vmax/2/TP->amax;
	t2 = (TP->smax - 2*A*t1/w)/TP->vmax + t1;

	ts = temp_Fine*0.0002;

	if(ts<(t1+t2))
	{
		if(ts<=t1)
		{
			*a = A*sin(w*ts);
			*v = -A/w*cos(w*ts) + A/w;
			Status = 1;
			//logMsg("Traj is Postive!\n",0,0,0,0,0,0);
		}
		if(t1<=ts && ts<=t2)
		{
			*a = 0;
			*v = TP->vmax;
			Status = 2;
			//logMsg("Traj is Speed!\n",0,0,0,0,0,0);
		}
		if(t2<=ts && ts<=(t2+t1))
		{
			*a = -A*sin(w*(ts-t2));
			*v = A/w*cos(w*(ts-t2)) + TP->vmax - A/w;
			Status = 3;
			//logMsg("Traj is negtive!\n",0,0,0,0,0,0);
		}
		*s = *s + *v*0.0002;;
		temp_Fine = temp_Fine + 1;
		//logMsg("The temp is %d\n",temp,0,0,0,0,0);	
	}
	else if(TP->WaitCyc > 0)
	{
		if(Fine_Wait >= TP->WaitCyc)
		{
			Status = 0;
			temp_Fine = 0;
			Fine_Wait = 0;	
			logMsg("The traj's waiting is finished!\n",0,0,0,0,0,0);
		}
		else
		{
			Fine_Wait++;
			Status = 4;
		}
	}
	else
	{
		Status = 0;
		temp_Fine = 0;
		Fine_Wait = 0;
		//szx
		*a=0.0;
		*v=0.0;
	}
	return Status; 
}

