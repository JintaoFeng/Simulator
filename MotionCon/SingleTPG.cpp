#include "StateMachine.h"
#include "math.h"
#include "SingleTPG.h"
#include "stdlib.h"
#include "stdio.h"
#include "GetData.h"

#define round(x) ((int)(x+0.5))

double SIGN(double x)
{
    if(fabs(x - 0.0) < tolerance) return 0.0;
    if(x < 0.0) return -1.0;
    if(x > 0.0) return 1.0;
    return 2.0;
}


int TPGen_4Order_P2P(const s_TParam *TP,s_TrajParam_4_Order_Poly *time)
//int TPGen_4Order_P2P(const s_TParam *TP, double *dd, double *Td, double *Tj, double *Ta, double *Tv, int cal_d_j,s_TrajParam_4_Order_Poly time)
{
	double a,b,c,e,f;
	double *dd=&a,*Td=&b,*Tj=&c,*Ta=&f,*Tv=&e;
	double d = TP->d;
	double t1, t2, t3, t4;
	double P,Q,D,R;
	double c1, c2, c3;
    double x, ddq, pp, dif, cor1, cor2, tt;
    long ti, cnt;
	//cal_d_j =  0;
//	printf("haha!!\n");
    //TP->d = 1000000;
	 /* TP->j = 700;
	TP->a = 0.2*9.8;
	TP->v = 100;
	TP->p = 0.01;
	TP->s = 100;
	TP->r = 100;

//		 TP.Ts = DEFAULT_TIMEINTERVAL;
    //TP.d = DEFAULT_SNAP;
    //TP.j = DEFAULT_JERK;
    //TP.a = DEFAULT_ACCELERATION;
   // TP.v = DEFAULT_VELOCITY;
   // TP.p = DEFAULT_DISPLACEMENT;*/

	if(((TP->p) < 0.0) || ((TP->v) < 0.0) || ((TP->a) < 0.0) || ((TP->j) < 0.0) || ((TP->d) < 0.0))
	{
		return e_ERRINPUT;
	}
	if(((TP->Ts) < 0.0) || ((TP->r) < 0.0) || ((TP->s) < 0.0))
	{
		return e_ERRINPUT;
	}

//	printf("hehe!!\n");
    //double d = TP->d;
// whether calculate the minimum jerk and snap. @Algorithm by Dongdong Yu
  /*  if(cal_d_j == 1)
    {
        double tmp, tmp1, tmp2;
        tmp1 = TP->p/TP->v - TP->v/TP->a;
        tmp2 = TP->v/TP->a;
        if(tmp1 < tmp2)
        {
            tmp=tmp1;
        }
        else
        {
            tmp=tmp2;
        }
        d = 4.0 * TP->a/ (tmp * tmp);
    }*/

////////////////////////////////////////////////////////////////////////////
//	*dd = d;
	

  //  double t1, t2, t3, t4;
// Calculation constant djerk phase duration: t1
    t1 = pow((1.0/8.0 * TP->p / d),(1.0/4.0)); //largest t1 with bound on derivative of jerk
    if(TP->Ts > 0)
    {
        t1 = ceil(t1 / TP->Ts) * TP->Ts;
        *dd = 1.0/8.0 * TP->p / pow(t1, 4.0);
		
    }
// velocity test
    if(TP->v < 2.0 * (*dd) * pow(t1, 3.0) ) //check if v bound is violated
    {
        t1 = pow((1.0/2.0 * TP->v / d), (1.0/3.0)); //t1 with bound on velocity not violated
        if(TP->Ts > 0)
        {
            t1 = ceil(t1 / TP->Ts) * TP->Ts;
            *dd = 1.0/2.0 * TP->v / pow(t1, 3.0);
			
        }
    }
// acceleration test
    if(TP->a < (*dd) * pow(t1, 2.0))    //check if a bound is violated
    {
        t1 = pow((TP->a / d), (1.0/2.0));   //t1 with bound on acceleration not violated
        if(TP->Ts > 0)
        {
            t1 = ceil(t1 / TP->Ts) * TP->Ts;
            *dd = TP->a / pow(t1, 2.0);
			
        }
    }
// jerk test
    if(TP->j < (*dd) * t1) //check if j bound is violated
    {
        t1 = TP->j / d; //t1 with bound on jerk not violated    为0
        if(TP->Ts > 0)
        {
            t1 = ceil(t1 / TP->Ts) * TP->Ts;
            *dd = TP->j / t1;// 为0
			
        }
    }
    d = *dd; //as t1 is now fixed, dd is the new bound on derivative of jerk

   // double P,Q,D,R;
    // Calculation constant jerk phase duration: t2
    P = -1.0/9.0 * pow(t1, 2.0);                            // calculations to determine
    Q = -1.0/27.0 * pow(t1,3.0) - TP->p / (4.0 * d * t1);       //positive real solution of
    D = pow(P, 3.0) + pow(Q, 2.0);                          //third order polynomial...
    R = pow((-Q + sqrt(D)), (1.0/3.0));
    t2 = R - P / R - 5.0/3.0 * t1;                          //largest t2 with bound on jerk
   
    if(fabs(t2) < tolerance)   t2 = 0.0;   //for continuous time case  为96450617283.
    if(TP->Ts > 0)
    {
	
        t2 = ceil(t2 / TP->Ts) * TP->Ts;
		
		
        *dd = TP->p / (8.0 * pow(t1, 4.0) + 16.0 * pow(t1, 3.0) * t2 + 10.0 * pow(t1, 2.0) * pow(t2, 2.0) + 2.0 * t1 * pow(t2, 3.0));
		
	}


    //velocity test     为23148148.
    if(TP->v < (2.0 * (*dd) * pow(t1, 3.0) + 3.0 * (*dd) * pow(t1, 2.0) * t2 + (*dd) * t1 * pow(t2, 2.0)))  //check if v bound is violated
    {
        t2 = pow((pow(t1, 2.0) / 4.0 + TP->v / d / t1), (1.0 / 2.0)) - 3.0 / 2.0 * t1;  //t2 with bound on velocity not violated
        if(fabs(t2) < tolerance)   t2 = 0.0;   //for continuous time case
        if(TP->Ts > 0)
        {
            t2 = ceil(t2 / TP->Ts) * TP->Ts;
            *dd = TP->v / (2.0 * pow(t1, 3.0) + 3.0 * pow(t1, 2.0) * t2 + t1 * pow(t2, 2.0));
			
        }
    }

//acceleration test   为5555555.
    if(TP->a < ((*dd) * pow(t1, 2.0) + (*dd) * t1 * t2)) //check if a bound is violated
    {
        t2 = TP->a / (d * t1) - t1; //t2 with bound on acceleration not violated
        if(fabs(t2) < tolerance)   t2 = 0.0;   //for continuous time case
        if(TP->Ts > 0)
        {
            t2 = ceil(t2 / TP->Ts) * TP->Ts;
            *dd = TP->a / (pow(t1, 2.0) + t1 * t2);
			
        }
    }

    d = *dd;    //as t2 is now fixed, dd is the new bound on derivative of jerk

    //Calculation constant acceleration phase duration: t3
    //double c1, c2, c3;
    c1 = pow(t1, 2.0) + t1 * t2;
    c2 = 6.0 * pow(t1, 3.0) + 9.0 * pow(t1, 2.0) * t2 + 3.0 * t1 * pow(t2, 2.0);
    c3 = 8.0 * pow(t1, 4.0) + 16.0 * pow(t1, 3.0) * t2 + 10.0 * pow(t1, 2.0) * pow(t2, 2.0) + 2.0 * t1 * pow(t2, 3.0);
    t3 = (-c2 + sqrt(pow(c2, 2.0) - 4.0 * c1 * (c3 - TP->p / d))) / (2.0 * c1);  //largest t3 with bound on acceleration
    if(fabs(t3) < tolerance)   t3 = 0.0;   //for continuous time case

    if(TP->Ts > 0)
    {
        t3 = ceil(t3 / TP->Ts) * TP->Ts;
        *dd = TP->p / (c1 * pow(t3, 2.0) + c2 * t3 + c3); //为96450617283
		
    }

    //velocity test
    if(TP->v < (*dd) * (2.0 * pow(t1, 3.0) + 3.0 * pow(t1, 2.0) * t2 + t1 * pow(t2, 2.0) + pow(t1, 2.0) * t3 + t1 * t2 * t3))    //check if v bound is violated
    {
        t3 = - (2.0 * pow(t1, 3.0) + 3.0 * pow(t1, 2.0) * t2 + t1 * pow(t2, 2.0) - TP->v / d) / (pow(t1, 2.0) + t1 * t2);   //t3, bound on velocity not violated
        if(fabs(t3) < tolerance)   t3 = 0.0;   //for continuous time case
        if(TP->Ts > 0)
        {
            t3 = ceil(t3 / TP->Ts) * TP->Ts;//为23148148.
            *dd = TP->v / (2.0 * pow(t1, 3.0) + 3.0 * pow(t1, 2.0) * t2 + t1 * pow(t2, 2.0) + pow(t1, 2.0) * t3 + t1 * t2 * t3);
			

		}
    }

    d = *dd;    //as t3 is now fixed, dd is the new bound on derivative of jerk

    //Calculation constant velocity phase duration: t4
    t4 = (TP->p - d * (c1 * pow(t3, 2.0) + c2 * t3 + c3)) / TP->v;  //t4 with bound on velocity
    if(fabs(t4) < tolerance) t4 = 0.0;   //for continuous time case
    if(TP->Ts > 0)
    {
        t4 = ceil(t4 / TP->Ts) * TP->Ts; //为96450617283
        *dd = TP->p / (c1 * pow(t3, 2.0) + c2 * t3 + c3 + t4 * (2.0 * pow(t1, 3.0) + 3.0 * pow(t1, 2.0) * t2 + t1 * pow(t2, 2.0) + pow(t1, 2.0) * t3 + t1 * t2 * t3));
		

	}

    //All time intervals are now calculated
    *Td = t1;
    *Tj = t2;
    *Ta = t3;
    *Tv = t4;
    //Quantization of dd and calculation of required position correction (decimal scaling)
   // double x, ddq, pp, dif, cor1, cor2, tt;
    //long ti, cnt;
    if(TP->Ts > 0)
    {
        x = ceil(log10(*dd));          //determine exponent of dd
        comData.dCoarseData[51] = x;
        ddq = *dd / pow(10.0, x);      //          % scale to 0-1
		//logMsg("value1=%x,value2=%x\n",ddq,1,0,0,0,0);
		

		//ddq = round(ddq * pow(10.0, TP->s)) / pow(10.0, TP->s);   //% round to s  decimals
		ddq =      (ddq * pow(10.0, TP->s)) / pow(10.0, TP->s); 
		//logMsg("value1=%x,value2=%x\n",ddq,2,0,0,0,0);
		


		ddq = ddq * pow(10.0, x);
        //actual displacement obtained with quantized dd
        pp = ddq * (c1 * pow(t3, 2.0) + c2 * t3 + c3 + t4 * (2.0 * pow(t1, 3.0) + 3.0 * pow(t1, 2.0) * t2 + t1 * pow(t2, 2.0) + pow(t1, 2.0) * t3 + t1 * t2 * t3));
        dif = TP->p - pp;          // position error due to quantization of dd
        //cnt = round(dif / TP->r);  // divided by resolution gives 'number of increments'
        cnt = (long)(dif / TP->r);
									// of required position correction
        // smooth correction obtained by dividing over entire trajectory duration
        tt = 8.0 * t1 + 4.0 * t2 + 2.0 * t3 + t4;
        //ti = round(tt / TP->Ts);                 //should be integer number of samples

        ti = (long)(tt / TP->Ts);				 //should be integer number of samples

		cor1 = SIGN(cnt) * floor(abs(cnt / ti)) * ti;   // we need cor1/ti increments correction at each
                                                        //    ... sample during trajectory
        cor2 = cnt - cor1;                                       // remaining correction: 1 increment per sample
                                                        // ... during first part of trajectory
        *dd = ddq;
    }
    else
    {
       cor1 = 0.0;
       cor2 = 0.0;
    }
	time->Ts = TP->Ts;
	time->d = *dd;
	time->Td =*Td;
	time->Tj =*Tj;
	time->Ta =*Ta;
	time->Tv =*Tv;
	
	
/*	comData.dCoarseData[38] = TP->Ts;//底轴1 Q限位采样home
 	comData.dCoarseData[39] = *dd;//底轴1 P限位采样
 
 	comData.dCoarseData[40] =	*Td ;//上轴1 Q限位采样home
 	comData.dCoarseData[41] = *Tj;//上轴1 P限位采样
 
 	comData.dCoarseData[42] = *Ta;//底轴1 Q限位采样home
 	comData.dCoarseData[43] = *Tv;//底轴1 P限位采样                             
 
 	comData.dCoarseData[44] =	cal_d_j ;//上轴1 Q限位采样home
 	comData.dCoarseData[45] = t2;
 	comData.dCoarseData[46] =	 t3 ;//上轴1 Q限位采样home
 	comData.dCoarseData[47] = t4;

 comData.dCoarseData[50] = TP->p;
 comData.dCoarseData[51] = TP->v;
 comData.dCoarseData[52] = TP->a;
 comData.dCoarseData[53] = TP->j;
 comData.dCoarseData[54] = TP->d;	*/
	return e_SUCCESS;
}






