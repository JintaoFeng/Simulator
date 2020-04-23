#include "trajplaner.h"
#include <math.h>
//#include "Rtcontrol.h"
//#include "StateMachine.h"

arcInterpolate_t *arcInterpolate;
lineInterpolate_t *lineInterpolate;

void mc_line(lineInterpolate_t *line)
{
   	line->len=sqrt(pow((line->endPoint.X -line->startPoint.X),2)+pow((line->endPoint.Y-line->startPoint.Y),2));
	line->FRN=line->vel/line->len;
	line->interval.X=line->Ts*line->FRN*(line->endPoint.X-line->startPoint.X);
	line->interval.Y=line->Ts*line->FRN*(line->endPoint.Y-line->startPoint.Y);
	line->current.X=line->startPoint.X;
	line->current.Y=line->startPoint.Y;
}

int lineCal(lineInterpolate_t *line)
{
	int judgeFlage=0;
	line->current.X+=line->interval.X;
	line->current.Y+=line->interval.Y;
	if(fabs(line->endPoint.X-line->current.X)<fabs(line->interval.X) || fabs(line->endPoint.Y-line->current.Y)<fabs(line->interval.Y))
	{
		line->current.X=line->endPoint.X;
		line->current.Y=line->endPoint.Y;
		judgeFlage=1;
	}

	return judgeFlage;
}
void mc_arc(arcInterpolate_t *arc)
{
	arc->FRN=arc->vel/arc->radius;
	arc->lamda=arc->Ts*arc->FRN;
	//将世界坐标系下的坐标转换到以圆心为原点的坐标系
	arc->current.X=arc->startPoint.X-arc->centerPoint.X;
	arc->current.Y=arc->startPoint.Y-arc->centerPoint.Y;
}
int arcCal(arcInterpolate_t *arc)
{//逆时针
	int judgeFlage=0;
	if(arc->dir == inverse)
	{
		arc->interval.X=arc->current.Y*arc->lamda+0.5*pow(arc->lamda,2)*arc->current.X;
		arc->interval.Y=arc->current.X*arc->lamda-0.5*pow(arc->lamda,2)*arc->current.Y;
		arc->current.X -= arc->interval.X;
		arc->current.Y += arc->interval.Y;
		//转换到绝对坐标系下
		arc->startPoint.X =arc->current.X+arc->centerPoint.X;
		arc->startPoint.Y =arc->current.Y+arc->centerPoint.Y;
		if(pow((arc->current.X-arc->endPoint.X),2)+pow((arc->current.Y-arc->endPoint.Y),2)<=pow((arc->Ts*arc->vel/2),2))
		{
			arc->startPoint.X=arc->endPoint.X;
			arc->startPoint.Y=arc->endPoint.Y;
			judgeFlage=1;
//			Machine.coarseStage.stageStatus = S_Stage_ALL_Close;
		}
	}//顺时针走
	else if(arc->dir==forward)
	{
		arc->interval.X=arc->current.Y*arc->lamda-0.5*pow(arc->lamda,2)*arc->current.X;
		arc->interval.Y=arc->current.X*arc->lamda+0.5*pow(arc->lamda,2)*arc->current.Y;
		arc->current.X += arc->interval.X;
		arc->current.Y -= arc->interval.Y;
		//转换到绝对坐标系下
		arc->startPoint.X =arc->current.X+arc->centerPoint.X;
		arc->startPoint.Y =arc->current.Y+arc->centerPoint.Y;
		if(pow((arc->startPoint.X-arc->endPoint.X),2)+pow((arc->startPoint.Y-arc->endPoint.Y),2)<=pow((arc->Ts*arc->vel/2),2))
		{
			arc->startPoint.X=arc->endPoint.X;
			arc->startPoint.Y=arc->endPoint.Y;
			judgeFlage=1;
//			Machine.coarseStage.stageStatus = S_Stage_ALL_Close;
		}
	}
//	Machine.coarseStage.arrAxis[0].dSetPoint=arc->startPoint.X;
//	Machine.coarseStage.arrAxis[1].dSetPoint=arc->startPoint.X;
//	Machine.coarseStage.arrAxis[2].dSetPoint=arc->startPoint.Y;
	return judgeFlage;
}