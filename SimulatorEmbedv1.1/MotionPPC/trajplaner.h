#ifndef _PLANER_H
#define _PLANER_H

typedef struct
{
	double X;
	double Y;
	double Z;
}position_t;

typedef enum
{
    line,
    arc
}interpolateType;

typedef enum
{
	forward,
	inverse
}Dir_t;

typedef struct
{
	position_t startPoint;
	position_t endPoint;
	position_t centerPoint;
    position_t interval;
	position_t current;
	float radius;
	float Ts;
	float FRN;
	float vel;
	float lamda;
    Dir_t dir;
}arcInterpolate_t;

typedef struct
{
	position_t startPoint;
	position_t endPoint;
    position_t interval;
	position_t current;
	float FRN;
	double len;
	float Ts;
    float vel;
}lineInterpolate_t;

extern arcInterpolate_t *arcInterpolate;
extern lineInterpolate_t *lineInterpolate;
void mc_line(lineInterpolate_t *line);
int lineCal(lineInterpolate_t *line);
void mc_arc(arcInterpolate_t *arc);
int arcCal(arcInterpolate_t *arc);

#endif