#include "Pt.h"



Pt::Pt()
{
	x = 0;
	y = 0;
	range=0;
   theta=0;
   velcity=0;
}
Pt::Pt(double _x, double _y)
{
	x = _x;
	y = _y;
	range = 0;
	theta = 0;
}

Pt::Pt(double _x, double _y, double _range, double _theta)
{
	x = _x;
	y = _y;
	range = _range;
	theta = _theta;
}
