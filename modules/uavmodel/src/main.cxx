#include <stdio.h>
#include <stdlib.h>

#include "uavmodel/navpoint.h"

int main(int argc, char* argv[]) 
{
	if (argc!=5) {
		printf("Syntax: %s x y z v_t\n", argv[0]);
		return -1;
	}
	
	double x1=atof(argv[1]);
	double y1=atof(argv[2]);
	double z1=atof(argv[3]);
	double tv=atof(argv[4]);
	//printf("Using x_1=%1.2lf,y_1=%1.2lf,v_t=%1.2lf\n",x1,y1,tv);
	NavPoint p0(0 ,0 , 0, 0, tv, 0);
	NavPoint p1(x1, y1, z1, 0, tv, 0);
	
	p0.getCostToPoint(p1);
	
	return 0;
}
