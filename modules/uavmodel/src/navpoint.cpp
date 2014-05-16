#include "uavmodel/navpoint.h"
#include "utilities/angles.h"
#include <math.h>
#include <stdio.h>

#define DEBUG_NAV (1)
#define MAX_ROT_VEL (0.1)  // [rad/sec]
#define ALPHA (0.9) // trade of between time to travel and angle to turn
#define ANGLE_EPSILON (0.01)
// see http://rossum.sourceforge.net/papers/CalculationsForRobotics/CirclePath.htm

#define EPS_VAL 0.0001;

void NavPoint::getArcParams(NavPoint &np, double& L, double& r_c, double& phi_g) {
    double x_0=x, y_0=y, z_0=z, th_0=norm_angle(th);
    double x_1=np.x, y_1=np.y, z_1=np.z;//, th_1=norm_angle(np.th);
	double dx = x_1-x_0;
	double dy = y_1-y_0;
	double temp_dx = dx;
    double th_0_inv = norm_angle(2*M_PI - th_0);
    //double th_0_inv = norm_angle(th_0);
    dx = dx * cos(th_0_inv) - dy * sin(th_0_inv);
    dy = temp_dx * sin(th_0_inv) + dy * cos(th_0_inv);
	//double dz = z_1-z_0;
    
	// Compute polar coordinates towards p_1
	bool leftTurn = false;
	bool rightTurn = false;
	bool straight = false;	
	
	double r_g = sqrt(dx*dx + dy*dy); 
	
	if (dx==0.0) dx += EPS_VAL;
	
	phi_g = norm_angle(atan2(dy,dx));
	
	// Compute desired radius (positive means left turn) and length
	L = 0.0;
	r_c = 0.0;
	if (fabs( sin(phi_g) )  < ANGLE_EPSILON ) {
		straight = true;
		leftTurn=false;
		rightTurn=false;
		r_c=0.0;
		// Compute length of the straight
		L = r_g;				
	}
	else {
		// Compute the radius
		r_c = r_g / (2.0 * sin(phi_g));
		// Compute length of the arc
		L = (r_g * phi_g) / (sin(phi_g));		
		if (r_c < 0.0) {
			r_c *= -1.0;
			straight=false;
			leftTurn=true;
			rightTurn=false;
		}
		else {
			straight=false;
			leftTurn=false;
			rightTurn=true;			
		}		
	}
    
}


double NavPoint::getCostToPoint(NavPoint &np) 
{
	double x_0=x, y_0=y, z_0=z, th_0=norm_angle(th);
    double x_1=np.x, y_1=np.y, z_1=np.z;//, th_1=norm_angle(np.th);
	double dx = x_1-x_0;
	double dy = y_1-y_0;
	// rotate to make relative to the Navpoints coordinate system
	if (DEBUG_NAV)
		printf("prior rot dx=%1.2lf, dy=%1.2lf, th=%1.2lf\n",dx,dy,th_0);
    double temp_dx = dx;
    double th_0_inv = norm_angle(2*M_PI - th_0);
    dx = dx * cos(th_0_inv) - dy * sin(th_0_inv);
    dy = temp_dx * sin(th_0_inv) + dy * cos(th_0_inv);
	double dz = z_1-z_0;
    
    if (DEBUG_NAV)
		printf("x=%1.2lf, y=%1.2lf, x1=%1.2lf, y1=%1.2lf\n",x_0,y_0,x_1,y_1);
    if (DEBUG_NAV)
        printf("local coor dx=%1.2lf, dy=%1.2lf, dz=%1.2lf, th=%1.2lf\n",dx,dy,dz,th_0);

	// Compute polar coordinates towards p_1
	bool leftTurn = false;
	bool rightTurn = false;
	bool straight = false;	
	double r_g = sqrt(dx*dx + dy*dy); 
	if (dx==0.0) dx += EPS_VAL;
	double phi_g = norm_angle(atan2(dy,dx));
	if (DEBUG_NAV)
		printf("phi_g=%1.2lf\n",phi_g);
	
	// Compute desired radius (positive means left turn) and length
	double L = 0.0;
	double r_c = 0.0;
	if (fabs( sin(phi_g) )  < ANGLE_EPSILON ) {
		straight = true;
		leftTurn=false;
		rightTurn=false;
		r_c=0.0;
		// Compute length of the straight
		L = r_g;				
	}
	else {
		// Compute the radius
		r_c = r_g / (2.0 * sin(phi_g));
		// Compute length of the arc
		L = (r_g * phi_g) / (sin(phi_g));		
		if (r_c < 0.0) {
			r_c *= -1.0;
			straight=false;
			leftTurn=true;
			rightTurn=false;
		}
		else {
			straight=false;
			leftTurn=false;
			rightTurn=true;			
		}		
	}
	
	if (DEBUG_NAV)
		printf("rad=%1.2lf, turnLeft=%d straight=%d\n",r_c,leftTurn,straight);

	// Compute time needed for travelling the arc
	if (tv==0.0) tv += EPS_VAL;
	
    // double dt = L / tv;
    // Approximation to add influence of altitude change:
    double dt = sqrt(dz*dz + L*L) / tv;

	
	// Compute the rotational velocity needed
	double theta = norm_angle(2.0 * phi_g);
    np.th = np.th + theta;
    np.th = norm_angle(np.th);
	double omega = theta / dt;
	if (!leftTurn) omega *= -1.0;

	if (DEBUG_NAV)	
		printf("L=%1.2lf  dt=%1.2lf  omega=%1.2lf \n",L,dt,omega);
	
	if (fabs(omega) > MAX_ROT_VEL) {
		if (DEBUG_NAV)	printf("Cannot reach point!\n");
		return HUGE_VAL;
	}

	
//	double x_test = x_0 - r_c * sin(th_0) + r_c * sin(omega*dth + th_0);
//	double y_test = y_0 - r_c * cos(th_0) - r_c * cos(omega*dth + th_0);
//		printf("Goal ist at (%1.2lf,%1.2lf) --> Will end up at (%1.2lf, %1.2lf)\n",
//		x_1,y_1,x_test,y_test);
		
	double cost = ALPHA * dt + (1-ALPHA) * fabs(theta);
	if (DEBUG_NAV)	printf("Cost: %1.2lf\n",cost);
	
	return cost;
}

double NavPoint::getSqrDistToPoint(NavPoint &np) {
    return pow(np.x-x,2) + pow(np.y-y,2);
}

double NavPoint::getDistToPoint(NavPoint &np) {
    return sqrt(pow(np.x-x,2) + pow(np.y-y,2));
}