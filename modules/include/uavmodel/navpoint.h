#ifndef _NAVPOINT_H
#define _NAVPOINT_H

/// Describes a point for navigation
class NavPoint 
{
  public:
    NavPoint(double _x, double _y, double _z, double _th, double _tv=0.0, double _rv=0.0)
        {x=_x;y=_y;z=_z;th=_th;tv=_tv;rv=_rv;locked=false;};
    NavPoint()
        {x=0;y=0;z=0;th=0;tv=0;rv=0;locked=false;};
    NavPoint(const NavPoint &np) 
        {x=np.x;y=np.y;z=np.z;th=np.th;tv=np.tv;rv=np.rv;locked=np.locked;};

    double getCostToPoint(NavPoint &np); 
    double getSqrDistToPoint(NavPoint &np); 
    double getDistToPoint(NavPoint &np); 
    void getArcParams(NavPoint &np, double& L, double& r_c, double& phi_g);
    

    bool locked;
    double x,y,z,th;     // Start position [m,m,m] and yaw orientaion [rad]
    double tv;           // Velocity in [m/s]
    double rv;           // Velocity in [rad/s]
};

#endif

