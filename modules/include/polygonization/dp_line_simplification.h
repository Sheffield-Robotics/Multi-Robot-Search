#ifndef DPLineSimplification_H
#define DPLineSimplification_H

#include "polygonization/polygon_environment.h"
#include <stack>

namespace DPLineSimplification {
    typedef double HOMOG[3];
    #define XX 0
    #define YY 1
    #define WW 2
    #define CROSSPROD_2CCH(p, q, r) /* 2-d cartesian to homog cross product */\
     (r)[WW] = CGAL::to_double((p).x()) * CGAL::to_double((q).y()) - CGAL::to_double((p).y()) * CGAL::to_double((q).x());\
     (r)[XX] = - CGAL::to_double((q).y()) + CGAL::to_double((p).y());\
     (r)[YY] =   CGAL::to_double((q).x()) - CGAL::to_double((p).x());
    #define DOTPROD_2CH(p, q)	/* 2-d cartesian to homog dot product */\
     CGAL::to_double((q)[WW]) + CGAL::to_double((p).x())*CGAL::to_double((q)[XX]) + CGAL::to_double((p).y())*CGAL::to_double((q)[YY])
    
	void
	find_split( int i, int j, int *split, double *dist, polygonization::Polygon& P);
	
	void 
	simplify( polygonization::Polygon& P, std::vector<int>& final_indices, double epsilon );
}

#endif