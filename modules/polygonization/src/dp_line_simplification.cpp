#include "polygonization/dp_line_simplification.h"

namespace DPLineSimplification {
    void 
    find_split( int i, int j, int *split, double *dist, polygonization::Polygon& P) {
        int k;
        HOMOG q;
        double tmp;
        *dist = -1;
        if (i + 1 < j) {
            CROSSPROD_2CCH(P[i], P[j], q); 
            for (k = i + 1; k < j; k++) {
                tmp = DOTPROD_2CH(P[k], q); 
                if (tmp < 0) tmp = - tmp;
                if (tmp > *dist) {
                    *dist  = tmp;
                    *split = k;
                }
            }
            *dist *= *dist/(q[XX]*q[XX] + q[YY]*q[YY]); 
        }
    }

    //Douglas Peucker Line Simplification
    void 
    simplify( polygonization::Polygon& P, std::vector<int>& final_indices, double epsilon ) {
        double epsilon_sq = pow(epsilon,2);
        bool outFlag = true;
        int split_index; 
        double dist_sq;
        
        // collect indices in stack
        std::stack<int> my_stack;
        //put last point into the stack 
        my_stack.push(P.size()-1); 
        // start with first point
        int i = 0;
        do {
            find_split(i, my_stack.top(), &split_index, &dist_sq, P );
            if ( dist_sq > epsilon_sq ) 
                my_stack.push(split_index);
            else {
                if ( outFlag ) { 
                    // add the first index
                    outFlag = false;
                    final_indices.push_back(i);
                }
                i = my_stack.top(); my_stack.pop();
                final_indices.push_back(i);
            }
        } while (! (my_stack.empty()) );
    }
}