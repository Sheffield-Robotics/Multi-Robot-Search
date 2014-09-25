#ifndef GRAPHCLEAR_CUT_H
#define GRAPHCLEAR_CUT_H

#include "graphclear/sg_typedefs.h"
#include <list>

namespace graphclear
{
    
class cut_t : public std::list<sg_base::vertex_descriptor>
{
public:
    cut_t() : rho(0),ag(0),b(0) {};
    ~cut_t ();

    int rho;
    int ag;
    int b;
    bool operator< ( const cut_t& y ) const
    {
        return y.rho < rho;
    }

private:
    /* data */
    
};

} /* graphclear */

#endif

