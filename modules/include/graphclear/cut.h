#ifndef GRAPHCLEAR_CUT_H
#define GRAPHCLEAR_CUT_H

#include "graphclear/sg_typedefs.h"
#include <list>
#include <unordered_set>

namespace graphclear
{
    
class cut_t : public std::list<sg_base::vertex_descriptor>
{
public:
    cut_t() : rho(0),ag(0),b(0) {};
    ~cut_t ();

    std::unordered_set<sg_base::vertex_descriptor> v_set;
    
    //push_back(sg_base::vertex_descriptor v)
    
    int helper_index;

    int rho;
    int ag;
    int b;
    
    bool operator< ( const cut_t& y ) const
    {
        if (y.rho < rho)
            return false;
        else if (y.rho > rho)
            return true;
        else {
            if (y.ag < ag)
                return false;
            else if (y.ag > ag)
                return true;
            else {
                if (y.b < b)
                    return false;
                else if (y.b > b)
                    return true;
                else {
                    if ( !this->empty() && !y.empty())
                        return this->back() < y.back();
                    else 
                        return false;
                }
            }
        }
    }

    friend std::ostream& operator<< (std::ostream& os, const cut_t& cut)
    {
        os << "ag=" << cut.ag 
            << " b=" << cut.b 
            << " rho=" << cut.rho << " ";
        cut_t::const_iterator it = cut.begin();
        while (it != cut.end() ) {
            os << " " << *it;
            it++;
        }
        return os;
    };
    
    void push_back (const sg_base::vertex_descriptor& val) 
    {
        if ( v_set.find(val) == v_set.end() ) {
            std::list<sg_base::vertex_descriptor>::push_back(val);
            v_set.insert(val);
        }
    }
    
private:
    /* data */
    
};

} /* graphclear */

#endif

