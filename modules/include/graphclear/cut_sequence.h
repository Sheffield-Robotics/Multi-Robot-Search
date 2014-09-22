#ifndef GRAPHCLEAR_CUT_SEQUENCE_H
#define GRAPHCLEAR_CUT_SEQUENCE_H

#include <deque>

#include "graphclear/cut.h"

namespace graphclear
{
    
class cut_sequence_t : public std::deque<cut_t>
{
public:
    int a;
    cut_sequence_t ();
    ~cut_sequence_t ();

private:
    /* data */
};

} /* cut_sequence */

#endif