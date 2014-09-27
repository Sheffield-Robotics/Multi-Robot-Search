#ifndef GRAPHCLEAR_CUT_SEQUENCE_H
#define GRAPHCLEAR_CUT_SEQUENCE_H

#include <deque>
#include <algorithm>
#include <iostream>

#include "graphclear/cut.h"
#include "graphclear/sg_typedefs.h"


namespace graphclear
{
    class surveillance_graph_t;

class cut_sequence_t : public std::list<cut_t>
{
public:
    int length;
    std::deque<sg_base::vertex_descriptor> vertex_sequence;
    std::set<cut_t> ordered_cuts;

    cut_sequence_t ();
    ~cut_sequence_t ();
    
    void add_cut( cut_t y );
    void add_cut_unordered( cut_t y );
    void 
        add( sg_base::vertex_descriptor v, int w, int w_v,
            surveillance_graph_t& g);
    void make_full();
    
    friend std::ostream& operator<< (std::ostream& os, 
            const cut_sequence_t& cut_seq)
    {
        os << " CUT SEQUENCE " << std::endl;
        cut_sequence_t::const_iterator it = cut_seq.begin();
        while (it != cut_seq.end() ) {
            os << " " << *it << std::endl;
            it++;
        }
        os << " vertex sequence " << std::endl;
        std::deque<sg_base::vertex_descriptor>::const_iterator 
            it2 = cut_seq.vertex_sequence.begin();
        while (it2 != cut_seq.vertex_sequence.end() ) {
            os << " " << *it2;
            it2++;
        }
        return os;
    };
    
private:
    /* data */
};

} /* cut_sequence */

template <class WeightMap,class CapacityMap>
class vertex_writer {
public:
  vertex_writer(WeightMap w, CapacityMap c) : wm(w),cm(c) {}
  template <class Vertex>
  void operator()(std::ostream &out, const Vertex& v) const {
    out << "[label=\"" << wm[v] << " " << v << "\"]";
  }
private:
  WeightMap wm;
  CapacityMap cm;
};

template <class WeightMap, class CapacityMap>
inline vertex_writer<WeightMap,CapacityMap> 
make_vertex_writer(WeightMap w,CapacityMap c) {
  return vertex_writer<WeightMap,CapacityMap>(w,c);
}

#endif