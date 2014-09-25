#ifndef SVG_EDGE
#define SVG_EDGE

// current labels include non_contiguous, hybrid heuristic and hybrid labels   
// contiguous and contiguous_mod

struct svg_edge
{
    int w;      // edge weight
    int w_inv;  // inverted edge weight for MST computation

    // Two labels, one for each direction.
    // Directions are either from
    // -) first to second  <=> [0]
    // -) second to first  <=> [1]
    //
    // A label from first to second represents robots coming
    // from the first and entering the second vertex
    // These labels should be initialized to -1
    int label[2];
    bool clear;
    bool is_in_mst;         // Is edge in minimum spanning tree?
    bool is_in_pipe;        // Helper for label computation
    bool shady;
    bool ignore;
    svg_edge()
    {
        label[0] = 0;
        label[1] = 0;
        clear = false;
        w = 1;
        is_in_mst = true;
        shady = false;
        ignore = false;
    };
};

#endif

