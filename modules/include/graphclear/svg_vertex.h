#ifndef SVG_VERTEX
#define SVG_VERTEX

#include "define.h"

struct svg_vertex
{
    int w;          // weight of vertex
    int mst_degree; // degree of vertex w.r.t. to MST

    bool clear;

    int color;
    int p;

    svg_vertex()
    {
        w = 0;
        mst_degree = 0;
        clear = false;
    };
};

#endif
