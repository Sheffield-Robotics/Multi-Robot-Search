#ifndef SVG_STRATEGY
#define SVG_STRATEGY

#include <boost/graph/adjacency_list.hpp>
#include <math.h>
#include <vector>
#include <iostream>
#include <fstream>
#include "define.h"
#include "svg_graph.h"
#include "svg_label_computer.h"

using namespace std;
using namespace boost;

typedef std::vector<svg_vertex_d> svg_vertex_vector;

class svg_strategy_step
{
public:
    svg_vertex_vector new_guards;
    svg_vertex_vector freed_guards;
    svg_vertex_d      new_vertex;
};

typedef std::vector<svg_strategy_step> svg_strategy_seq;

class svg_strategy
{
private:

    svg_strategy_seq sequence;
    svg_strategy_seq sequence_compressed;
    svg_graph* SVGp;

    void
    clean_up_graph();
    edge_vector
    collecting_neighbors( svg_vertex_d current_v, svg_edge_d current_e, bool starting);

    bool
    has_contam_nonmst_neighbor( svg_vertex_d vd );
    bool
    has_clear_nonmst_neighbor( svg_vertex_d vd );
    void
    collect_freed_nonmst_neighbors( svg_vertex_d vd, svg_strategy_step& new_step);

    void
    collect_freed_mst_neighbors( svg_vertex_d vd, svg_strategy_step& new_step);

    bool
    is_to_be_freed( svg_vertex_d vd );


    void
    visit_vertex( svg_vertex_d current_v, svg_edge_d current_e, bool starting, bool last_child );

    bool
    is_clear( svg_vertex_d vd );

public:
    svg_strategy( svg_graph* SVGp , svg_vertex_d start_v );

    int
    compress_strategy();
    int
    get_newguards( int step );
    int
    get_freeguards( int step );

    int
    number_of_robots();
    void
    cout_strategy();
    void
    cout_schedule();
    int
    find_free_robot( std::vector< std::vector< int > >& schedule, int step );
    void
    free_robot( std::vector< std::vector< int > >& schedule, svg_vertex_d at_v, int step );



    svg_vertex_d
    get_vertex( int step );

    svg_vertex_d
    get_vertex( int step, svg_vertex_d& from );

    int
    size();

    void
    print_schedule_to_file(const char* filename );

    svg_strategy_step*
    get_step( int step );
    int robots_max;

};

#endif
