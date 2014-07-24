#include "polygonization/Segment_Visibility_Graph.h"

using namespace boost;
using namespace std;

std::list<Segment_Visibility_Graph::vertex> 
Segment_Visibility_Graph::get_shortest_path(
    Segment_Visibility_Graph::vertex start, 
    Segment_Visibility_Graph::vertex goal )
{
    std::vector<vertex> p(num_vertices(*g));
    std::vector<cost_t> d(num_vertices(*g));
    weightmap = boost::get(&seg_edge::distance, *g);
    list<vertex> shortest_path;
    try {
        astar_goal_visitor<vertex> astar_visitor(goal);
        boost::astar_search
            (*g, start,
            distance_heuristic(g, goal),          
            boost::weight_map(weightmap).
            predecessor_map(&p[0]).
            visitor(astar_visitor)
            );
        
    } catch(found_goal fg) 
    { 
        // found a path to the goal
        
        for(vertex v = goal;; v = p[v]) {
            shortest_path.push_front(v);
            if(p[v] == v)
                break;
        }
        list<vertex>::iterator spi = shortest_path.begin();
        for(++spi; spi != shortest_path.end(); ++spi) 
        {
            cout << " vertex " 
                << (*g)[*spi].segment_index << " "
                << (*g)[*spi].p_x << ":"
                << (*g)[*spi].p_y << " "
                << (*g)[*spi].type << " "
                    << endl;
        }
        cout << endl << "Total travel time: " << d[goal] << endl;
    }
    return shortest_path;
}

Segment_Visibility_Graph::vertex
Segment_Visibility_Graph::add_vertex(seg_vertex vertex_info)
{
    vertex v;
    bool inserted;
    v = boost::add_vertex(vertex_info,*g);
    return v;
}

Segment_Visibility_Graph::edge_descriptor
Segment_Visibility_Graph::add_edge(vertex v1, vertex v2)
{
    edge_descriptor e;
    seg_edge p;
    bool inserted;
    tie(e,inserted) = boost::add_edge(v1,v2,p,*g);
    // TODO
    //weightmap[e] = weights[j];
    return e;
}

void
Segment_Visibility_Graph::add_edge(vertex v1, vertex v2, double d)
{    
    edge_descriptor e = this->add_edge(v1,v2);
    // set d x,y in e properties
}

void
Segment_Visibility_Graph::add_edge(vertex v1, vertex v2, double x, double y, double d)
{
    seg_edge p;
    p.p_x = x;
    p.p_y = y;
    p.distance = d;
    edge_descriptor e;
    bool inserted;
    tie(e,inserted) = boost::add_edge(v1,v2,p,*g);
    
    // set d x,y in e properties
}


Segment_Visibility_Graph::Segment_Visibility_Graph( )
{
    g = new mygraph_t;
}



Segment_Visibility_Graph::Segment_Visibility_Graph( int n_v, int n_e, float* edge_weights, edge* edge_array )
{
    // build the graph   
    g = new mygraph_t(n_v);
    
    
    //for(std::size_t j = 0; j < n_e; ++j) {
    //  edge_descriptor e; bool inserted;
    //  tie(e, inserted) = add_edge(edge_array[j].first,
    //                              edge_array[j].second, *g);
    //  weightmap[e] = weights[j];
    //}
    
}
