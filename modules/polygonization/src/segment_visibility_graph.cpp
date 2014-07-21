#include "polygonization/Segment_Visibility_Graph.h"

using namespace boost;
using namespace std;

int 
Segment_Visibility_Graph::get_shortest_path(
    Segment_Visibility_Graph::vertex start, 
    Segment_Visibility_Graph::vertex end )
{
    vector<mygraph_t::vertex_descriptor> p(num_vertices(*g));
    vector<cost_t> d(num_vertices(*g));
    //location locations[] = { // lat/long
    //    {42.73, 73.68}, {44.28, 73.99}, {44.70, 73.46},
    //    {44.93, 74.89}, {43.97, 75.91}, {43.10, 75.23},
    //    {43.04, 76.14}, {43.17, 77.61}, {42.89, 78.86},
    //    {42.44, 76.50}, {42.10, 75.91}, {42.04, 74.11},
    //    {40.67, 73.94}
    //  };
      
    try {
      // call astar named parameter interface
      //astar_search
      //  (*g, start,
      //   distance_heuristic<mygraph_t, cost_t, location*>
      //    (locations, end),
      //   predecessor_map(&p[0]).distance_map(&d[0]).
      //   visitor(astar_goal_visitor<vertex>(end)));
    } catch(found_goal fg) 
    { 
        // found a path to the goal
        list<vertex> shortest_path;
        for(vertex v = end;; v = p[v]) {
            shortest_path.push_front(v);
            if(p[v] == v)
                break;
        }
        list<vertex>::iterator spi = shortest_path.begin();
        for(++spi; spi != shortest_path.end(); ++spi) 
        {
            
        }
        cout << endl << "Total travel time: " << d[end] << endl;
        return 0;
    }
    return 0;
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
    //weightmap = boost::get(edge_weights, *g);
    //for(std::size_t j = 0; j < n_e; ++j) {
    //  edge_descriptor e; bool inserted;
    //  tie(e, inserted) = add_edge(edge_array[j].first,
    //                              edge_array[j].second, *g);
    //  weightmap[e] = weights[j];
    //}
    
}
