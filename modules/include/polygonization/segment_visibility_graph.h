#ifndef SEGMENT_VISIBILITY_GRAPH_H
#define SEGMENT_VISIBILITY_GRAPH_H

#include <boost/graph/astar_search.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/random.hpp>
#include <boost/random.hpp>
#include <boost/graph/graphviz.hpp>
#include <sys/time.h>
#include <vector>
#include <list>
#include <iostream>
#include <fstream>
#include <math.h>    // for sqrt

class Segment_Visibility_Graph
{
  public:
      
    struct seg_vertex
    {
        int segment_index;
        int type;
        seg_vertex(int i,int t) : segment_index(i), type(t) {};
    };
    struct seg_edge
    {
        int segment_index;
        int segment_index2;
        double distance;
    };
    
    typedef float cost_t;
    typedef boost::adjacency_list
        <boost::listS, boost::vecS, boost::undirectedS, 
         seg_vertex, seg_edge 
        > mygraph_t;
    typedef boost::property_map
        <mygraph_t, 
         double seg_edge::* 
        >::type WeightMap;
    typedef mygraph_t::vertex_descriptor vertex;
    typedef mygraph_t::edge_descriptor edge_descriptor;
    typedef mygraph_t::vertex_iterator vertex_iterator;
    typedef std::pair<int, int> edge;

    int 
    get_shortest_path(
        Segment_Visibility_Graph::vertex start, 
        Segment_Visibility_Graph::vertex end );
    
    Segment_Visibility_Graph( int n_v, int n_e, float* edge_weights, edge* edge_array );
    Segment_Visibility_Graph( );

    struct location
    {
      float y, x; // lat, long
    };
    
    // euclidean distance heuristic
    template <class Graph, class CostType, class LocMap>
    class distance_heuristic : public boost::astar_heuristic<Graph, CostType>
    {
    public:
      typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
      distance_heuristic(LocMap l, Vertex goal)
        : m_location(l), m_goal(goal) {}
      CostType operator()(Vertex u)
      {
        CostType dx = m_location[m_goal].x - m_location[u].x;
        CostType dy = m_location[m_goal].y - m_location[u].y;
        return ::sqrt(dx * dx + dy * dy);
      }
    private:
      LocMap m_location;
      Vertex m_goal;
    };
    
    struct found_goal {}; // exception for termination
    
    // visitor that terminates when we find the goal
    template <class Vertex>
    class astar_goal_visitor : public boost::default_astar_visitor
    {
    public:
      astar_goal_visitor(Vertex goal) : m_goal(goal) {}
      template <class Graph>
      void examine_vertex(Vertex u, Graph& g) {
        if(u == m_goal)
          throw found_goal();
      }
    private:
      Vertex m_goal;
    };
    
    
private:
    mygraph_t* g;
    WeightMap weightmap;// = get(edge_weight, g);
    
};

#endif