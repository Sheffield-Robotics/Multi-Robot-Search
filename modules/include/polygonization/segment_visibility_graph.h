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
      
    class seg_vertex
    {
    public:
        int segment_index;
        int type; // vertices are either type 1 (corresponds to segment) or type 2 (corresponds to a vertex)
        bool reflexive;
        double p_x;
        double p_y;
        double seg_x;
        double seg_y;
        double seg_x2;
        double seg_y2;
        boost::default_color_type color;
        seg_vertex(int i,int t) : segment_index(i), type(t) {};
        seg_vertex(int i,int t,double x, double y) 
            : segment_index(i), type(t), p_x(x), p_y(y) {reflexive = false;};
        seg_vertex(int i,int t,double x, double y, double x1, double y1, double x2, double y2) 
            : segment_index(i), type(t), p_x(x), p_y(y), seg_x(x1), seg_y(y1), seg_x2(x2), seg_y2(y2)  
                {reflexive = false;};
        seg_vertex() { reflexive = false;};
    };
    class seg_edge
    {
    public:
        int segment_index;
        int segment_index2;
        double distance;
        double distance_temp;
        double p_x;
        double p_y;
        double p_x2;
        double p_y2;
        bool point_is_set;
        bool point_is_set2;
        double edge_weight_t; 
        seg_edge(int i, int j, double d) : segment_index(i), segment_index2(j), distance(d) { point_is_set = false;};
        seg_edge(int i, int j, double d, double x, double y) 
            : segment_index(i), segment_index2(j), distance(d), p_x(x), p_y(y) { point_is_set = true;};
        seg_edge() {point_is_set = false;};
    };
    

    typedef double cost_t;
    typedef boost::adjacency_list
        <boost::listS, boost::vecS, boost::undirectedS, 
         seg_vertex, seg_edge 
        > mygraph_t;
    typedef boost::property_map
        <mygraph_t, double seg_edge::*>::type WeightMap;
    typedef boost::property_map
        <mygraph_t, boost::default_color_type seg_vertex::*>::type ColorMap;
    //typedef std::vector<boost::default_color_type> ColorMap;
    //std::vector<default_color_type> color_vec( num_vertices(graph) );
    typedef mygraph_t::vertex_descriptor vertex;
    typedef mygraph_t::edge_descriptor edge_descriptor;
    typedef mygraph_t::vertex_iterator vertex_iterator;
    typedef std::pair<int, int> edge;

    std::list<vertex> 
    get_shortest_path(
        Segment_Visibility_Graph::vertex start, 
        Segment_Visibility_Graph::vertex end );
    
    Segment_Visibility_Graph( int n_v, int n_e, float* edge_weights, edge* edge_array );
    Segment_Visibility_Graph( );
    vertex add_vertex(seg_vertex vertex_info);
    edge_descriptor add_edge(vertex v1, vertex v2);
    edge_descriptor add_edge(vertex v1, vertex v2, double d);
    edge_descriptor add_edge(vertex v1, vertex v2, double x, double y, double x2, double y2, double d);
    edge_descriptor get_edge_shortest(vertex v, vertex w);

    struct location
    {
      double y, x; // lat, long
    };
    
    // euclidean distance heuristic
    class distance_heuristic : public boost::astar_heuristic<mygraph_t, double>
    {
    public:
      
      distance_heuristic(mygraph_t* g, mygraph_t::vertex_descriptor goal)
        : m_graph(g), m_goal(goal) {}
       double operator()(mygraph_t::vertex_descriptor u)
       {
          double dx = (*m_graph)[m_goal].seg_x - (*m_graph)[u].p_x;
          double dy = (*m_graph)[m_goal].seg_y - (*m_graph)[u].p_y;
          double dx2 = (*m_graph)[m_goal].seg_x2 - (*m_graph)[u].p_x;
          double dy2 = (*m_graph)[m_goal].seg_y2 - (*m_graph)[u].p_y; 
          return fmin( ::sqrt(dx * dx + dy * dy),::sqrt(dx2 * dx2 + dy2 * dy2));
      }
    private:
      mygraph_t* m_graph;
      mygraph_t::vertex_descriptor m_goal;
    };
    
    struct found_goal {}; // exception for termination
    
    // visitor that terminates when we find the goal
    template <class Vertex, class Edge>
    class astar_goal_visitor : public boost::default_astar_visitor
    {
    public:
      astar_goal_visitor(Vertex goal, Vertex start, ColorMap* colormap) : m_goal(goal),m_start(start), m_colormap(colormap) {}
      template <class Graph>
      void examine_vertex(Vertex u, Graph& g) {
          //std::cout << " examine_vertex " << u << std::endl;
          if(u == m_goal)
              throw found_goal();
      }
      
      // template <class Graph>
      // void initialize_vertex(Vertex u, Graph& g) {
      //     typedef typename boost::property_traits<ColorMap>::value_type ColorValue;
      //     typedef boost::color_traits<ColorValue> Color;
      //     std::cout << " initialize_vertex " << u;
      //     std::cout << " type " << g[u].type;
      //     std::cout << " Segment Index " << g[u].segment_index << std::endl;
      //     //if ( g[u].type == 1 && u != m_goal && u!= m_start ) {
      //     //    std::cout << " put color to black " << u << std::endl;
      //     //    put(*m_colormap, u, Color::black());
      //     //}
      // }
      //
      // template <class Graph>
      // void discover_vertex(Vertex u, Graph& g) {
      //     std::cout << " discover_vertex " << u << std::endl;
      //     std::cout << " Type " << g[u].type;
      //     std::cout << " Segment Index " << g[u].segment_index << std::endl;
      // }
      //
      // template <class Graph>
      // void edge_relaxed(Edge e, Graph& g) {
      //     std::cout << " edge_relaxed " << e ;
      //     std::cout << "     edge-d " << g[e].distance;
      //     std::cout << "     sour type " << g[source(e,g)].type;
      //     std::cout << "     targ type " << g[target(e,g)].type;
      //     std::cout << "     sour seg " << g[source(e,g)].segment_index;
      //     std::cout << "     targ seg " << g[target(e,g)].segment_index;
      //     std::cout << std::endl;
      // }
      //
      // template <class Graph>
      // void examine_edge(Edge e, Graph& g) {
      //     std::cout << " examine_edge " << e ;
      //     std::cout << "     edge-d " << g[e].distance;
      //     std::cout << "     sour type " << g[source(e,g)].type;
      //     std::cout << "     targ type " << g[target(e,g)].type;
      //     std::cout << "     sour seg " << g[source(e,g)].segment_index;
      //     std::cout << "     targ seg " << g[target(e,g)].segment_index;
      //     std::cout << std::endl;
      // }
      //
      //
      // template <class Graph>
      // void black_target(Edge e, Graph& g) {
      //     std::cout << " black_target " << e << std::endl;
      //}      
      
    private:
      Vertex m_goal;
      Vertex m_start;
      ColorMap* m_colormap;
    };
    
    
public:
    mygraph_t* g;
    WeightMap weightmap;// = get(edge_weight, g);
    ColorMap colormap;
    
};

#endif