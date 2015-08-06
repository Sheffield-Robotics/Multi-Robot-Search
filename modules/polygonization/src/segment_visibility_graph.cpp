#include "polygonization/Segment_Visibility_Graph.h"

using namespace boost;
using namespace std;

std::list<Segment_Visibility_Graph::vertex>
Segment_Visibility_Graph::get_shortest_path(
    Segment_Visibility_Graph::vertex start,
    Segment_Visibility_Graph::vertex goal) {
  std::vector<vertex> p(num_vertices(*g));
  std::vector<cost_t> d(num_vertices(*g));
  weightmap = boost::get(&seg_edge::distance, *g);
  colormap = boost::get(&seg_vertex::color, *g);
  list<vertex> shortest_path;
  try {
    astar_goal_visitor<vertex, edge_descriptor> astar_visitor(goal, start,
                                                              &colormap);
    boost::astar_search(*g, start, distance_heuristic(g, goal),
                        boost::weight_map(weightmap).predecessor_map(&p[0]).
                        // color_map(colormap).
                        visitor(astar_visitor));

  } catch (found_goal fg) {
    // found a path to the goal

    for (vertex v = goal;; v = p[v]) {
      shortest_path.push_front(v);
      if (p[v] == v)
        break;
    }
    // list<vertex>::iterator spi = shortest_path.begin();
    // cout << " shortest path print: "  << std::endl;
    // for(++spi; spi != shortest_path.end(); ++spi)
    //{
    //    cout << " vertex seg index "
    //        << (*g)[*spi].segment_index << " "
    //        << (*g)[*spi].p_x << ":"
    //        << (*g)[*spi].p_y << " "
    //        << "type " << (*g)[*spi].type << " "
    //            << endl;
    //}
    // cout << endl << "Total travel time: " << d[goal] << endl;
  }
  return shortest_path;
}

// TODO: remove all non-shortest edges after polygon construction
// Segment_Visibility_Graph::edge_descriptor
// Segment_Visibility_Graph::remove_non_shortest_edges(vertex v, vertex w)
//{
//
//}

std::pair<Segment_Visibility_Graph::edge_descriptor, bool>
Segment_Visibility_Graph::get_edge_shortest(vertex v, vertex w) {
  Segment_Visibility_Graph::mygraph_t::out_edge_iterator ei, ei_end;
  tie(ei, ei_end) = boost::out_edges(v, *g);
  double previous_distance = -1;
  edge_descriptor return_e;
  bool found = false;
  for (; ei != ei_end; ++ei) {
    vertex v_source = boost::source(*ei, *g);
    vertex v_target = boost::target(*ei, *g);
    if (v_target == w) {
      found = true;
      // std::cout <<  "      fr " << (*g)[*ei].segment_index
      //    <<  " to " << (*g)[*ei].segment_index2
      //    <<  " d " << (*g)[*ei].distance
      //    <<  " " << (*g)[*ei].p_x
      //    <<  ":" << (*g)[*ei].p_y
      //    <<  " " << (*g)[*ei].p_x2
      //    <<  ":" << (*g)[*ei].p_y2
      //    << std::endl;
      if ((*g)[*ei].distance < previous_distance || previous_distance == -1) {
        if ( previous_distance != -1 ) {
          // remove the previous edge
          //remove_edge(return_e, *g);
        }
        previous_distance = (*g)[*ei].distance;
        return_e = *ei;
      } else {
        // we don't need this edge
        //remove_edge(ei, *g);
      }
    }
  }
  return std::make_pair(return_e,found);
}

Segment_Visibility_Graph::vertex
Segment_Visibility_Graph::add_vertex(seg_vertex vertex_info) {
  vertex v;
  bool inserted;
  v = boost::add_vertex(vertex_info, *g);
  return v;
}

Segment_Visibility_Graph::edge_descriptor
Segment_Visibility_Graph::add_edge(vertex v1, vertex v2) {
  edge_descriptor e;
  seg_edge p;
  bool inserted;
  tie(e, inserted) = boost::add_edge(v1, v2, p, *g);
  // TODO
  // weightmap[e] = weights[j];
  return e;
}

Segment_Visibility_Graph::edge_descriptor
Segment_Visibility_Graph::add_edge(vertex v1, vertex v2, double d) {
  edge_descriptor e = this->add_edge(v1, v2);
  // set d x,y in e properties
  return e;
}

Segment_Visibility_Graph::edge_descriptor
Segment_Visibility_Graph::add_edge(vertex v1, vertex v2, double x, double y,
                                   double x2, double y2, double d) {
  seg_edge p;
  p.p_x = x;
  p.p_y = y;
  p.p_x2 = x2;
  p.p_y2 = y2;
  if (x != 0 && y != 0)
    p.point_is_set = true;
  else
    p.point_is_set = false;
  if (x2 != 0 && y2 != 0)
    p.point_is_set2 = true;
  else
    p.point_is_set2 = false;
  p.distance = d;
  p.distance_temp = d;
  p.segment_index = (*g)[v1].segment_index;
  p.segment_index2 = (*g)[v2].segment_index;
  edge_descriptor e;
  bool inserted;
  tie(e, inserted) = boost::add_edge(v1, v2, p, *g);
  return e;
}

Segment_Visibility_Graph::Segment_Visibility_Graph() { g = new mygraph_t; }

Segment_Visibility_Graph::Segment_Visibility_Graph(int n_v, int n_e,
                                                   float *edge_weights,
                                                   edge *edge_array) {
  // build the graph
  g = new mygraph_t(n_v);

  // for(std::size_t j = 0; j < n_e; ++j) {
  //  edge_descriptor e; bool inserted;
  //  tie(e, inserted) = add_edge(edge_array[j].first,
  //                              edge_array[j].second, *g);
  //  weightmap[e] = weights[j];
  //}
}
