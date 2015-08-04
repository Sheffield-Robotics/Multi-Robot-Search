#include "graphclear/surveillance_graph.h"

using namespace graphclear;

boost::mt19937 rng(std::time(0));

void surveillance_graph_t::cut_strategy()
{
  std::deque< surveillance_graph_t::vertex_descriptor > q;
    
  // add all leaves to the queue
  surveillance_graph_t::vertex_iterator vi, vi_end;
  for (boost::tie(vi, vi_end) = vertices(*this); vi != vi_end; ++vi) {
    if ( out_degree(*vi,*this) == 1 ) 
      q.push_back(*vi);
  }
  if ( graphclear::DEBUG_LVL >= 1 ) {
    std::cout << " Leaves pushed into queue " << q.size() << std::endl;
    std::cout << std::endl;
  }
    
  surveillance_graph_t::out_edge_iterator ei, ei_end;
    
  while ( ! q.empty() ) {
    surveillance_graph_t::vertex_descriptor v_y, v_x;
    v_y = q.front();
    q.pop_front();
        
    if ( out_degree(v_y,*this) == 1 ) {
      boost::tie(ei, ei_end) = out_edges(v_y,*this);
      v_x = this->get_other(ei,v_y);

      if ( !get_cut_sequence(v_x,v_y)->empty() ) 
        continue;

      get_cut_sequence(v_x,v_y)->add(v_x, (*this)[*ei].w,
                                     (*this)[v_x].w + (*this)[*ei].w, *this);
      get_cut_sequence(v_x,v_y)->add(v_y, 0,
                                     (*this)[v_y].w + (*this)[*ei].w, *this);
      if ( graphclear::DEBUG_LVL >= 2 ) {
        std::cout << "Computed cut seq from " << v_x 
                  << " to " << v_y << std::endl;
      }
      (*this)[v_x].outgoing_completed++;
      if ( (*this)[v_x].outgoing_completed >= out_degree(v_x,*this)-1 )
        {
          q.push_back(v_x);
        }
            
    } else {
      int out_completed = (*this)[v_y].outgoing_completed;
      int degree = out_degree(v_y,*this);
      if ( graphclear::DEBUG_LVL >= 2 ) {
        std::cout << " Non leaf out_completed=" << out_completed 
                  << ":degree=" << degree  << std::endl;
      }
      if ( out_completed == degree-1 ) 
        {
          // we have a sufficient number of outgoing cut sequences       
          // so that we can build ONE incoming cut sequence
          // find the v_x for which to build it
          boost::tie(ei, ei_end) = out_edges(v_y,*this);
          for ( ; ei != ei_end; ++ei ) {
            v_x = this->get_other(ei,v_y);
            if ( get_cut_sequence(v_y,v_x)->empty() == true ) {
              // this is the right v_x
              this->construct_full_cut_sequence(v_x,v_y);
              if ( (*this)[v_x].outgoing_completed 
                   >= out_degree(v_x,*this)-1 ) 
                {
                  q.push_back(v_x);          
                }
            }
          }
        } else if ( out_completed == degree ) {
        if ( graphclear::DEBUG_LVL >= 2 ) {
            std::cout << " we can build ALL incoming cut sequences";
            std::cout << std::endl;
        }
        boost::tie(ei, ei_end) = out_edges(v_y,*this);
        for ( ; ei != ei_end; ++ei ) 
          {
            v_x = this->get_other(ei,v_y);
            if ( get_cut_sequence(v_x,v_y)->empty() == true ) {
              this->construct_full_cut_sequence(v_x,v_y);
              if ( (*this)[v_x].outgoing_completed 
                   >= out_degree(v_x,*this)-1 ) 
                {
                  q.push_back(v_x);          
                }
            }
          }
      }
    }
    if ( graphclear::DEBUG_LVL >= 2 ) {
      std::cout << std::endl;       
    }
  }
    
}

graphclear::cut_sequence_t*
surveillance_graph_t::find_best_strategy()
{
    
  // go through all vertices and test them as a root
  int ag_min = INT_MAX;
  graphclear::cut_sequence_t *c, *best_c;
  surveillance_graph_t::vertex_iterator vi, vi_end;
  for (boost::tie(vi, vi_end) = vertices(*this); vi != vi_end; ++vi) {
    c = this->best_strategy_at_vertex( *vi );
    if ( graphclear::DEBUG_LVL >= 2 ) {
      std::cout << *vi << " ag=" << c->back().ag << std::endl;
    }
        
    if ( c->back().ag < ag_min ) {
      best_c = c;
      ag_min = c->back().ag;
    } else {
      delete c;
    }
  }
    
  if ( graphclear::DEBUG_LVL >= 1 ) {
    std::cout << " Best strategy " << std::endl;
    std::cout << " ag=" << best_c->back().ag << std::endl;
    std::cout << " last cut=" << best_c->back() << std::endl;    
    std::cout << *best_c << std::endl;
  }
  return best_c;
}

graphclear::cut_sequence_t*
surveillance_graph_t::best_strategy_at_vertex( vertex_descriptor v)
{
  if ( graphclear::DEBUG_LVL >= 1 ) {
    std::cout << "best_strategy_at_vertex " << v << std::endl;
  }
  graphclear::cut_sequence_t* new_c = new cut_sequence_t();
    
  surveillance_graph_t::vertex_descriptor v_x;
  surveillance_graph_t::out_edge_iterator ei, ei_end;
  boost::tie(ei, ei_end) = out_edges(v,*this);
  int b = 0;
  std::vector<int> b_s;
  std::vector<edge_descriptor> e_s;
  std::vector<graphclear::cut_sequence_t*> c_s;
  std::vector<graphclear::cut_sequence_t::iterator> c_s_its;
  if ( graphclear::DEBUG_LVL >= 2 ) {
    std::cout << " Edges " << std::endl;
  }
  for ( ; ei != ei_end; ++ei ) {
    v_x = this->get_other(ei,v);
    b += (*this)[*ei].w;
    b_s.push_back((*this)[*ei].w);
    e_s.push_back(*ei);
    graphclear::cut_sequence_t* cc = get_cut_sequence(v,v_x);         
    c_s.push_back(cc);
    c_s_its.push_back(cc->begin()); 
    graphclear::cut_sequence_t::iterator it = cc->begin();
    if ( graphclear::DEBUG_LVL >= 2 ) {
      std::cout << " Cut sequence from  " << v << " to " 
                << v_x << " " << std::endl;
      std::cout << "   ignoring " << *it  << std::endl;
    }
    it++;
    while ( it != cc->end() ) {
      it->helper_index = c_s_its.size()-1;
      if ( graphclear::DEBUG_LVL >= 3 ) {
        std::cout << *it << std::endl;
      }
      new_c->add_cut(*it);
      it++;
    }
  }
  int b1  = b;
  int ag1 = (*this)[v].w + b;
  new_c->add(v, b1,ag1, *this);
    
  if ( graphclear::DEBUG_LVL >= 2 ) {
    std::cout << " Going through ordered cuts " << std::endl;
  }
  std::set<cut_t>::iterator cut_set_it = new_c->ordered_cuts.begin();
  while ( cut_set_it !=  new_c->ordered_cuts.end() ) {        
    if ( graphclear::DEBUG_LVL >= 3 ) {
      std::cout << *cut_set_it << std::endl;
    }
    cut_t new_cut = new_c->back();
                
    // compute the cost and block of the new cut based on the old one
    int b_change = b_s[cut_set_it->helper_index] - cut_set_it->b;
    new_cut.ag = b + cut_set_it->ag - b_s[cut_set_it->helper_index];
    b -= b_change;
    new_cut.b = b;
    b_s[cut_set_it->helper_index] = cut_set_it->b;
        
    // add the cut vertices to the new cut
    cut_t::const_iterator cut_iterator = cut_set_it->begin();
    while ( cut_iterator != cut_set_it->end() ) {
      new_cut.push_back(*cut_iterator);
      cut_iterator++;
    }
        
    new_c->add_cut_unordered( new_cut );

    cut_set_it++;
  }
    
  new_c->make_full();
    
  return new_c;
}


// bool
//     is_target(vertex_descriptor v, edge_iterator e)
// {
//     if ( v == target(e,*this) )
//         return true;
//     return false;
// }

graphclear::cut_sequence_t*
surveillance_graph_t::get_cut_sequence(vertex_descriptor from, vertex_descriptor to)
{
  edge_descriptor e = edge(from,to,*this).first;
  if ( from < to )
    return (*this)[ e ].cut_sequence_source_to_target;
  else
    return (*this)[ e ].cut_sequence_target_to_source;
}



/*
  builds the full cut sequence around the vertex to,
  attached to edge [from,to]
*/
void 
surveillance_graph_t::construct_full_cut_sequence(vertex_descriptor from, vertex_descriptor to)
{
  if ( graphclear::DEBUG_LVL >= 2 ) {
    std::cout << "construct_full_cut_sequence " 
              << from << ":" << to << std::endl;
  }
  graphclear::cut_sequence_t* new_c = get_cut_sequence(from,to);
  new_c->clear();
    
  edge_descriptor e = edge(from,to,*this).first;
  int b_from_to = (*this)[e].w;
    
    
  surveillance_graph_t::vertex_descriptor v_x;
  surveillance_graph_t::out_edge_iterator ei, ei_end;
  int b = 0;
  std::vector<int> b_s;
  std::vector<edge_descriptor> e_s;
  std::vector<graphclear::cut_sequence_t*> c_s;
  std::vector<graphclear::cut_sequence_t::iterator> c_s_its;
  if ( graphclear::DEBUG_LVL >= 2 ) {
    std::cout << " going through edges" << std::endl;
  }
  boost::tie(ei, ei_end) = out_edges(to,*this);
  for ( ; ei != ei_end; ++ei ) {
    v_x = this->get_other(ei,to);
    if ( v_x != from ) {
      b += (*this)[*ei].w;
      b_s.push_back((*this)[*ei].w);
      if ( graphclear::DEBUG_LVL >= 3 ) {
        std::cout << " w=" << (*this)[*ei].w;
      }
      e_s.push_back(*ei);
      graphclear::cut_sequence_t* cc = get_cut_sequence(to,v_x);         
      c_s.push_back(cc);
      c_s_its.push_back(cc->begin()); 
      graphclear::cut_sequence_t::iterator it = cc->begin();
      it++;
      if ( graphclear::DEBUG_LVL >= 3 ) {
        std::cout << "   going through cuts " << std::endl;
      }
      while ( it != cc->end() ) {
        it->helper_index = c_s_its.size()-1;
        if ( graphclear::DEBUG_LVL >= 3 ) {
          std::cout << *it << std::endl;
        }
        new_c->add_cut(*it);
        it++;
      }
    }
  }
  if ( graphclear::DEBUG_LVL >= 1 ) {
    std::cout << " edges considered " << b_s.size() << std::endl;
    std::cout << " ordered cuts " << new_c->ordered_cuts.size() << std::endl;
  }
    
  int b1  = b_from_to;
  int ag1 = (*this)[from].w + b_from_to;
    
  // now lets go through the first cuts of all cut sequences
  // and 'activate' them (implicitly done through clearing v_y)
    
  int ag2 = (*this)[to].w + b_from_to + b;
  int b2 = b;
    
  //std::cout << " from to edge weight " << (*this)[e].w << std::endl;
  new_c->add(from, b1,ag1, *this);
  new_c->add(to, b2,ag2, *this);
    
  // now continuing along the next cut 
  // inside ordered_cuts the cuts should be ordered already
  // helper index refers to the index from the vectors
  // created when parsing through the edges  
  if ( graphclear::DEBUG_LVL >= 1 ) {
    std::cout << " Going through ordered cuts " << std::endl;
  }
  std::set<cut_t>::iterator cut_set_it = new_c->ordered_cuts.begin();
  while ( cut_set_it !=  new_c->ordered_cuts.end() ) {
        
    //if ( cut_set_it->helper_index < 0 
    //     || cut_set_it->helper_index > b_s.size())
    //    std::cout << "ERROR" << std::endl;
        
    if ( graphclear::DEBUG_LVL >= 2 ) {
      std::cout << " helper " << cut_set_it->helper_index << std::endl;
      std::cout << *cut_set_it << std::endl;
    }
        
    cut_t new_cut = new_c->back();
        
    // compute the cost and block of the new cut based on the old one
        
    int b_change = b_s[cut_set_it->helper_index] - cut_set_it->b;
    if ( graphclear::DEBUG_LVL >= 2 ) {
      std::cout << " b_change " << b_change << std::endl;
    }
    new_cut.ag = b + cut_set_it->ag - b_s[cut_set_it->helper_index];
    b -= b_change;
    new_cut.b = b;
    b_s[cut_set_it->helper_index] = cut_set_it->b;
        
    // add the cut vertices to the new cut
    cut_t::const_iterator cut_iterator = cut_set_it->begin();
    while ( cut_iterator != cut_set_it->end() ) {
      new_cut.push_back(*cut_iterator);
      cut_iterator++;
    }
        
    // and then add the finished cut 
    new_c->add_cut_unordered( new_cut );
    if ( graphclear::DEBUG_LVL >= 2 ) {
      std::cout << " new cut " << new_cut << std::endl;
    }

    cut_set_it++;
  }
    
    
  // got all cuts in new_c ordered_cuts;
  // now turn it into a full cutsequence
  new_c->make_full();
    
  if ( graphclear::DEBUG_LVL >= 2 ) {
    std::cout << "Computed cut seq from " << from 
              << " to " << to << std::endl;
  }
    
  (*this)[from].outgoing_completed++;
    
}

int surveillance_graph_t::play_through_strategy(cut_t& strategy, std::string filename)
{
  if ( graphclear::DEBUG_LVL >= 1 ) {
    std::cout << " play_through_strategy " << strategy << std::endl;
  }
    
  std::ofstream out_file;
  std::string filename1 = filename + "1";
  if ( filename != "" ) {
    out_file.open(filename1.c_str());
  }
    
  //cut is interpreted as a list of vertices
  cut_t::iterator it = strategy.begin();
  int total_max_cost = 0;
  int total_cost = 0, total_block_cost = 0, vertex_clear_cost = 0;
  bool first = true;
  int initial = 0;
  int dist = 0;
  int count = 0;
  while ( it != strategy.end() ) {
    vertex_clear_cost = (*this)[*it].w;
        
    surveillance_graph_t::out_edge_iterator ei, ei_end;
    boost::tie(ei, ei_end) = out_edges(*it,*this);
    for ( ; ei != ei_end; ++ei ) 
      {
        if ( ! (*this)[*ei].blocked ) {
          (*this)[*ei].blocked = true;
          total_block_cost += (*this)[*ei].w;
        }
      }
        
    // clear vertex
    (*this)[*it].cleared = 1;
    total_cost = total_block_cost + vertex_clear_cost;
    total_max_cost = std::max(total_cost,total_max_cost);
    if ( graphclear::DEBUG_LVL >= 2 ) {
      std::cout << " cost=" << total_cost 
                << " (b=" << total_block_cost << std::endl;
    }
        
    if ( filename != "" ) {
      surveillance_graph_t::vertex_iterator v_i, v_i_end;
      surveillance_graph_t::edge_iterator e_i, e_i_end;
      boost::tie(v_i, v_i_end) = vertices(*this);
      boost::tie(e_i, e_i_end) = edges(*this);
      for ( ; v_i != v_i_end; ++v_i ) {
        if ( *v_i == *it ) {
          out_file << (*this)[*v_i].w << ", ";
          if (first) {
            initial = count;
          }
        } else
          out_file << "0, ";
        count++;
      }
      for ( ; e_i != e_i_end;  )  {
        if ( (*this)[*e_i].blocked) {
          out_file << (*this)[*e_i].w;
          if(first) {
            if(dist < (*this)[*e_i].w) {
              dist = (int)(*this)[*e_i].w;
            }
          }
        } else
          out_file << "0";
        ++e_i;
        if ( e_i == e_i_end ) {
          out_file << std::endl;
        } else {
          out_file << ", ";
        }
      }
      first = false;
    }
        
        
    // remove all edge blocks to cleared vertices
    boost::tie(ei, ei_end) = out_edges(*it,*this);
    for ( ; ei != ei_end; ++ei ) 
      {
        if ( (*this)[this->get_other(ei,*it)].cleared ) {
          (*this)[*ei].blocked = false;
          total_block_cost -= (*this)[*ei].w;
        }
      }
    it++;
  }

  out_file << "\n1";
  out_file.close(); 
  std::string filename2 = filename + "2";
  if ( filename != "" ) {
    out_file.open(filename2.c_str());
    out_file << initial << "\n";
    out_file.close(); 
  }

  if ( graphclear::DEBUG_LVL >= 1 ) {
    std::cout << " total_max_cost " << total_max_cost << std::endl;
  }
  return total_max_cost;
}

int surveillance_graph_t::count_outgoing_sequences()
{
  return 1;
}

surveillance_graph_t::vertex_descriptor 
surveillance_graph_t::get_other( out_edge_iterator ei, vertex_descriptor v)
{
  if ( boost::source(*ei,*this) == v )
    return boost::target(*ei,*this);
  else 
    return boost::source(*ei,*this);
}

void
surveillance_graph_t::print_graph_to_txt_file(const char* filename)
{
  std::ofstream out_file;
  edge_iterator e_it, e_it_end, t_e_it, t_e_it_end;
  vertex_iterator vert_it, vert_it_end, v_it, v_it_end;

  // Prepare the file into which to write the graph
  out_file.open(filename);//formerly out_file

  std::string filename3(filename);
  filename3 += "3";
  std::ofstream out_file3;
  out_file3.open(filename3.c_str());

  int nvertices = 0;
  boost::tie(vert_it, vert_it_end) = vertices(*this);
  for (; vert_it != vert_it_end ; ++vert_it ) {
    nvertices++;
  }
  int nedges = 0;
  boost::tie( t_e_it, t_e_it_end ) = edges( *this );
  for (; t_e_it != t_e_it_end; t_e_it++) {
    nedges++;
  }

  out_file << nvertices << ", " << nedges << std::endl;
  out_file << std::endl;
  boost::tie(vert_it, vert_it_end) = vertices(*this);
  for (; vert_it != vert_it_end ; ) {
    out_file << (*this)[*vert_it].w;
    vert_it++;
    if ( vert_it == vert_it_end ) {
      out_file << std::endl;
    } else {
      out_file << ", ";
    }
  }
  out_file << std::endl;
    
  //  Edges
  boost::tie( t_e_it, t_e_it_end ) = edges( *this );
  for (; t_e_it != t_e_it_end;) {
    out_file << (*this)[*t_e_it].w;
    t_e_it++;
    if ( t_e_it == t_e_it_end ) {
      out_file << std::endl;
    } else {
      out_file << ", ";
    }
  }
  out_file << std::endl;

  //distance matrix n+m x n+m
  boost::tie(vert_it, vert_it_end) = vertices(*this);
  for (; vert_it != vert_it_end ; ++vert_it ) {
    boost::tie(v_it, v_it_end) = vertices(*this);
    for (; v_it != v_it_end ; ++v_it ) {
      out_file << this->distance_between(*vert_it,*v_it);
      out_file << ", ";
    }
    boost::tie( e_it, e_it_end) = edges( *this );
    double dist = 0;
    for (; e_it != e_it_end; ) {
      out_file << this->distance_between(*vert_it,*e_it);
      if((source(*e_it,*this) == *vert_it || target(*e_it,*this) == *vert_it) && dist < distance_between(*vert_it, *e_it))
        dist = distance_between(*vert_it, *e_it);
      e_it++;
      if ( e_it == e_it_end ) {
        out_file << std::endl;
      } else {
        out_file << ", ";
      }
    }
    out_file3 << ((int)dist) << "\n";
  }
  //out_file << std::endl;
  boost::tie( t_e_it, t_e_it_end ) = edges( *this );
  for (; t_e_it != t_e_it_end; t_e_it++) {
    boost::tie(v_it, v_it_end) = vertices(*this);
    for (; v_it != v_it_end ; ++v_it ) {
      out_file << this->distance_between(*v_it,*t_e_it);
      out_file << ", ";
    }
    boost::tie( e_it, e_it_end) = edges( *this );
    for (; e_it != e_it_end; ) {
      out_file << this->distance_between(*t_e_it,*e_it);
      e_it++;
      if ( e_it == e_it_end ) {
        out_file << std::endl;
      } else {
        out_file << ", ";
      }
    }
  }
  out_file << std::endl;
    
    
  // vertex to edge adjacency
  boost::tie(vert_it, vert_it_end) = vertices(*this);
  for (; vert_it != vert_it_end ; ++vert_it ) {
    boost::tie( e_it, e_it_end) = edges( *this );
    for (; e_it != e_it_end; ) {
      if ( source(*e_it,*this) == *vert_it 
           || target(*e_it,*this) == *vert_it) {
        out_file << "1";
      } else {
        out_file << "0";
      }
      e_it++;
      if (e_it == e_it_end )
        out_file << std::endl;
      else
        out_file << ", ";
    }
  }
  out_file << std::endl;
    
  // edge to vertex adjacency
  boost::tie( e_it, e_it_end) = edges( *this );
  for (; e_it != e_it_end; e_it++) {
    boost::tie(vert_it, vert_it_end) = vertices(*this);
    for (; vert_it != vert_it_end ;) {
      if ( source(*e_it,*this) == *vert_it 
           || target(*e_it,*this) == *vert_it) {
        out_file << "1";
      } else {
        out_file << "0";
      }
      vert_it++;
      if (vert_it == vert_it_end )
        out_file << std::endl;
      else
        out_file << ", ";
    }
  }
  out_file << std::endl;
    
  // vertex to vertex adjacency
  double dist = 0;
  boost::tie(vert_it, vert_it_end) = vertices(*this);
  for (; vert_it != vert_it_end ; ++vert_it ) 
    {
      boost::tie(v_it, v_it_end) = vertices(*this);
      for (; v_it != v_it_end ; ) {
        if ( edge(*v_it, *vert_it, *this).second ) {
          out_file << "1";
					if(dist < distance_between(*vert_it,*v_it))
						dist = distance_between(*vert_it,*v_it);
        } else {
          out_file << "0";
        }
        ++v_it;
        if (v_it == v_it_end )
          out_file << std::endl;
        else
          out_file << ", ";
      }
    }
  out_file3 << ((int)dist)*2 << "\n";
  out_file << std::endl;
    
  out_file3.close();
  out_file.close(); 
}

float surveillance_graph_t::distance_between( vertex_descriptor v, vertex_descriptor w ) {
  float distance =  pow( (*this)[v].x - (*this)[w].x, 2 ) 
    + pow( (*this)[v].y - (*this)[w].y , 2 ) ;
  distance = sqrt( distance );
  return distance;
}

float surveillance_graph_t::distance_between( vertex_descriptor v, edge_descriptor e ) {
  float distance =  pow( (*this)[v].x - (*this)[e].x, 2 ) 
    + pow( (*this)[v].y - (*this)[e].y, 2 ) ;
  distance = sqrt( distance );
  return distance;
}

float surveillance_graph_t::distance_between( edge_descriptor ee, edge_descriptor e ) {
  float distance =  pow( (*this)[ee].x - (*this)[e].x, 2 ) 
    + pow( (*this)[ee].y - (*this)[e].y , 2 ) ;
  distance = sqrt( distance );
  return distance;
}



namespace graphclear
{

  void graph_to_tree(surveillance_graph_t& g, surveillance_graph_t& tree_of_g)
  {
    tree_of_g.clear();
    std::vector < surveillance_graph_t::edge_descriptor > spanning_tree;
    kruskal_minimum_spanning_tree(g, std::back_inserter(spanning_tree),
                                  boost::weight_map(boost::get(&surveillance_graph_edge::w, g))
                                  );
    for ( int i = 0 ; i < spanning_tree.size(); ++i ) {
      g[spanning_tree[i]].spanning_tree = true;
    } 
    
    surveillance_graph_t::vertex_iterator v_i,v_end;
    boost::tie(v_i,v_end) = vertices(g);
    for ( ; v_i != v_end; ++v_i ) {
      add_vertex( g[*v_i], tree_of_g);
    }

    surveillance_graph_t::edge_iterator e_i,e_end;
    boost::tie(e_i,e_end) = edges(g);
    for ( ; e_i != e_end; ++e_i ) {
      if (g[*e_i].spanning_tree == true)
        {
          surveillance_graph_t::edge_descriptor 
            e = add_edge(source(*e_i,g),target(*e_i,g),g[*e_i], tree_of_g).first;
          tree_of_g[e].cut_sequence_source_to_target = 
            new cut_sequence_t();
          tree_of_g[e].cut_sequence_target_to_source = 
            new cut_sequence_t();
        }
    }
  }

  void cleanup_tree(surveillance_graph_t& tree_of_g) {
    //std::cout << " cleanup_tree " << std::endl;
    surveillance_graph_t::edge_iterator e_i,e_end;
    boost::tie(e_i,e_end) = edges(tree_of_g);
    for ( ; e_i != e_end; ++e_i ) {
      delete tree_of_g[*e_i].cut_sequence_source_to_target;
      delete tree_of_g[*e_i].cut_sequence_target_to_source;
    }
    tree_of_g.clear();
  }

  int roll_die(int a,int b) {
    boost::random::uniform_int_distribution<> dist(a, b);
    return dist(rng);
  }

  void gen_rand_graph(surveillance_graph_t& g, int nV, int nE, int min_v_w,int max_v_w, int min_e_w, int max_e_w)
  {
    g.clear();
    //boost::mt19937 rng(std::time(0));
    boost::generate_random_graph(g, nV, nE, rng, true, true);
    
    // make graph connected
    std::vector<int> component(nV);
    int num_con_com = boost::connected_components(g, &component[0]);
    std::cout << "Number of connected components " << num_con_com << std::endl;
    std::vector<int> vertex_of_component(num_con_com);
    for (int i = 0; i < component.size(); ++i ) {
      vertex_of_component[component[i]]  = i;
    }
    for (int i = 0; i < vertex_of_component.size()-1; ++i ) {
      add_edge(vertex_of_component[i],vertex_of_component[i+1], g);
    }
  
    int num_con_com_final = boost::connected_components(g, &component[0]);
    std::cout << "Number of final con components " << num_con_com_final << std::endl;
    
    surveillance_graph_t::vertex_iterator v_i,v_end;
    boost::tie(v_i,v_end) = vertices(g);
    for ( ; v_i != v_end; ++v_i ) {
      g[*v_i].w = roll_die(min_v_w,max_v_w);
    }    
    surveillance_graph_t::edge_iterator e_i,e_end;
    boost::tie(e_i,e_end) = edges(g);
    for ( ; e_i != e_end; ++e_i ) {
      int a = g[source(*e_i,g)].w;
      int b = g[target(*e_i,g)].w;
      int m = (a<b) ? a : b ;
      m = (m<max_e_w) ? m : max_e_w;
      g[*e_i].w = roll_die(min_e_w,m);
    }
  }

  void gen_rand_physical_graph(surveillance_graph_t& g, double grid, int nV, int min_v_w,int max_v_w, int min_e_w, int max_e_w, double connect_thres, double all_connect_d)
  {
    g.clear();
    
    std::vector< std::pair<double,int> > dummy(nV);
    std::vector< std::vector <std::pair<double,int> > > d_mat(nV,dummy);
    
    for ( int i = 0; i < nV; i++ ) {
      // generate the i-th vertex
      surveillance_graph_t::vertex_descriptor v = add_vertex(g);
      g[v].x = roll_die(0,grid);
      g[v].y = roll_die(0,grid);
      for ( int j = 0; j < i; j++ ) {
        d_mat[i][j].first = 
          sqrt(pow(g[v].x - g[j].x,2) + pow(g[v].y - g[j].y,2));
        d_mat[i][j].second = j;
      }
    }
    // for ease of use - mirror the matrix across its diagonal
    for ( int i = 0; i < nV; i++ ) {
      d_mat[i][i].first = 0;
      for ( int j = i+1; j < nV; j++ ) {
        d_mat[i][j] = d_mat[j][i];
        d_mat[i][j].second = j;
      }
    }
    

    for ( int i = 0; i < nV; i++ ) {
      // connect the 3 closest vertices
      // if all closer than 60 units
      // connect more than 3 if they are closer than 40
      std::vector<std::pair<double,int> > to_be_sorted = d_mat[i];
      std::sort(to_be_sorted.begin(), to_be_sorted.end());
      for ( int j = 1; j < nV && (j < 4 || to_be_sorted[j].first < all_connect_d); j++) {
        // connect 
        if ( to_be_sorted[j].first < connect_thres ) {
          if (!edge(i,to_be_sorted[j].second,g).second ) {
            surveillance_graph_t::edge_descriptor e;
            e = add_edge(i,to_be_sorted[j].second,g).first;
            g[e].x = (g[i].x + g[to_be_sorted[j].second].x)/2;
            g[e].y = (g[i].y + g[to_be_sorted[j].second].y)/2;    
            //std::cout << " Edge between " 
            //    << source(e,g) << ":"<< target(e,g) 
            //    << " " << g[e].x << ":" << g[e].y << " "
            //    << g[source(e,g)].x << ":" << g[source(e,g)].y << " "
            //    << g[target(e,g)].x << ":" << g[target(e,g)].y << " "
            //        << std::endl;
          }
        }
      }
    }
    
    // Oh Oh code replication (TODO: fix that)
    // make graph connected
    std::vector<int> component(nV);
    int num_con_com = boost::connected_components(g, &component[0]);
    std::cout << "Number of connected components " << num_con_com << std::endl;
    std::vector<int> vertex_of_component(num_con_com);
    for (int i = 0; i < component.size(); ++i ) {
      vertex_of_component[component[i]]  = i;
    }
    for (int i = 0; i < vertex_of_component.size()-1; ++i ) {
      surveillance_graph_t::edge_descriptor e;
      int v_1 = vertex_of_component[i];
      int v_2 = vertex_of_component[i+1];
      e = add_edge(v_1,v_2, g).first;        
      g[e].x = (g[v_1].x + g[v_2].x)/2;
      g[e].y = (g[v_1].y + g[v_2].y)/2;    
    }
  
    int num_con_com_final = boost::connected_components(g, &component[0]);
    std::cout << "Number of final con components after connecting" << num_con_com_final << std::endl;
    
    surveillance_graph_t::vertex_iterator v_i,v_end;
    boost::tie(v_i,v_end) = vertices(g);
    for ( ; v_i != v_end; ++v_i ) {
      g[*v_i].w = roll_die(min_v_w,max_v_w);
    }    
    surveillance_graph_t::edge_iterator e_i,e_end;
    boost::tie(e_i,e_end) = edges(g);
    for ( ; e_i != e_end; ++e_i ) {
      int a = g[source(*e_i,g)].w;
      int b = g[target(*e_i,g)].w;
      int m = (a<b) ? a : b ;
      m = (m<max_e_w) ? m : max_e_w;
      g[*e_i].w = roll_die(min_e_w,m);
    }
    
  }

  void write_tree_to_file(surveillance_graph_t& tree_of_g){
    std::ofstream file_stream("graphvizfile.dot", std::ios_base::out);
    boost::write_graphviz(file_stream, tree_of_g,   
                          make_vertex_writer(get(&surveillance_graph_vertex::w, tree_of_g),
                                             boost::get(&surveillance_graph_vertex::w, tree_of_g)),
                          make_label_writer(get(&surveillance_graph_edge::w, tree_of_g)));
    file_stream.close();
  }

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

}

