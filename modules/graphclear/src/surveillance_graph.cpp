#include "graphclear/surveillance_graph.h"

using namespace graphclear;

void surveillance_graph_t::cut_strategy()
{
    std::deque< surveillance_graph_t::vertex_descriptor > q;
    
    // add all leaves to the queue
    surveillance_graph_t::vertex_iterator vi, vi_end;
    for (boost::tie(vi, vi_end) = vertices(*this); vi != vi_end; ++vi)
    {
        if ( out_degree(*vi,*this) == 1 ) 
            q.push_back(*vi);
    }
    std::cout << " Leaves pushed into queue " << q.size() << std::endl;
    
    surveillance_graph_t::out_edge_iterator ei, ei_end;
    
    while ( ! q.empty() )
    {
        surveillance_graph_t::vertex_descriptor v_y, v_x;
        v_y = q.front();
        q.pop_front();
        
        if ( out_degree(v_y,*this) == 1 ) {
            boost::tie(ei, ei_end) = out_edges(v_y,*this);
            v_x = this->get_other(ei,v_y);
            std::cout << " Adding cut sequences " << std::endl;
            
            get_cut_sequence(v_x,v_y)->add(v_x, (*this)[*ei].w,
                (*this)[v_x].w, *this);
            get_cut_sequence(v_x,v_y)->add(v_y, 0,
                (*this)[v_y].w, *this);
            
            (*this)[v_x].outgoing_completed++;
            if ( (*this)[v_x].outgoing_completed >= out_degree(v_x,*this)-1 )
            {
                q.push_back(v_x);
            }
                
        } else {
            int out_completed = (*this)[v_y].outgoing_completed;
            int degree = out_degree(v_y,*this);
            std::cout << " Non leaf out_c" << out_completed 
                << ":" << degree  << std::endl;
            if ( out_completed == degree-1 ) 
            {
                // we have a sufficient number of outgoing cut sequences       
                // so that we can build ONE incoming cut sequence
                // find the v_x for which to build it
                boost::tie(ei, ei_end) = out_edges(v_y,*this);
                for ( ; ei != ei_end; ++ei ) 
                {
                    v_x = this->get_other(ei,v_y);
                    if ( get_cut_sequence(v_y,v_x)->empty() == true ) {
                        // this is the right v_x
                        this->construct_full_cut_sequence(v_x,v_y);
                    }
                    if ( (*this)[v_x].outgoing_completed 
                          >= out_degree(v_x,*this)-1 ) 
                    {
                        q.push_back(v_x);          
                    }
                }
            } 
            else if ( out_completed == degree ) 
            {
                // we can build ALL incoming cut sequences
            }
        }       
    }
    
}

// bool
//     is_target(vertex_descriptor v, edge_iterator e)
// {
//     if ( v == target(e,*this) )
//         return true;
//     return false;
// }

graphclear::cut_sequence_t*
    surveillance_graph_t::get_cut_sequence( 
        vertex_descriptor from, vertex_descriptor to)
{
    edge_descriptor e = edge(from,to,*this).first;
    if ( source(e,*this) == from )
        return (*this)[ e ].cut_sequence_source_to_target;
    else
        return (*this)[ e ].cut_sequence_target_to_source;
}



/*
    builds the full cut sequence around the vertex to,
    attached to edge [from,to]
*/
void 
    surveillance_graph_t::construct_full_cut_sequence(
        vertex_descriptor from, vertex_descriptor to)
{
    graphclear::cut_sequence_t* new_c = get_cut_sequence(from,to);
    new_c->clear();
    
    edge_descriptor e = edge(from,to,*this).first;
    int b_from_to = (*this)[e].w;
    new_c->add(from, (*this)[e].w,(*this)[from].w, *this);
    new_c->add(to, 0,(*this)[to].w, *this);
    
    surveillance_graph_t::vertex_descriptor v_x;
    surveillance_graph_t::out_edge_iterator ei, ei_end;
    boost::tie(ei, ei_end) = out_edges(to,*this);
    int b = 0;
    std::vector<int> b_s;
    std::vector<edge_descriptor> e_s;
    std::vector<graphclear::cut_sequence_t*> c_s;
    for ( ; ei != ei_end; ++ei ) 
    {
        v_x = this->get_other(ei,to);
        if ( v_x != from ) {
            b += (*this)[*ei].w;
            b_s.push_back(b);
            e_s.push_back(*ei);
            graphclear::cut_sequence_t* c = get_cut_sequence(to,v_x);         
            c_s.push_back(c);
            //graphclear::cut_sequence_t::iterator it;
            //it = c->begin(); it++;
            //while ( it != c->end() ) {
            //    new_c->add_cut(*it);
            //    it++;
            //}
        }
    }
    
    b_from_to;
    (*this)[from].w + b_from_to;
    
    b + b_from_to + (*this)[to].w
    
    
    // got all cuts in new_c ordered_cuts;
    // now turn it into a full cutsequence
    new_c->make_full();
    
    
    
    (*this)[from].outgoing_completed++;
    
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

int main (int argc, char const *argv[])
{
    /* code */
    std::cout << " Testing surv graph " << std::endl;
    
    typedef boost::small_world_iterator<boost::minstd_rand, 
        surveillance_graph_t> SWGen;
    boost::minstd_rand gen;
    // Create graph with 100 nodes 
    
    surveillance_graph_t g(SWGen(gen, 100, 6, 0.03), SWGen(), 100);    
    surveillance_graph_t g_2; 
    
    std::cout << num_vertices(g) << " vertices " << std::endl;
    std::cout << num_edges(g) << " edges " << std::endl;
    
    surveillance_graph_t::vertex_iterator v_i,v_end;
    tie(v_i,v_end) = vertices(g);
    for ( ; v_i != v_end; ++v_i ) {
        g[*v_i].w = rand() % 20;
        add_vertex( g[*v_i], g_2);
        //add_vertex( get(surveillance_graph_vertex(), g, *v_i), g_2);
    }
    
    std::cout << num_vertices(g_2) << " vertices " << std::endl;
    
    surveillance_graph_t::edge_iterator e_i,e_end;
    tie(e_i,e_end) = edges(g);
    for ( ; e_i != e_end; ++e_i ) {
        g[*e_i].w = rand() % 10;
    }
    
    std::vector < surveillance_graph_t::edge_descriptor > spanning_tree;
    kruskal_minimum_spanning_tree(g, std::back_inserter(spanning_tree),
        boost::weight_map(boost::get(&surveillance_graph_edge::w, g))
        );
    for ( int i = 0 ; i < spanning_tree.size(); ++i ) {
        g[spanning_tree[i]].spanning_tree = true;
    }    
    
    tie(e_i,e_end) = edges(g);
    for ( ; e_i != e_end; ++e_i ) {
        if (g[*e_i].spanning_tree == true)
        {
            surveillance_graph_t::edge_descriptor  e = add_edge(source(*e_i,g),target(*e_i,g),g[*e_i], g_2).first;
            g_2[e].cut_sequence_source_to_target = 
                new cut_sequence_t();
            g_2[e].cut_sequence_target_to_source = 
                new cut_sequence_t();
        }
    }
    
    std::cout << num_edges(g_2) << " edges " << std::endl;
    
    g_2.cut_strategy();
        
    return 0;
}