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
    std::cout << std::endl;
    
    surveillance_graph_t::out_edge_iterator ei, ei_end;
    
    while ( ! q.empty() )
    {
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
            std::cout << "Computed cut seq from " << v_x 
                 << " to " << v_y << std::endl;
            (*this)[v_x].outgoing_completed++;
            if ( (*this)[v_x].outgoing_completed >= out_degree(v_x,*this)-1 )
            {
                q.push_back(v_x);
            }
            
        } else {
            int out_completed = (*this)[v_y].outgoing_completed;
            int degree = out_degree(v_y,*this);
            std::cout << " Non leaf out_completed=" << out_completed 
                << ":degree=" << degree  << std::endl;
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
                        if ( (*this)[v_x].outgoing_completed 
                              >= out_degree(v_x,*this)-1 ) 
                        {
                            q.push_back(v_x);          
                        }
                    }
                }
            } 
            else if ( out_completed == degree ) 
            {
                std::cout << " we can build ALL incoming cut sequences" <<
                std::cout << std::endl;
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
        std::cout << std::endl;       
    }
    
}

graphclear::cut_sequence_t*
surveillance_graph_t::find_best_strategy()
{
    
    // go through all vertices and test them as a root
    int ag_min = INT_MAX;
    graphclear::cut_sequence_t *c, *best_c;
    surveillance_graph_t::vertex_iterator vi, vi_end;
    for (boost::tie(vi, vi_end) = vertices(*this); vi != vi_end; ++vi)
    {
        c = this->best_strategy_at_vertex( *vi );
        std::cout << *vi << " ag=" << c->back().ag << std::endl;
        if ( c->back().ag < ag_min ) {
            best_c = c;
            ag_min = c->back().ag;
        } else {
            delete c;
        }
    }
    
    std::cout << " Best strategy " << std::endl;
    std::cout << " ag=" << best_c->back().ag << std::endl;
    std::cout << " last cut=" << best_c->back() << std::endl;
    
    std::cout << *best_c << std::endl;
    return best_c;
}

graphclear::cut_sequence_t*
surveillance_graph_t::best_strategy_at_vertex( vertex_descriptor v)
{
    std::cout << "best_strategy_at_vertex " << v << std::endl;
    graphclear::cut_sequence_t* new_c = new cut_sequence_t();
    
    surveillance_graph_t::vertex_descriptor v_x;
    surveillance_graph_t::out_edge_iterator ei, ei_end;
    boost::tie(ei, ei_end) = out_edges(v,*this);
    int b = 0;
    std::vector<int> b_s;
    std::vector<edge_descriptor> e_s;
    std::vector<graphclear::cut_sequence_t*> c_s;
    std::vector<graphclear::cut_sequence_t::iterator> c_s_its;
    std::cout << " Edges " << std::endl;
    for ( ; ei != ei_end; ++ei ) 
    {
        v_x = this->get_other(ei,v);
        b += (*this)[*ei].w;
        b_s.push_back((*this)[*ei].w);
        e_s.push_back(*ei);
        graphclear::cut_sequence_t* cc = get_cut_sequence(v,v_x);         
        c_s.push_back(cc);
        c_s_its.push_back(cc->begin()); 
        graphclear::cut_sequence_t::iterator it = cc->begin();
        std::cout << " Cut sequence from  " << v << " to " 
            << v_x << " " << std::endl;
        std::cout << "   ignoring " << *it  << std::endl;
        it++;
        while ( it != cc->end() ) {
            it->helper_index = c_s_its.size()-1;
            std::cout << *it << std::endl;
            new_c->add_cut(*it);
            it++;
        }
    }
    int b1  = b;
    int ag1 = (*this)[v].w + b;
    new_c->add(v, b1,ag1, *this);
    
    std::cout << " Going through ordered cuts " << std::endl;
    std::set<cut_t>::iterator cut_set_it = new_c->ordered_cuts.begin();
    while ( cut_set_it !=  new_c->ordered_cuts.end() ) {        
        std::cout << *cut_set_it << std::endl;
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
    surveillance_graph_t::get_cut_sequence( 
        vertex_descriptor from, vertex_descriptor to)
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
    surveillance_graph_t::construct_full_cut_sequence(
        vertex_descriptor from, vertex_descriptor to)
{
    std::cout << "construct_full_cut_sequence " 
        << from << ":" << to << std::endl;
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
    std::cout << " going through edges" << std::endl;
    boost::tie(ei, ei_end) = out_edges(to,*this);
    for ( ; ei != ei_end; ++ei ) 
    {
        v_x = this->get_other(ei,to);
        if ( v_x != from ) {
            b += (*this)[*ei].w;
            b_s.push_back((*this)[*ei].w);
            std::cout << " w=" << (*this)[*ei].w;
            e_s.push_back(*ei);
            graphclear::cut_sequence_t* cc = get_cut_sequence(to,v_x);         
            c_s.push_back(cc);
            c_s_its.push_back(cc->begin()); 
            graphclear::cut_sequence_t::iterator it = cc->begin();
            it++;
            std::cout << "   going through cuts " << std::endl;
            while ( it != cc->end() ) {
                it->helper_index = c_s_its.size()-1;
                std::cout << *it << std::endl;
                new_c->add_cut(*it);
                it++;
            }
        }
    }
    std::cout << " edges considered " << b_s.size() << std::endl;
    std::cout << " ordered cuts " << new_c->ordered_cuts.size() << std::endl;
    
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
    std::cout << " Going through ordered cuts " << std::endl;
    std::set<cut_t>::iterator cut_set_it = new_c->ordered_cuts.begin();
    while ( cut_set_it !=  new_c->ordered_cuts.end() ) {
        
        //if ( cut_set_it->helper_index < 0 
        //     || cut_set_it->helper_index > b_s.size())
        //    std::cout << "ERROR" << std::endl;
        
        std::cout << " helper " << cut_set_it->helper_index << std::endl;
        std::cout << *cut_set_it << std::endl;
        
        cut_t new_cut = new_c->back();
        
        // compute the cost and block of the new cut based on the old one
        
        int b_change = b_s[cut_set_it->helper_index] - cut_set_it->b;
        std::cout << " b_change " << b_change << std::endl;
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
        std::cout << " new cut " << new_cut << std::endl;

        cut_set_it++;
    }
    
    
    // got all cuts in new_c ordered_cuts;
    // now turn it into a full cutsequence
    new_c->make_full();
    
    std::cout << "Computed cut seq from " << from 
         << " to " << to << std::endl;
    
    (*this)[from].outgoing_completed++;
    
}

void surveillance_graph_t::play_through_strategy(
    cut_t& strategy)
{
    std::cout << " play_through_strategy " << strategy << std::endl;
    //cut is interpreted as a list of vertices
    cut_t::iterator it = strategy.begin();
    int total_max_cost = 0;
    int total_cost = 0, total_block_cost = 0, vertex_clear_cost = 0;
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
        std::cout << " cost=" << total_cost 
            << " (b=" << total_block_cost << std::endl;
        
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
    std::cout << " total_max_cost " << total_max_cost << std::endl;
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
    
    surveillance_graph_t g(SWGen(gen, 20, 4, 0.03), SWGen(), 20);    
    surveillance_graph_t tree_of_g; 
    
    std::cout << num_vertices(g) << " vertices " << std::endl;
    std::cout << num_edges(g) << " edges " << std::endl;
    
    surveillance_graph_t::vertex_iterator v_i,v_end;
    tie(v_i,v_end) = vertices(g);
    for ( ; v_i != v_end; ++v_i ) {
        g[*v_i].w = rand() % 20 + 10;
        add_vertex( g[*v_i], tree_of_g);
        //add_vertex( get(surveillance_graph_vertex(), g, *v_i), tree_of_g);
    }
    
    std::cout << num_vertices(tree_of_g) << " vertices " << std::endl;
    
    surveillance_graph_t::edge_iterator e_i,e_end;
    tie(e_i,e_end) = edges(g);
    for ( ; e_i != e_end; ++e_i ) {
        g[*e_i].w = rand() % 10 + 1;
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
            surveillance_graph_t::edge_descriptor 
                e = add_edge(source(*e_i,g),target(*e_i,g),g[*e_i], tree_of_g).first;
            tree_of_g[e].cut_sequence_source_to_target = 
                new cut_sequence_t();
            tree_of_g[e].cut_sequence_target_to_source = 
                new cut_sequence_t();
        }
    }
    
    std::cout << num_edges(tree_of_g) << " edges " << std::endl;
    
    tree_of_g.cut_strategy();
    std::ofstream file_stream("graphvizfile.dot", std::ios_base::out);
    boost::write_graphviz(file_stream, tree_of_g,   
        make_vertex_writer(get(&surveillance_graph_vertex::w, tree_of_g),
            boost::get(&surveillance_graph_vertex::w, tree_of_g)),
        make_label_writer(get(&surveillance_graph_edge::w, tree_of_g)));
    file_stream.close();
    graphclear::cut_sequence_t* best_c = tree_of_g.find_best_strategy();
    tree_of_g.play_through_strategy(best_c->back());
    g.play_through_strategy(best_c->back());
    return 0;
}