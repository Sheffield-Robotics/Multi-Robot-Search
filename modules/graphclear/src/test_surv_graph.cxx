#include "graphclear/surveillance_graph.h"

using namespace graphclear;

int main (int argc, char const *argv[])
{
    /* code */
    std::cout << " Testing surv graph " << std::endl;
    typedef boost::small_world_iterator<boost::minstd_rand, 
        surveillance_graph_t> SWGen;
    boost::minstd_rand gen;
    
    // Create graph 
    surveillance_graph_t g(SWGen(gen, 20, 4, 0.03), SWGen(), 20);    
    std::cout << num_vertices(g) << " vertices " << std::endl;
    std::cout << num_edges(g) << " edges " << std::endl;
    surveillance_graph_t::vertex_iterator v_i,v_end;
    tie(v_i,v_end) = vertices(g);
    for ( ; v_i != v_end; ++v_i ) {
        g[*v_i].w = rand() % 20 + 10;
    }    
    surveillance_graph_t::edge_iterator e_i,e_end;
    tie(e_i,e_end) = edges(g);
    for ( ; e_i != e_end; ++e_i ) {
        g[*e_i].w = rand() % 10 + 1;
    }
    
    graphclear::cut_sequence_t* best_c;
        
    surveillance_graph_t tree_of_g; 
    graph_to_tree(g,tree_of_g);
    tree_of_g.cut_strategy();
    best_c = tree_of_g.find_best_strategy();
    tree_of_g.play_through_strategy(best_c->back());
    g.play_through_strategy(best_c->back());
    
    
    //astar_search
    //  (VertexListGraph &g,
    //   typename graph_traits<VertexListGraph>::vertex_descriptor s,
    //   AStarHeuristic h, const bgl_named_params<P, T, R>& params);
    
    surveillance_graph_t ran_graph;
    for ( int i = 0; i < 2; i++ ) {
        graphclear::gen_rand_graph(ran_graph, 10,20, 5,20, 1,10);
        std::cout<< "Graph has " ;
        std::cout<< " " << num_vertices(ran_graph) << " vertices" ;
        std::cout<< " " << num_edges(ran_graph) << " edges" << std::endl;
            
        graph_to_tree(ran_graph,tree_of_g);
        
        write_tree_to_file(tree_of_g);
        
        tree_of_g.cut_strategy();
        best_c = tree_of_g.find_best_strategy();
        tree_of_g.play_through_strategy(best_c->back());
        ran_graph.play_through_strategy(best_c->back());
        
        cleanup_tree(tree_of_g);
    }
    
    char filename[200];
    for ( int i = 0; i < 2; i++ ) {
        graphclear::gen_rand_physical_graph(ran_graph, 50, 5,20, 1,10, 200,100);
        std::cout<< "Graph has " ;
        std::cout<< " " << num_vertices(ran_graph) << " vertices" ;
        std::cout<< " " << num_edges(ran_graph) << " edges" << std::endl;
            
        graph_to_tree(ran_graph,tree_of_g);
        
        write_tree_to_file(tree_of_g);
        
        sprintf( filename, "output/ran_graph_%d_.txt",i);
        ran_graph.print_graph_to_txt_file(filename);
        
        tree_of_g.cut_strategy();
        best_c = tree_of_g.find_best_strategy();
        tree_of_g.play_through_strategy(best_c->back());
        ran_graph.play_through_strategy(best_c->back(),filename);
        
        cleanup_tree(tree_of_g);
    }
    
    
//template<class Property, class G, class RandomGenerator>
//    void randomize_property(G& g, RandomGenerator& rg);
    
    return 0;
}