#include "graphclear/surveillance_graph.h"
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
using namespace graphclear;

int main (int argc, char **argv)
{
  /* Options for this little utility */
  int n_graphs_generated = 1;
  int n_vertices = 20;
  int min_v_w = 2 ;
  int max_v_w = 10; 
  int min_e_w = 1 ;
  int max_e_w = 4 ;
  double connect_thres = 200;
  double all_connect_d = 100;
  char c;
  //./graphclear_exe -n 10 -v 20 -q 2 -w 10 -e 1 -r 4 -t 200 -y 100
  while((c = getopt(argc, argv, "n:v:q:w:e:r:t:y:")) != EOF) {
      switch(c) {
      case 'n':
        n_graphs_generated = atoi(optarg);
        break;
      case 'v':
        n_vertices = atoi(optarg);
        break;
      case 'q':
        min_v_w = atoi(optarg);
        break;
      case 'w':
        max_v_w = atoi(optarg);
        break;
      case 'e':
        min_e_w = atoi(optarg);
        break;
      case 'r':
        max_e_w = atoi(optarg);
        break;
      case 't':
        std::cout << "connect_thres=" << std::endl;
        connect_thres = double(atof(optarg));
        std::cout << connect_thres << std::endl;
        break;
      case 'y':
        std::cout << "all_connect_d=" << std::endl;
        all_connect_d = double(atof(optarg));
        std::cout << all_connect_d << std::endl;
        break;
      default:
      case 'h':
      case 'H':
        printf("\nOptions:\n");
        printf("--------------\n");
        printf("Oh Oh evil me.\n");
        printf("\n");
        exit(0);
        break;
      }
    }
    
    
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
    
  char filename[200], filename1[200], filename2[200], filename3[200], filename4[200];
  for ( int i = 0; i < n_graphs_generated; i++ ) {
    graphclear::gen_rand_physical_graph(ran_graph, 
                                        n_vertices, 
                                        min_v_w,max_v_w, min_e_w,max_e_w, 
                                        connect_thres, all_connect_d);
    std::cout<< "Random Graph has " ;
    std::cout<< " " << num_vertices(ran_graph) << " vertices" ;
    std::cout<< " " << num_edges(ran_graph) << " edges" << std::endl;
            
    graph_to_tree(ran_graph,tree_of_g);
        
    write_tree_to_file(tree_of_g);
        
    sprintf( filename, "output/.ran_graph_%d_.input",i);
    ran_graph.print_graph_to_txt_file(filename);
        
    tree_of_g.cut_strategy();
    best_c = tree_of_g.find_best_strategy();
    tree_of_g.play_through_strategy(best_c->back());
    sprintf( filename1, "output/.ran_graph_%d_.input1",i);
    sprintf( filename2, "output/.ran_graph_%d_.input2",i);
    sprintf( filename4, "output/.ran_graph_%d_.input3",i);
    int graph_cost = ran_graph.play_through_strategy(best_c->back(),filename);
    std::cout << "Total cost on graph:" << graph_cost << std::endl;
    cleanup_tree(tree_of_g);

    sprintf( filename3, "output/ran_graph_%d_.input",i);
    std::ofstream out_file;
    out_file.open(filename3);
    out_file << graph_cost << ", ";
    std::ifstream in_file, in_file3;
    in_file.open(filename);
    in_file3.open(filename4);
    std::string x;
    std::getline(in_file, x);
    out_file << x << ", "; 
    std::ifstream in_file1;
    in_file1.open(filename1);
    std::ifstream in_file2;
    in_file2.open(filename2);
    std::getline(in_file2, x);
    out_file << x << ", "; 
    int k = std::stoi(x);
    for(int t=0; t<num_vertices(ran_graph); t++) {
      std::getline(in_file3, x);
			if(t==k)
				out_file << x << ", ";
    }    
    std::getline(in_file3, x);
    out_file << x << "\n";		
    while(std::getline(in_file, x)) {
      out_file << x << "\n";
    }
    while(std::getline(in_file1, x)) {
      out_file << x << "\n";
    }     
      
    in_file.close();
    in_file1.close();
    in_file2.close();
    in_file3.close();
    out_file.close();
    //std::remove(filename);
    //std::remove(filename1);
    //std::remove(filename2);
    //std::remove(filename4);
  }
    
    
  //template<class Property, class G, class RandomGenerator>
  //    void randomize_property(G& g, RandomGenerator& rg);
    
  return 0;
}
