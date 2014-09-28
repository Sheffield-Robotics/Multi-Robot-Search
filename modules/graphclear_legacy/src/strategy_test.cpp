#include <opencv/cv.h>
#include <opencv/cvaux.h>
#include <opencv/highgui.h>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <string>

#include "graphclear/surveillance_graph.h"

//**** Config
#include <libconfig.h++>
using namespace libconfig;
#include "graphclear_legacy/sg_label_computer.h"
#include "graphclear_legacy/sg_map.h"
#include "time.h"
#include "graphclear_legacy/define.h"

#define USAGE \
  "USAGE: maps2graph_test -f <map_filename>\n" \
  "       [-f <map_filename>] : name of the img file to process \n" \
  "       [-l <lower_threshold>] : \n" \
  "       [-d <dist_minimum>] : \n" \
  "       [-r <min_clearance>] : \n" \
  "       [-s <sense_range_pix>] : \n" \
  "       [-e <min_decreasing>] : "
//e.g. ./strategy_map_test -d 60 -r 10 -c 12 -e 1
// ./strategy_map_test -f maps/ucm3.bmp -d 20 -r 8 -c 6 -e 0.4
// ./strategy_map_test -f maps/sdr_tr180.bmp -d 20 -r 8 -c 3 -e 0.2
// ./strategy_map_test -f maps/sdr_180t_2pxGB_180t.bmp -d 15 -r 3 -c 4 -e 0.2

int main(int argc, char **argv) {

  //******* Setting the Options  *******
  std::string map_img_filename ="";
  std::string cfg_filename ="";

  char c;
  while((c = getopt(argc, argv, "m:c:")) != EOF)
  {
     switch(c)
     {
        case 'm':
           map_img_filename = optarg;
           break;
        case 'c':
           cfg_filename = optarg;
           break;
        default:
        case 'h':
        case 'H':
           printf("\nOptions:\n");
           printf("--------------\n");
           printf("-m <filename> map file.\n");
           printf("-c <filename> cfg file.\n");
           printf("\n");
           exit(0);
           break;
     }
  }
  if (map_img_filename == "") {
     printf("You need to provide a map file!\n");
     exit(0);
  }
  if (cfg_filename == "") {
     printf("You need to provide a cfg file!\n");
     exit(0);
  }

  Config* robot_cfg = new Config; 
  robot_cfg->readFile(cfg_filename.c_str());
  float lower_thres;
	robot_cfg->lookupValue( "map.lower_thres", lower_thres );
	
  //******* Main *******
  DEBUG_1("Creating sg_map");
  sg_map* the_map;
  the_map = new sg_map( robot_cfg );
  the_map->process_img( map_img_filename.c_str() , double(lower_thres) );
  DEBUG_1("Processing the sg_map");
  the_map->process_map();

  //******* Strategy *******
  sg_label_computer the_strategy;

  DEBUG_1("Calling compute strategy");
  the_strategy.compute_labels(*(the_map->get_graph()),2);
  the_map->display_graph(1, 1,"output/str_graph1_m0.bmp", "output/str_graph2_m0.bmp");
  the_map->display_graph(1, 0,"output/str_graph1c_m0.bmp", "output/str_graph2c_m0.bmp");
  the_map->save_img_graph("output/final_str_graph_start.bmp");
  DEBUG_1("Find best starting vertex");
  sg_vertex_d best_vd;
  int cost;
  best_vd = the_strategy.get_best_start_vd( cost , 2, *(the_map->get_graph()) );
  	int prev_vertex_number = the_map->count_vertices();

  bool continue_loop = true, continue_loop2 = true;
  int j = 1;
  char filename1[50], filename2[50];
  while( continue_loop == true || continue_loop2 == true) {
    DEBUG_1("Calling easy_merge_leaves");
    continue_loop = the_map->easy_merge_leaves();
    continue_loop2 = the_map->easy_merge_twos();
    DEBUG_1("Calling compute strategy");
    the_strategy.compute_labels(*(the_map->get_graph()),2);
    sprintf( filename1 , "output/str_graph_a_m%d.bmp" , j);
    sprintf( filename2 , "output/str_graph_ab_m%d.bmp" , j);
    the_map->display_graph(1, 1, filename1, filename2);
    sprintf( filename1 , "output/str_graph_c_m%d.bmp" , j);
    sprintf( filename2 , "output/str_graph_cb_m%d.bmp" , j);
    the_map->display_graph(0, 0, filename1, filename2);
    sprintf( filename1 , "output/str_graph_b_m%d.bmp" , j);
    sprintf( filename2 , "output/str_graph_bb_m%d.bmp" , j);
    the_map->display_graph(0, 1, filename1, filename2);
    sprintf( filename1 , "output/final_str_graph_m%d.bmp" , j);
    the_map->save_img_graph(filename1);
    ++j;
  }
    
	DEBUG_1("Find best starting vertex");
	sg_vertex_d best_vd2;
	int cost2;
	best_vd2 = the_strategy.get_best_start_vd( cost2 , 2, *(the_map->get_graph()) );
	cout << " First best vertex is " << best_vd << " cleared at cost " << cost << endl;
	cout << " Best vertex is " << best_vd2 << " cleared at cost " << cost2 << endl;
	cout << " prev vertex number " << prev_vertex_number << " now: " << the_map->count_vertices();
	DEBUG_1("Printing graph to files and images");
	the_map->get_graph()->print_graph_to_file("output/graph_strategy_mst2.gdl", 2, 0);
	the_map->get_graph()->print_graph_to_file("output/graph_strategy2.gdl", 2, 1);
	the_map->display_graph(1,1, "output/s_graph_1final.bmp", "output/s_graph_2final.bmp");
	the_map->display_graph(1,0, "output/s_graph_1cfinal.bmp", "output/s_graph_2cfinal.bmp");
	the_map->cout_graph_summary();
    
    the_map->get_graph()->print_graph_to_file_txt("output/graph.txt");
    
    
    sg_graph* old_graph = the_map->get_graph();
    std::vector<int> old_to_new(num_vertices(*old_graph));
    graphclear::surveillance_graph_t surv_g;
    sg_vertex_it v_it,v_it_end;
    tie(v_it, v_it_end) = vertices(*old_graph);
    for ( ; v_it != v_it_end; v_it++ ) {
        if ( (*old_graph)[*v_it].alive ) {
            graphclear::surveillance_graph_t::vertex_descriptor v;
            v = add_vertex(surv_g);
            surv_g[v].w = (*old_graph)[*v_it].w;
            old_to_new[*v_it] = v;
            std::cout << "v " << v << " w=" << surv_g[v].w << std::endl;
        }
    }
    
    sg_edge_it e_it,e_it_end;
    tie(e_it, e_it_end) = edges(*old_graph);
    for ( ; e_it != e_it_end; e_it++ ) {
        if ( old_graph->edge_alive(*e_it) ) {
            graphclear::surveillance_graph_t::edge_descriptor e;
            e = add_edge( old_to_new[source(*e_it,*old_graph)], old_to_new[target(*e_it,*old_graph)], surv_g).first;
            surv_g[e].w = (*old_graph)[*e_it].w;
            std::cout << "edge " << e << " w=" << surv_g[e].w << std::endl;
        }
    }
    
	std::vector<int> component(num_vertices(surv_g));
	int num_con_com = boost::connected_components(surv_g, &component[0]);
	std::cout << "Number of connected components " << num_con_com << std::endl;
    std::cout << "Number of vertices " << num_vertices(surv_g) << std::endl;
    std::cout << "Number of edges " << num_edges(surv_g) << std::endl;
    
    
    std::cout << "Finding best strategy " << std::endl;
    graphclear::cut_sequence_t* best_c;
    graphclear::surveillance_graph_t tree_of_g; 
    graphclear::graph_to_tree(surv_g,tree_of_g);
    graphclear::write_tree_to_file(tree_of_g);
    tree_of_g.cut_strategy();
    best_c = tree_of_g.find_best_strategy();
    tree_of_g.play_through_strategy(best_c->back());
    surv_g.play_through_strategy(best_c->back(),"output/graph_strat.txt");
    
       
	return 0;
}