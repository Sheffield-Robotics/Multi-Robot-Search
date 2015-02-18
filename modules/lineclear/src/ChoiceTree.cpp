#include "lineclear/ChoiceTree.h"
#include "heightmap/perimeter.h"
#include "utilities/math/bresenham.h"
#include "utilities/Yaml_Config.h"

namespace lineclear {
    using namespace Yaml_Config;
    
ChoiceTree::ChoiceTree(Environment* e) {
    _e = e;
    _n = _e->get_obstacle_number();
    _sg = NULL;
    _zero_choiceset = new ChoiceSet();
    start_step_for_split_obstacle.resize(_n+1);
}

ChoiceTree::ChoiceTree(Environment* e, Visibility* v, polygonization::Polygon_Environment* pe) {
    _pol_env = pe;
    _e = e;
    _v = v;
    _sg = NULL;
    _n = _e->get_obstacle_number();
    _zero_choiceset = new ChoiceSet();
    start_step_for_split_obstacle.resize(_n+1);
}

ChoiceTree::ChoiceTree(Environment* e, Visibility* v) {
    
}

void ChoiceTree::init_choice_tree() {
    _sg = NULL;
    start_step_for_split_obstacle.resize(_n+1);
    // TODO: use proximity queries to only query costs that are close
    std::cout << " init_choice_tree with obstacles _n=" << _n << std::endl;
    _choiceSetMatrix = new ChoiceSet**[_n];
    _cost_updated = new bool**[_n];
    double range = yaml_param["range_for_cost"].as<float>();
    int use_poly_costs = yaml_param["use_poly_environment_costs"].as<int>();
    for (int k = 1; k < _n; ++k ) {
        _choiceSetMatrix[k] = new ChoiceSet*[_n+1];
        _cost_updated[k] = new bool*[_n+1];
        for (int i = 1; i <= _n; ++i ) {
            std::cout << "   k=" << k << "   i=" << i << std::endl;
            _cost_updated[k][i] = new bool[k+1];
            _choiceSetMatrix[k][i] = new ChoiceSet(_n,i,k);
            int b;
            if ( Params::g_compute_UAV_cost_everywhere ) {
                this->update_costs(i,k);
            } else if ( use_poly_costs ) {
                if ( _pol_env->is_necessary_block(i-1,i+k) ) {
                    b = _pol_env->get_block_cost(i-1,i+k,range);
                } else  {
                    b = -1;
                }
            } else {
                b = _e->get_shortest_line_inside_cost(i-1,i+k);
            }
            _choiceSetMatrix[k][i]->set_b(b);
            M_INFO1_D(DEBUG_CHOICETREE,2,"Block cost %d \n ",b);
            if ( b == -1 ) {
                _choiceSetMatrix[k][i]->set_all_c(-1);
            }
            else {
                M_INFO1_D(DEBUG_CHOICETREE,2,"Computing split costs \n ");
                for ( int j = 1; j <= k; j++ ) {
                    _cost_updated[k][i][j] = false;
                    int o = _choiceSetMatrix[k][i]->get_obstacle_for_choice(j);
                    if ( Params::g_compute_UAV_cost_everywhere ) {
                        this->update_costs(i, k, j);
                    } else if ( use_poly_costs ) {
                        int c = -1;
                        if ( _pol_env->is_necessary_split(i-1, i+k, o) ) {
                            std::cout << "   k=" << k << "   i=" << i 
                                << "  j=" << j << std::endl;
                            c = _pol_env->get_split_cost(i-1, i+k, o, range);  
                        }
                        M_INFO2_D(DEBUG_CHOICETREE,3,"%d ",c);
                        _choiceSetMatrix[k][i]->set_c_at(j,c);
                    } else {
                        int c;
                        c = _e->get_shortest_extension_cost(i-1, i+k, o);  
                        _choiceSetMatrix[k][i]->set_c_at(j,c);
                    }
                }
            }
        }
    }
    std::cout << "Finished cost assignment --- Processing... " << std::endl;
    this->process_all_sets();
}

/*
 * TODO @ALEX: This function updates the costs of selected 
 * choice sets. Choice sets are already allocated in init_choice_tree
 * or via a load and can be accessed via _choiceSetMatrix[k][i]
 * or get_choice_set_at(i,k) for a choice set T_k^i. Here k
 * is the number of choices in T_k^i and i, i+1, ..., i+k-1 the choices of 
 * obstacle indices that are possible. 
 *  
 * 
 *
 * Author: Andreas Kolling ( Thu Sep 13 04:35:53 CEST 2012 ) 
 */

int ChoiceTree::update_costs() {
    // pick the best strategies and re-evaluate their costs 
    
    int o;
    CutSequence* cutseq = this->get_optimal_cut_sequence(o);
    int new_cost = this->update_costs( cutseq,o );

    std::cout << "updated UAV costs to " << new_cost << std::endl;
    return new_cost;
}

void ChoiceTree::update_costs(int i, int k) {
    int ii = i-1;
    int kk = i+k;
    int b;
    _e->fix_index(ii);
    _e->fix_index(kk);
    if ( _e->are_adjacent(ii,kk) ) {  
        b = 0;  
    }
    else {
        Segment shortL = _e->get_shortest_line_inside(ii,kk);
        if (shortL.squared_length() > 0 ) {
            b = this->computeElevationMapBorderCoverageCostOften( shortL);
        } else {
            b = -1;
        }
    }
    _choiceSetMatrix[k][i]->set_b(b);
    //std::cout << " Updated blocking cost to b=" << b << std::endl;
}

void ChoiceTree::update_costs(int i, int k, int j) {
    
    //c = _e->get_shortest_extension_cost(i-1, i+k, o);  
    
    //std::cout << " Updating costs " << i <<":"<<k<<" j=" << j <<std::endl;
    //_e->fix_index(i);
    int o = _choiceSetMatrix[k][i]->get_obstacle_for_choice(j);
    Segment l1,l2;
    int c = _e->get_shortest_extension(i-1, i+k, o, l1, l2);
    if ( c !=-1 ) {
        c = 0;
        if ( l1.squared_length() > 0 )
            c += this->computeElevationMapBorderCoverageCost(l1);
        if ( l2.squared_length() > 0 )
            c+= this->computeElevationMapBorderCoverageCost(l2);
    }
    _choiceSetMatrix[k][i]->set_c_at(j,c);
}

int ChoiceTree::update_costs(CutSequence* cut_seq, int first_o) {
    
    M_INFO1(" Parsing through cut sequence to update cost\n");
    M_INFO1(" Current cost: %d \n",cut_seq->get_final_cost());
    M_INFO1(" First o: %d \n",first_o);
    double new_sensing_diameter = 2 * _v->get_max_steps();
    std::list<int> o_sequence = cut_seq->get_obstacle_sequence();
    o_sequence.push_front(first_o);
    if ( o_sequence.size() == 0 ) { return 0; }
    
    std::list<int>::iterator o_it = o_sequence.begin();     
    std::list<int>::iterator i;
    std::list<int> o_cleared;
    std::map< std::pair<int,int>, int> blocking_lines_map;
    std::map< std::pair<int,int>, int> blocking_lines_parent;
    std::map< std::pair<int,int>, int> blocking_lines_parent_left;
    int current_blocking_cost = 0;
    int current_cost = 0;
    int max_cost = -1;
    //int ext_cost = 0;
    int _sg_index = 0;
    if ( _sg == NULL ) {
        _sg = new SurveillanceGraph( o_sequence.size() );
    }
    // add the first two obstacles free of charge ;-)
    int first_obs;
    o_cleared.push_back( *o_it );
    _sg_index = *o_it;
    first_obs = *o_it;
    _sg->nodes[_sg_index].obstacle_index = _sg_index;
    _sg->nodes[_sg_index]._n_children = 1;
    _sg->root = &(_sg->nodes[_sg_index]);
    o_it++;
    o_cleared.push_back( *o_it );
    _sg_index = *o_it;
    _sg->nodes[_sg_index].obstacle_index = _sg_index;
    _sg->root->children[0] = & ( _sg->nodes[_sg_index] );
    o_it++;
    o_cleared.sort();
    
    std::pair<int,int> block_between_r1(_sg_index,first_obs);
    std::pair<int,int> block_between_l1(first_o,_sg_index);
    blocking_lines_parent[block_between_r1] = _sg_index;
    blocking_lines_parent_left[block_between_r1] = 0;
    //blocking_lines_parent[block_between_l] = _sg_index;
    //blocking_lines_parent_left[block_between_l] = 1;
    
    // int i, int k
    //for ( int j = 1; j <= k; j++ ) {
    //cs_l = this->get_choice_set_at(i  , j-1);
    //cs_r = this->get_choice_set_at(i+j, k-j);
    
    while ( o_it != o_sequence.end() ) {
        int new_o = *o_it;
        
        if ( DEBUG_TRAJECTORY_CREATION >= 1 ) 
            M_INFO1(" new_o: %d \n",new_o);
        
        int left_o = 0, right_o = 0;
        i = o_cleared.begin();
        while ( new_o > *i && i != o_cleared.end() ) { 
            left_o = *i;
            i++;
        }
        bool there_is_no_smaller = false;
        if ( left_o == 0 ) {
            there_is_no_smaller = true;
            left_o = o_cleared.back();
        } 
        bool there_is_no_larger = false;
        if ( i == o_cleared.end() ) {
            there_is_no_larger = true;
            right_o = o_cleared.front();
        } else {
            right_o = *i;
        }
        
        if ( DEBUG_TRAJECTORY_CREATION >= 1 ) {
            M_INFO2(" get_shortest_extension: ");
            M_INFO2("left %d right %d ",left_o,right_o);
            M_INFO2("new %d \n", new_o);
        }
        
        std::pair<int,int> block_between_r(new_o,right_o);
        std::pair<int,int> block_between_l(left_o,new_o);
        blocking_lines_parent[block_between_r] = new_o;
        blocking_lines_parent_left[block_between_r] = 0;
        blocking_lines_parent[block_between_l] = new_o;
        blocking_lines_parent_left[block_between_l] = 1;
        
        std::pair<int,int> block_between_start(left_o,right_o);
        std::map<std::pair<int,int>, int>::iterator map_it,map_it2;
        map_it = blocking_lines_parent.find( block_between_start );
        map_it2 = blocking_lines_parent_left.find( block_between_start );
        if ( map_it != blocking_lines_parent.end() ) {
            _sg_index = new_o;
            _sg->nodes[_sg_index].obstacle_index = _sg_index;
            int parent_id = map_it->second;
            if ( DEBUG_TRAJECTORY_CREATION >= 1 ) {
                M_INFO2("pare %d of %d left? %d",parent_id, _sg_index, map_it2->second);
            }
            if ( map_it2->second == 0 ) { // we were on the left
                _sg->nodes[parent_id].children[0] = & ( _sg->nodes[_sg_index] );
            }
            else {
                _sg->nodes[parent_id].children[1] = & ( _sg->nodes[_sg_index] );
            }
        }
        
        int b_l_vissample = 0, b_r_vissample = 0;
        Segment l1,l2,l3,l4;
        
        start_step_for_split_obstacle[new_o] = all_lines_in_time.size();
        if ( new_o == 0) {
            M_INFO1(" new_o is zero. that's odd  \n");
        }
        
        int ext_cost = _e->get_shortest_extension(left_o,
            right_o, new_o,l1,l2);
        if ( DEBUG_TRAJECTORY_CREATION >= 1 ) {
            M_INFO1(" 2d extension cost %d \n",ext_cost);
        }
        int old_ext_cost = ceil ( ext_cost / new_sensing_diameter );
        ext_cost = 0;
        if ( l1.target() != l1.source() 
          && Params::g_visibility_line_sampling_type == 2 ) {
            ext_cost += computeElevationMapBorderCoverageCostOften(l1);
        }
        if ( l2.target() != l2.source() 
          && Params::g_visibility_line_sampling_type == 2 ) {
            ext_cost += computeElevationMapBorderCoverageCostOften(l2);
        }
        if ( Params::g_visibility_line_sampling_type == 1 ) {
            ext_cost = sampled_cost_split(l1,l2,left_o,right_o, new_o, b_l_vissample, b_r_vissample);
        }
        
        if ( old_ext_cost > ext_cost ) {
            M_INFO1(" OH OH visibility is lower cost %d > %d\n",old_ext_cost,ext_cost);
            ext_cost = old_ext_cost;
        } else if (old_ext_cost < ext_cost) {
            M_INFO1(" Uh uh visibility is larger cost %d < %d\n",old_ext_cost,ext_cost);
        }
        // NOTE: IGNORE 2.5 VISIBILITY - TODO : remove this later
        ext_cost = old_ext_cost;
        _sg->nodes[_sg_index].split_cost = old_ext_cost;
        
        if ( DEBUG_TRAJECTORY_CREATION >= 1 ) {
            M_INFO1(" 2.5d updated extension cost %d \n",ext_cost);
        }
        
        // this b_removed will be overwritten later
        int b_removed = _e->get_shortest_line_inside_cost(
            left_o, right_o);
        if ( b_removed != 0 && b_removed != -1 ) {
            M_INFO2(" removing old block %d -- %d \n ",left_o,right_o);
            std::pair<int,int> b_p(left_o, right_o);
            b_removed = blocking_lines_map[b_p];
            blocking_lines_map.erase(b_p);
            _sg->nodes[_sg_index].incoming_blocking_cost = b_removed;
            if ( b_removed != -1 )
                current_blocking_cost -= b_removed ;
        }
        
        current_cost = ext_cost + current_blocking_cost;
        
        if ( current_cost > max_cost ) { max_cost = current_cost; }
        
        if ( DEBUG_TRAJECTORY_CREATION >= 1 ) {
            std::cout << " COST= " << current_cost << std::endl;
            std::cout << " ext_cost= " << ext_cost << std::endl;
            std::cout << " cur_block_c= " << current_blocking_cost << std::endl;
            std::cout << " max_cost= " << max_cost << std::endl;
            std::cout << " just removed block= " << b_removed << std::endl;
        }
        
        
        // compute new blocking costs and add them
        l3 = _e->get_shortest_line_inside(new_o,right_o);
        if ( l3.target() != l3.source() ) {
            
            M_INFO1(" 2.5d right_o %d \n",right_o);
            int b_r = 0;
            if ( Params::g_visibility_line_sampling_type == 2 ) {
                b_r = computeElevationMapBorderCoverageCostOften(l3);
            }
            else if ( Params::g_visibility_line_sampling_type == 1 ) {
                b_r = b_r_vissample;
            }
            M_INFO1(" 2.5d right_o block cost %d \n",b_r);
            std::pair<int,int> block_between(new_o,right_o);
            blocking_lines_map[block_between] = b_r;
            
            if ( b_r > 0 ) {
                current_blocking_cost += b_r;
            }
        }
        l4 = _e->get_shortest_line_inside(left_o,new_o);
        if ( l4.target() != l4.source() ) {
            std::pair<int,int> block_between(left_o,new_o);
            M_INFO1(" 2.5d left_o %d \n",left_o);
            int b_l = 0;
            if ( Params::g_visibility_line_sampling_type == 2 ) {
                b_l = computeElevationMapBorderCoverageCostOften(l4);
            }
            else if ( Params::g_visibility_line_sampling_type == 1 ) {
                b_l = b_l_vissample;
            }
            blocking_lines_map[block_between] = b_l;
            
            M_INFO1(" 2.5d left_o block cost %d \n",b_l);
            if ( b_l > 0 ) {
                current_blocking_cost += b_l;
            }
        }
        
        o_cleared.push_back(new_o);
        o_cleared.sort();
        o_it++;
        M_INFO3("\n\n *** NEXT OBSTACLE*** \n\n\n");
    }
    M_INFO3("\n *** New updated cost is %d.*** \n\n\n",max_cost);
    return max_cost;
    
}


int ChoiceTree::computeElevationMapBorderCoverageCost(Segment& l1) {
    return this->computeElevationMapBorderCoverageCost( 
        floor( CGAL::to_double( l1.source().x())), 
        floor( CGAL::to_double( l1.source().y())),
        floor( CGAL::to_double( l1.target().x())), 
        floor( CGAL::to_double( l1.target().y())) );
} 

//int ChoiceTree::sample_random_bias

int ChoiceTree::computeElevationMapBorderCoverageCostOften(Segment& l1)
{
    //std::cout << " computeElevationMapBorderCoverageCostOften " << std::endl;
    int best = -1;
    int current;
    for( int count=0;count< Params::g_resample_UAV_times;count++) {
        current = this->computeElevationMapBorderCoverageCost(l1);
        if ( current < best || best == -1 ) {
            best = current;
        }
    }
    //std::cout << " computeElevationMapBorderCoverageCostOften DONE"
    //    << std::endl;
    return best;
}

int ChoiceTree::computeElevationMapBorderCoverageCost(int x1, int y1, int x2, int y2) 
{
    //std::cout << " computeElevationMapBorderCoverageCost " 
    //    << x1 << ":" << y1 << " " << x2 << ":" << y2
    //    << std::endl;
	HeightMap* _map = _v->getHeightMap();
	deque<Perimeter::Cell> cells;
	Perimeter peri(_map, _v, 0,0,0);	

	// Determine cells on line
	bresenham_param_t line;
    get_bresenham_parameters(x1, y1, x2, y2, &line);
    while (get_next_point(&line)) {
		int x,y;get_current_point(&line, &x, &y);
	    if (!peri.verify(x,y))
			continue;
		if ( _map->getCellsMM()[x][y].getClass() 
              == HeightCell::ELC_FLAT_GROUND 
           ) {
            cells.push_back(Perimeter::Cell(x,y));
            _map->getCellsMM()[x][y].togglePerimeterOn();
        }
	}

	// Compute number of needed agents to cover
	peri.computePerimeterRegions(cells);	
	return peri.getNumVertices();
}

int ChoiceTree::random_sampled_cost(int x1, int y1, int x2, int y2, Pos_list& poses) {
    HeightMap* _map = _v->getHeightMap();
    deque<Perimeter::Cell> cells;
    Perimeter peri(_map, _v, 0,0,0);	
    
    // Determine cells on line
    bresenham_param_t line;
    get_bresenham_parameters(x1, y1, x2, y2, &line);
    while (get_next_point(&line)) {
        int x,y;get_current_point(&line, &x, &y);
        if (!peri.verify(x,y))
            continue;
        if ( _map->getCellsMM()[x][y].getClass() 
              == HeightCell::ELC_FLAT_GROUND 
           ) {
            cells.push_back(Perimeter::Cell(x,y));
            _map->getCellsMM()[x][y].togglePerimeterOn();
        }
    }
    
    // Compute number of needed agents to cover
    peri.computePerimeterRegions(cells);	
    return peri.getNumVertices();
}


int ChoiceTree::random_cost_bias_sampled_cost(int x1, int y1, int x2, int y2, vector<NavPoint> current_poses, Pos_list& poses) 
{
    HeightMap* _map = _v->getHeightMap();
	deque<Perimeter::Cell> cells;
	vector<double> cells_probability;
	Perimeter peri(_map, _v, 0,0,0);	

	bresenham_param_t line;
    get_bresenham_parameters(x1, y1, x2, y2, &line);
    while (get_next_point(&line)) {
		int x,y;get_current_point(&line, &x, &y);
	    if (!peri.verify(x,y))
			continue;
		if ( _map->getCellsMM()[x][y].getClass() 
              == HeightCell::ELC_FLAT_GROUND 
           ) {
            // COMPUTE A COST FUNCTION - probability of selection
            // [x][y]
            double height = _map->getCellsMM()[x][y].getHeight();
            NavPoint sample_point(double(x), double(y), height, 0);
            double prob = 1;
            double min_cost = HUGE_VAL;
            int up_to_i = int(current_poses.size());
            for ( int i = 0; i < up_to_i; i++ ) {
                double c = sample_point.getCostToPoint( current_poses[i]);
                if ( c < min_cost ) 
                    min_cost = c;
            }
            if ( min_cost == 0 ) {
                min_cost = 0.00001; // HACK: TODO: CAREFUL
            }
            prob = 1 / min_cost;
            cells.push_back(Perimeter::Cell(x,y));
            cells_probability.push_back( prob );
            _map->getCellsMM()[x][y].togglePerimeterOn();
        }
	}

	// Compute number of needed agents to cover
	peri.computePerimeterRegionsBIAS(cells, cells_probability, poses);
	return peri.getNumVertices();
}

int ChoiceTree::sampled_cost_split( Segment& to_line1, Segment& to_line2, int left_o, int right_o, int new_o, int& b_l, int& b_r ) {
    
    lineclear::Segment l_block = _e->get_shortest_line_inside(left_o,right_o);
    bool move_to_triangle_base = true;
    if ( l_block.squared_length() == 0 ) {
        move_to_triangle_base = false;
    }
    // move on segments left_o,right_o to new_o
    
    // find out on which segment we will move faster
    Point dummy = l_block.source();
    Point left_block_point, right_block_point;
    if ( move_to_triangle_base ) {
        if ( _e->is_on_line(left_o,right_o,dummy) == left_o ) {
            left_block_point = l_block.source();
            right_block_point = l_block.target();
        } else {
            left_block_point = l_block.target();
            right_block_point = l_block.source();
        }
    }
    
    dummy = to_line1.target();
    Point left_split_point, right_split_point;
    if ( _e->is_on_line(left_o,right_o,dummy) == left_o ) {
        left_split_point = to_line1.target();
        right_split_point = to_line2.target();
    } else {
        left_split_point = to_line2.target();
        right_split_point = to_line1.target();
    }

    int max_c = 0;
    if ( DEBUG_TRAJECTORY_CREATION > 2 ) {
        M_INFO1("1 1 1 ***** STEP 1 TRIANGLE BASE.  \n");
        std::cout << " move_to_triangle_base? " << move_to_triangle_base;
        std::cout << " left_split_point " << left_split_point;
        std::cout << " right_split_point " << right_split_point << std::endl;
    }
    Segment triangle_base(left_split_point,right_split_point);
    if ( move_to_triangle_base ) { 
        int last_c;
        int c = move_between_two_lines( left_block_point, right_block_point,
            left_split_point, right_split_point, left_o, right_o, last_c);
        if ( c > max_c ) {
            max_c = c;
        }
    }
    
    // Step 2 - move triangle base to split point    
    if ( to_line1.source() != to_line2.source() ) {
        M_ERR("ARGGGH: source of both split lines is not identical\n");
    }
    
    Point split_point = to_line1.source();
    Point base_point;
    if ( move_to_triangle_base ) {
        base_point = _e->get_closest_point(split_point, triangle_base);
        if ( DEBUG_TRAJECTORY_CREATION > 2 ) {
            M_INFO3("\t base_point = closest point to split on triangle \n");            
        }
    } else {
        base_point = triangle_base.source(); // target==source on base
        if ( DEBUG_TRAJECTORY_CREATION > 2 ) {
            M_INFO3("\t base_point = closest point to split on triangle \n");            
        }
    }
    
    if ( DEBUG_TRAJECTORY_CREATION > 4 ) {
        std::cout << "split_point " << split_point << std::endl;
        std::cout << "left_split_point " << left_split_point << std::endl;
        std::cout << "right_split_point " << right_split_point << std::endl;
    }
    
    bool left_no_block = false;
    bool right_no_block = false;
    if ( split_point == left_split_point ) {
        left_no_block = true;
        if ( DEBUG_TRAJECTORY_CREATION > 3 ) 
            M_INFO2("\t new split point equal to left split point  \n");            
    }
    if ( split_point == right_split_point ) {
        right_no_block = true;
        if ( DEBUG_TRAJECTORY_CREATION > 3 ) 
            M_INFO2("\t new split point equal to left split point  \n");            
    }
    
    M_INFO2("2 2 2 ***** STEP 2 : Moving to split point  \n");
    if ( !left_no_block && !right_no_block ) {
        Vector move_base(base_point, split_point );
        int steps = ceil( 
            sqrt(CGAL::to_double(squared_distance(base_point, split_point))) 
                / Params::g_trajectory_discretization );
        int start_recording = all_lines_in_time.size();
        bool first_step_remove_old_block = true;
        if ( DEBUG_TRAJECTORY_CREATION > 2 )
            M_INFO3("Steps %d \n",steps);
            
        for ( int t = 0; t <= steps && steps != 0; t++ ) {
            if ( DEBUG_TRAJECTORY_CREATION > 2 ) {
                M_INFO3("\t Preparing new move point towards split. \n");            
                std::cout << "\t Point base_point " << base_point << std::endl;
                std::cout << "\t Point split_point " << split_point;
                std::cout << std::endl;
            }
            Point a = base_point + (float (t)/ float (steps)) * move_base;
            if ( t == 0 ) {
                a = base_point;
            } else if ( t == steps ) {
                a = split_point;
            } 
        
            if ( DEBUG_TRAJECTORY_CREATION > 2 ) {
                std::cout << "\t Point a " << a << std::endl;
                std::cout << "\t Point left " << left_split_point << std::endl;
                std::cout << "\t Point right " << right_split_point;
                std::cout << std::endl;
            }
        
            Segment left_line(left_split_point, a);
            Segment right_line(a,right_split_point );
            Pos_list poses_left, poses_right;
        
            int c_l = 0, c_r = 0;
            if ( left_line.squared_length() > 0 ) { 
                if ( DEBUG_TRAJECTORY_CREATION > 2 ) {
                    M_INFO3(" --> Sampling visibility costs left. \n");
                }
                c_l = visibility_sampled_cost(poses_left, left_line);
                record_poses(left_o,new_o,poses_left,start_recording);
            }
                
            if ( right_line.squared_length() > 0 ) { 
                if ( DEBUG_TRAJECTORY_CREATION > 2 ) {
                    M_INFO3(" --> Sampling visibility costs right. \n");
                }
                c_r = visibility_sampled_cost(poses_right, right_line);
                record_poses(new_o,right_o,poses_right,start_recording);
            }
            
            int c = c_l + c_r;
        
            if ( c > max_c  ) {
                max_c = c;
            }
            if ( first_step_remove_old_block ) {
                // the time start_recording stil carries the old block line
                first_step_remove_old_block = false;
                remove_poses(left_o,right_o,start_recording);
            }
            start_recording++;
        }
    } else { 
        M_INFO2("2 2 2 ***** STEP 2 SKIPPED - shuffling block poses \n");
        int last_index = all_lines_in_time.size()-1;
        Pos_list oldposes = get_poses(left_o,right_o,last_index);
        int old_c = oldposes.size();
        M_INFO2("2 2 2 ***** STEP 2 SKIPPED - old poses %d \n",old_c);
        if ( old_c > max_c  ) {
            max_c = old_c;
        }
        if ( right_no_block ) {
            record_poses(new_o,left_o,oldposes,last_index);
        } else if ( left_no_block ) {
            record_poses(new_o,right_o,oldposes,last_index);
        }
        remove_poses(left_o,right_o,last_index);
    }
    int start_recording = all_lines_in_time.size();
    
    M_INFO2("\n\t ___STEP 3 - move to blocking positions\n\n");
    M_INFO2("\t ___STEP 3a) - left side\n");
    int max_b_l = 0;
    lineclear::Segment new_block_l = _e->get_shortest_line_inside(left_o,new_o);
    if ( true || new_block_l.target() != new_block_l.source() ) {
        Point dumm = new_block_l.source();;
        Point to_l, to_r;
        if ( _e->is_on_line(left_o,new_o,dumm) == left_o ) {
            to_l = new_block_l.source();
            to_r = new_block_l.target();
        } else {
            to_l = new_block_l.target();
            to_r = new_block_l.source();
        }
        max_b_l = move_between_two_lines( left_split_point, split_point,
            to_l, to_r , left_o, new_o, b_l, start_recording);
    } else {
        b_l = 0;
    }
    
    M_INFO2("\t ___STEP 3b) - right side\n");
    int max_b_r = 0;
    lineclear::Segment new_block_r = _e->get_shortest_line_inside(new_o, right_o);
    if ( true || new_block_r.target() != new_block_r.source() ) {
        Point dumm = new_block_r.source();;
        Point to_l, to_r;
        if ( _e->is_on_line(right_o,new_o,dumm) == right_o ) {
            to_l = new_block_r.target(); to_r = new_block_r.source();
        } else {
            to_l = new_block_r.source(); to_r = new_block_r.target();
        }
        max_b_r = move_between_two_lines( split_point, right_split_point,
            to_l, to_r , new_o, right_o, b_r, start_recording);
    } else {
        b_r = 0;
        // make sure to kill the line between right_o and new_o
    }
    if ( max_b_r + max_b_l  > max_c )
        max_c = max_b_r + max_b_l;
    
    M_INFO2("\n\t\t\t ** SAMPLED COST OF SPLIT %d \n\n",max_c);
    return max_c;
}

int ChoiceTree::move_between_two_lines( Point left_start, Point right_start,
        Point left_end, Point right_end, int left_o, int right_o, int& last_c, int start_recording) {

    double left_dist = CGAL::to_double(squared_distance( left_end, 
        left_start));
    double right_dist = CGAL::to_double(squared_distance(right_end,
        right_start));
    double move_on_dist;
    Segment move_on_other_segment;
    int move_on_o;
    Point move_on_from, move_on_to;
    // move larger segment 
    if ( left_dist > right_dist ) {
        move_on_o = left_o;
        move_on_from = left_start;
        move_on_to = left_end;
        move_on_dist = left_dist;
        move_on_other_segment = _e->get_edge( right_o );
    } else { 
        move_on_o = right_o;
        move_on_from = right_start;
        move_on_to = right_end;
        move_on_dist = right_dist;
        move_on_other_segment = _e->get_edge( left_o );
    }
    int max_c = 0;
    Vector move_v(move_on_from, move_on_to );
    int steps = ceil( sqrt(move_on_dist) / 
        Params::g_trajectory_discretization ) + 1; // TODO: check the +1
    if ( steps == 0 ) {
        M_INFO1("NO MOTION NECESSARY. \n");
        return 0;
    }
    if ( DEBUG_TRAJECTORY_CREATION > 2 ) {
        M_INFO1("move_between_two_lines IN %d STEPS. \n", steps);
        M_INFO1("move_on_o %d . \n", move_on_o);
        std::cout << " left_start " << left_start << " ";
        std::cout << " right_start " << right_start << " ";
        std::cout << " left_end " << left_end << " ";
        std::cout << " right_end " << right_end << " " << std::endl;
        M_INFO1("left_o %d left_o %d . \n", left_o,right_o);
    }
    for ( int t = 0; t <= steps; t++ ) {
        if ( DEBUG_TRAJECTORY_CREATION > 2 ) {
            M_INFO3("Preparing new move point towards triangle base. \n");
        }
        Point a = move_on_from + (float (t)/ float (steps)) * move_v;
        if ( t == steps ) {
            a = move_on_to;
        } else if ( t == 0 ) {
            a = move_on_from;
        }
        Point b = _e->get_closest_point(a, move_on_other_segment);
    
        if ( DEBUG_TRAJECTORY_CREATION > 2 ) {
            M_INFO3("\t Place UAVs on the line between ");
            std::cout << a << " and " << b << std::endl;
        }
        
        // a is on move_on_o
        Segment moving_line(a,b);
        // invert if segment goes from right to left
        if ( move_on_o == right_o ) {
            moving_line = moving_line.opposite();
        }
        
        Pos_list poses;
        if ( DEBUG_TRAJECTORY_CREATION > 2 ) {
            M_INFO3("\t Now sampling visibility costs ...\n");
        }
        int c  = visibility_sampled_cost(poses, moving_line);
        if ( c > 0 ) 
            record_poses(left_o,right_o,poses,start_recording);
        else {
            remove_poses(left_o,right_o,start_recording);
        }
        last_c = c;
        if ( c > max_c  ) {
            max_c = c;
        }
        if ( start_recording != -1 )
            start_recording++;
    }
    return max_c;
}

void ChoiceTree::remove_poses(int o1, int o2, int at) {
    int o_1 = o1,o_2 = o2;
    if (o1 > o2 ) { o_1 = o2; o_2 = o1; }
    
    if ( DEBUG_RECORD_POSES > 2)
        M_INFO1(" REMOVE POSES between %d and %d at time %d \n",o_1,o_2,at);
        
    std::pair<int,int> line_between(o_1,o_2);
    if ( at >= int(all_lines_in_time.size()) ) 
        at = -1;
    if ( at != -1 ) 
        all_lines_in_time[at].erase(line_between);
    else
        all_lines_in_time.back().erase(line_between);
}

Pos_list ChoiceTree::get_poses(int o1, int o2, int at) {
    int o_1 = o1,o_2 = o2;
    if (o1 > o2 ) { o_1 = o2; o_2 = o1; }
    
    if ( DEBUG_RECORD_POSES > 2)
        M_INFO1(" GET POSES between %d and %d at time %d \n",o_1,o_2,at);
    
    if ( all_lines_in_time.size() == 0 ) {
        Pos_list empty_poslist;
        return empty_poslist;
    }
    
    std::pair<int,int> line_between(o_1,o_2);
    if ( at != -1 ) 
        return all_lines_in_time[at][line_between];
    else
        return all_lines_in_time.back()[line_between];
}

void ChoiceTree::record_poses(int o1, int o2, Pos_list& poses, int at) {
    
    // RECORD POSES IN GLOBAL STRUCTURE
    all_frequin_poses.push_back(poses);
    
    int o_1 = o1,o_2 = o2;
    if (o1 > o2 ) { o_1 = o2; o_2 = o1; }
    
    M_INFO3_D(DEBUG_RECORD_POSES, 2, 
        " REC POSES between %d and %d\n at time %d \n",o_1,o_2,at);
        
    // create a new sweep state, load it and add to it
    Sweep_state new_sweep_state;
    if ( at != -1 
      && at < int(all_lines_in_time.size()) ) {
        if ( DEBUG_RECORD_POSES > 2) {
            M_INFO3("Loading sweep state at time %d \n",at);
        }
        new_sweep_state = all_lines_in_time[at];
    }
    else if (all_lines_in_time.size() > 0 ) {
        if ( DEBUG_RECORD_POSES > 2) {
            M_INFO3("Loading LAST sweep state at %d \n", 
            all_lines_in_time.size()-1);
        }
        new_sweep_state = all_lines_in_time.back();
    } else {
        // at is 0 and we will just push_back the first sweep state (below)
    }
    std::pair<int,int> line_between(o_1,o_2);
    new_sweep_state[line_between] = poses;

    if ( at != -1 
      && at < int(all_lines_in_time.size()) ) {
        all_lines_in_time[at] = new_sweep_state;
    }
    else {
        all_lines_in_time.push_back( new_sweep_state );
    }
}

int ChoiceTree::visibility_sampled_cost( Pos_list& poses, Segment& l1 ) {
    if ( l1.squared_length() <= 2 ) {
        return 0;
    }
    if ( Params::g_visibility_line_sampling_type_method == 1 ) {
        return _v->get_visibility_line_cost(
            floor( CGAL::to_double( l1.source().x())), 
            floor( CGAL::to_double( l1.source().y())),
            floor( CGAL::to_double( l1.target().x())), 
            floor( CGAL::to_double( l1.target().y())),
            poses );
    } else if ( Params::g_visibility_line_sampling_type_method == 2 ) {
        vector<NavPoint> current_nav_points;
        // TODO: fill this vector with the points of the previous line
        return this->random_cost_bias_sampled_cost(
            floor( CGAL::to_double( l1.source().x())), 
            floor( CGAL::to_double( l1.source().y())),
            floor( CGAL::to_double( l1.target().x())), 
            floor( CGAL::to_double( l1.target().y())),
            current_nav_points,
            poses);
    } else if ( Params::g_visibility_line_sampling_type_method == 3 ) {
        return this->random_sampled_cost(
            floor( CGAL::to_double( l1.source().x())), 
            floor( CGAL::to_double( l1.source().y())),
            floor( CGAL::to_double( l1.target().x())), 
            floor( CGAL::to_double( l1.target().y())),
            poses);
    }
    return 0;
}

void ChoiceTree::save_to_file( std::string filename ) {
    std::ofstream outFile;
    outFile.open(filename.c_str(),std::ios::out);
    outFile <<  _n << std::endl;
    for (int k = 1; k < _n; ++k ) {
        for (int i = 1; i <= _n; ++i ) {
            outFile << _choiceSetMatrix[k][i]->get_b() << " ";
            for ( int j = 1; j <= k; j++ ) {
                outFile << _choiceSetMatrix[k][i]->get_c_at(j) << " ";
            }
            if ( k != _n -1 || i != _n )
                outFile << std::endl;
        }
    }
    outFile.close();
}

void ChoiceTree::load_from_file( std::string filename ) {
    std::ifstream file (filename.c_str(), std::ios::in);
    if (!file.is_open()) {
        std::cout << " FAILED TO LOAD " << std::endl;
        return;
    } else {
        std::cout << " LOADING FILE " << filename.c_str() << std::endl;
    }
    
    double new_sensing_diameter = 2*_v->get_max_steps();
    // previously Params::g_new_sensing_range;
    std::cout << " new_sensing_diameter " << new_sensing_diameter << std::endl;
    
    std::string line;
    std::istringstream in_Stream;
    
    getline(file, line);
    in_Stream.str(line);
    in_Stream >> _n;    
    std::cout << " LOADING WITH " << _n << " obstacles." << std::endl;
    
    // build the choiceSetMatrix
    _choiceSetMatrix = new ChoiceSet**[_n];
    _cost_updated = new bool**[_n];
    for (int k = 1; k < _n; ++k ) {
        _choiceSetMatrix[k] = new ChoiceSet*[_n+1];
        _cost_updated[k] = new bool*[_n+1];
        for (int i = 1; i <= _n; ++i ) {
            _cost_updated[k][i] = new bool[k+1];
            getline(file, line);
            std::istringstream inStream;
            inStream.str(line);
            int b;
            inStream >> b;
            M_INFO1_D(DEBUG_CT_LOAD,1,"b in pix b=%d ",b);
            if ( Yaml_Config::yaml_param["use_new_sensing_range"].as<int>() ) {
                if ( b != 0 && b != -1 )
                    b = ceil ( b / new_sensing_diameter );
            }
            M_INFO1_D(DEBUG_CT_LOAD,1,"CS %d:%d b=%d ",i,k,b);
            _choiceSetMatrix[k][i] = new ChoiceSet(_n,i,k,b);
            for ( int j = 1; j <= k; j++ ) {
                _cost_updated[k][i][j] = false;
                int c;
                inStream >> c;
                if ( Yaml_Config::yaml_param["use_new_sensing_range"].as<int>() ) {
                    if ( c != 0 && c != -1 )
                        c = ceil ( c / new_sensing_diameter );
                }
                _choiceSetMatrix[k][i]->set_c_at(j,c);
                if ( DEBUG_CT_LOAD >= 2 ) {
                    int o = _choiceSetMatrix[k][i]->get_obstacle_for_choice(j);
                    std::cout << " c(" << o << ")=" << c << " ";
                    
                }
                //M_INFO1_D(DEBUG_CT_LOAD,1," c(%d)=%d ",j,c);
            }
            M_INFO1_D(DEBUG_CT_LOAD,1," \n ");
        }
    }
    
    std::cout << " FINISHED LOADING" << std::endl;
    std::cout << " Now processing ..." << std::endl;
    this->process_all_sets();
    
    int proper;
    std::cout << " average_n_cutsequences " <<
        this->average_n_cutsequences(proper) << std::endl;
    std::cout << " average_n_cutsequences proper " <<
        proper << std::endl;
    std::cout << " optimal cost " << this->get_optimal_cost() << std::endl;
}

void ChoiceTree::process_all_sets() {
    std::cout << "Processing all sets " << std::endl;
    for (int k = 1; k < _n; ++k ) {
        for (int i = 1; i <= _n; ++i ) {
            this->process_set(i,k);            
        }
    }
}

double ChoiceTree::average_n_cutsequences(int&proper_average) {
    std::cout << "Getting average number of cut sequences " << std::endl;
    double avg2 = 0;
    double pavg2 = 0;
    double ll = 0;
    for (int k = 1; k < _n; ++k ) {
        double avg = 0;
        double pavg = 0;
        double l = 0;
        for (int i = 1; i <= _n; ++i ) {
            if ( this->get_choice_set_at(i,k)->get_b() != -1 ) {
                l++;
                pavg += this->get_choice_set_at(i,k)->cut_sequences_size();
            }
            avg += this->get_choice_set_at(i,k)->cut_sequences_size();
        }
        avg = avg / _n;
        if ( l > 0 ) {
            pavg = pavg / l;
            pavg2 += pavg;
            ll++;
        }
        avg2 += avg;
    }
    avg2 = avg2 / (_n-1);
    if ( ll > 0 ) {
        pavg2 = pavg2 / ll;
    }
    proper_average = pavg2;
    return avg2;
}


void ChoiceTree::print_set( int i, int k) {
    if ( DEBUG_CHOICETREE >= 5 ) {
        std::cout << " Printing set " << i << " : " << k << std::endl;
    }
    ChoiceSet *cs = this->get_choice_set_at(i,k);
    CutSequence* cut_seq = cs->get_best_cut_sequence();
    if ( cut_seq == NULL ) 
        return;
    if ( DEBUG_CHOICETREE >= 5 ) {
        cut_seq->print();
    }
    if ( DEBUG_CHOICETREE >= 5 ) {
        std::cout << " Getting final cost " << i << " : " << k << std::endl;
    }
    int c = cut_seq->get_final_cost();
    M_INFO1_D(DEBUG_CHOICETREE,4," best cost %d \n",c);
}

/*
 * take choice set T_k^i and process it's choices
 * Choices 1 to k: 
 * Choice of obstacle indices i to i+k-1.
 */
void ChoiceTree::process_set( int i, int k) {

    if ( DEBUG_CHOICETREE >= 2 ) {
        std::cout << std::endl;
        std::cout << " ********______Processing set " << i << ":" << k;
        std::cout << std::endl;
    }
    // TODO: instead of best cut sequence take many
    
    ChoiceSet *cs = this->get_choice_set_at(i,k);
    //CutSequence* best_cs = NULL;
    //int best_j = 0;
    //int min_mu = -1;
    for ( int j = 1; j <= k; j++ ) {
        if ( DEBUG_CHOICETREE >= 2 ) {
            std::cout << " ********__________ Processing choice " << j 
                << " o=" << cs->get_obstacle_for_choice(j) << std::endl;
        }
        if (cs->get_c_at(j) == -1) {
            if ( DEBUG_CHOICETREE >= 2 ) {
                std::cout << " c too low" << cs->get_c_at(j) << std::endl;
            }
            continue;
        }
        
        // note that the zero_choiceset has exactly one cutsequence with one cut
        ChoiceSet *cs_l,*cs_r;
        cs_l = this->get_choice_set_at(i  , j-1);
        cs_r = this->get_choice_set_at(i+j, k-j);
        if ( DEBUG_CHOICETREE >= 1 ) {
            std::cout << " CS left has " << cs_l->cut_sequences_size() 
                << " cut sequences " << std::endl;
            std::cout << " CS right has " << cs_r->cut_sequences_size() 
                << " cut sequences " << std::endl;
        }
        
        CutSequence *cut_seq_l,*cut_seq_r;
        for ( int l_i = 0; l_i < cs_l->cut_sequences_size(); l_i++ ) {
            cut_seq_l = cs_l->get_cut_sequence(l_i);
            for ( int r_i = 0; r_i < cs_r->cut_sequences_size() ; r_i++ ) {                
                cut_seq_r = cs_r->get_cut_sequence(r_i);
                
                CutSequence* new_cs;
                new_cs = new_cutsequence_from(
                    cs,j, cs_l->get_b(), cs_r->get_b(), 
                    cut_seq_l, cut_seq_r );
                
                int cut_seq_c = new_cs->get_final_cost();
                if ( DEBUG_CHOICETREE >= 3 ) {
                    std::cout << " new cut seq cost " << cut_seq_c << std::endl;
                }
                if ( cut_seq_c != 0 && ! cs->is_dominated_weakly( *new_cs )) {
                    cs->add_cut_sequence( *new_cs, j );
                }
            }
        }
        if ( DEBUG_CHOICETREE >= 1 ) {
            std::cout << " have " << cs->cut_sequences_size() 
                << " cut sequences" << std::endl;
        }
        if ( DEBUG_CHOICETREE >= 2 ) {
            std::cout << " ********__________ DONE choice " << j << std::endl;
        }
    }
    
    cs->remove_dominated();
    
    //if ( best_cs != NULL ) {
    //    std::cout << " FOUND BEST CUT SEQUENCE T(" 
    //              << i << ":" << k << ")" << std::endl;
    //    best_cs->print();
    //    cs->add_cut_sequence( *best_cs, best_j );
    //} else { 
    //    std::cout << " FOUND NO BEST CUT SEQUENCE T(" 
    //             << i << ":" << k << ")" << std::endl;
    //}
}

CutSequence* 
ChoiceTree::new_cutsequence_from(ChoiceSet *cs, int j, int b_l, int b_r, 
                                 CutSequence *cut_seq_l, 
                                 CutSequence *cut_seq_r )
{
    if ( DEBUG_CHOICETREE >= 3 ) {
        std::cout << " ____****______ new_cutsequence_from " << std::endl;
    }
    int b = cs->get_b();
    int mu = cs->get_c_at(j);
    int last_o = cs->get_obstacle_for_choice(j);
    int b_current = b;
    int mu_current = mu;
    int mu_max = mu_current;
    bool candidate_cut_exists = false;
    Cut candidate_cut;
    if ( DEBUG_CHOICETREE >= 3 ) {
        std::cout << " ____****b=" << b << std::endl;
        std::cout << " ____****mu_max=" << mu_max << std::endl;
    }
    
    CutSequence* new_cut_sequence = new CutSequence();
    new_cut_sequence->set_base(b,mu);
    new_cut_sequence->set_left(cut_seq_l, b_l);
    new_cut_sequence->set_right(cut_seq_r, b_r);
    if ( DEBUG_CHOICETREE >= 3 ) {
        std::cout << "      1) ADDING base obstacle index" 
            << cs->get_obstacle_for_choice(j)
            << std::endl;
    }
    new_cut_sequence->add_obstacle_index( last_o );
    
    CutSequence::CutSequenceIterator r_i = cut_seq_r->begin();
    CutSequence::CutSequenceIterator l_i = cut_seq_l->begin();
    CutSequence::CutSequenceIterator r_end = cut_seq_r->end();
    CutSequence::CutSequenceIterator l_end = cut_seq_l->end();
    CutSequence::CutSequenceOIterator r_o_i = cut_seq_r->o_begin();
    CutSequence::CutSequenceOIterator l_o_i = cut_seq_l->o_begin();
    CutSequence::CutSequenceOIterator r_o_end = cut_seq_r->o_end();
    CutSequence::CutSequenceOIterator l_o_end = cut_seq_l->o_end();

    if ( DEBUG_CHOICETREE >= 3 ) {
        std::cout << " cut_seq_l " << std::endl;
        cut_seq_l->print();
        std::cout << " cut_seq_r " << std::endl;
        cut_seq_r->print();
    }
    
    // recall - the zero cut sequence has no obstacle indices
    bool building_cutsequence = true;
    bool l_done = false;
    bool r_done = false;
    if ( cut_seq_r->o_size() == 0 ) 
        r_done = true;
    if ( cut_seq_l->o_size() == 0 ) 
        l_done = true;
    if ( l_done && r_done )
        building_cutsequence = false;
    
    while( building_cutsequence ) {
        if ( DEBUG_CHOICETREE >= 3 ) {
            std::cout << " ** PARSING left/right cuts " << std::endl;
            std::cout << " ** b_l= " << b_l << std::endl;
            std::cout << " ** b_r= " << b_r << std::endl;
            std::cout << " ** right cut= ";
            if ( !r_done )
                r_i->print();
            else 
                std::cout << " done " << std::endl;
            std::cout << " ** left cut= ";
            if ( !l_done )
                l_i->print();
            else 
                std::cout << " done " << std::endl;
            std::cout  << "      l_done " << l_done << std::endl;
            std::cout  << "      r_done " << r_done << std::endl;
        }
        if ( b_r+b_l < b_current ) {
            // found a state in which we have lower blocking cost
            if ( DEBUG_CHOICETREE >= 3 ) {
                std::cout << " b_r+b_l < b_current " << std::endl;
                std::cout << "  - gen candidate cut " << std::endl;
            }
            candidate_cut = Cut(b_r+b_l, mu_max, last_o);
            candidate_cut_exists = true;
        }
        if ( l_done 
          || (!r_done && r_i->get_rho() <= l_i->get_rho() ) ) {
            // right side's turn - add all o-indices until reaching cut
            if ( DEBUG_CHOICETREE >= 3 ) {
                std::cout << "   RIGHT ob=" << *r_o_i << std::endl;
            }
            while ( r_i->get_o() != *r_o_i && r_o_end != r_o_i) {
                if ( DEBUG_CHOICETREE >= 4 ) {
                    std::cout << "      1a) RIGHT ob=" << *r_o_i << std::endl;
                }
                new_cut_sequence->add_obstacle_index( *r_o_i );
                r_o_i++;
            }
            new_cut_sequence->add_obstacle_index( r_i->get_o() );
            b_r =  r_i->get_b();
            mu_current = b_l + r_i->get_mu();
            last_o = r_i->get_o();
            r_i++; // increment right side cut for next loop
            r_o_i++;
        } else if ( r_done 
          || (!l_done && l_i->get_rho() <= r_i->get_rho() ) ) {
            // left side's turn - add all o-indices until reaching cut
            if ( DEBUG_CHOICETREE >= 3 ) {
                std::cout << "   LEFT ob=" << *l_o_i << std::endl;
            }
            while ( l_i->get_o() != *l_o_i  && l_o_end != l_o_i ) {
                if ( DEBUG_CHOICETREE >= 4 ) {
                    std::cout << "      1a) LEFT ob=" << *l_o_i << std::endl;
                }
                new_cut_sequence->add_obstacle_index( *l_o_i );
                l_o_i++;
            }
            new_cut_sequence->add_obstacle_index( l_i->get_o() );
            b_l = l_i->get_b();
            mu_current = b_r + l_i->get_mu();
            last_o = l_i->get_o();
            l_i++;
            l_o_i++;
        }
        if ( DEBUG_CHOICETREE >= 3 ) {
            std::cout << " mu_current " << mu_current << std::endl;
            std::cout << " b_l " << b_l << std::endl;
            std::cout << " b_r " << b_r << std::endl;
        }
        // check if the last candidate cut is indeed a full cut
        if ( mu_current > mu_max ) {
            if ( candidate_cut_exists ) {
                b_current = candidate_cut.get_b();
                if ( DEBUG_CHOICETREE >= 3 ) {
                    std::cout << "      3) New cut " 
                        << candidate_cut.get_mu() << ","
                        << candidate_cut.get_b() << std::endl;
                }
                new_cut_sequence->push_back( candidate_cut );
            }
            mu_max = mu_current; 
        }
        if ( r_i == r_end ) { r_done = true; }
        if ( l_i == l_end ) { l_done = true; }
        if ( l_done && r_done ) {building_cutsequence = false;}
    }
    if ( DEBUG_CHOICETREE >= 3 ) {
        std::cout << "      3) FINAL cut " << mu_max <<","<<last_o << std::endl;
    }
    new_cut_sequence->push_back( Cut(0,mu_max,last_o) ); // add the final cut
    if ( DEBUG_CHOICETREE >= 3 ) {
        std::cout << "      " << std::endl;
        new_cut_sequence->print();
        std::cout << "      " << std::endl;
    }
    
    return new_cut_sequence;
}

ChoiceSet* ChoiceTree::get_choice_set_at(int i, int k) {
    if ( DEBUG_CHOICETREE >= 2 ) {
        std::cout << "get_choice_set_at " << i << ":" << k << std::endl;
    }
    _e->fix_index(i);
    if ( DEBUG_CHOICETREE >= 3 ) {
        std::cout << "get_choice_set_at fixed " << i << ":" << k << std::endl;
    }
    if ( k == 0 || i == 0 ) {
        return _zero_choiceset;
    }
    if ( assert_indices(i,k) ) {
        return _choiceSetMatrix[k][i];
    }
    return NULL;
}

bool ChoiceTree::get_cost_updated(int i, int k, int j) {
    _e->fix_index(i);
    if ( k == 0 || i == 0 ) {
        return true;
    }
    if ( assert_indices(i,k) ) {
         return _cost_updated[k][i][j];
    }
    return false;
}

bool ChoiceTree::assert_indices(int i, int k) {
    if ( 0 < i && i <= _n && 0 < k && k < _n ) {
        return true;
    }
    return false;
}

ChoiceSet* ChoiceTree::get_optimal_choice_set(int &best_start) {
    std::cout << " get_optimal_choice_set " << std::endl;
    ChoiceSet* choice_set;
    ChoiceSet* best_choice_set = NULL;
    CutSequence* cut_seq;
    CutSequence* best_cut_seq;
    int min_cost = -1,c;
    
    // go through all base choice sets
    for (int i = 1; i <= _n; ++i ) { 
        choice_set = get_choice_set_at(i,_n-1);
        cut_seq = choice_set->get_best_cut_sequence();
        if ( cut_seq == NULL ) 
            continue;
        if ( DEBUG_CHOICETREE >= 3 ) {
            cut_seq->print();
        }
        c = cut_seq->get_final_cost();
        M_INFO1_D(DEBUG_CHOICETREE,2," best cost %d \n",c);
        if ( c < min_cost || min_cost == -1 ) {
            best_start = i-1;
            if ( best_start == 0 )
                best_start = _n;
            min_cost = c;
            best_cut_seq = cut_seq;
            M_INFO1_D(DEBUG_CHOICETREE,2," best choice set i=%d \n",i);
            best_choice_set = choice_set;
        }
    }
    if ( best_choice_set == NULL ) 
    {
        M_INFO1_D(DEBUG_CHOICETREE,1," ERROR null best choice set\n");
    }
    
    if ( DEBUG_CHOICETREE >= 4 ) {
        // print all cut sequences of all choice sets
        std::cout << "PRINTING all sets " << std::endl;
        for (int k = 1; k < _n; ++k ) {
            for (int i = 1; i <= _n; ++i ) {
                this->print_set(i,k);            
            }   
        }
        std::cout << "DONE WITH all sets " << std::endl;   
    }
    return best_choice_set;
}

CutSequence* ChoiceTree::get_optimal_cut_sequence(int &best_start) {
    std::cout << " get_optimal_cut_sequence " << std::endl;
    return get_optimal_choice_set(best_start)->get_best_cut_sequence();
}

std::list<int> ChoiceTree::get_optimal_obstacle_sequence(int &best_start) {
    std::cout << " get_optimal_obstacle_sequence " << std::endl;
    return get_optimal_cut_sequence(best_start)->get_obstacle_sequence();
}

int ChoiceTree::get_optimal_cost() {
    int best_start;
    return get_optimal_cut_sequence(best_start)->get_final_cost();
}


int ChoiceTree::get_number_of_obstacles() {
    return _n;
}

}