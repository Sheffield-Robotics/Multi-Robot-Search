// 
//  ChoiceTree.h
//  lineClear
//  
//  Created by andreas on 2012-02-12.
//  Copyright 2012 andreas. All rights reserved.
// 

#ifndef CHOICETREE_H
#define CHOICETREE_H

#define DEBUG_CHOICETREE 6
#define DEBUG_CT_LOAD 6
#define DEBUG_TRAJECTORY_CREATION 0
#define DEBUG_RECORD_POSES 0

#include "lineclear/ChoiceSet.h"
#include "lineclear/Environment.h"
#include "lineclear/ObstacleSequence.h"
#include "lineclear/SurveillanceGraph.h"
#include "polygonization/polygon_environment.h"
#include "heightmap/visibility.h"
#include "uavmodel/navpoint.h"
#include <assert.h>
#include <iostream>
#include <string>
#include <list>
#include <vector>

#include "lineclear/CGALTypes.h"

namespace lineclear {
    
    typedef std::list<Visibility::Pos> Pos_list;
    typedef std::map< std::pair<int,int>, Pos_list> Sweep_state;
    
    class ChoiceTree {
        
      public:
    
        ChoiceTree(Environment* e);
        ChoiceTree(Environment* e, Visibility* v);
        ChoiceTree(Environment* e, Visibility* v, 
            polygonization::Polygon_Environment* pe);
        void init_choice_tree();
        ChoiceSet* get_choice_set_at(int i, int k);
        void save_to_file( std::string filename );
        void load_from_file( std::string filename );
        
        ChoiceSet* get_optimal_choice_set(int &best_start);
        CutSequence* get_optimal_cut_sequence( int &best_start);
        std::list<int> get_optimal_obstacle_sequence(int &best_start);
        int get_optimal_cost();
    
        int update_costs();
        void update_costs(int i, int k);
        void update_costs(int i, int k, int j);
        int update_costs(CutSequence* cut_seq, int first_o);
        int get_number_of_obstacles();
        
        int computeElevationMapBorderCoverageCost(Segment& l1);
        int computeElevationMapBorderCoverageCost(int x1, int y1, int x2, int y2);
        int computeElevationMapBorderCoverageCostOften(Segment& l1);
        double average_n_cutsequences(int&proper_average);
        int random_sampled_cost(int x1, int y1, int x2, int y2, Pos_list& poses);
        int random_cost_bias_sampled_cost(int x1, int y1, int x2, int y2, vector<NavPoint> current_poses, Pos_list& poses);
        
        int visibility_sampled_cost( Pos_list& poses, Segment& l1 );
        int sampled_cost_split( Segment& to_line1, Segment& to_line2, int left_o, int right_o, int new_o, int& b_l, int& b_r );
        
        int move_between_two_lines( Point left_start, Point right_start, Point left_end, Point right_end, int left_o, int right_o, int& last_c, int start_recording = -1);
        void record_poses(int o1, int o2, Pos_list& poses, int at);
        void remove_poses(int o1, int o2, int at);
        Pos_list get_poses(int o1, int o2, int at = -1);
        
        inline
        bool are_adjacent(int i, int j ) {
            int val = abs(i-j);
            if ( val < 2 || val >= _n-1 ) { 
                return true; 
            }
            return false;
        }
        
        std::vector<Pos_list> all_frequin_poses;
        Sweep_state current_lines;
        vector<Sweep_state> all_lines_in_time;
        
        vector<int> start_step_for_split_obstacle;
        
        SurveillanceGraph* _sg;
        
      private:
        
        ChoiceSet* _zero_choiceset;
        Environment* _e;
        polygonization::Polygon_Environment* _pol_env;
        Visibility* _v;
        
        int _n;

        /* k is the first index (opposite of get_choice_set_at)*/
        ChoiceSet*** _choiceSetMatrix; // lovin' triple pointers 
        bool*** _cost_updated; // lovin' triple pointers 
        ChoiceSet* _root;
        
        void process_all_sets();
        void process_set( int i, int k);
        void print_set( int i, int k);
        
        CutSequence* 
        new_cutsequence_from(ChoiceSet *cs, int j, int b_l, int b_r, 
                             CutSequence *cut_seq_l, 
                             CutSequence *cut_seq_r );
        bool get_cost_updated(int i, int k, int j);
        bool assert_indices(int i, int k);
    };
}
#endif