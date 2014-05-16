#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <sstream>
#include <cstring>
#include <cstdlib>

using namespace std;

namespace Params
{
   // General stuff
   double g_start_world_x = 0; 
   double g_start_world_y = 0; 

   // Height map
   bool g_classify_everything_as_flat_ground = false;
   bool g_classify_from_mask_only = false;

   // Height difference that cannot be overcome 
   double g_minWallHeight = 2.0;

   // Allowed variance of height values on flat ground
   double g_flatHeight = 1.0;

   // Definition of ramp
   double g_minRampAngle = 45.0;
   double g_maxRampAngle = 90.0;

   double g_heightDecrementPerUpdate = 0.1;
   double g_maxCellAge = 500.0;
   double g_ClampedRescaleLength = 40.0;
   bool g_ClampedRescale = true;

   // Planner
   double g_obstacleMinDist = 0.25;
   double g_obstacleMaxDist = 2.0;
   bool g_skillExploration = false;
   bool g_greedyExploration = false;
   bool g_draw_agent_trajectories;
   double g_min_dist_gps_poses = -1.0;

   // Motion Planner
   bool g_motion_planner = false;

   // Scheduler
   bool g_use_compressed_strategy = true;
   bool g_show_visibility_during_schedule = true;
   int g_num_spanning_trees = 1; 
   bool g_bias_spanning_trees = false; 
   bool g_compute_exhaustive_spanning_trees = false;
   bool g_compute_shady_edges = false;
   bool g_generate_regular_instead_of_sparse_edges = false;

   // Visibility
   bool g_use_improved_sampling = true;
   double g_imporved_sampling_min_area_size = 500; 
   bool g_use_circular_fov = true;
   double g_pursuer_height = 2.0;
   double g_target_height = 2.0;
   double g_pursuer_range = 1000.0;
   bool g_always_recompute_graph = false; 
   bool g_use_shrubs = true;
   double g_shrub_height = 1.0;
   int g_improved_sampling_down_level_search_width = 4;
   bool g_use_only_one_region_per_pixel = true;
   double g_min_size_for_polygon_at_vertex;
   bool g_use_visibility_cache = false;
   double g_neg_value_fraction = 1;
   bool g_allow_multiple_polygons = true;

   // Poligonization
   double g_epsilon = 3.0;
   double g_alpha = 3.0;
   int g_startx = 10;
   int g_starty = 10;
   double g_min_polygon_area = 10.0;
   bool g_construct_master = true;
   bool g_compute_UAV_cost_everywhere = false;
   bool g_use_new_sensing_range = false;
   double g_new_sensing_range = 30.0;
   int g_resample_UAV_times = 3;
   bool g_only_use_best_cut_sequence = 0;
   int g_visibility_line_sampling_type = 1;
   double g_trajectory_discretization = 4.0;
   int g_visibility_line_sampling_type_method = 1;
   int g_motion_cost_computation = 1;
   double g_UAV_max_velocity = 0.1;
   double g_distance_sum_cutoff = 200;
   int g_blow_up_obstacles = 1;
   double g_uav_distance_to_obstacles = 0.01;
   double g_min_dist_to_other_uavs = 0.01;
   bool g_use_min_distances = false;

   // Gui drawing
   bool g_use_multiple_agent_colors = true; 
   bool g_draw_large_agents = true;

   // File IO
   bool g_always_save_existing_graph = true;
   bool g_readWriteBorderAndRegionInfo = false;

   // Experiment
   bool g_run_experiment_mode = false;

   void readConfFile(const char* fileName) 
   {
      // open file for reading
      ifstream file (fileName, ios::in);
      if (!file.is_open()) {
         cerr << "Error: could not open file: \"" << fileName << "\" for reading." << endl;
         cerr << flush;
         exit(1);
      }

      cerr << "reading configuration data from file: \"" << fileName << "\":" << endl;
      cerr << flush;

      string entryLine;
      istringstream inStream;
      char messageName[64];

      while (!file.eof()) {
         getline (file, entryLine);

         if (entryLine.find('#') == 0 || entryLine.size() < 3) {
            continue;
         }

         inStream.clear();
         inStream.str(entryLine);

         inStream >> messageName;

         if (strcmp(messageName, "min_wall_height") == 0) {
            inStream >> g_minWallHeight;
            cerr << "min_wall_height = " << g_minWallHeight << endl;
         } else if (strcmp(messageName, "flat_height") == 0) {
            inStream >> g_flatHeight;
            cerr << "flat_height = " << g_flatHeight << endl;
         } else if (strcmp(messageName, "min_ramp_angle") == 0) {
            inStream >> g_minRampAngle;
            cerr << "min_ramp_angle = " << g_minRampAngle << endl;
         } else if (strcmp(messageName, "max_ramp_angle") == 0) {
            inStream >> g_maxRampAngle;
            cerr << "max_ramp_angle = " << g_maxRampAngle << endl;
         } else if (strcmp(messageName, "height_decrement_per_update") == 0) {
            inStream >> g_heightDecrementPerUpdate;
            cerr << "height_decrement_per_update = " << g_heightDecrementPerUpdate << endl;
         } else if (strcmp(messageName, "max_cell_age") == 0) {
            inStream >> g_maxCellAge;
            cerr << "max_cell_age = " << g_maxCellAge << endl;
         } else if (strcmp(messageName, "clamped_rescale_length") == 0) {
            inStream >> g_ClampedRescaleLength;
            cerr << "clamped_rescale_length = " << g_ClampedRescaleLength << endl;
         } else if (strcmp(messageName, "clamped_rescale") == 0) {
            inStream >> g_ClampedRescale;
            cerr << "clamped_rescale = " << g_ClampedRescale << endl;
         } else if (strcmp(messageName, "obstacle_min_dist") == 0) {
            inStream >> g_obstacleMinDist;
            cerr << "obstacle_min_dist = " << g_obstacleMinDist << endl;
         } else if (strcmp(messageName, "obstacle_max_dist") == 0) {
            inStream >> g_obstacleMaxDist;
            cerr << "obstacle_max_dist = " << g_obstacleMaxDist << endl;
         } else if (strcmp(messageName, "skill_exploration") == 0) {
            inStream >> g_skillExploration;
            cerr << "skill_exploration = " << g_skillExploration << endl;
         } else if (strcmp(messageName, "greedy_exploration") == 0) {
            inStream >> g_greedyExploration;
            cerr << "greedy_exploration = " << g_greedyExploration << endl;
         } else if (strcmp(messageName, "shrub_height") == 0) {
            inStream >> g_shrub_height;
            cerr << "shrub_height = " << g_shrub_height << endl;
         } else if (strcmp(messageName, "pursuer_height") == 0) {
            inStream >> g_pursuer_height;
            cerr << "pursuer_height = " << g_pursuer_height << endl;
         } else if (strcmp(messageName, "target_height") == 0) {
            inStream >> g_target_height;
            cerr << "target_height = " << g_target_height << endl;
         } else if (strcmp(messageName, "pursuer_range") == 0) {
            inStream >> g_pursuer_range;
            cerr << "pursuer_range = " << g_pursuer_range << endl;
         } else if (strcmp(messageName, "use_shrubs") == 0) {
            inStream >> g_use_shrubs;
            cerr << "use_shrubs = " << g_use_shrubs << endl;
         } else if (strcmp(messageName, "use_circular_fov") == 0) {
            inStream >> g_use_circular_fov;
            cerr << "use_circular_fov = " << g_use_circular_fov << endl;
         } else if (strcmp(messageName, "use_improved_sampling") == 0) {
            inStream >> g_use_improved_sampling;
            cerr << "use_improved_sampling = " << g_use_improved_sampling << endl;
         } else if (strcmp(messageName, "always_save_existing_graph") == 0) {
            inStream >> g_always_save_existing_graph;
            cerr << "always_save_existing_graph = " << g_always_save_existing_graph << endl;
         } else if (strcmp(messageName, "read_write_border_and_region_info") == 0) {
            inStream >> g_readWriteBorderAndRegionInfo;
            cerr << "read_write_border_and_region_info = " << g_readWriteBorderAndRegionInfo << endl;
         } else if (strcmp(messageName, "improved_sampling_min_area_size") == 0) {
            inStream >> g_imporved_sampling_min_area_size;
            cerr << "improved_sampling_min_area_size = " << g_imporved_sampling_min_area_size << endl;
         } else if (strcmp(messageName, "classify_everything_as_flat_ground") == 0) {
            inStream >> g_classify_everything_as_flat_ground;
            cerr << "classify_everything_as_flat_ground = " << g_classify_everything_as_flat_ground << endl;
         } else if (strcmp(messageName, "always_recompute_graph") == 0) {
            inStream >> g_always_recompute_graph;
            cerr << "always_recompute_graph = " << g_always_recompute_graph << endl;
         } else if (strcmp(messageName, "use_compressed_strategy") == 0) {
            inStream >> g_use_compressed_strategy;
            cerr << "use_compressed_strategy = " << g_use_compressed_strategy << endl;
         } else if (strcmp(messageName, "start_world_x") == 0) {
            inStream >> g_start_world_x;
            cerr << "start_world_x = " << g_start_world_x << endl;
         } else if (strcmp(messageName, "start_world_y") == 0) {
            inStream >> g_start_world_y;
            cerr << "start_world_y = " << g_start_world_y << endl;
         } else if (strcmp(messageName, "show_visibility_during_schedule") == 0) {
            inStream >> g_show_visibility_during_schedule;
            cerr << "show_visibility_during_schedule = " << g_show_visibility_during_schedule << endl;
         } else if (strcmp(messageName, "draw_agent_trajectories") == 0) {
            inStream >> g_draw_agent_trajectories;
            cerr << "draw_agent_trajectories = " << g_draw_agent_trajectories << endl;
         } else if (strcmp(messageName, "num_spanning_trees") == 0) {
            inStream >> g_num_spanning_trees;
            cerr << "num_spanning_trees = " << g_num_spanning_trees << endl;
         } else if (strcmp(messageName, "bias_spanning_trees") == 0) {
            inStream >> g_bias_spanning_trees;
            cerr << "bias_spanning_trees = " << g_bias_spanning_trees << endl;
         } else if (strcmp(messageName, "compute_exhaustive_spanning_trees") == 0) {
            inStream >> g_compute_exhaustive_spanning_trees;
            cerr << "compute_exhaustive_spanning_trees = " << g_compute_exhaustive_spanning_trees << endl;
         } else if (strcmp(messageName, "compute_shady_edges") == 0) {
            inStream >> g_compute_shady_edges;
            cerr << "compute_shady_edges = " << g_compute_shady_edges << endl;
         } else if (strcmp(messageName, "generate_regular_instead_of_sparse_edges") == 0) {
            inStream >> g_generate_regular_instead_of_sparse_edges;
            cerr << "generate_regular_instead_of_sparse_edges = " << g_generate_regular_instead_of_sparse_edges << endl;
         } else if (strcmp(messageName, "min_dist_gps_poses") == 0) {
            inStream >> g_min_dist_gps_poses;
            cerr << "min_dist_gps_poses = " << g_min_dist_gps_poses << endl;
         } else if (strcmp(messageName, "use_multiple_agent_colors") == 0) {
            inStream >> g_use_multiple_agent_colors;
            cerr << "use_multiple_agent_colors = " << g_use_multiple_agent_colors << endl;
         } else if (strcmp(messageName, "draw_large_agents") == 0) {
            inStream >> g_draw_large_agents;
            cerr << "draw_large_agents = " << g_draw_large_agents << endl;
         } else if (strcmp(messageName, "improved_sampling_down_level_search_width") == 0) {
            inStream >> g_improved_sampling_down_level_search_width;
            cerr << "improved_sampling_down_level_search_width = " << g_improved_sampling_down_level_search_width << endl;
         } else if (strcmp(messageName, "classify_from_mask_only") == 0) {
            inStream >> g_classify_from_mask_only;
            cerr << "classify_from_mask_only = " << g_classify_from_mask_only << endl;
         } else if (strcmp(messageName, "use_only_one_region_per_pixel") == 0) {
            inStream >> g_use_only_one_region_per_pixel;
            cerr << "use_only_one_region_per_pixel = " << g_use_only_one_region_per_pixel<< endl;
         } else if (strcmp(messageName, "min_size_for_polygon_at_vertex") == 0) {
            inStream >> g_min_size_for_polygon_at_vertex;
            cerr << "min_size_for_polygon_at_vertex = " << g_min_size_for_polygon_at_vertex<< endl;
         } else if (strcmp(messageName, "use_visibility_cache") == 0) {
            inStream >> g_use_visibility_cache;
            cerr << "use_visibility_cache = " << g_use_visibility_cache << endl;
         } else if (strcmp(messageName, "neg_value_fraction") == 0) {
            inStream >> g_neg_value_fraction;
            cerr << "g_neg_value_fraction = " << g_neg_value_fraction << endl;
         } else if (strcmp(messageName, "allow_multiple_polygons") == 0) {
            inStream >> g_allow_multiple_polygons;
            cerr << "g_allow_multiple_polygons = " << g_allow_multiple_polygons << endl;
         } else if (strcmp(messageName, "run_experiment_mode") == 0) {
            inStream >> g_run_experiment_mode;
            cerr << "run_experiment_mode = " << g_run_experiment_mode << endl;
         } else if (strcmp(messageName, "alpha") == 0) {
             inStream >> g_alpha;
             cerr << "alpha = " << g_alpha << endl;
         } else if (strcmp(messageName, "epsilon") == 0) {
            inStream >> g_epsilon;
            cerr << "epsilon = " << g_epsilon << endl;
         } else if (strcmp(messageName, "startx") == 0) {
            inStream >> g_startx;
            cerr << "startx = " << g_startx << endl;
         } else if (strcmp(messageName, "starty") == 0) {
            inStream >> g_starty;
            cerr << "starty = " << g_starty << endl;
         } else if (strcmp(messageName, "min_polygon_area") == 0) {
            inStream >> g_min_polygon_area;
            cerr << "min_polygon_area = " << g_min_polygon_area << endl;
         } else if (strcmp(messageName, "construct_master") == 0) {
            inStream >> g_construct_master;
            cerr << "construct_master = " << g_construct_master << endl;
         } else if (strcmp(messageName, "compute_UAV_cost_everywhere") == 0) {
            inStream >> g_compute_UAV_cost_everywhere;
            cerr << "compute_UAV_cost_everywhere = " << g_compute_UAV_cost_everywhere << endl;
         } else if (strcmp(messageName, "new_sensing_range") == 0) {
            inStream >> g_new_sensing_range;
            cerr << "new_sensing_range = " << g_new_sensing_range << endl;
         } else if (strcmp(messageName, "use_new_sensing_range") == 0) {
            inStream >> g_use_new_sensing_range;
            cerr << "use_new_sensing_range = " << g_use_new_sensing_range << endl;
         } else if (strcmp(messageName, "resample_UAV_times") == 0) {
            inStream >> g_resample_UAV_times;
            cerr << "resample_UAV_times = " << g_resample_UAV_times << endl;
         } else if (strcmp(messageName, "only_use_best_cut_sequence") == 0) {
            inStream >> g_only_use_best_cut_sequence;
            cerr << "only_use_best_cut_sequence = " << g_only_use_best_cut_sequence << endl;
         } else if (strcmp(messageName, "visibility_line_sampling_type") == 0) {
            inStream >> g_visibility_line_sampling_type;
            cerr << "visibility_line_sampling_type = " << g_visibility_line_sampling_type << endl;
         } else if (strcmp(messageName, "trajectory_discretization") == 0) {
            inStream >> g_trajectory_discretization;
            cerr << "trajectory_discretization = " << g_trajectory_discretization << endl;
         } else if (strcmp(messageName, "visibility_line_sampling_type_method") == 0) {
            inStream >> g_visibility_line_sampling_type_method;
            cerr << "visibility_line_sampling_type_method = " << g_visibility_line_sampling_type_method << endl;
         } else if (strcmp(messageName, "motion_cost_computation") == 0) {
            inStream >> g_motion_cost_computation;
            cerr << "motion_cost_computation = " << g_motion_cost_computation << endl;
         } else if (strcmp(messageName, "UAV_max_velocity") == 0) {
            inStream >> g_UAV_max_velocity;
            cerr << "UAV_max_velocity = " << g_UAV_max_velocity << endl;
         } else if (strcmp(messageName, "distance_sum_cutoff") == 0) {
            inStream >> g_distance_sum_cutoff;
            cerr << "distance_sum_cutoff = " << g_distance_sum_cutoff << endl;
         } else if (strcmp(messageName, "blow_up_obstacles") == 0) {
            inStream >> g_blow_up_obstacles;
            cerr << "blow_up_obstacles = " << g_blow_up_obstacles << endl;
         } else if (strcmp(messageName, "uav_distance_to_obstacles") == 0) {
             inStream >> g_uav_distance_to_obstacles;
             cerr << "uav_distance_to_obstacles = " << g_uav_distance_to_obstacles << endl;
         } else if (strcmp(messageName, "min_dist_to_other_uavs") == 0) {
              inStream >> g_min_dist_to_other_uavs;
              cerr << "min_dist_to_other_uavs = " << g_min_dist_to_other_uavs << endl;
          } else if (strcmp(messageName, "use_min_distances") == 0) {
              inStream >> g_use_min_distances;
              cerr << "use_min_distances = " << g_use_min_distances << endl;
          }
      }

      file.close();

      cerr << "done" << endl;
   }
}

