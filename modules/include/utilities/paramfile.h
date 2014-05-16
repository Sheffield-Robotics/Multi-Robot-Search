#ifndef PARAM_FILE_H
#define PARAM_FILE_H

namespace Params {

   // General stuff
   extern double g_start_world_x; 
   extern double g_start_world_y; 

   // Height map
   extern double g_minWallHeight;   // in [m]
   extern double g_flatHeight;      // in [m]
   extern double g_minRampAngle;    // in [deg]
   extern double g_maxRampAngle;    // in [deg]
   extern double g_heightDecrementPerUpdate;    // in [m]
   extern double g_maxCellAge;          // in [ms]
   extern double g_ClampedRescaleLength;    // in [m]
   extern bool g_ClampedRescale;    // [1|0]

   extern bool g_classify_everything_as_flat_ground; // [1|0]
   extern bool g_classify_from_mask_only; // [1|0]

   // Planner
   extern double g_obstacleMinDist; // in [m]
   extern double g_obstacleMaxDist; // in [m]
   extern bool g_skillExploration;  // [1|0]
   extern bool g_greedyExploration; // [1|0]
   extern bool g_draw_agent_trajectories; // [1|0]
   extern double g_min_dist_gps_poses; // [m] 

   // Motion Planner
   extern bool g_motion_planner;

   // Scheduler
   extern bool g_use_compressed_strategy; //[1|0]
   extern int g_num_spanning_trees; // 1..1000000
   extern bool g_bias_spanning_trees; // [1|0]
   extern bool g_show_visibility_during_schedule; // [1|0]
   extern bool g_compute_exhaustive_spanning_trees; // [1|0]
   extern bool g_compute_shady_edges; // [1|0]
   extern bool g_generate_regular_instead_of_sparse_edges; // [1|0]

   // Visibility
   extern bool g_use_improved_sampling;     // [1|0]
   extern double g_imporved_sampling_min_area_size; // in [m^2]
   extern int g_improved_sampling_down_level_search_width; // number of cells 
   extern bool g_use_circular_fov;      // [1|0]
   extern bool g_use_shrubs;        // [1|0]
   extern double g_shrub_height;    // in [m]
   extern double g_pursuer_height;  // in [m]
   extern double g_target_height;   // in [m]
   extern double g_pursuer_range;   // in [m]
   extern bool g_always_recompute_graph; // [1|0]
   extern bool g_use_only_one_region_per_pixel;
   extern double g_min_size_for_polygon_at_vertex;
   extern bool g_use_visibility_cache;
   extern double g_neg_value_fraction;
   extern bool g_allow_multiple_polygons;

   // Poligonizatoin
   extern double g_epsilon;
   extern double g_alpha;
   extern int g_startx;
   extern int g_starty;
   extern double g_min_polygon_area;
   extern bool g_construct_master;
   extern bool g_compute_UAV_cost_everywhere;
   extern bool g_use_new_sensing_range;
   extern double g_new_sensing_range;
   extern int g_resample_UAV_times;
   extern bool g_only_use_best_cut_sequence;
   extern int g_visibility_line_sampling_type;
   extern double g_trajectory_discretization;
   extern int g_visibility_line_sampling_type_method;
   extern int g_motion_cost_computation;
   extern double g_UAV_max_velocity;
   extern double g_distance_sum_cutoff;
   extern int g_blow_up_obstacles;
   extern double g_uav_distance_to_obstacles;
   extern double g_min_dist_to_other_uavs;
   extern bool g_use_min_distances;

   // Gui drawing
   extern bool g_use_multiple_agent_colors; // [1|0]
   extern bool g_draw_large_agents; // [1|0]

   // File IO
   extern bool g_always_save_existing_graph;    // [1|0]
   extern bool g_readWriteBorderAndRegionInfo;  // [1|0]

   // Experiment mode
   extern bool g_run_experiment_mode;

   // Functions
   void readConfFile(const char* fileName);
};

#endif
