#ifndef SG_DEFINE
#define SG_DEFINE

// Some important testing options
#define MST_DEGREE 0
#define OLD_VORONOI 1
#define LINE_DISCRETE_STEPS 100
#define INTER_POSE_DISTANCE 3
#define SG_GRAPH_MAX_VERTEX_DEGREE 3
#define ERROR_THRESHOLD 0.2
// Here we define all graph_clear debugging options
// vtrav is for voronoi diagram traversal
// ptrav is for pipeline traversal
// mrg is for merging of vertices in the improvement step
// DEBUG_FO_PA is for parsing fortune voronoi diagram for segments
#define DEBUG_SWEEP_COMP 0
#define DEBUG_CLOSES_OBSTACLE 0
#define DEBUG_MEMBER 5
#define DEBUG 5
#define DEBUG_MRG 5
#define DEBUG_LBL 5
#define DEBUG_SHOW_START_VERTEX_COMP 0
#define DEBUG_MAP 1 
#define DEBUG_VTRAV 5
#define DEBUG_PTRAV 5
#define DEBUG_FO_PA 5 
#define DEBUG_VOR_MIN 5
#define DEBUG_GR_CON 5
#define DEBUG_TASK 5
#define DEBUG_1(string) if(DEBUG >= 1) cout << #string << endl
#define DEBUG_2(string) if(DEBUG >= 2) cout << #string << endl
#define DEBUG_3(string) if(DEBUG >= 3) cout << #string << endl
#define DEBUG_4(string) if(DEBUG >= 4) cout << #string << endl

#endif
