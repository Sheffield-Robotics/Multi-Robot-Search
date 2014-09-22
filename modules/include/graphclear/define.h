#ifndef SVG_DEFINE
#define SVG_DEFINE

#define DEF_MAX(a,b) ((a) < (b) ? (b) : (a))

// Some important testing options
#define GENERATE_N_TREES 100
#define MST_DEGREE 0
// Here we define all graph_clear debugging options
// vtrav is for voronoi diagram traversal
// ptrav is for pipeline traversal
// mrg is for merging of vertices in the improvement step
// DEBUG_FO_PA is for parsing fortune voronoi diagram for segments

#define SVG_STRATEGY_DEBUG 0

#define DEBUG_LBL 0
#define DEBUG_VTRAV 0
#define DEBUG_PTRAV 0
#define DEBUG 0
#define DEBUG_SHOW_START_VERTEX_COMP 0

#define DEBUG_1(string) if(DEBUG >= 1) cout << #string << endl
#define DEBUG_2(string) if(DEBUG >= 2) cout << #string << endl
#define DEBUG_3(string) if(DEBUG >= 3) cout << #string << endl
#define DEBUG_4(string) if(DEBUG >= 4) cout << #string << endl


#endif
