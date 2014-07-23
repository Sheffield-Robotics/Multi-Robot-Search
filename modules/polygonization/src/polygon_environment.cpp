#include "polygonization/polygon_environment.h"
#include "polygonization/voronoi_diagram.h"
#include "utilities/paramfile.h"

#define DEBUG_POLYGONENVIRONMENT 1

namespace polygonization {
    
void Polygon_Environment::construct( Alpha_shape* alphaShape, float epsilon )
{
    _epsilon = epsilon;
    _visi_epsilon = 0.00000000001;
    
    A = alphaShape;
    Alpha_shape::vertex_iterator AS_vert_it;
    AS_vert_it = A->alpha_shape_vertices_begin();
    std::list< Alpha_shape::vertex_handle > vertex_list;
    for ( ; AS_vert_it != A->alpha_shape_vertices_end() ; ++AS_vert_it ) 
    {
        vertex_list.push_back( *AS_vert_it );
        vertex_list.back()->info() = false;
    }

    Alpha_shape::Alpha_shape_edges_iterator e_it 
        = A->alpha_shape_edges_begin();
    Alpha_shape::vertex_handle vertex_h;
    this->n_polygons = 0; 
    while ( vertex_list.size() > 0 ) {
        // pick the first vertex
        vertex_h = vertex_list.front();
        vertex_list.pop_front();
        
        if ( vertex_h->info() == true )
            continue;
        if ( A->classify( vertex_h ) != Alpha_shape_base::REGULAR )
            continue;
        
        extract_polygon( vertex_h );
    }
    if ( open_polygons > 0 )
        M_WARN("\nalpha_shape - have OPEN POLYGONS!!!\n");
    
    std::list< std::pair<int,double> > area_list;
    for ( unsigned int i = 0; i < this->size(); i++ ) { 
        double area = fabs(CGAL::to_double(this->at(i).area()));
        area_list.push_back( std::pair<int,double>(i,area));
        if ( DEBUG_POLYGONENVIRONMENT >=3 )
            std::cout << " poly " << i << " area " << area << std::endl;
    }
    area_list.sort(pair_comparison);
    std::list< std::pair<int,double> >::iterator list_i;
    list_i = area_list.begin();
    int larget_poly_i = list_i->first;
    M_INFO2("Erasing largest polygon (faulty/redundant outside border).\n");
    this->erase(this->begin()+larget_poly_i);
    area_list.clear();
    
    M_INFO2("Grabbing the new largest polygon to define outer boundary.\n");
    for ( unsigned int i = 0; i < this->size(); i++ ) {  
        area_list.push_back( std::pair<int,double>(i,
            fabs(CGAL::to_double(this->at(i).area()))));
    }
    area_list.sort(pair_comparison);
    list_i = area_list.begin();
    unsigned int second_largest_poly_i = ((unsigned int) list_i->first);
    
    //Polygon* second_poly = &( this->at(second_largest_poly_i) );
    
    // connect the remaining polygons to make a master polygon
    master_polygon = new Polygon();
    
    M_INFO1("Constructing Voronoi Diagram\n");
    VD = new Voronoi_Diagram();
    VD->construct(this);
    M_INFO1("Constructed Voronoi Diagram with %d edges and %d vertices \n",
        VD->number_of_halfedges(), VD->number_of_vertices());
    distance_mat = new double*[this->size()];
    
    for ( unsigned int i = 0; i < this->size();i++ ) {
        distance_mat[i] = new double[i+1];
        for ( unsigned int j = 0; j < i+1;j++ ) {
            distance_mat[i][j] = -1;
        }
    }
    M_INFO1("Parsing Voronoi Diagram to create distance matrix\n");
    segment_mat = VD->parse(distance_mat,this);
    
    //replicate distance matrix
    distance_mat2 = new double*[this->size()];
    for ( unsigned int i = 0; i < this->size();i++ ) {
        distance_mat2[i] = new double[i+1];
        for ( unsigned int j = 0; j < i+1;j++ ) {
            distance_mat2[i][j] = distance_mat[i][j];
            if ( DEBUG_POLYGONENVIRONMENT >=3 )
                std::cout << distance_mat[i][j] << " ";
        }
        if ( DEBUG_POLYGONENVIRONMENT >=3 )
            std::cout << std::endl;
    }
    
    compute_to_master_distances(second_largest_poly_i);

    if ( DEBUG_POLYGONENVIRONMENT >=3 ) {
        M_INFO1("Show master distances to %d \n",second_largest_poly_i);
        for ( unsigned int i = 0; i < this->size();i++ ) {
            std::cout << get_distance2(second_largest_poly_i,i) << " ";
        }
        std::cout << std::endl;
    }
    M_INFO1("Computing polygon distances to Master Polygon for %d polygons.\n", 
        this->size());
    std::list< std::pair<int,double> > distance_list;
    for ( unsigned int i = 0; i < this->size(); i++ ) {
        double d = 0;
        if ( i != second_largest_poly_i ) {
            d = get_distance2(second_largest_poly_i,i);
            if ( DEBUG_POLYGONENVIRONMENT >=2 ) {
                std::cout << i << " d= " << d << std::endl;
            }
        }
        if ( d != -1 )
            distance_list.push_back( std::pair<int,double>(i,d));
    }
    
    M_INFO2("Sorting polygons by distance to master.\n");
    distance_list.sort(pair_comparison_inv);

    if ( !Params::g_construct_master )
        return;
    
    M_INFO1("Adding polygons in order of distance to master (ascending).\n");
    list_i = distance_list.begin();
    while ( list_i != distance_list.end() ) {
        M_INFO2("Adding polygon %d with distance %f \n",
            list_i->first,list_i->second);
        //if ( list_i->first == 118 )
        //    break;
        add_to_master( &this->at(list_i->first),list_i->first, second_largest_poly_i );
        list_i++;
    }
    M_INFO3("Finished constructing Master Polygon (%d vertices).\n", master_polygon->size());
    
    //M_INFO3("Building visibility graph.\n");
    //if ( visibility_graph != NULL ) 
    //    delete visibility_graph;
    //visibility_graph = new Vis_graph(master_polygon->vertices_begin(), master_polygon->vertices_end());
    
    seg_vis_graph = new Segment_Visibility_Graph();
    
    this->process_master_polygon();
        
    return;
}

void Polygon_Environment::process_master_polygon()
{
    M_INFO3(" Polygon_Environment::process_master_polygon \n");
    // for every vertex, i.e., endpoint of segment/edge in the 
    // master polygon we need to compute the visibility polygon
    // to determine visibility of segments
    VisiLibity::Polygon a_poly;

    // converting master polygon to a visilibity polygon
    seg_vis_graph_type1_vertices.clear();
    seg_vis_graph_type2_vertices.clear();
    int i=0;
    double next_x = 0, next_y = 0;
    double first_x = 0, first_y = 0;
    Polygon::Vertex_iterator it = master_polygon->vertices_begin();
    first_x = CGAL::to_double((*it).x());
    first_y = CGAL::to_double((*it).y());
    for ( ; it != master_polygon->vertices_end();  ) 
    {
        VisiLibity::Point poi( 
            CGAL::to_double((*it).x()),
            CGAL::to_double((*it).y()) );
        a_poly.push_back( poi );
        
        it++;
        if (it == master_polygon->vertices_end() )
        {
            next_x = first_x; next_y = first_y;    
        } else {
            next_x = CGAL::to_double((*it).x());
            next_y = CGAL::to_double((*it).y());
        }   
            
        Segment_Visibility_Graph::vertex v_desc;
        Segment_Visibility_Graph::seg_vertex 
            a_vertex(i,1, (next_x+poi.x())/2,(next_y+poi.y())/2);
        v_desc = seg_vis_graph->add_vertex( a_vertex );
        seg_vis_graph_type1_vertices.push_back(v_desc);
        Segment_Visibility_Graph::seg_vertex 
            b_vertex(i,2, poi.x(), poi.y() );
        v_desc = seg_vis_graph->add_vertex( b_vertex );
        seg_vis_graph_type2_vertices.push_back(v_desc);
        i++;
    }
    
    M_INFO3("Added %d vertices.\n", seg_vis_graph_type2_vertices.size());
    
    M_INFO3("Building Visibility Graph with VisiLibity.\n");
    //double eps = 0.00000000001;
    my_environment = new VisiLibity::Environment(a_poly);
    M_INFO3("Is it valid? %d.\n",my_environment->is_valid( _visi_epsilon ) );
    VisiLibity::Polygon* env_outer_poly = &((*my_environment)[0]);
    
    M_INFO3("Processing vertices to compute visibility polygons.\n");
    for ( int i = 0; i < env_outer_poly->n() ; i++ ) {
        this->process_vertex( (*env_outer_poly)[i] );
    }
    
    M_INFO3("N vertices %d.\n",env_outer_poly->n());
    
    //// For every visiblity polygon
    for ( int ii = 0; ii < env_outer_poly->n() ; ii++ ) {
        M_INFO3("Computing visibility flags for vertex %d.\n",ii);
        // 1) find the min and max indices 
        
        KERNEL::Point_2 
            v( this->get_visi_vertex(ii).x(), this->get_visi_vertex(ii).y() );
        
        M_INFO3("Visi poly %d has area %f \n",ii,visi_polies[ii].area());
        //// area > 0 => vertices listed ccw, 
        //// area < 0 => cw 
        //// need ccw for the method below to work
        this->process_visibility_polygon( ii, v, visi_polies[ii] );   
        //if ( ii == 3 ) 
        //    return;
    }
    
    M_INFO3("Finished with all visi polies\n");
    M_INFO3("Connecting reflexive vertices that are neighbors\n");
    Segment_Visibility_Graph::mygraph_t* g;
    Segment_Visibility_Graph::vertex v1,v2;
    g = this->seg_vis_graph->g;
    for ( int i = 0; i < this->seg_vis_graph_type2_vertices.size(); i++ ) {
        int next_i = i+1;
        if ( next_i == this->seg_vis_graph_type2_vertices.size())
            next_i = 0;
        v1 = this->seg_vis_graph_type2_vertices[next_i];
        v2 = this->seg_vis_graph_type2_vertices[next_i];
        if ( (*g)[v1].reflexive && (*g)[v2].reflexive ) 
        {
            // add an edge between the two
            double dis = 
                pow( (*g)[v1].p_x-(*g)[v2].p_x, 2 )
              + pow( (*g)[v1].p_y-(*g)[v2].p_y, 2 );
            dis = sqrt(dis);
            this->add_edge_to_visibility_graph(i, 2, next_i,2, dis);
        }
    }
}

/**
 * v_index corresponds ot vertex v
 */

void
Polygon_Environment::process_visibility_polygon(
    int v_index,
    KERNEL::Point_2 v, 
    VisiLibity::Visibility_Polygon &v_poly)
{
    
    KERNEL::Point_2 v1 = Point_2_from_poly_vertex(v_poly[0]);
    std::cout << "v1 (" << v1.x() << "," << v1.y() << ") ";
    std::cout << std::endl;
    
    int v_index_in_visi = -1;
    for ( int i = 0; i < v_poly.n() ; i++) 
    {
        KERNEL::Point_2 v_i( v_poly[i].x(), v_poly[i].y() );
        if ( v_i == v ) {
            v_index_in_visi = i;
        }
        std::cout << "v_" << i << " (" << v_i.x() << "," << v_i.y() << ") ";
        std::cout << std::endl;
    }
    std::cout << " v is v_" << v_index_in_visi << std::endl;
    std::cout << std::endl;
    
    M_INFO3("Check whether we have a reflexive vertex \n");
    bool v_is_reflexive = this->vertex_is_reflexive( v_index_in_visi, v_poly );
    (*(this->seg_vis_graph->g))
        [this->seg_vis_graph_type2_vertices[v_index]].reflexive = v_is_reflexive;
    
    M_INFO3("Going through visibility polygon for vertex v %d\n", v_index);
    for ( int i = 0; i < v_poly.n(); i++) 
    {
        M_INFO1("    v_i: vertex %d of visi poly\n",i);
        
        int i_next = i+1;
        if ( i == v_poly.n() - 1 )
            i_next = 0;
        
        if ( v_index_in_visi == i || v_index_in_visi == i_next ) 
        {
            M_INFO2(" vertex is startpoint or endpoint  ");
            continue;
        }
        
        M_INFO1("Finding the segment index for visi poly segment\n");
        KERNEL::Point_2 v_i( v_poly[i].x(), v_poly[i].y() );
        KERNEL::Point_2 v_next( v_poly[i_next].x(), v_poly[i_next].y() );
        int endpoint_segment_index = this->is_endpoint(v_i);
        int endpoint_segment_index_next = this->is_endpoint(v_next);
        M_INFO3("%d %d \n",endpoint_segment_index,endpoint_segment_index_next);
        int v_i_index;
        if ( endpoint_segment_index == -1  )
        {
            v_i_index = get_segment_index_for_point(v_poly[i]);
        } else {
            v_i_index = endpoint_segment_index;
        }
        if ( v_i_index < 0 )
            return;
        M_INFO1("... found segment index %d \n",v_i_index);
        
        M_INFO1("Finding closest point from v to segment");
        KERNEL::Segment_2 s(v_i,v_next);
        KERNEL::Point_2 closest_p;
        double dis = this->shortest_distance_between( s, v, closest_p );
        M_INFO2("Closest point at distance %f  \n",dis);
        std::cout << "Closest point is " << closest_p << std::endl;
        
        M_INFO1("Adding edges to seg visi graph...\n");
        
        this->add_edge_to_visibility_graph(v_index, 1, v_i_index,1, dis,
            CGAL::to_double(closest_p.x()),CGAL::to_double(closest_p.y()));
        
        if ( v_is_reflexive )
        {
            this->add_edge_to_visibility_graph(v_index, 2, v_i_index, 1, dis,
                CGAL::to_double(closest_p.x()), CGAL::to_double(closest_p.y()));
            bool v_i_is_reflexive = this->vertex_is_reflexive(i, v_poly );
            if ( v_i_is_reflexive ) {
                this->add_edge_to_visibility_graph(v_index, 2, v_i_index, 2, dis);
            }
        }
        
    }
    M_INFO2("Done with adding edges to seg visi graph for this vertex\n");
}

void
Polygon_Environment::add_edge_to_visibility_graph
    ( int i,  int type_i, int j, int type_j, double d, double x, double y )
{
    Segment_Visibility_Graph::vertex v, w;
    if ( type_i == 1 )
        v = this->seg_vis_graph_type1_vertices[i];
    else
        v = this->seg_vis_graph_type2_vertices[i];
    if ( type_j == 1 )
        w = this->seg_vis_graph_type1_vertices[j];
    else
        w = this->seg_vis_graph_type2_vertices[j];
    seg_vis_graph->add_edge(v, w, x, y, d);
}

int
Polygon_Environment::is_endpoint( KERNEL::Point_2 p )
{
    Polygon::Vertex_iterator it = master_polygon->vertices_begin();
    int i=0;
    double eps = this->_visi_epsilon*this->_visi_epsilon;
    for ( ; it != master_polygon->vertices_end(); it++) 
    {
        double d = CGAL::to_double(CGAL::squared_distance( *it, p ));
        if ( d < eps ) { return i; }
        i++;
    }
    return -1;
}

bool
Polygon_Environment::vertex_is_reflexive( 
    int i, VisiLibity::Visibility_Polygon& v_poly ) 
{
    bool is_reflexive = false;
    KERNEL::Point_2 v( v_poly[i].x(), v_poly[i].y() );
    int v_neigh1_i,v_neigh2_i;
    if ( i == 0 ) v_neigh1_i = v_poly.n()-1;
    else  v_neigh1_i = i-1;
    if ( i == v_poly.n() ) v_neigh2_i = 0;
    else  v_neigh2_i = i+1;
    KERNEL::Point_2 v_neigh1( v_poly[v_neigh1_i].x(), v_poly[v_neigh1_i].y() );
    KERNEL::Point_2 v_neigh2( v_poly[v_neigh2_i].x(), v_poly[v_neigh2_i].y() );
    KERNEL::Orientation orientation_at_v 
        = CGAL::orientation(v_neigh1,v,v_neigh2);
    if ( orientation_at_v == CGAL::LEFT_TURN ) {
        is_reflexive = false;
    } else if (orientation_at_v == CGAL::COLLINEAR ) {
        is_reflexive = false;
    } else {
        is_reflexive = true;
    }
    return is_reflexive;
}

// bool
// Polygon_Environment::vertex_is_reflexive(
//     KERNEL::Point_2 p1, KERNEL::Point_2 p2, KERNEL::Point_2 p3)
// {
//     bool is_reflexive = false;
//     KERNEL::Orientation orientation_at_v
//         = CGAL::orientation(p1,p2,p3);
//     if ( orientation_at_v == CGAL::LEFT_TURN ) {
//         is_reflexive = false;
//     } else if (orientation_at_v == CGAL::COLLINEAR ) {
//         is_reflexive = false;
//     } else {
//         is_reflexive = true;
//     }
//     return is_reflexive;
// }

//double xline = v_poly[i].x() - CGAL::to_double(v.x());
//double yline = v_poly[i].y() - CGAL::to_double(v.y());
//double xline_next = v_poly[i_next].x() - CGAL::to_double(v.x());
//double yline_next = v_poly[i_next].y() - CGAL::to_double(v.y());
//double a1 = norm_angle(atan2(yline,xline));
//double a2 = norm_angle(atan2(yline_next,xline_next));
//M_INFO3("angles %f and %f \n", a1,a2);
// TODO: is there is a CGAL way to do this? 
// Answer: not unless we use the original Kernel points
// fabs(diff_angles(a1,a2)) < _visi_epsilon
    

double
Polygon_Environment::shortest_distance_between( KERNEL::Segment_2 s, KERNEL::Point_2 p,
 KERNEL::Point_2& closest_point )
{
    
    CGAL::squared_distance( s, p);
    
    KERNEL::Point_2 l1Points[] = { s.source(), s.target() };
    KERNEL::Point_2 l2Points[] = { p };
    
    Polytope_distance pd(l1Points, l1Points+2, l2Points, l2Points+1);
    assert (pd.is_valid());
    
    Polytope_distance::Coordinate_iterator coord_it;
    KERNEL::RT x,y,w;
    coord_it = pd.realizing_point_p_coordinates_begin();
    x = *(coord_it++);
    y = *(coord_it++);
    w = (*coord_it);
    Point p1(x,y,w);
    
    closest_point = p1;
    
    coord_it = pd.realizing_point_q_coordinates_begin();
    x = *(coord_it++);
    y = *(coord_it++);
    w = (*coord_it);
    Point p2(x,y,w);
    
    return CGAL::to_double(CGAL::squared_distance( s, p));
}

KERNEL::Point_2 
Polygon_Environment::Point_2_from_poly_vertex( VisiLibity::Point& p )
{
    return KERNEL::Point_2(p.x(),p.y());
}

int 
Polygon_Environment::get_segment_index_for_point( VisiLibity::Point p ) 
{
    // this is a vertex index - starting at 0, for segment [vertex_0,vertex_1]
    return p.index_of_projection_onto_boundary_of( *my_environment );
}


void Polygon_Environment::process_vertex(VisiLibity::Point &p)
{
    //M_INFO3("Processing vertex\n");
    p.snap_to_boundary_of(*my_environment,_visi_epsilon);
    p.snap_to_vertices_of(*my_environment,_visi_epsilon);
    VisiLibity::Visibility_Polygon vis_poly = 
        VisiLibity::Visibility_Polygon(p,*my_environment, _visi_epsilon); 
    visi_polies.push_back(vis_poly);    
}

VisiLibity::Point Polygon_Environment::get_visi_vertex(int i)
{
    return (*my_environment)[0][i];
}

void 
    Polygon_Environment::get_visibility_polygon(Segment s)
{
    
}

double 
Polygon_Environment::shortest_path_distance_between(double x1,double y1,double x2, double y2) {
    VisiLibity::Polyline my_shortest_path;
    VisiLibity::Point start(x1,y1);
    VisiLibity::Point finish(x2,y2);
    my_shortest_path = my_environment->shortest_path(start, finish, _epsilon);
    return my_shortest_path.length();
}
    
void
Polygon_Environment::compute_to_master_distances(int goal) {
    
    //compute distance_mat2 values towards goal
    int min_i;
    unsigned int counter = 0;
    bool have_unreached = true;
    bool* done = new bool[this->size()];
    for (unsigned int i = 0; i < this->size(); i++ ) {
        done[i] = false;
    }
    M_INFO1(" Computing master distances with goal polygon %d",goal);
    while ( have_unreached && counter < this->size()+1 ) {
        counter++;
        double min_d = -2;
        for ( int i = 0; i < ((int) this->size());i++) {
            if ( (get_distance2(i,goal) < min_d || min_d == -2 )
                && !done[i]
                && get_distance2(i,goal) != -1 
                && get_distance2(i,goal) != 0
                && i != goal ) {
                min_i = i;
                min_d = get_distance2(i,goal);
            }
            // found a new minimum, adjust neigbhors
        }
        done[min_i] = true;
        if ( min_d == -2 ) {
            have_unreached = false;
        } else {
            //set_distance2(min_i,goal,0);
            if ( DEBUG_POLYGONENVIRONMENT >=3 ) {
                std::cout << " new min " << min_i << " " << min_d << std::endl;
            }
            // can any of the others connect to the goal faster through me?
            for ( int i = 0; i < ((int) this->size());i++) {
                if ( min_i == i || i == goal ) 
                    continue;
                if ( get_distance(min_i,i) != -1 ) {
                    // can connect to this neighbr
                    double hop_d = min_d + get_distance(i,min_i);
                    if ( hop_d < get_distance2(i,goal)
                      || get_distance2(i,goal) == -1 ) {
                        set_distance2(i,goal,hop_d);
                        swap_segment(i,goal,min_i);
                        if ( DEBUG_POLYGONENVIRONMENT >=3 ) {
                            std::cout << " found better hop to " << i 
                                << " at " << hop_d << std::endl;
                        }
                    }
                }
            }
        }
    }
    delete[] done;
}

inline
double Polygon_Environment::get_distance(int i, int j) {
    if ( i < j ) {
        return distance_mat[j][i];
    } else {
        return distance_mat[i][j];
    }
}

inline
void Polygon_Environment::set_distance(int i, int j, double d) {
    if ( i < j ) {
        distance_mat[j][i] = d;
    } else {
        distance_mat[i][j] = d;
    }
}


inline
double Polygon_Environment::get_distance2(int i, int j) {
    if ( i < j ) {
        return distance_mat2[j][i];
    } else {
        return distance_mat2[i][j];
    }
}

inline
void Polygon_Environment::set_distance2(int i, int j, double d) {
    if ( i < j ) {
        distance_mat2[j][i] = d;
    } else {
        distance_mat2[i][j] = d;
    }
}

inline
Segment Polygon_Environment::get_segment(int i, int j, int k) {
    if ( i < j ) {
        return segment_mat[j][i][k];
    } else {
        return segment_mat[i][j][k];
    }
}

inline
void Polygon_Environment::set_segment(int i, int j, int k, Segment s) {
    if ( i < j ) {
        segment_mat[j][i][k] = s;
    } else {
        segment_mat[i][j][k] = s;
    }
}

inline
void Polygon_Environment::swap_segment( int i, int g, int min_i ) {
    // instead of with the goal, connect the polygon with 
    // the master direcltly pretending min_i is already in the master
    set_segment(i, g, 0, get_segment(i,min_i,0));
    set_segment(i, g, 1, get_segment(i,min_i,1));
    set_segment(i, g, 2, get_segment(i,min_i,2));
}

void Polygon_Environment::add_to_master(Polygon* poly, int poly_id, int goal) {
    if ( master_polygon->size() == 0 ) {
        M_INFO1(" Adding first polygon to Master Polygon.\n");
        master_polygon->insert( master_polygon->vertices_begin(), 
            poly->vertices_begin(), poly->vertices_end());
        if( master_polygon->is_clockwise_oriented() ) { 
            master_polygon->reverse_orientation(); 
        }
    }
    else {
        double jiggle_factor = 0.2; // TODO: introduce parameter
        bool is_simple = poly->is_simple();
        M_INFO2("Polygon is poly->is_simple()? %d \n",is_simple);
        if ( !is_simple ) { return; }
        if( poly->is_clockwise_oriented() ) { poly->reverse_orientation(); }
        
        //poly->test_bounded_side(master_polygon);
        
        if ( poly->test_bounded_side(master_polygon) ) {
            M_WARN(" Polygon is not inside and not added to master.\n");
            return;
        }
        
        if ( DEBUG_POLYGONENVIRONMENT >=1 ) {
            std::cout << " add_to_m " << poly_id << " goal " << goal << std::endl;
        }
        M_INFO1(" Adding another polygon (id=%d) to Master Polygon.\n",goal);
        
        if ( DEBUG_POLYGONENVIRONMENT >=4 )
            master_polygon->print();
        if ( DEBUG_POLYGONENVIRONMENT >=4 )
            poly->print();
        
        Segment master_s, poly_s, shortest_s;
        Polygon::Vertex_iterator it, poly_i,master_i;
        
        // find out if source or target of short segment are on poly or master
        shortest_s = get_segment(goal,poly_id,0);
        it = poly->find_in( shortest_s.target() );
        if ( it == poly->vertices_end() ) {
            if ( DEBUG_POLYGONENVIRONMENT >=2 ) {
                std::cout << shortest_s.target() 
                    << " shortest target not on poly " << std::endl;
            }
            shortest_s = shortest_s.opposite();
        }
        
        // shortest_s is now properly oriented from master to poly
        
        // this will give us the pointer to the target of a segment
        // that contains or is very close to the point
        master_i = master_polygon->find_in( shortest_s.source(), master_s );
        if ( master_i == master_polygon->vertices_end() ) {
            M_WARN(" WARNING: polygon changed. now using closest point.\n");
            master_i = master_polygon->find_in_closest( shortest_s.source(), 
                2*jiggle_factor*jiggle_factor, master_s );
            if ( master_i == master_polygon->vertices_end() ) {
                M_ERR(" CRITICAL ERROR: no segment on master polygon found.\n");
                return;
            }
        }
        poly_i = poly->find_in( shortest_s.target(), poly_s );
        
        if ( DEBUG_POLYGONENVIRONMENT >=2 ) {
            std::cout << " connections between polygons " << std::endl;
            std::cout << " shortest_s " << shortest_s << std::endl;
            std::cout << " poly_s " << poly_s << std::endl;
            std::cout << " poly_i " << *poly_i << std::endl;
            std::cout << " master_s " << master_s << std::endl;
            std::cout << " master_i " << *master_i << std::endl;
        }
        // just for kicks, recompute the shortest segment
        Segment shortest_s2 = polygonization::get_shortest_line(
            poly_s,master_s);
        if ( DEBUG_POLYGONENVIRONMENT >=2 ) {
            std::cout << " shortest_s2 " << shortest_s2 << std::endl;
        }
        
        // Points to be added along the segments
        Point p1,p2,p3,p4;

        Vector v( shortest_s );
        if ( DEBUG_POLYGONENVIRONMENT >=2 ) 
            std::cout << " original v=" << v << std::endl;
        v = v.perpendicular(CGAL::LEFT_TURN);
        if ( DEBUG_POLYGONENVIRONMENT >=2 )
            std::cout << " turned v=" << v << std::endl;
        double d = sqrt( CGAL::to_double(v.squared_length()) );
        if ( d == 0 ) {
            M_ERR("CRITICAL ERROR: D=0\n");
        }
        v = v / d;
        if ( DEBUG_POLYGONENVIRONMENT >=2 ) {
            std::cout << " normal. v=" << v << std::endl;
            std::cout << "Building connection points." << std::endl;
        }
        p1 = shortest_s.source(); 
        p2 = shortest_s.target();
        p3 = shortest_s.target() + jiggle_factor * v;
        p4 = shortest_s.source() + jiggle_factor * v;
        if ( DEBUG_POLYGONENVIRONMENT >=2 ) {
            std::cout << " p1 " << p1 << std::endl;
            std::cout << " p2 " << p2 << std::endl;
            std::cout << " p3 " << p3 << std::endl;
            std::cout << " p4 " << p4 << std::endl;
            std::cout << "Checking conditions." << std::endl;
        }
        bool do_not_add_p1 = false;
        bool do_not_add_p2 = false;
        if ( master_s.target() == shortest_s.source() ) {
            do_not_add_p1 = true;
        } else if ( master_s.source() == shortest_s.source() ) { 
            // move master_i to source
            if ( DEBUG_POLYGONENVIRONMENT >=3 ) {
                std::cout << " MOVED MASTER_I " << std::endl;
            }
            master_i = master_polygon->find( master_s.source() );
            do_not_add_p1 = true;
        } else {
            // need a correction of the master_i
            // TODO: why do we not need this correction?
            //std::cout << " CORRECTING master_i from " << *master_i
            //    << std::endl;
            //master_i = master_polygon->find_in( shortest_s.source() );
            //if ( master_i == master_polygon->vertices_end() ) {
            //    M_ERR("Could not correct master pointer. \n");
            //    return;
            //}
            //std::cout << " to " << *master_i << std::endl;
        }
        if ( poly_s.target() == shortest_s.target() ) {
            do_not_add_p2 = true;
        } else if ( poly_s.source() == shortest_s.target() ) {
            if ( DEBUG_POLYGONENVIRONMENT >=3 ) {
                std::cout << " MOVED POLY_I " << std::endl;
            }
            poly_i = poly->find( poly_s.source() );
            do_not_add_p2 = true;
        } else {
            if ( DEBUG_POLYGONENVIRONMENT >=3 ) {
                std::cout << " CORRECTING poly_i from " << *master_i
                    << std::endl;
            }
            poly_i = poly->find_in( shortest_s.target() );
            if ( DEBUG_POLYGONENVIRONMENT >=3 ) {
                std::cout << " to " << *poly_i << std::endl;
            }
        }
        
        if ( DEBUG_POLYGONENVIRONMENT >=3 ) {
            std::cout << " NEW master_i " << *master_i << std::endl;
            std::cout << " NEW poly_i " << *poly_i << std::endl;
        }
        
        it = master_i;
        if ( !do_not_add_p1 ) {
            if ( DEBUG_POLYGONENVIRONMENT >=3 ) {
                std::cout << " insert p1 " << p1 << " before " << *it << std::endl;
            }
            it = master_polygon->insert( it, p1 );
        }
        if ( !do_not_add_p2 ) {
            if ( DEBUG_POLYGONENVIRONMENT >=3 ) {
                std::cout << " insert p2 " << p2 << " before " << *it 
                    << std::endl;
            }
            it = master_polygon->insert( it, p2 );
        }
        
        if ( DEBUG_POLYGONENVIRONMENT >=2 ) {
            std::cout << " first way around " << std::endl;
        }
        Polygon::Vertex_iterator it2 = poly_i;
        while (it2 != poly->vertices_end()) {
            if ( DEBUG_POLYGONENVIRONMENT >=2 ) {
                std::cout << *it2 << " " << " before " << *it << " ";
            }
            it = master_polygon->insert( it, *it2);
            it2++;
        }
        if ( DEBUG_POLYGONENVIRONMENT >=2 ) {
            std::cout << std::endl << " other way around " << std::endl;
        }
        it2 = poly->vertices_begin();
        while (it2 != poly_i) {
            if ( DEBUG_POLYGONENVIRONMENT >=2 ) {
                std::cout << *it2 << " " << " before " << *it << " ";
            }
            it = master_polygon->insert( it, *it2);
            it2++;
        }
        if ( DEBUG_POLYGONENVIRONMENT >=2 ) {
            std::cout << std::endl;
            std::cout << " insert p3 " << p3 << " before " << *it << std::endl;
        }
        it = master_polygon->insert( it, p3 );
        
        if ( DEBUG_POLYGONENVIRONMENT >=2 ) {
            std::cout << " insert p4 " << p4 << " before " << *it << std::endl;
        }
        it = master_polygon->insert( it, p4 );
        
        artificial_points.push_back(p1);
        artificial_points.push_back(p2);
        artificial_points.push_back(p3);
        artificial_points.push_back(p4);
        
        //it = master_polygon->vertices_begin();
        //Point lastPoint = *it;
        //it++;
        //M_INFO2("  Removing redundant vertices.\n");
        //while ( it != master_polygon->vertices_end() ) {
        //    if ( *it == lastPoint ) {
        //        M_INFO2("  Redundant vertex removed.");
        //        std::cout << *it << std::endl;
        //        master_polygon->erase(it);
        //        it++;
        //    } else {
        //        lastPoint = *it;
        //        it++;
        //    }
        //}
    }
    M_INFO1("  Added polygon to Master Polygon successfully.\n");
    M_INFO1("  **** MASTER IS SIMPLE? %d \n",master_polygon->is_simple());
}

bool Polygon_Environment::is_artificial( Point p ) {
    if ( 
        find( artificial_points.begin(), artificial_points.end(), p)
        != artificial_points.end() ) {
            return true;
    } 
    return false;
}

int Polygon_Environment::is_artificial2( Point p ) {
    std::vector<Point>::iterator it;
    it = find( artificial_points.begin(), artificial_points.end(), p);
    if (  it != artificial_points.end() ) {    
        return it - artificial_points.begin();;
    } 
    return -1;
}

bool Polygon_Environment::is_same_articifial( int i, int j ) {
    int i_mod = floor( i / 4 );
    int j_mod = floor( j / 4 );
    if ( i_mod != j_mod ) {
        return false;
    } else {
        return true;
    }
}

bool Polygon_Environment::is_artificial( int i ) {
    if ( is_artificial( master_polygon->edge(i-1).target() )
      && is_artificial( master_polygon->edge(i-1).source() ) ) {
          return true;
    }
    return false;
}

bool Polygon_Environment::is_artificial( Segment s ) {
    if ( is_artificial(s.target() )
      && is_artificial(s.source() ) ) {
          return true;
    }
    return false;
}

/*
 * Figures out where and how the connection segments are located and connected
 *
 * Author: Andreas Kolling ( Thu Sep 13 12:52:51 CEST 2012 ) 
 */
bool Polygon_Environment::get_connections_segments( Segment& shortest_s, Segment& master_s, Segment&poly_s, Polygon::Vertex_iterator& poly_i, Polygon::Vertex_iterator& master_i, int poly_id, int goal, Polygon* poly) {
    
    // These segments are those used initially for the original
    // Master Polygon
    shortest_s = get_segment(goal,poly_id,0);
    Segment from_s = get_segment(goal,poly_id,1);
    Segment to_s = get_segment(goal,poly_id,2);
    if ( DEBUG_POLYGONENVIRONMENT >=2 ) {
        std::cout << " FROM " << from_s << std::endl;
        std::cout << " TO " << to_s << std::endl;
    }
    
    int dir; // source to target
    Polygon::Vertex_iterator it;
    if ( is_in_polygon( from_s, poly, it, dir ) ) {
        //M_INFO1(" from_s is in poly, source_to_target? %d.\n",dir);
        if ( dir == 1) {
            poly_s = from_s;
        } else if ( dir == 0) {
            poly_s = from_s.opposite();
        } else if ( dir == 2 ){
            // we are in polygon, but not sure whether segment is oriented
            // the right way
            // TODO: might not even need this with new above correction
        }
        poly_i = it; // it points to the segment target;
    }
    else if ( is_in_polygon( from_s, master_polygon, it, dir ) ) {
        //M_INFO1(" from_s is in master, source_to_target? %d.\n",dir);
        if ( dir == 1) {
            master_s = from_s;
        } else if ( dir == 0) {
            master_s = from_s.opposite();
        }
        master_i = it;
    }
    if ( is_in_polygon( to_s, poly, it, dir ) ) {
        //M_INFO1(" to_s is in poly, source_to_target? %d.\n",dir);
        if ( dir == 1) {
            poly_s = to_s;
        } else if ( dir == 0) {
            poly_s = to_s.opposite();
        }
        poly_i = it;
    }
    else if ( is_in_polygon( to_s, master_polygon, it, dir ) ) {
        //M_INFO1(" to_s is in master, source_to_target? %d.\n",dir);
        if ( dir == 1 ) {
            master_s = to_s;
        } else if ( dir == 0) {
            master_s = to_s.opposite();
        }
        master_i = it;
    }
    
    // swap target,source of shortest_s so that source is on master
    if ( !master_s.has_on( shortest_s.source() ) ) {
        shortest_s = shortest_s.opposite();
        if ( !master_s.has_on( shortest_s.source() ) ) {
            M_ERR("CRITICAL ERROR:");
            M_ERR("MSG: source+target of short_s not on master_\n");
            return false;
        }
    }
    
    if ( master_s.target() != *master_i ) {
        M_ERR("CRITICAL ERROR:");
        M_ERR("MSG: master_s.target() != *master_i - not pointing to end \n");
        return false;
    }
    
    if ( poly_s.target() != *poly_i ) {
        M_ERR("CRITICAL ERROR:");
        M_ERR("MSG: poly_s.source() != *poly_i - not pointing to end  \n");
        return false;
    }
    
    return true;
    
}

bool Polygon_Environment::is_in_polygon( Segment s, Polygon* poly, Polygon::Vertex_iterator& it, int& source_to_target ) {
    
    // TODO: there is a problem here with when new polygons
    // have been added to the master, the target may be further
    // away even if it is source to target
    
    it = poly->find( s.target() );
    if ( it == poly->vertices_end() ) {
        return false;
    }
    
    Polygon::Vertex_iterator it2 = it;
    
    Point a,b;
    it2--;
    if ( it2 != poly->vertices_end() ) {
        a = *it2;
    } else {
        //std::cout << " 'a' cannot be accessed" << std::endl;
        a = poly->vertex(poly->size() - 1);
    }
    it2 = it; it2++;
    if ( it2 != poly->vertices_end() )
        b = *it2;
    else {
        //std::cout << " 'b' cannot be accessed" << std::endl;
        b = poly->vertex(0);
    }
    
    //std::cout << " s=" << s << " a=" << a << " b=" << b << std::endl;
    if ( s.has_on(a) ) {
        M_INFO3(" Source to target, all is well.\n");
        source_to_target = 1;
    } else if ( s.has_on(b) ) {
        M_INFO3(" Target to source, flipping pointer.\n");
        source_to_target = 0;
        // will return pointer to source rather than target;
        it = poly->find( s.source() );
    } else {
        M_WARN(" neither forward nor backward point is on segment.");
        if ( DEBUG_POLYGONENVIRONMENT >=2 ) {
            std::cout << " s=" << s << " a=" << a << " b=" << b << std::endl;
        }
        source_to_target = 2;
        // REDO search for proper pointer
    }
    
    return true;
}

// comparison, not case sensitive.
bool pair_comparison (std::pair<int,double> a, std::pair<int,double> b)
{
    return a.second > b.second;
}

bool pair_comparison_inv (std::pair<int,double> a, std::pair<int,double> b)
{
    return a.second < b.second;
}

void Polygon_Environment::extract_polygon( Alpha_shape::vertex_handle v )
{
    Polygon new_poly;
    this->push_back( new_poly );
    this->n_polygons++;
    visit_alpha_vertex( v );
    
    simplify_polygon(this->size()-1,_epsilon);
    
    this->back().push_back( this->back().vertex(0) );
    this->back().erase( this->back().vertices_begin() );
    this->back().push_back( this->back().vertex(0) );
    this->back().erase( this->back().vertices_begin() );

    simplify_polygon(this->size()-1,_epsilon);
    
    if ( DEBUG_POLYGONENVIRONMENT >=2 ) {
        std::cout << " POLY AREA " << CGAL::to_double(this->back().area()) 
            << std::endl;
    }
    if ( fabs(CGAL::to_double(this->back().area())) <= Params::g_min_polygon_area ) {
        this->pop_back();
        --(this->n_polygons);
    }
}

void Polygon_Environment::visit_alpha_vertex( Alpha_shape::vertex_handle v_start )
{
    Alpha_shape::vertex_handle v = v_start, v1,v2,v_neigh;
    Alpha_shape::vertex_handle v_reset;
    Alpha_shape::vertex_handle v_prev;
    // find all other segments that connect to v
    Alpha_shape::edge_circulator e_circ1,e_circ2;
    
    (*this)[this->n_polygons-1].push_back( v_start->point() );
    v_start->info() = true;
    
    int polygon_edges = 0;
    bool found_next = true;
    bool reset = false;
    while ( found_next == true ) 
    {
        bool found = false;
        int reg_edges = A->regular_edges( v );
        if ( polygon_edges > 0 && reg_edges != 2 ) {
            found = find_next_regular_vertex( v, v_prev, v_neigh );
            if ( reset == true )
                v_reset->info() = false; // can be visited again
            v_reset = v;
            reset = true;
        }
        else {
            found = find_next_regular_vertex( v, v_neigh );
            if ( reset == true ) 
            {
                v_reset->info() = false; // can be visited again
                reset = false;
            }
        }
        
        if ( found == true ) {
            v_prev = v; v = v_neigh;
            (*this)[this->n_polygons-1].push_back( v->point() );
            ++polygon_edges;
            v->info() = true;
        }
        else {
            // no more traversable alpha edges!!!
            if ( !find_last( v, v_start ) )
                ++open_polygons;
            // need to close polygon
            found_next = false;
        }
    }
    
    if ( polygon_edges == 0 ) {
        --(this->n_polygons);
        this->pop_back();
        return;
    }
}

bool Polygon_Environment::find_next_regular_vertex( 
   Alpha_shape::vertex_handle v, 
   Alpha_shape::vertex_handle& vr ) {
    Alpha_shape::vertex_handle v_neigh;
    Alpha_shape::edge_circulator e_circ1,e_circ2;
    e_circ1 = A->incident_edges( v );
    e_circ2 = e_circ1;
    do {
        if ( A->is_regular( e_circ1 ) ) {
            v_neigh = A->get_neighbor_vertex( e_circ1, v );
            if ( v_neigh->info() == false ) { // not used 
                vr = v_neigh;
                return true;
            }
        }
        ++e_circ1;
    }
    while ( e_circ1 != e_circ2 );
    return false;
}

bool
Polygon_Environment::find_next_regular_vertex( Alpha_shape::vertex_handle v,
                                              Alpha_shape::vertex_handle v_prev,
                                              Alpha_shape::vertex_handle& vr ) 
{
    Alpha_shape::vertex_handle v_neigh;
    Alpha_shape::edge_circulator e_circ1,e_circ2;
    e_circ1 = A->incident_edges( v );
    e_circ2 = e_circ1;
    do
    {
        if( A->is_regular( e_circ1 ) )
        {
            v_neigh = A->get_neighbor_vertex( e_circ1, v );
            if ( v_neigh == v_prev )
            {
                int limit = 0;
                while( limit < 100 )
                {
                    ++e_circ1;
                    if ( A->is_regular( e_circ1 ) )
                    {
                        v_neigh = A->get_neighbor_vertex( e_circ1, v );
                        if ( !(v_neigh->info()) )
                        {
                            vr = v_neigh;
                            return true;
                        }
                    }
                    ++limit;
                } 
                M_ERR(" ERROR did not find free next vertex\n");
                return false;
            }
        }
        ++e_circ1;
    }
    while ( e_circ1 != e_circ2 );
    return false;
}

bool
Polygon_Environment::find_last( Alpha_shape::vertex_handle v, 
                                Alpha_shape::vertex_handle v_start ) {
    Alpha_shape::vertex_handle v_neigh;
    Alpha_shape::edge_circulator e_circ1,e_circ2;
    e_circ1 = A->incident_edges( v );
    e_circ2 = e_circ1;
    do {
        if( A->is_regular( e_circ1 ) ) {
            v_neigh = A->get_neighbor_vertex( e_circ1, v );
            if ( v_neigh == v_start )
                return true;
        }
        ++e_circ1;
    }
    while ( e_circ1 != e_circ2 );
    return false;
}

void Polygon_Environment::simplify( double epsilon ) {
    // Simplify Polygons
    M_INFO1("-SimplyEpsilon=%f \n",epsilon);
    for ( int i = 0; i < this->n_polygons; ++i ) 
        simplify_polygon( i, epsilon );
}

void Polygon_Environment::simplify_polygon( int poly, double epsilon ) {
    std::vector<int> final_indices;
    DPLineSimplification::simplify( (*this)[ poly ], final_indices, epsilon );
    sort( final_indices.begin(), final_indices.end() );
    Polygon simplified_poly;
    for( int i = 0; i < int(final_indices.size()) ; ++i ) 
        simplified_poly.push_back( (*this)[ poly ][final_indices[i]] );
    
    (*this)[poly] = simplified_poly;
}

void Polygon_Environment::make_simply_connected() {
    
}

}