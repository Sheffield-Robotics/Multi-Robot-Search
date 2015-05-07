#include "polygonization/polygon_environment.h"
#include "polygonization/voronoi_diagram.h"
#include "utilities/paramfile.h"

#define DEBUG_POLYGONENVIRONMENT 0

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
    M_INFO2("Size of Alpha shape vertex list %d\n",vertex_list.size());

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
    M_INFO2("area_list number of polygons %d",area_list.size());
    int larget_poly_i = list_i->first;
    if ( area_list.size() > 1 ) {
        M_INFO2("Erasing largest polygon (faulty/redundant outside border).\n");
        this->erase(this->begin()+larget_poly_i);
    }
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
    
    if ( this->size() == 1 ) {
        master_polygon = &((*this)[0]);
    } else {
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
    }
    
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
    M_INFO1(" converting master polygon to a visilibity polygon \n");
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
            a_vertex(i,1, (next_x+poi.x())/2,(next_y+poi.y())/2, 
            poi.x(), poi.y(), next_x, next_y);
        v_desc = seg_vis_graph->add_vertex( a_vertex );
        seg_vis_graph_type1_vertices.push_back(v_desc);
        Segment_Visibility_Graph::seg_vertex 
            b_vertex(i,2, poi.x(), poi.y() );
        v_desc = seg_vis_graph->add_vertex( b_vertex );
        seg_vis_graph_type2_vertices.push_back(v_desc);
        i++;
    }
    M_INFO1("Added %d vertices.\n", seg_vis_graph_type2_vertices.size());
    int n_segs = seg_vis_graph_type2_vertices.size();
     Segment_Visibility_Graph::seg_vertex 
         a_special_vertex(n_segs,2, 0,0,0,0,0,0);
    special_v = seg_vis_graph->add_vertex( a_special_vertex );
    
    std::vector<KERNEL::Segment_2> dummy(n_segs+1);
    seg_to_seg_segments 
        = new std::vector< std::vector<KERNEL::Segment_2> >(n_segs+1,dummy);
    std::vector<bool > dummy2(n_segs+1);
    seg_to_seg_visible
        = new std::vector< std::vector<bool> >(n_segs+1,dummy2);
    std::vector<double > dummy3(n_segs+1);
    seg_to_seg_distance
        = new std::vector< std::vector<double> >(n_segs+1,dummy3);
    for (int i = 0; i < n_segs+1; i++ ) {
        for (int j = 0; j < n_segs+1; j++ ) {
            (*seg_to_seg_visible)[i][j] = false;
            (*seg_to_seg_distance)[i][j] = 0;
        }
    }
    
    std::vector < bool > c_dum1(n_segs);
    std::vector < double >  c_dum2(n_segs);
    std::vector < std::list<KERNEL::Segment_2> > c_dum3(n_segs);
    get_shortest_path_cache_filled 
        = new std::vector< std::vector < bool > >(n_segs,c_dum1);
    get_shortest_path_distance_cache 
        = new std::vector< std::vector < double > >(n_segs,c_dum2);
    get_shortest_path_cache
        = new std::vector< std::vector < std::list<KERNEL::Segment_2> > >(n_segs,c_dum3);
    for (int i = 0; i < n_segs; i++ ) {
        for (int j = 0; j < n_segs; j++ ) {
            (*get_shortest_path_cache_filled)[i][j] = false;
        }
    }
    
    M_INFO1("Building Visibility Graph with VisiLibity.\n");
    my_environment = new VisiLibity::Environment(a_poly);
    M_INFO1("Is it valid? %d.\n",my_environment->is_valid( _visi_epsilon ) );
    VisiLibity::Polygon* env_outer_poly = &((*my_environment)[0]);
    
    M_INFO1("Processing vertices to compute visibility polygons.\n");
    for ( int i = 0; i < env_outer_poly->n() ; i++ ) {
        this->compute_visibility_polygon_at_vertex( (*env_outer_poly)[i] );
    }
    M_INFO1("N vertices %d.\n",env_outer_poly->n());
    
    //// For every visiblity polygon
    M_INFO1("Going through visibility polygons for each vertex.\n");
    for ( int ii = 0; ii < env_outer_poly->n() ; ii++ ) {
        //M_INFO2("Computing visibility flags for vertex %d.\n",ii);
        //M_INFO2("Visi poly %d has area %f \n",ii,visi_polies[ii].area());
        //// area > 0 => vertices listed ccw, 
        //// area < 0 => cw 
        //// need ccw for the method below to work
        
        KERNEL::Point_2 
            v( this->get_visi_vertex(ii).x(), this->get_visi_vertex(ii).y() );
        this->process_visibility_polygon( ii, v, visi_polies[ii] );   
    }
    
    M_INFO1("Finished with all visibility polygons\n");
    M_INFO1("Additionally connecting reflexive vertices that are neighbors\n");
    Segment_Visibility_Graph::mygraph_t* g;
    Segment_Visibility_Graph::vertex v1,v2;
    g = this->seg_vis_graph->g;
    for ( int i = 0; i < this->seg_vis_graph_type2_vertices.size(); i++ ) {
        int next_i = i+1;
        if ( next_i == this->seg_vis_graph_type2_vertices.size())
            next_i = 0;
        v1 = this->seg_vis_graph_type2_vertices[i];
        v2 = this->seg_vis_graph_type2_vertices[next_i];
        if ( (*g)[v1].reflexive && (*g)[v2].reflexive ) 
        {
            // add an edge between the two
            double dis = 
                pow( (*g)[v1].p_x-(*g)[v2].p_x, 2 )
              + pow( (*g)[v1].p_y-(*g)[v2].p_y, 2 );
            dis = sqrt(dis);
            this->add_edge_to_visibility_graph(i, 2, next_i,2, dis, (*g)[v1].p_x,(*g)[v1].p_y,(*g)[v2].p_x,(*g)[v2].p_y );
        }
    }
    M_INFO1("\n\n");
}

bool
Polygon_Environment::check_path_cache(int i, int j) {
    if ( i > j ) { std::swap(i,j); };
    if ( (*get_shortest_path_cache_filled)[i][j] ) {
        return true;
    } else {
        return false;
    }
}

std::list<KERNEL::Segment_2>
Polygon_Environment::get_path_cache(int i, int j) {
    if ( i > j ) { std::swap(i,j); };
    if ( (*get_shortest_path_cache_filled)[i][j] ) {
        return (*get_shortest_path_cache)[i][j];
    } else {
        std::list< KERNEL::Segment_2> empty;
        return empty;
    }
}

double
Polygon_Environment::get_path_distance(int i, int j) {
    if ( i > j ) { std::swap(i,j); };
    if ( (*get_shortest_path_cache_filled)[i][j] ) {
        return (*get_shortest_path_distance_cache)[i][j];
    } else {
        return 0;
    }
}

void
Polygon_Environment::set_path_cache(int i, int j, std::list<KERNEL::Segment_2> l, double d)
{
    if ( i > j ) { std::swap(i,j); };
    if ( i < get_shortest_path_cache_filled->size() 
        && j < get_shortest_path_cache_filled->size() ) {
        (*get_shortest_path_cache_filled)[i][j] = true;   
        (*get_shortest_path_cache)[i][j] = l;   
        (*get_shortest_path_distance_cache)[i][j] = d;   
    }
    return;
}

KERNEL::Point_2 
    Polygon_Environment::get_point_of_segment_on_segment( KERNEL::Segment_2 s, int k, bool& success ) 
{
    double d;
    d = CGAL::to_double ( 
        CGAL::squared_distance(s.target(), 
        master_polygon->edge(k) ) );
    if ( DEBUG_POLYGON_ENVIRONMENT >= 4 ) {
        M_INFO3("get_point_of_segment_on_segment\n");
        std::cout << s 
            << " edge k " << k << "  " 
            << master_polygon->edge(k)  << std::endl;
    }
    if ( d < _epsilon ) {
        success = true;
        return s.target();
    }
    d = CGAL::to_double ( 
        CGAL::squared_distance(s.source(), 
        master_polygon->edge(k) ) );
    if ( d < _epsilon ) {
        success = true;
        return s.source();
    }
    success = false;
    return KERNEL::Point_2(0,0);
}

double 
Polygon_Environment::get_block_distance(int i, int j)
{
    if ( !index_bound_check(i) 
        || !index_bound_check(j) )
    {
        M_ERR("Index is out of bounds"); return -1;
    }
    double final_dist;
    get_shortest_path(i,j, final_dist);
    return final_dist;
}

bool
Polygon_Environment::is_necessary_split(int i, int j, int k)
{
    // splits are only necessary if the block cost between i,k and j,k
    // are not touching another obstacle
    return 
        is_necessary_block(i,k) && is_necessary_block(j,k);
    
    //fix_index(i);fix_index(j);fix_index(k);
    //i--;j--;k--; // indices for cost functions start at 1 but pol env at 0    
}

/*
* determines whether a block between i and j has to be considered
* i.e., whether it could ever originate from a previous split
*/
bool 
Polygon_Environment::is_necessary_block(int i, int j) 
{
    // determine which side of the choice set is contaminated 
    // and which is cleared
    int original_i = i, original_j = j;
    fix_index(i);fix_index(j); // fixes the index to be within range 
    i--;j--; // fixes the 1 to 0 address issue (choice sets start at 1, segment indices at 0)
    
    // same obstacle always has a 0 block
    if ( i == j ) 
        return true;
    
    // if the segments are directly visible, the block is necessary 
    if ( (*seg_to_seg_visible)[i][j] || (*seg_to_seg_visible)[j][i] ) {
        return true;
    }
    
    //blocks between neighbors are automatically necessary to consider
    if ( abs(j-i) == 1 
        || (i==0 && j==master_polygon->size()-1) 
        || (j==0 && i==master_polygon->size()-1) 
        ) 
    {
        return true;            
    }  
    
    //blocks that contain no contaminated obstacle indices are necessary!    
    double d;
    std::list<KERNEL::Segment_2> p = get_shortest_path(i,j,d);
    if ( p.size() <= 1 ) 
        return true;
    std::list<KERNEL::Segment_2>::iterator it = p.begin(), it_end=p.end();
    int n_non_zero_segments = 0;
    for ( ; it != it_end; it++ ) {
        int seg_index = 
            get_segment_index_for_point( 
                VisiLibity::Point( CGAL::to_double(it->target().x()),
                     CGAL::to_double(it->target().y()) )
                );
        // check if the segment index is contaminated
        bool seg_index_contaminated = false; 
        //called as is_necessary_block(original_i = i-1,original_i = i+k) )
        if ( original_i < seg_index && seg_index < original_j ) {
            seg_index_contaminated = true;
        } else if ( original_j != j && seg_index < j+1  ) {
            // j was fixed from above the index range to below i
            seg_index_contaminated = true;
        }
        
        if ( !seg_index_contaminated ) {
            if ( seg_index != i && seg_index != j ) {
                return true; 
            }
        } else { 
            if ( seg_index != i && seg_index != j ) {
                return false; 
            }
        }
        // if ( it->squared_length() > 0 ) {
        //     n_non_zero_segments++;
        //     if ( n_non_zero_segments > 2 && seg_index_contaminated ) {
        //         return false;
        //     }
        // }
    }

    return false;
}
    
int 
Polygon_Environment::get_block_cost(int i, int j, double r)
{
    fix_index(i);fix_index(j);
    i--;j--;
    if ( !index_bound_check(i) 
        || !index_bound_check(j) )
    {
        M_ERR("Index is out of bounds"); return -1;
    }
    double d = this->get_block_distance(i,j);
    int cost = int(ceil(d/ double(r)));
    std::cout << "get_block_cost " << i << "-" << j << " " << cost << std::endl;
    return cost;
}

bool Polygon_Environment::index_bound_check( int i ) {
    return ( 0 <= i && i < master_polygon->size() );
}

bool
Polygon_Environment::get_split_distances(int i, int j, int k, 
    double& d1, double& d2, double r)
{
    int split_point_index;
    shortest_split_costs(i,j,k,d1,d2,split_point_index);
    return true;
}


int
Polygon_Environment::get_split_cost(int i, int j, int k, double r)
{
    fix_index(i);fix_index(j);fix_index(k);
    i--;j--;k--; // indices for cost functions start at 1 but pol env at 0
    
    if ( !index_bound_check(i) || !index_bound_check(j) 
        || !index_bound_check(k) )
    {
        M_ERR("Index is out of bounds"); return -1;
    }
    double d1, d2;
    get_split_distances(i,j,k,d1,d2,r);
    int cost = int ( ceil(d1/r))+int(ceil(d2/r));
    if ( DEBUG_POLYGON_ENVIRONMENT >= 2 ) 
    {
        std::cout << "get_split_cost " << i << "-" << j << " " << k << " " 
            << cost << std::endl;
    }
    
    return cost;
}

std::list<KERNEL::Segment_2>
    Polygon_Environment::shortest_split_cost(int i, int j, int k, double& cost) 
{
    double cost1,cost2;
    int split_point_index;
    std::list<KERNEL::Segment_2> l = shortest_split_costs(i,j,k, cost1, cost2,split_point_index);
    cost = cost1 + cost2;
    return l;
}

bool
Polygon_Environment::is_sequence_split(int i, int j, int k)
{
    //  check if k is in the middle between i and j (or between j and i)
    if ( is_sequential(i,k) && is_sequential(j,k) ) {
        return true;
    }
    return false;
//    if ( get_next_index(i) == k && get_next_index(k) == j) {
//         return true;
//     } else if ( get_next_index(j) == k && get_next_index(k) == i) {
//         return true;
//     } else {
//         return false;
//     }
}

bool
Polygon_Environment::is_sequential(int i, int j)
{
    if ( get_next_index(i) == j || get_next_index(j) == i ) {
        return true;
    }
    return false;
}

/*
 * pick a point on segment k, between block line of i,k and j,k
 */
std::list<KERNEL::Segment_2>
Polygon_Environment::shortest_split_costs(
    int i, int j, int k, double& cost1, double& cost2, int& split_point_index) 
{
    if ( DEBUG_POLYGON_ENVIRONMENT >= 1 ) {
        M_INFO1("\n shortest_split_costs %d %d %d\n",i,j,k);
    }
    
    if ( is_sequential(i,k) && is_sequential(j,k) ) {
        if ( DEBUG_POLYGON_ENVIRONMENT >= 2 )
            M_INFO1("     all indices are sequential  \n");
        int smallest, largest;
        smallest = ( i < k ) ? i : j;
        largest = ( i < k ) ? j : i;
        KERNEL::Orientation o1 = 
                CGAL::orientation( 
                    master_polygon->vertex(smallest), 
                    master_polygon->vertex(k),
                    master_polygon->vertex(largest));
        KERNEL::Orientation o2 = 
                CGAL::orientation( 
                    master_polygon->vertex(k),
                    master_polygon->vertex(largest),
                    master_polygon->edge(largest).target());
        std::cout << o1 << " " << o2 << std::endl;
        if ( o1 != CGAL::RIGHT_TURN || o2 != CGAL::RIGHT_TURN ) {
            // we can skip the steps below and just return the length of edge k
            cost1 = sqrt( CGAL::to_double( master_polygon->edge(k).squared_length() ) );
            cost2 = 0;
            split_point_index = 0;
            std::list<KERNEL::Segment_2> return_this;
            return_this.push_back(  master_polygon->edge(k) );
            return return_this;
        }          
    } else if ( is_sequential(i,k) || is_sequential(j,k) ) {
        if ( DEBUG_POLYGON_ENVIRONMENT >= 2 )
            M_INFO1("     two indices are sequential  \n");
        
        // take the common endpoint of sequential segments/edges
        int seque_index = is_sequential(i,k) ? i : j;
        int other_index = (seque_index == i) ? j : i;
        KERNEL::Point_2 common_p;
        if ( seque_index < k ) {
            common_p = master_polygon->vertex(k);
        } else {
            common_p = master_polygon->vertex(seque_index);
        }
        double cost;
        std::list<KERNEL::Segment_2> return_this;
        return_this = get_shortest_path( common_p, other_index, k, cost );
        if ( i == seque_index ) {
            cost1 = 0; cost2 = cost; split_point_index = return_this.size()-1;
        } else {
            cost2 = 0; cost1 = cost; split_point_index = 0;
        }
        return return_this;
    }
    
    std::list<KERNEL::Segment_2> list_i_k, list_j_k;
    double final_d_i, final_d_j;
    list_i_k = get_shortest_path(i, k,final_d_i);
    list_j_k = get_shortest_path(j, k,final_d_j);
    //find point on k
    bool success = false;
    KERNEL::Point_2 point_on_k_from_i;
    point_on_k_from_i 
        = get_point_of_segment_on_segment( list_i_k.front(), k, success);
    if ( !success ) {
        point_on_k_from_i 
            = get_point_of_segment_on_segment( list_i_k.back(), k, success);
    }
    if (!success) 
        M_INFO3("ERROR - no point on segment found\n");
    KERNEL::Point_2 point_on_k_from_j;
    point_on_k_from_j 
        = get_point_of_segment_on_segment( list_j_k.front(), k, success);
    if ( !success ) {
        point_on_k_from_j 
            = get_point_of_segment_on_segment( list_j_k.back(), k, success);
    }
    if (!success) 
        M_INFO3("ERROR - no point on segment found\n");
    KERNEL::Vector_2 vec(point_on_k_from_i,point_on_k_from_j);
    if ( DEBUG_POLYGON_ENVIRONMENT >= 4 ) {
        std::cout << " point_on_k_from_i  " << point_on_k_from_i << std::endl;
        std::cout << " point_on_k_from_j  " << point_on_k_from_j << std::endl;
    }    
    // interpolate between point_on_k_from_i and point_on_k_from_j
    double step_size = 5;
    double search_distance = sqrt(CGAL::to_double( vec.squared_length()));
    int n_steps = int ( ceil( search_distance ) / step_size );
    if ( DEBUG_POLYGON_ENVIRONMENT >= 4 ) {
        M_INFO3("Number of interpolations steps %d\n",n_steps);
        M_INFO3("Interpolating\n");
    }
    
    if ( n_steps == 0 ) {
        // no search should be done. return what we already know
        cost1 = final_d_i; cost2 = final_d_j;
        split_point_index = list_i_k.size();
        list_i_k.insert(list_i_k.end(),list_j_k.begin(),list_j_k.end());
        return list_i_k;    
    }
    
    double final_d_i2_best = std::numeric_limits<double>::max();
    double final_d_j2_best = std::numeric_limits<double>::max();
    std::list<KERNEL::Segment_2> l1_best;
    
    char fname[200];
    sprintf( fname, "test_%d_%d_%d.txt",i,j,k);
    std::ofstream out_file;
    if ( DEBUG_POLYGON_ENVIRONMENT_PRINT == 1 ) {
        out_file.open(fname, std::ios_base::app);
        out_file << std::endl;
        out_file << i << " " << j << " " << k << " " << std::endl;
    }
    
    double angle_diff_prev = 0;
    if (n_steps == 0) 
        n_steps = 1;
    if ( DEBUG_POLYGON_ENVIRONMENT_PRINT == 1 ) 
        out_file << n_steps << std::endl;
    for ( int step =0; step <= n_steps-1; step++ ) 
    {
        KERNEL::Point_2 v;
        double r = (double(step)/double(n_steps-1));
        if ( n_steps == 1 || step == 0 ) {
            v = point_on_k_from_i + 0.0001 * vec;
        } else if ( step == n_steps-1 ){
            v = point_on_k_from_j - 0.0001 * vec;
        } else {
            v = point_on_k_from_i + r * vec;
        }
        if ( DEBUG_POLYGON_ENVIRONMENT >= 5 ) {
                M_INFO1(" Interpolation step %d\n", step);
                M_INFO1("     Got v %f:%f\n", 
                    CGAL::to_double(v.x()), CGAL::to_double(v.y()));
        }
        
        VisiLibity::Point p(CGAL::to_double(v.x()),CGAL::to_double(v.y()));
        p.snap_to_boundary_of(*my_environment,_visi_epsilon);
        p.snap_to_vertices_of(*my_environment,_visi_epsilon);
        VisiLibity::Visibility_Polygon vis_poly = 
            VisiLibity::Visibility_Polygon(p,*my_environment, _visi_epsilon); 
        int n_segs = seg_vis_graph_type2_vertices.size();
        remove_special_vertex_direct_visible();
        remove_special_vertex_edges();
        Segment_Visibility_Graph::mygraph_t* g = seg_vis_graph->g;
        (*g)[special_v].p_x = p.x(); (*g)[special_v].p_y = p.y();
        (*g)[special_v].segment_index = k;
        
        // this adds the right edges between special_v and all others
        process_visibility_polygon( n_segs, v, vis_poly, true);
        
        // now plan paths from v to i,j
        double final_d_i2, final_d_j2;
        std::list<KERNEL::Segment_2> l1,l2;
        l1 = get_shortest_path(n_segs,i,final_d_i2);
        l2 = get_shortest_path(n_segs,j,final_d_j2);
        
        KERNEL::Segment_2 s_of_i_to_k = get_segment_to_k(l1,k);
        KERNEL::Segment_2 s_of_j_to_k = get_segment_to_k(l2,k);
        double dist1 = sqrt(CGAL::to_double(s_of_i_to_k.squared_length() ) );
        double dist2 = sqrt(CGAL::to_double(s_of_j_to_k.squared_length() ) );
        KERNEL::Vector_2 v1 = s_of_i_to_k.to_vector();
        KERNEL::Vector_2 v2 = s_of_j_to_k.to_vector();
        std::cout << " s_of_i_to_k " << s_of_i_to_k << std::endl;
        std::cout << " s_of_j_to_k " << s_of_j_to_k << std::endl;
        std::cout << " v1 " << v1 << std::endl;
        std::cout << " v2 " << v2 << std::endl;
        std::cout << " vec " << vec << std::endl;
        double dotp1;
        double dotp2;
        if ( dist2 != 0 && dist1 != 0 ) {
            dotp1 = CGAL::to_double( vec*v1 ) / (dist1*search_distance);
            dotp2 = CGAL::to_double( vec*v2 ) / (dist2*search_distance);    
        }
        
        std::cout << dotp1 << std::endl;
        std::cout << dotp2 << std::endl;
        
        double angle1 = acos( dotp1);
        double angle2 = acos( dotp2);
        if ( angle1 > M_PI/2 )
            angle1 = M_PI - angle1;
        if ( angle2 > M_PI/2 )
            angle2 = M_PI - angle2;
        
        if ( DEBUG_POLYGON_ENVIRONMENT >= 4 ) {
            std::cout << " dist1 " << dist1 << std::endl;
            std::cout << " dist2 " << dist2 << std::endl;
            std::cout << " angle1 " << angle1 << std::endl;
            std::cout << " angle2 " << angle2 << std::endl;
            std::cout << " * " << dist1*cos(angle1) << std::endl;
            std::cout << " * " << dist2*cos(angle2) << std::endl;
        }
        double angle_diff = angle1 - angle2;
        std::cout << " angle_diff " << angle_diff << std::endl;
                            
        // prepare the parts to be remembered
        split_point_index = l1.size();
        l1.insert(l1.end(),l2.begin(),l2.end());
        if ( DEBUG_POLYGON_ENVIRONMENT >= 6 ) {
            std::cout << " final_d_i2 " << final_d_i2 << std::endl;
            std::cout << " final_d_j2 " << final_d_j2 << std::endl;
        }
        
        if ( final_d_i2 + final_d_j2 < final_d_i2_best + final_d_j2_best ) {
            final_d_i2_best = final_d_i2;
            final_d_j2_best = final_d_j2;
            l1_best = l1;
        } else {
            // we're getting worse - but we're convex, so no use climbing 
            // further in this direction
            if ( DEBUG_POLYGON_ENVIRONMENT >= 4 ) {
                M_INFO3("Getting worse \n ");
            }
            //break;
        }
        angle_diff_prev = angle_diff;
        if ( DEBUG_POLYGON_ENVIRONMENT_PRINT == 1 )
            out_file << step << " " << final_d_i2 + final_d_j2 <<  std::endl;
    }
    
    cost1 = final_d_i2_best;
    cost2 = final_d_j2_best;
    out_file.close();
    return l1_best;
}

KERNEL::Segment_2
Polygon_Environment::get_segment_to_k(std::list<KERNEL::Segment_2> &l, int k )
{
    if ( is_k_in_front(l,k) ) {
        return l.front();
    } else {
        return l.back();
    }
}

bool
Polygon_Environment::is_k_in_front(std::list<KERNEL::Segment_2> &l, int k )
{
    bool success = false;
    get_point_of_segment_on_segment( l.front(), k, success);
    if ( !success ) {
        return false;
    }
    return true;
} 

void
Polygon_Environment::remove_special_vertex_edges() 
{    
    //std::cout << "removing "  << std::endl;
    Segment_Visibility_Graph::mygraph_t::out_edge_iterator ei, ei_end;
    //std::cout << "getting out edges"  << std::endl;
    tie(ei, ei_end) = boost::out_edges(special_v,*(seg_vis_graph->g));
    std::vector< Segment_Visibility_Graph::edge_descriptor > edge_del;
    for ( ; ei != ei_end; ei++ ) {
        edge_del.push_back(*ei);        
    }
    for ( int i=0; i < edge_del.size(); i++ ) {
        //std::cout << "removing " << edge_del[i] << std::endl;
        remove_edge(edge_del[i],*(seg_vis_graph->g));
    }
}

void
    Polygon_Environment::remove_special_vertex_direct_visible()
{
    int n_segs = seg_vis_graph_type2_vertices.size();
    for ( int i = 0 ; i < seg_to_seg_visible->size(); i++ ) {
        (*seg_to_seg_distance)[i][n_segs] = 0;
        (*seg_to_seg_distance)[n_segs][i] = 0;
        (*seg_to_seg_visible)[i][n_segs] = false;
        (*seg_to_seg_visible)[n_segs][i] = false;
    }
}

/**
 * v_index corresponds ot vertex v
 */
void
Polygon_Environment::process_visibility_polygon(
    int v_index,
    KERNEL::Point_2 v, 
    VisiLibity::Visibility_Polygon &v_poly,
    bool special_run)
{
    int n_segs = seg_vis_graph_type2_vertices.size();
    
    if ( DEBUG_POLYGON_ENVIRONMENT >= 4 )
        M_INFO3("Find the index of point v in visibility polygon\n");
    int v_index_in_visi = -1;
    for ( int i = 0; i < v_poly.n() ; i++) 
    {
        KERNEL::Point_2 v_i( v_poly[i].x(), v_poly[i].y() );
        if ( v_i == v ) { v_index_in_visi = i; }
    }
    bool v_is_reflexive = false;
    if ( v_index != n_segs ) {
        if ( DEBUG_POLYGON_ENVIRONMENT >= 4 )
            M_INFO3("Check whether we have a reflexive vertex \n");
        v_is_reflexive = this->vertex_is_reflexive( v_index_in_visi, v_poly );
        (*(this->seg_vis_graph->g))
            [this->seg_vis_graph_type2_vertices[v_index]].reflexive 
                = v_is_reflexive;
    }
    
    if ( DEBUG_POLYGON_ENVIRONMENT >= 4 )
        M_INFO3("Parsing visibility polygon for vertex-segment %d\n", v_index);
    for ( int i = 0; i < v_poly.n(); i++) 
    {
        if ( DEBUG_POLYGON_ENVIRONMENT >= 5 )
            M_INFO1("    \n\nv_i: vertex %d of visi poly\n",i);
        
        int i_next = (i == v_poly.n() - 1) ? 0 : i+1;
        
        if ( (v_index_in_visi == i || v_index_in_visi == i_next)  
            && v_index != n_segs ) 
        {
            if ( DEBUG_POLYGON_ENVIRONMENT >= 5 )
                M_INFO1("    v_index in visi touched --- CONTINUING \n",i);
            continue;
        }
        
        KERNEL::Point_2 v_i( v_poly[i].x(), v_poly[i].y() );
        KERNEL::Point_2 v_next( v_poly[i_next].x(), v_poly[i_next].y() );
        if ( DEBUG_POLYGON_ENVIRONMENT >= 5 ) {
            std::cout << " Point of visi vertex " << v_i << std::endl;
            std::cout << " Point at end of segment " << v_next << std::endl;
            M_INFO1("     Finding the segment index for visi poly segment\n");
        }
        int endpoint_segment_index = this->is_endpoint(v_i);
        int endpoint_segment_index_next = this->is_endpoint(v_next);
        if ( DEBUG_POLYGON_ENVIRONMENT >= 5 ) {
            M_INFO1("     Endpoint segment indices %d - %d \n", 
                endpoint_segment_index, endpoint_segment_index_next);
        }
        
        int v_i_index;
        if ( endpoint_segment_index == -1  ) {
            v_i_index = get_segment_index_for_point(v_poly[i]);
        } else {
            v_i_index = endpoint_segment_index;
        }
        int segment_index_next;
        if ( endpoint_segment_index_next == -1  ) {
            segment_index_next = get_segment_index_for_point(v_poly[i_next]);
        } else {
            segment_index_next = endpoint_segment_index_next;
        }
        if ( DEBUG_POLYGON_ENVIRONMENT >= 5 ) {
            M_INFO1("     Final (mid_endpoint) segment indices %d - %d \n", 
                v_i_index, segment_index_next);
        }
                
        bool in_air = !is_sequential(v_i_index,segment_index_next) 
            && v_i_index != segment_index_next;
        
        if ( v_i_index < 0 )
            return;
        if ( DEBUG_POLYGON_ENVIRONMENT >= 5 ) {
            M_INFO1("     ... found segment index for v_i %d \n",v_i_index);
            M_INFO1("    Finding closest, v to visibility polygon segment\n");
        }
        
        KERNEL::Point_2 closest_p;
        double dis;
        if ( in_air ) {
            if ( DEBUG_POLYGON_ENVIRONMENT >= 5 ) 
                M_INFO1("     ... segment is in air \n");
            dis = CGAL::to_double(CGAL::squared_distance(v_i,v));
            closest_p = v_i;
        } else {
            KERNEL::Segment_2 s(v_i,v_next);
            dis = this->shortest_distance_between( s, v, closest_p );
        }
        dis = sqrt(dis);
        if ( DEBUG_POLYGON_ENVIRONMENT >= 5 ) {
            M_INFO1("     Closest point at distance %f  \n",dis);
            std::cout << "      Closest point is " << closest_p << std::endl;
            M_INFO1("      ---> Adding edges to seg visi graph...\n");
            M_INFO1("      ---> v_index %d and n_segs %d.\n", v_index, n_segs);
        }
        KERNEL::Segment_2 seg_to_seg_seg(v,closest_p);
        
        if ( v_index != n_segs ) {
            update_seg_to_seg(v_index, v_i_index, seg_to_seg_seg, dis);
            int v_prev_index = get_prev_index(v_index);
            update_seg_to_seg(v_prev_index, v_i_index, seg_to_seg_seg, dis);
        } else {
            // single special vertex with index n_segs
            update_seg_to_seg(n_segs, v_i_index, seg_to_seg_seg, dis);
        }
            
        if ( v_is_reflexive )
        {
            this->add_edge_to_visibility_graph(v_index, 2, v_i_index, 1, dis,
                CGAL::to_double(v.x()),CGAL::to_double(v.y()),
                CGAL::to_double(closest_p.x()), CGAL::to_double(closest_p.y()));
        } 
        
        if ( v_is_reflexive || v_index == n_segs ) {
            // if v_i 's index is also a reflexive vertex, then add a 
            //type 2 type 2 connections            
            bool v_i_is_reflexive = this->vertex_is_reflexive(i, v_poly );
            if ( v_i_is_reflexive ) {
                dis = CGAL::to_double(CGAL::squared_distance(v_i,v));
                dis = sqrt(dis);
                this->add_edge_to_visibility_graph(v_index, 2, v_i_index, 2,
                     dis, 
                    CGAL::to_double(v.x()),CGAL::to_double(v.y()),
                    CGAL::to_double(v_i.x()),CGAL::to_double(v_i.y())
                    );
                //M_INFO3("visi poly vertex v_i_is_reflexive i=%d, seg_ind=%d \n",
                //    i,v_i_index);
            }
        }
        if ( special_run && in_air ) {
            // we have a case not covered in the seg graph 
            // need to discover when landing on v_i_index collinear
            // to an edge
            // we know whether the endpoints of the visibility edge
            // is on an endpoint of an edge or not 
            // and we know the indices of the endpoints 
            
            // discovered indices in the visibility polygon are 
            // always increasing due to a clockwise ordering (within)
            // the circular order i.e 6,7,8, 4, 5 works
            // but 6,7,8,4,5,3 does not 
            
            v_i_index = endpoint_segment_index;
            segment_index_next = endpoint_segment_index_next;
            if ( endpoint_segment_index_next != -1 ) {
                // landed through air on an endpoint 
                int other_edge_index = endpoint_segment_index_next-1;
                fix_index(other_edge_index);
                // test if the previous segment is parallel to edge
                double sqrd_d = CGAL::to_double(CGAL::squared_distance( 
                    master_polygon->vertex(other_edge_index),
                    KERNEL::Segment_2(v_i,v_next)
                    ));
                if ( sqrd_d < _visi_epsilon*_visi_epsilon ) {
                    if ( DEBUG_POLYGON_ENVIRONMENT >= 5 ) {
                        M_INFO1(" Pretty close %f  \n",sqrd_d);
                    }
                    // need to add a 
                    KERNEL::Segment_2 seg_to_seg_seg2(
                        v,
                        master_polygon->vertex(other_edge_index));
                    double dis2;
                    dis2 = sqrt(
                        CGAL::to_double(seg_to_seg_seg2.squared_length()));
                    update_seg_to_seg(n_segs, other_edge_index,
                         seg_to_seg_seg2, dis2);
                    this->add_edge_to_visibility_graph(v_index, 2,
                        other_edge_index, 2,
                        dis2, 
                        CGAL::to_double(v.x()),CGAL::to_double(v.y()),
                        CGAL::to_double(seg_to_seg_seg2.target().x()),
                        CGAL::to_double(seg_to_seg_seg2.target().y())
                    );
                }
            }
        }
        
    }
    //M_INFO2("Done with adding edges to seg visi graph for this vertex\n");

}

void
Polygon_Environment::update_seg_to_seg(int i, int j,KERNEL::Segment_2 s, double d, bool recall )
{
    if ( DEBUG_POLYGON_ENVIRONMENT >= 5 ) {
        M_INFO1("            update_seg_to_seg %d to %d  \n",i,j);
        std::cout << s << std::endl;
        M_INFO1("            update_seg_to_seg d=%f  \n",d);
    }
    if ( (*seg_to_seg_visible)[i][j] ) {
        // already visible, compare distances
        if ( d < (*seg_to_seg_distance)[i][j] ) {
            (*seg_to_seg_segments)[i][j] = s;
            (*seg_to_seg_distance)[i][j] = d;
        }
    } else {
        (*seg_to_seg_visible)[i][j] = true;
        (*seg_to_seg_segments)[i][j] = s;
        (*seg_to_seg_distance)[i][j] = d;
    }
    if ( recall ) {
        update_seg_to_seg(j,i,s,d,false);
    }
    
}

int 
Polygon_Environment::get_next_index( int i ) 
{
    return (i == (*my_environment)[0].n() - 1) ? 0 : i+1;
}

int 
Polygon_Environment::get_prev_index( int i ) 
{
    return (i == 0) ? (*my_environment)[0].n() - 1 : i-1;
}

//std::list<KERNEL::Segment_2> 
//Polygon_Environment::get_shortest_path_to_special_v(int i)
//{
//    // special vertex special_v
//    // check k's direct visibility to segments
//    Segment_Visibility_Graph::vertex w;
//    w = this->seg_vis_graph_type1_vertices[i];
//    reset_edges_type_1_from_infty();
//    set_edges_type_1_to_infty(i,i);
//    this->add_extra_edges(i,i);
//    double total_path_distance;
//    segment_list_path = plan_in_svg(special_v,w,total_path_distance);
//    this->remove_extra_edges();
//}

std::list<KERNEL::Segment_2> 
Polygon_Environment::get_shortest_path(KERNEL::Point_2 v, int j, int k, double& final_dist)
{
    // initialization and bookkeeping
    VisiLibity::Point p(CGAL::to_double(v.x()),CGAL::to_double(v.y()));
    p.snap_to_boundary_of(*my_environment,_visi_epsilon);
    p.snap_to_vertices_of(*my_environment,_visi_epsilon);
    VisiLibity::Visibility_Polygon vis_poly;
    vis_poly = VisiLibity::Visibility_Polygon(
        p,*my_environment, _visi_epsilon);
    int n_segs = seg_vis_graph_type2_vertices.size();    
    remove_special_vertex_direct_visible();
    remove_special_vertex_edges();
    Segment_Visibility_Graph::mygraph_t* g = seg_vis_graph->g;
    
    // set up the special vertex for planning
    (*g)[special_v].p_x = p.x(); (*g)[special_v].p_y = p.y();
    (*g)[special_v].segment_index = k; 
   
    // this adds the right edges between special_v and all others
    process_visibility_polygon( n_segs, v, vis_poly, true);
    
    // now plan paths from v to i,j
    std::list<KERNEL::Segment_2> l1;
    l1 = get_shortest_path(n_segs,j, final_dist);
    return l1;
}

std::list<KERNEL::Segment_2> 
Polygon_Environment::get_shortest_path(int i, int j, double& final_dist)
{
    if ( DEBUG_POLYGON_ENVIRONMENT >= 2 )
        M_INFO2("get_shortest_path %d %d\n",i,j);
    if ( i < master_polygon->size() && j < master_polygon->size() ) {
        if ( DEBUG_POLYGON_ENVIRONMENT >= 3 )
            M_INFO2("checking cache\n");
        if ( check_path_cache(i,j) ) {
            if ( DEBUG_POLYGON_ENVIRONMENT >= 3 )
                M_INFO2(" cache is full - grab it \n");
            final_dist = get_path_distance(i,j);
            return get_path_cache(i,j);
        }
    }
    
    std::list<KERNEL::Segment_2> segment_list;
    std::list<KERNEL::Segment_2> segment_list_path;
    // The quick options
    if ( i == j ) {
        if ( DEBUG_POLYGON_ENVIRONMENT >= 3 )
            M_INFO2("i==j returning 0 and empty segment list\n");
        final_dist = 0; return segment_list;
    } else if ( ( abs(j-i) == 1 
          || (i==0 && j==master_polygon->size()-1) 
          || (j==0 && i==master_polygon->size()-1) ) 
         && master_polygon->size() != i && master_polygon->size() != j ) 
    {
        //the common point of j and i is large one
        int iv = i > j ? i : j;
        if ( (i==0 && j==master_polygon->size()-1) 
          || (j==0 && i==master_polygon->size()-1) ) {
              iv = 0;
        }
        
        
        if ( DEBUG_POLYGON_ENVIRONMENT >= 2 )
            M_INFO2("Taking vertex %d directly\n",iv);
        if ( iv < master_polygon->size() ) {
            KERNEL::Segment_2 seg(master_polygon->vertex(iv), master_polygon->vertex(iv));
            std::cout << master_polygon->vertex(iv) << std::endl;
            segment_list.push_back ( seg );
            final_dist = 0; return segment_list;
        } else {
            M_ERR("ERROR index i or j out of bounds %d %d >= %d\n",i,j,int(master_polygon->size()) );
        }
    }
    
    double shortest_distance = std::numeric_limits<double>::max();
    int n_segs = this->seg_vis_graph_type1_vertices.size();
    //M_INFO2("seg_to_seg_visible->size() %d \n", seg_to_seg_visible->size() );
    //M_INFO2("(*seg_to_seg_visible)[i].size()  %d \n ", (*seg_to_seg_visible)[i].size() );
    bool tr = (*seg_to_seg_visible)[i][j];
    //M_INFO2("visible?  %d \n",  tr);
    if ( i < seg_to_seg_visible->size() 
            && j < (*seg_to_seg_visible)[i].size() 
            && (*seg_to_seg_visible)[i][j] ) {
        if ( DEBUG_POLYGON_ENVIRONMENT >= 3 ) {
            M_INFO2("Segments VISIBLE %d-%d - checking distances\n", i,j);
        }
        segment_list.push_back( (*seg_to_seg_segments)[i][j]);
        shortest_distance = (*seg_to_seg_distance)[i][j];
        if ( DEBUG_POLYGON_ENVIRONMENT >= 3 ) {
            M_INFO2("shortest_distance %f \n", shortest_distance);
        }
    } 
    
    Segment_Visibility_Graph::vertex v, w;
    v = (i==n_segs) ? special_v : this->seg_vis_graph_type1_vertices[i];
    w = this->seg_vis_graph_type1_vertices[j];
    reset_edges_type_1_from_infty();
    set_edges_type_1_to_infty(i,j);
    this->add_extra_edges(i,j);
    double total_path_distance;
    segment_list_path = plan_in_svg(v,w,total_path_distance);
    this->remove_extra_edges();
    
    if ( DEBUG_POLYGON_ENVIRONMENT >= 2 ) {
            M_INFO2("Path planned - len %d \n", segment_list_path.size() );
            M_INFO2("Path planned - dis %f \n", total_path_distance );    
            std::list<KERNEL::Segment_2>::iterator p_it;
            p_it = segment_list_path.begin();
            for ( ; p_it != segment_list_path.end(); p_it++ ) 
                std::cout << " Segment " << *p_it << std::endl;
    }
    
    if ( total_path_distance < shortest_distance 
            && segment_list_path.size() != 0 )
    {
        final_dist = total_path_distance;
        if ( i < master_polygon->size() && j < master_polygon->size() )
            set_path_cache(i,j,segment_list_path, final_dist);
        if ( DEBUG_POLYGON_ENVIRONMENT >= 2 )
            M_INFO2("final distance of path %f\n",final_dist);
        return segment_list_path;
    } else if (  segment_list.size() != 0 ) {
        final_dist = shortest_distance;
        if ( i < master_polygon->size() && j < master_polygon->size() )
            set_path_cache(i,j,segment_list, final_dist);
        if ( DEBUG_POLYGON_ENVIRONMENT >= 2 )
            M_INFO2("final distance (directly visible)%f\n",final_dist);
        return segment_list;
    } else {
        if ( DEBUG_POLYGON_ENVIRONMENT >= 2 )
            M_ERR("ERROR failed planning the shortest path ");
        return segment_list;
    }
}

std::list<KERNEL::Segment_2>
Polygon_Environment::plan_in_svg(Segment_Visibility_Graph::vertex v,
    Segment_Visibility_Graph::vertex w, double& total_path_distance)
{
    std::list<KERNEL::Segment_2> segment_list_path;
    std::list<Segment_Visibility_Graph::vertex> shortest_path;      
    shortest_path = seg_vis_graph->get_shortest_path(v,w);
    std::list<Segment_Visibility_Graph::vertex>::iterator spi =    
        shortest_path.begin();
    Segment_Visibility_Graph::vertex last_v = v;
    Segment_Visibility_Graph::mygraph_t* g = seg_vis_graph->g;
    total_path_distance = 0;
    for(++spi; spi != shortest_path.end(); ++spi) 
    { 
        Segment_Visibility_Graph::edge_descriptor short_e = 
            seg_vis_graph->get_edge_shortest(last_v,*spi);
        KERNEL::Point_2 p1((*g)[short_e].p_x, (*g)[short_e].p_y);
        KERNEL::Point_2 p2((*g)[short_e].p_x2, (*g)[short_e].p_y2);
        KERNEL::Segment_2 s( p1,p2);
        if ( DEBUG_POLYGON_ENVIRONMENT >= 6 ) {
            std::cout << "Adding segment to path " << s << std::endl;
        }
        if ( s.squared_length() == 0 && segment_list_path.size() != 0 ) {
            if ( DEBUG_POLYGON_ENVIRONMENT >= 6 ) {
                M_INFO2("s.squared_length() == 0 ignoring segment in path \n");
                std::cout << (*g)[short_e].distance << std::endl;
            }                
            if ( (*g)[short_e].distance != 0 )
                M_ERR("ERROR (*g)[short_e].distance not zero\n");
        } else {
            segment_list_path.push_back(s); 
            total_path_distance += (*g)[short_e].distance;
        }   
        last_v = *spi;         
    }
    if ( DEBUG_POLYGON_ENVIRONMENT >= 6 )
        M_INFO2("segment_list_path.size() %d  \n", segment_list_path.size());
    return segment_list_path;
}

void Polygon_Environment::set_edges_type_1_to_infty(int i,int j)
{
    Segment_Visibility_Graph::mygraph_t::edge_iterator 
        ei, e_end;
    Segment_Visibility_Graph::mygraph_t* g = seg_vis_graph->g;
    tie(ei,e_end) = edges( *g );
    for ( ; ei != e_end; ei++ ) {           
        if (  ( (*g)[ target(*ei,*g) ].type == 1 
            && (*g)[ target(*ei,*g) ].segment_index != i 
            && (*g)[ target(*ei,*g) ].segment_index != j ) 
            || ( (*g)[ source(*ei,*g) ].type == 1 
                && (*g)[ source(*ei,*g) ].segment_index != i
                && (*g)[ source(*ei,*g) ].segment_index != j ))
        {
            //std::cout << " Edge " << *ei << " to infty " << std::endl;
            (*g)[*ei].distance = std::numeric_limits<double>::max();
        } else {
            //std::cout << " Edge " << *ei << " NOT TO infty " << std::endl;
        }
    }
}

void Polygon_Environment::reset_edges_type_1_from_infty() {
    Segment_Visibility_Graph::mygraph_t::edge_iterator 
        ei, e_end;
    Segment_Visibility_Graph::mygraph_t* g = seg_vis_graph->g;
    tie(ei,e_end) = edges( *g );
    for ( ; ei != e_end; ei++ ) {
        (*g)[*ei].distance = (*g)[*ei].distance_temp;
    }
}

void Polygon_Environment::remove_extra_edges() {
    for ( int i = 0; i < extra_edges.size(); i++ ) {
            remove_edge( extra_edges[i], *(this->seg_vis_graph->g) );
    }
    extra_edges.clear();
}


void Polygon_Environment::add_extra_edges(int i, int j) {
    extra_edges.clear();
    add_extra_edges_for(i);
    add_extra_edges_for(j);    
}

void Polygon_Environment::add_extra_edges_for(int seg_ind) {
    Segment_Visibility_Graph::mygraph_t* g = this->seg_vis_graph->g;
    Segment_Visibility_Graph::vertex v, w,z;
    int n_segs = seg_vis_graph_type2_vertices.size();
    int next_index;
    if ( seg_ind != n_segs) {
        next_index = get_next_index(seg_ind);
    } else {
        next_index = get_next_index( (*g)[special_v].segment_index);
    }
        
    if ( seg_ind != n_segs ) {
        v = this->seg_vis_graph_type1_vertices[seg_ind];
        w = this->seg_vis_graph_type2_vertices[seg_ind];
        z  =  this->seg_vis_graph_type2_vertices[next_index];
    } else {
        v = special_v;
        w = this->seg_vis_graph_type2_vertices[(*g)[special_v].segment_index];
        z = this->seg_vis_graph_type2_vertices[next_index];
    }
    
    Segment_Visibility_Graph::edge_descriptor e;
    if ( (*g)[w].reflexive ) {
        if ( seg_ind == n_segs ) {
            double dis = sqrt( pow( (*g)[w].p_x-(*g)[special_v].p_x,2) + pow( (*g)[w].p_y- (*g)[special_v].p_y, 2) );
            e = this->add_edge_to_visibility_graph(
               seg_ind, 2, 
               (*g)[special_v].segment_index, 2, 
               dis, 
               (*g)[special_v].p_x, (*g)[special_v].p_y, 
               (*g)[w].p_x, (*g)[w].p_y);
        } else {
            e = this->add_edge_to_visibility_graph(
               seg_ind, 2, 
               seg_ind, 1, 
               0, (*g)[w].p_x, (*g)[w].p_y, (*g)[w].p_x, (*g)[w].p_y);
        }
        extra_edges.push_back(e);
    }
    if ( (*g)[z].reflexive ) {
        if ( seg_ind == n_segs ) {
            double dis = sqrt( pow( (*g)[z].p_x-(*g)[special_v].p_x,2) + pow( (*g)[z].p_y- (*g)[special_v].p_y, 2) );
            e = this->add_edge_to_visibility_graph(
               seg_ind, 2, 
               next_index, 2, 
               dis, 
               (*g)[special_v].p_x, (*g)[special_v].p_y, 
               (*g)[z].p_x, (*g)[z].p_y);
        } else {
            e = this->add_edge_to_visibility_graph(
               next_index, 2, 
               seg_ind, 1, 
               0, (*g)[z].p_x, (*g)[z].p_y, (*g)[z].p_x, (*g)[z].p_y);
        }
        extra_edges.push_back(e);
    }
}

Segment_Visibility_Graph::vertex
Polygon_Environment::get_segment_visibility_vertex(int i, int type_i)
{
    Segment_Visibility_Graph::vertex v, w;
    if ( type_i == 1 )
        v = this->seg_vis_graph_type1_vertices[i];
    else
        v = this->seg_vis_graph_type2_vertices[i];
    return v;
}


Segment_Visibility_Graph::edge_descriptor
Polygon_Environment::add_edge_to_visibility_graph( int i,  int type_i, int j, int type_j, double d, double x, double y, double x2,double y2 )
{
    //std::cout 
    //    << "  i=" << i << ", " << type_i
    //    << " j=" << j << ", " << type_j 
    //    << " d=" << d << " " 
    //    << x << ":" << y << " " << x2 << ":" << y2 
    //    << std::endl;
    Segment_Visibility_Graph::vertex v, w;
    if ( type_i == 1 && i < seg_vis_graph_type1_vertices.size())
        v = this->seg_vis_graph_type1_vertices[i];
    else if ( i < seg_vis_graph_type2_vertices.size() )
        v = this->seg_vis_graph_type2_vertices[i];
    else if ( i == seg_vis_graph_type1_vertices.size() )
        v = special_v;
    if ( type_j == 1 && j < seg_vis_graph_type1_vertices.size())
        w = this->seg_vis_graph_type1_vertices[j];
    else if ( j < seg_vis_graph_type2_vertices.size())
        w = this->seg_vis_graph_type2_vertices[j];
    else if ( j == seg_vis_graph_type1_vertices.size() )
        w = special_v;
    Segment_Visibility_Graph::edge_descriptor e;
    e = seg_vis_graph->add_edge(v, w, x, y, x2,y2, d);
    return e;
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
    
    int v_neigh1_i,v_neigh2_i;
    if ( i == 0 ) v_neigh1_i = v_poly.n()-1;
    else  v_neigh1_i = i-1;
    if ( i == v_poly.n() ) v_neigh2_i = 0;
    else  v_neigh2_i = i+1;
    
    KERNEL::Point_2 v( v_poly[i].x(), v_poly[i].y() );
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
    
    //CGAL::squared_distance( s, p);
    
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

/*
 * // this is a vertex index - starting at 0, for segment [vertex_0,vertex_1]
 */
int 
Polygon_Environment::get_segment_index_for_point( VisiLibity::Point p ) 
{
    return p.index_of_projection_onto_boundary_of( *my_environment );
}


void Polygon_Environment::compute_visibility_polygon_at_vertex(VisiLibity::Point &p)
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