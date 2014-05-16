#ifndef VORONOI_DIAGRAM_H
#define VORONOI_DIAGRAM_H

#include "utilities/misc.h"
#include "geometry_types.h"
#include "polygonization/polygon_environment.h"
#include <iostream>

#include <CGAL/Kernel/global_functions_internal_2.h> 
#include <CGAL/basic.h>
#include <CGAL/squared_distance_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <CGAL/Triangulation_face_base_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>
#include <CGAL/Triangulation_default_data_structure_2.h>
#include <CGAL/Triangulation_hierarchy_2.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Segment_Delaunay_graph_traits_2.h>
#include <CGAL/Segment_Delaunay_graph_2.h>
#include <CGAL/Voronoi_diagram_2.h>
#include <CGAL/Segment_Delaunay_graph_adaptation_traits_2.h>
#include <CGAL/Segment_Delaunay_graph_adaptation_policies_2.h>
#include <CGAL/Segment_Delaunay_graph_hierarchy_vertex_base_2.h>
#include <CGAL/Identity_policy_2.h>
#include <CGAL/intersections.h>
#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Triangulation_hierarchy_3.h>
#include <CGAL/Alpha_shape_3.h>
#include <CGAL/Segment_Delaunay_graph_hierarchy_2.h>

#include <map>
#include <utility>

namespace polygonization {
    class Polygon_Environment;
    
    class VoronoiVertexInfo {
    public:
       VoronoiVertexInfo() : isVoronoi(false), visited(false) {}
       bool isVoronoi;
       bool visited;
    };

    typedef CGAL::Segment_Delaunay_graph_filtered_traits_without_intersections_2
                <KERNEL, CGAL::Field_with_sqrt_tag>
      Traits_Filtered;
    typedef CGAL::Segment_Delaunay_graph_storage_traits_2< Traits_Filtered > 
      Storage_traits;
    typedef CGAL::Triangulation_data_structure_2 < 
        CGAL::Segment_Delaunay_graph_hierarchy_vertex_base_2< 
            CGAL::Segment_Delaunay_graph_vertex_base_2<Storage_traits> >,
        CGAL::Triangulation_face_base_with_info_2<  
            VoronoiVertexInfo,Traits_Filtered> > 
      Triangulation_structure;

    typedef CGAL::Segment_Delaunay_graph_hierarchy_2<Traits_Filtered, 
        Storage_traits, CGAL::Tag_false, Triangulation_structure>
      Graph_hierarchy;

    typedef CGAL::Segment_Delaunay_graph_adaptation_traits_2<Graph_hierarchy> 
      Adaption_traits;
    typedef CGAL::Identity_policy_2<Graph_hierarchy, Adaption_traits>       
      Identity_policy;
    typedef CGAL::Voronoi_diagram_2< Graph_hierarchy, Adaption_traits, 
        Identity_policy>
      Voronoi_Diagram_base;
      
    class Voronoi_Diagram : public Voronoi_Diagram_base {
      public:
        typedef std::pair<double,double>
            Bad_Point;
        typedef std::pair< Bad_Point,Bad_Point > 
            Bad_Segment;
        std::map< Bad_Segment,int> site_to_polygon;
        std::map< Segment,int> site_to_polygon2;
        bool construct( Polygon_Environment* e );
        Bad_Segment site_to_bad_segment( Voronoi_Diagram::Site_2 a );
        
        Segment*** parse( double** distance_mat, Polygon_Environment* poly_env);
    };
}


#endif