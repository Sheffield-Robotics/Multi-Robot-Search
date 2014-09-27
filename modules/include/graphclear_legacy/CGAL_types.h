#ifndef MY_CGAL_CUSTOM_TYPES
#define MY_CGAL_CUSTOM_TYPES

// IMPORTANT: this fixes undefined constants for squared_distance_2
#include <CGAL/Kernel/global_functions_internal_2.h> 
#include <CGAL/basic.h>
#include <CGAL/squared_distance_2.h>
#include <CGAL/Cartesian.h>
#include <CGAL/Alpha_shape_vertex_base_2.h>
#include <CGAL/Triangulation_face_base_2.h>
#include <CGAL/Alpha_shape_face_base_2.h>
#include <CGAL/Triangulation_default_data_structure_2.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Alpha_shape_2.h>
#include <CGAL/Polygon_2.h>
//******* Voronoi Diagram Adaptor *******
// includes for defining the Voronoi diagram adaptor
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Segment_Delaunay_graph_2.h>
#include <CGAL/Segment_Delaunay_graph_traits_2.h>

#include <CGAL/Voronoi_diagram_2.h>
#include <CGAL/Segment_Delaunay_graph_adaptation_traits_2.h>
#include <CGAL/Segment_Delaunay_graph_adaptation_policies_2.h>
#include <CGAL/Identity_policy_2.h>
#include <CGAL/intersections.h>
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

typedef CGAL::Cartesian< double >                         CGAL_K;
typedef CGAL_K::Point_2                                   CGAL_Point;
typedef CGAL_K::Segment_2                                 CGAL_Segment;
typedef CGAL::Alpha_shape_vertex_base_2<CGAL_K>           CGAL_Av;
typedef CGAL::Triangulation_face_base_2<CGAL_K>           CGAL_Tf;
typedef CGAL::Alpha_shape_face_base_2< CGAL_K, CGAL_Tf >  CGAL_Af;
typedef CGAL::Triangulation_default_data_structure_2<CGAL_K,CGAL_Av,CGAL_Af> 
  CGAL_Tds;
typedef CGAL::Delaunay_triangulation_2<CGAL_K,CGAL_Tds>   CGAL_Dt;
typedef CGAL::Alpha_shape_2<CGAL_Dt>                      CGAL_Alpha_shape_2;
typedef CGAL::Polygon_2< CGAL_K >                         Polygon;
typedef std::vector< Polygon >                            Polygons;

//******* Voronoi Diagram Adaptor typedefs *******
class VoronoiVertexInfo {
public:
   VoronoiVertexInfo() : isVoronoi(false), visited(false) {}
   bool isVoronoi;
   bool visited;
};

typedef CGAL::Exact_predicates_inexact_constructions_kernel 
    SDG_Kernel;

typedef CGAL::Segment_Delaunay_graph_filtered_traits_without_intersections_2
            <SDG_Kernel, CGAL::Field_with_sqrt_tag>
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
  SDG;

typedef CGAL::Segment_Delaunay_graph_adaptation_traits_2<SDG> 
  Adaption_traits;
typedef CGAL::Identity_policy_2<SDG, Adaption_traits>       
  Identity_policy;
typedef CGAL::Voronoi_diagram_2< SDG, Adaption_traits, 
    Identity_policy>
  CGAL_VD;

typedef Adaption_traits::Site_2       VD_Site_2;
typedef Adaption_traits::Point_2      VD_Point_2;
typedef SDG_Kernel::Line_2               VD_Line_2;
typedef SDG_Kernel::Ray_2                VD_Ray_2;
typedef SDG_Kernel::Vector_2             VD_Vector;
//typedef SDG::Point_2                     VD_Point;
typedef SDG_Kernel::Segment_2            VD_Segment_2;

typedef CGAL_VD::Vertex                  VD_Vertex;
typedef CGAL_VD::Halfedge                VD_Halfedge;
typedef CGAL_VD::Locate_result           VD_Locate_result;
typedef CGAL_VD::Vertex_handle           VD_Vertex_handle;
typedef CGAL_VD::Face_handle             VD_Face_handle;
typedef CGAL_VD::Halfedge_handle         VD_Halfedge_handle;
typedef CGAL_VD::Ccb_halfedge_circulator VD_Ccb_halfedge_circulator;

#endif