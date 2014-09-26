#include "sg_map.h"

//******* Constructor and Options *******
sg_map::sg_map( Config* the_config ) {
	//DEBUG_1("SG_MAP Constructor");
	//***	 Reading configuration (fold)
	robot_cfg = the_config;
	the_config->lookupValue("map.ncols", ncols); 
	the_config->lookupValue("map.nrows", nrows);
	the_config->lookupValue("map.resolution", resolution);
	the_config->lookupValue("map.sense_range_pix", sense_range_pix);
	the_config->lookupValue("map.sense_delta", sense_delta);
	the_config->lookupValue("map.minimum_gap", minimum_gap);
	the_config->lookupValue("map.show_maps", show_maps); 
	the_config->lookupValue("map.save_imgs", save_imgs); 
	the_config->lookupValue("map.base_img_dir", base_img_dir); 
	the_config->lookupValue("map.simplify_epsilon", simplify_epsilon); 
	the_config->lookupValue("map.alpha_value", alpha_value); 
	// Reading configuration (end)
	//*** Initialization of matrices (fold)
	this->occ.set_dimensions(nrows,ncols);
	this->occ.set_elements(0);	
	this->boundary.set_dimensions(nrows,ncols);
	this->boundary.set_elements(0);	
	to_polygon.set_dimensions(nrows,ncols);
	to_polygon.set_elements(-1);
	vertex_visited.set_dimensions(nrows,ncols);
	vertex_visited.set_elements(0);
	edge_visited.set_dimensions(nrows,ncols);
	edge_visited.set_elements(0);
	voronoi_matrix.set_dimensions(nrows,ncols);
	voronoi_matrix.set_elements(0);
	this->wave_count.set_dimensions(nrows,ncols);
	this->wave_count.set_elements(0);
	this->clearance_value.set_dimensions(nrows,ncols);
	this->clearance_value.set_elements(0);
	this->wavepoints_hitdir.set_dimensions(nrows,ncols);
	this->wavepoints_hitdir.set_elements(-1);
	this->fortune_voronoi.set_dimensions(nrows,ncols);
	this->fortune_voronoi.set_elements(0);
	this->fortune_v_segment.set_dimensions(nrows,ncols);
	this->fortune_v_segment.set_elements(0);
	this->belongs_to_segment.set_dimensions(nrows,ncols);
	this->belongs_to_segment.set_elements(0);
	this->voronoi_minima.set_dimensions(nrows,ncols);
	this->voronoi_minima.set_elements(0);
	this->visited.set_dimensions(nrows,ncols);
	this->visited.set_elements(0);
	this->visited_close_min.set_dimensions(nrows,ncols);
	this->visited_close_min.set_elements(0);
	this->blocked_for_minimum.set_dimensions(nrows,ncols);
	this->blocked_for_minimum.set_elements(0);
	this->path_points.set_dimensions(nrows,ncols);
	this->path_points.set_elements(0);
	leaf_mergers = 0; 
	two_mergers	 = 0;	 
	n_polygons = 0;
	std::vector< map_point_list > dummy_v(ncols);
	closest_obstacle = new map_point_grid(nrows, dummy_v);
	std::vector< sg_vertex_d > dummy_v2(ncols);
	vertex_membership = new vertexgrid(nrows, dummy_v2);
	//DEBUG_1("SG_MAP Constructor - done");
	// Initialization to ZERO (end)
}

//******* The entire graph construction process ******* (fold)

/*
* Parses image and creaets occupancy information in matrices
*/

void sg_map::process_img(const char* img_filename, double lower_thres) {
	IplImage* img_map     = cvLoadImage( img_filename, -1);
	img_map_size.width    = img_map->width;
	img_map_size.height   = img_map->height;
	IplImage* thresholded = cvCreateImage( img_map_size, 8, 1 );
	int threshold = 230, pixel = 0;
	cvThreshold( img_map, thresholded, lower_thres, 
	             double(threshold), CV_THRESH_BINARY );
	// build the occupancy set - reset occupancy variables/information
	n_occupied_cells = 0;
	occ_points.clear();
	this->occ.set_elements(0);	
	CGAL_occ_vector.clear();
	// build new occupancy information
	int i, j;
	for ( i = 0; i < thresholded->height; i++ ) {
		for ( j = 0; j < thresholded->width; j++ ) {
			pixel = int (((uchar*)(thresholded->imageData + thresholded->widthStep*i))[j]);
			if (pixel == threshold) {
				this->set_occ( i, j, 1 ); //setting occ matrix entry to 1
				this->add_occ_pt( i, j ); //adding point to occ_points vector
				CGAL_occ_vector.push_back( CGAL_Point(i,j) );
				n_occupied_cells++;
			}
		}
	}
	cout << " Occupied pixels " << n_occupied_cells << endl;
	cvReleaseImage( &img_map ); cvReleaseImage( &thresholded );
}

void sg_map::process_map() {
	if ( DEBUG_MAP >= 1 ) 
		cout << " 1) Compute Alpha Shape & Simplify Boundary" << endl;
	alpha_shape();
	if ( DEBUG_MAP >= 1 ) 
		cout << " 2) Build Voronoi Diagram and Delaunay Graph" << endl;
	build_CGAL_voronoi_diagram();
	if ( DEBUG_MAP >= 1 ) 
		cout << " 2a) Imprint Voronoi Diagram " << endl;
	imprint_CGAL_voronoi_diagram();
	if ( DEBUG_MAP >= 1 ) 
		cout << " 3) Process Voronoi Diagram to create a SG" << endl;
	process_CGAL_voronoi_diagram();
}
//*** (end)

void sg_map::alpha_shape() {
	cout << " Building alpha shape from CGAL_occ_vector" << endl;
	CGAL_Alpha_shape_2 A( CGAL_occ_vector.begin(), CGAL_occ_vector.end(),
	                      3, CGAL_Alpha_shape_2::GENERAL );
	
	A.set_alpha( alpha_value );
	//for ( int count = 0; count < A.number_of_alphas() ; ++count ) {
	//  cout << A.get_nth_alpha(count) << endl; 
	//}
	
	CGAL_Alpha_shape_2::Alpha_shape_vertices_iterator it, it2;
	it  = A.alpha_shape_vertices_begin();
	it2 = A.alpha_shape_vertices_end();
	
	
	IplImage* img = cvCreateImage( img_map_size, 8, 3 );
	int i,j;
	for ( ; it != it2 ; ++it ) {    
		i = int ( (*it)->point().x() );
		j = int ( (*it)->point().y() );
		if ( bound_check(i,j) ) {
			  ((uchar *)(img->imageData + i*img->widthStep))[j*img->nChannels + 0] = 200;// B
			  ((uchar *)(img->imageData + i*img->widthStep))[j*img->nChannels + 1] = 200; // G
			  ((uchar *)(img->imageData + i*img->widthStep))[j*img->nChannels + 2] = 0; // R  
		 }
	}  
	char filename[100];  
	strcpy( filename, base_img_dir );
	cvSaveImage( strcat(filename, "save_img_alpha_vertices.BMP"), img );
	cvReleaseImage(&img);  
	
	CGAL_Alpha_shape_2::Alpha_shape_edges_iterator eit, eit2;
	eit  = A.alpha_shape_edges_begin();
	eit2 = A.alpha_shape_edges_end();
  
	CGAL_Point prev_target, new_target, new_source;
	n_polygons = 0;
	
	// collect all segments in alpha_segments
	std::list< CGAL_Segment > alpha_segments;
	for ( ; eit != eit2 ; ++eit ) {    
		alpha_segments.push_back( A.segment(*eit) );
	}
	
	std::list< CGAL_Segment >::iterator l_it;; 
	CGAL_Segment segment1;
	while ( alpha_segments.size() > 0 ) {
		// pick the first segment
		segment1 = alpha_segments.front();
		alpha_segments.pop_front();
		Polygon new_poly;
		poly_boundary.push_back(new_poly);
		poly_boundary[n_polygons].push_back(segment1.source());
		poly_boundary[n_polygons].push_back(segment1.target());
		n_polygons++;
		// find all other segments that connect to this one
		bool found_next = true;
		while (found_next == true ) {
			// need to find a next one or abort
			found_next = false;
			l_it = alpha_segments.begin();
			for ( ; l_it != alpha_segments.end(); ++l_it ) {
				if ( (*l_it).source() == segment1.target() ) {
					segment1 = (*l_it);
					alpha_segments.erase(l_it);
					poly_boundary[n_polygons-1].push_back( segment1.target() );
					found_next = true;
					break;
				}
			}
		}
	}
	cout << " " << n_polygons << " polygons are the boundary " << endl;
	
	for ( i = 0; i < n_polygons; ++i ) {
		simplify_polygon( i );
	}
	
	sprintf( filename, "%s%s", base_img_dir, "save_img_poly_boundary.BMP");
	save_img_poly_boundary_simple( filename );
}

void sg_map::build_CGAL_voronoi_diagram() {
	// Build the Voronoi Diagram. ( a CGAL_SVD2)
	cout << " Building sites for VD from poly_boundary_simple" << endl;
	CGAL_Point P1,P2;
	VD_Site_2 a_site;
	std::vector< VD_Site_2 > site_v;
	for ( int i = 0 ; i < n_polygons; ++i ) {
		P1 = poly_boundary_simple[i][0];
		for ( int j = 1; j < poly_boundary_simple[i].size(); ++j ) {
			P2 = poly_boundary_simple[i][j];
			a_site = a_site.construct_site_2( VD_Point_2(P1.x(),P1.y()), 
			                                  VD_Point_2(P2.x(),P2.y()));
			if ( a_site.is_defined() )
				site_v.push_back(a_site);
			P1 = P2;
		}
		// Add segment from last point to first point to close the polygon
		P2 = poly_boundary_simple[i][0];
		a_site = a_site.construct_site_2( VD_Point_2(P1.x(),P1.y()), 
		                                  VD_Point_2(P2.x(),P2.y()));
		if ( a_site.is_defined() )
			site_v.push_back(a_site);
	}
	
	/* Insert all segments from the simplified polygon boundary */
	std::vector< VD_Site_2 >::iterator site_it, site_it_end;
	site_it     = site_v.begin();
	site_it_end = site_v.end();
	cout << " Inserting sites for Voronoi Diagram" << endl;
	voronoi_diagram.insert( site_it, site_it_end );
	
	/* Diagnostics */
	cout << " V VERTICES " << voronoi_diagram.number_of_vertices() << endl;
	cout << " V Halfedges " << voronoi_diagram.number_of_halfedges() << endl;
	cout << " V Faces " << voronoi_diagram.number_of_faces() << endl;
	cout << endl;
	char filename[100];
	sprintf( filename, "%s%s", base_img_dir, "save_img_CGAL_Voronoi.BMP");
	save_img_CGAL_voronoi_diagram( filename );
	
	//CGAL_VD::Vertex_iterator v_it, v_it_end; 
	//v_it     = voronoi_diagram.vertices_begin();
	//v_it_end = voronoi_diagram.vertices_end();
	//cout << " Go through vertices and check degree " << endl;
	//for ( int i = 0 ; v_it != v_it_end; ++v_it, ++i) {
	//	cout << " ~~~~~~~~VERTEX~~~~~~~~ " << i << endl;
	//	cout << v_it->point().x() << ":" << v_it->point().y() 
	//	     << " d=" << v_it->degree() << endl;
	//}
	//VD_Point_2 VP1;
	//CGAL_VD::Edge_iterator edge_it, edge_it_end; 
	//edge_it_end = voronoi_diagram.edges_end();
	//edge_it     = voronoi_diagram.edges_begin();
	//cout << " Go through edges " << endl;
	//for ( int i = 0 ; edge_it != edge_it_end; ++edge_it, ++i) {
	//	cout << " ~~~~~~~~(HALF)-EDGE~~~~~~~~ " << i << endl;
	//	if ( edge_it->has_source() ) {
	//		VP1 = edge_it->source()->point();
	//		cout << VP1.x() << ":" << VP1.y() << endl;
	//	}
	//	else
	//		cout << " infity vertex " << endl;
	//	if ( edge_it->has_target() ) {
	//		VP1 = edge_it->target()->point();
	//		cout << VP1.x() << ":" << VP1.y() << endl;
	//	}
	//	else {
	//		cout << " infity vertex " << endl;
	//	}
	//}
	
	/* Draw the dual of the dual into a stream for an IplImage */
	IplImage* my_img = cvCreateImage( img_map_size, 8, 3 );
	custom_ostream my_stream( my_img );
	voronoi_diagram.dual().draw_dual( my_stream );
	for ( int i = 0; i < n_polygons; ++i )
		write2img_polygon( my_img, poly_boundary_simple[i] );
	write2img_CGAL_voronoi_vertices( my_img );
	
	sprintf( filename, "%s%s", base_img_dir, "save_img_CGAL_Real_Voronoi.BMP");
	cvSaveImage( filename, my_img );
	cvReleaseImage(&my_img);
}

void sg_map::imprint_CGAL_voronoi_diagram() {
	char filename[100];
	IplImage* my_img = cvCreateImage( img_map_size, 8, 1 );
	custom_ostream my_stream( my_img );	
	
	write2img_occupancy( my_img );
	//voronoi_diagram.dual().draw_dual( my_stream );
	voronoi_diagram.dual().draw_skeleton( my_stream );

	sprintf( filename, "%s%s", base_img_dir, "save_img_voronoi_IMPRINT.BMP");
	cvSaveImage( filename, my_img );
	
	IplImage* thresholded = cvCreateImage( img_map_size, 8, 1 );
	int threshold = 230, pixel = 0;
	
	cvThreshold( my_img, thresholded, 3, 
	             double(threshold), CV_THRESH_BINARY );
	
	int i, j;
	for ( i = 0; i < thresholded->height; i++ ) { //nrows
		for ( j = 0; j < thresholded->width; j++ ) { //ncols
			pixel = int (((uchar*)(thresholded->imageData + thresholded->widthStep*i))[j]);
			if (pixel == threshold) {
				voronoi_matrix(i,j) = 1;
			}
		}
	}
	cvReleaseImage(&thresholded);
	cvReleaseImage(&my_img);
}

/* Author: Andreas Kolling
 * Parses voronoi_diagram and finds critical points for each edge
 */

void sg_map::process_CGAL_voronoi_diagram() {
	// find an edges whose vertex points are both in free space
	VD_Point_2 VP1, VP2;
	CGAL_VD::Edge_iterator edge_it, edge_it_end; 
	edge_it_end = voronoi_diagram.edges_end();
	edge_it     = voronoi_diagram.edges_begin();
	for ( ; edge_it != edge_it_end; ++edge_it ) {
		if ( edge_it->is_segment() ) {
			VP1 = edge_it->source()->point();
			VP2 = edge_it->target()->point();
			if ( occ( int(VP1.x()), int(VP1.y()) ) == 0 
			  && occ( int(VP2.x()), int(VP2.y()) ) == 0  ) {
				break;
			}
		}
	}
	//Start visiting the graph
	//cout << " - visit vertices in CGAL Voronoi Diagram " << endl;
	visit_VD_vertex( *(edge_it->source()) );
	char filename[100];
	//cout << " - saving SG_graph & critical points/lines ... ";
	sprintf( filename, "%s%s", base_img_dir, "save_img_SG_graph.BMP");
	save_img_graph( filename );	
	
	cout << " - construct vertex weights is SG_graph " << endl;
	construct_weights_SG_graph();
	
	//Print graph
	//cout << " - saving SG_graph & critical points/lines ... ";
	sprintf( filename, "%s%s", base_img_dir, "save_img_SG_graph.BMP");
	save_img_graph( filename );	
	//sprintf( filename, "%s%s", base_img_dir, "save_img_SG_crits_all.BMP");
	//save_img_sg_vertex_all_crits( filename );	
	//sprintf( filename, "%s%s", base_img_dir, "save_img_SG_crits0.BMP");
	//save_img_sg_vertex_crits( filename, 0 );
	//sprintf( filename, "%s%s", base_img_dir, "save_img_SG_crits1.BMP");
	//save_img_sg_vertex_crits( filename, 1 );
	//sprintf( filename, "%s%s", base_img_dir, "save_img_SG_crits2.BMP");
	//save_img_sg_vertex_crits( filename, 2 );
	cout << " done " << endl;
}

sg_vertex_d sg_map::visit_VD_vertex( VD_Vertex vert ) {
	//cout << "*VISIT V " << vert.point().x() << ":" << vert.point().y() << endl;
	set_vertex_visited( vert );
	// this is the new vertex in the surveillance graph G_g for vert
	sg_vertex_d sg_v1 = add_new_vertex( vert.point().x(), vert.point().y() );
	//cout << " VISITED VERTEX AND CREATED  SG_V " << sg_v1 << endl;
	vd_sg_pairs.push_back( vd_sg_vertex(vert,sg_v1) );
	G_g[sg_v1].true_x = vert.point().x(); 
	G_g[sg_v1].true_y = vert.point().y(); 
	//cout << " true x y " << G_g[sg_v1].true_x << ":" << G_g[sg_v1].true_y << endl;
	//cout << " int  x y " << G_g[sg_v1].x << ":" << G_g[sg_v1].y << endl;
	G_g[sg_v1].w = 1;
	G_g[sg_v1].v = vert;
	// check all unvisited edges
	VD_Vertex::Halfedge_around_vertex_circulator edge_circ1, edge_circ2;
	edge_circ1 = vert.incident_halfedges();
	edge_circ2 = edge_circ1;
	
	VD_Vertex neigh_v;
	int n_halfedge = 1;
	do {
		//cout << sg_v1 << " check Halfedge " << n_halfedge << endl; 
		++n_halfedge;
		// start visiting the edge (dereference is a halfedge)
		if ( is_edge_valid( *edge_circ1 ) == true ) {
			//cout << "         valid ";
			if ( is_edge_visited( *edge_circ1 ) == false ) {
				//cout << " unvisited" << endl;
				// VISIT EDGE *edge_circ1
				set_edge_visited( *edge_circ1 );
				if ( vert == *(edge_circ1->source()) ) 
					neigh_v = *(edge_circ1->target());
				else
					neigh_v = *(edge_circ1->source());
				// Neigh_v may already be visited
				if ( is_vertex_visited( neigh_v ) ) {
					// Really only a cycle close
					//cout << " Both HalfEdges unvisited, but neigh vertex is visited" << endl;
					//cout << " Hence a cycle " << endl;
					sg_vertex_d sg_v2;
					bool success = find_sg_vertex( neigh_v, sg_v2 );
					if ( success ) {
						//cout << " Closing cycle " << sg_v1 << " <-> " << sg_v2 << endl;
						sg_edge_d   sg_e  = add_new_edge( sg_v1, sg_v2 );
						G_g[sg_e].e = *edge_circ1;
						G_g[sg_e].w = 1;
					}
					else {
						//cout << " ERROR VD_VERTEX SHOULD EXIST " << endl;
					}
					
					//cout << " Try CYC CLOS " << sg_v1 << " " << sg_v2 << endl;
					//cout << "   " << neigh_v.point().x() << " " << neigh_v.point().y() << endl;
					//if ( (!success) || sg_v2 == sg_v1 ) {
					//	if ( success ) {
					//		// then sg_v1 has indeed exactly the same coordinates as this new
					//		// vertex and is degenerate
					//		// VISITE EDGE - GO TO VERTEX
					//	}
					//	else {
					//		// we did not find an existing vertex with exactly these coordinates
					//		// hence the is_vertex_visited should be false
					//		// VISITE EDGE - GO TO VERTEX
					//		
					//	}
					//	cout << "No Cycle - vertex is just very close " << endl;
					//	// we got a problem, a degenerate vertex
					//	sg_vertex_d sg_v2 = visit_VD_vertex( neigh_v );
					//	// Create an edge for this vertex;
					//	sg_edge_d   sg_e  = add_new_edge( sg_v1, sg_v2 );
					//	G_g[sg_e].e = *edge_circ1;
					//	G_g[sg_e].w = 1;
					//}
					//else if ( success ) {
					//	// another vertex may be on the same spot if success == false
					//	cout << "Closing cycle between vertices " << sg_v1 << " & " << sg_v2 << endl;
					//	sg_edge_d   sg_e  = add_new_edge( sg_v1, sg_v2 );
					//	G_g[sg_e].e = *edge_circ1;
					//	G_g[sg_e].w = 1;
					//}
				}
				else {
					// VISITE EDGE - GO TO VERTEX
					sg_vertex_d sg_v2 = visit_VD_vertex( neigh_v );
					// Create an edge for this vertex;
					sg_edge_d   sg_e  = add_new_edge( sg_v1, sg_v2 );
					G_g[sg_e].e = *edge_circ1;
					G_g[sg_e].w = 1;
				}
			}
			else {
				//cout << " halfedge already visited " << endl;
			}
			//cout << endl;
		}
		else {
			//cout << " halfedge not valid " << endl;
		} 
	} while ( ++edge_circ1 != edge_circ2 );
	//cout << "-RETURN F " << vert.point().x() << ":" << vert.point().y() << endl;
	return sg_v1;
}

bool 
sg_map::find_sg_vertex( VD_Vertex v , sg_vertex_d& vd ) {
	vd_sg_vertex_vector::iterator v_it,v_it_end;
	v_it_end = vd_sg_pairs.end();
	v_it = vd_sg_pairs.begin();
	for ( ; ;++v_it) {
		if ( v_it->first == v ) {
			vd = v_it->second;
			return true;
		}
	}
	return false;
}


bool 
sg_map::find_sg_vertex( double x, double y, sg_vertex_d& vd ) {
	//sg_vertex_it vert_it, vert_it_end;
	//tie(vert_it, vert_it_end) = vertices(G_g);
	//for (; vert_it != vert_it_end ; ++vert_it ) {
	//	vd = vertex(*vert_it, G_g); //get the vertex descriptor
	//	if ( G_g[vd].true_x == x && G_g[vd].true_y == y) {
	//		cout << " Found SG VERTEX " << x << ":" << y << endl;
	//		return true;
	//	}
	//}
	return false;
}

void sg_map::construct_weights_SG_graph() {
	// go through all vertices
	sg_vertex_d  vd;
	sg_vertex_it vert_it, vert_it_end;
	sg_o_edge_it out_edge_it, out_edge_it_end;
	VD_Vertex    VD_v;
	VD_Halfedge  VD_e;
	tie(vert_it, vert_it_end) = vertices(G_g);
	for (; vert_it != vert_it_end; ++vert_it) {
		vd   = *vert_it;
		tie(out_edge_it, out_edge_it_end) = out_edges( vd, G_g);
		int n_edges = 0;
		for (;out_edge_it != out_edge_it_end; out_edge_it++) {
			//cout << "V=" << vd << " " << G_g[vd].x << ":" << G_g[vd].y << endl;
			VD_e = G_g[*out_edge_it].e;
			critical_points( VD_e.dual(), vd, n_edges );
			++n_edges;
		}
	}
	
	sg_edge_d ed;
	sg_edge_it edge_it, edge_it_end;
	tie(edge_it,edge_it_end) = edges(G_g);
	for (; edge_it != edge_it_end; ++edge_it) {
		critical_edge_points( G_g[*edge_it].e.dual(), *edge_it );
	}
}

void sg_map::critical_points( SDG::Edge e, sg_vertex_d vd, int n_edges ) {
	/* Sites A, B, C define one vertex, Sites A, B, D the other */
	//cout << "   *** Examining crit pts for vertex " << int(vd)  << endl;
	VD_Vertex VD_v = G_g[vd].v;
	SDG::Vertex_handle v[] = 
	     { e.first->vertex( voronoi_diagram.dual().ccw(e.second) ),
	       e.first->vertex( voronoi_diagram.dual().cw(e.second) ),
	       e.first->vertex( e.second ),
	       voronoi_diagram.dual().tds().mirror_vertex(e.first, e.second) };
	double dis1, dis2, dist_total, min_dist = std::numeric_limits<double>::max();
	double min_dis1, min_dis2;
	SDG::Point_2 pt_crit1, pt_crit2, pt_crit3;
	// check whether third site is 2(C) or 3(D) for v
	int v_site_i = 0; 
	for ( int site_i = 2 ; (site_i < 4) && (v_site_i == 0) ; ++site_i ) {
		for ( int i = 0; i < 3 ; ++i ) {
			if ( VD_v.site(i) == v[site_i] )
				v_site_i = site_i;
		}
	}
	// some diagnostics
	//cout << " defining sites " << endl;
	//cout << v[0]->site() << endl;
	//cout << v[1]->site() << endl;
	//cout << v[v_site_i]->site() << endl;
	// some diagnostics - end
	if ( v[v_site_i]->site().is_segment() ) {
		VD_Segment_2 seg = v[v_site_i]->site().segment();
		// We need to walk on the segment to find the minimum distance pt
		SDG::Point_2 pt_src = seg.source();
		SDG::Point_2 pt_tgt = seg.target();
		double v_x = pt_tgt.x() - pt_src.x();
		double v_y = pt_tgt.y() - pt_src.y();
		SDG::Point_2 pt_tmp_crit1, pt_tmp_crit2, pt_tmp_crit3;
		// going through points
		//cout << " Opposite site C/D is a segment " << endl;
		double facto;
		for ( int i = 0; i <= LINE_DISCRETE_STEPS ; ++i ) {
			facto = double(i)/double(LINE_DISCRETE_STEPS);
			//cout << "FAKTOR " << facto << " " << i << " " << endl;
			SDG::Point_2 pt(pt_src.x()+facto*v_x, pt_src.y()+facto*v_y);
			//cout << " Checking point " << pt << " on segment ";
			// not much to do but to calculate the distances
			dis1 = distance_to_site( v[0]->site(), pt, pt_tmp_crit1 );
			dis2 = distance_to_site( v[1]->site(), pt, pt_tmp_crit3 );
			dist_total = dis1 + dis2;
			//cout << " dist is " << dist_total << endl;
			pt_tmp_crit2 = pt;
			if ( dist_total < min_dist ) {
				pt_crit1 = pt_tmp_crit1; 
				pt_crit2 = pt_tmp_crit2; 
				pt_crit3 = pt_tmp_crit3;
				min_dist = dist_total; min_dis1 = dis1; min_dis2 = dis2;
			}
		}
	}
	// SITE C or D is a point
	if ( v[v_site_i]->site().is_point() ) {
		
		SDG::Point_2 pt = v[v_site_i]->site().point();
		// not much to do but to calculate the distances
		dis1 = distance_to_site( v[0]->site(), pt, pt_crit1 );
		dis2 = distance_to_site( v[1]->site(), pt, pt_crit3 );
		dist_total = dis1 + dis2;
		// the center critical points is pt itself
		pt_crit2 = pt;
		min_dist = dist_total; min_dis1 = dis1; min_dis2 = dis2;
	}
	// We now have 
	
	G_g[vd].ww[n_edges] = number_of_robots( min_dis1 ) 
	                    + number_of_robots( min_dis2 );
	//cout << " -------> " << n_edges << " direction crits are:" << endl;
	G_g[vd].crit_p[n_edges][0] = pt_crit1;
	G_g[vd].crit_p[n_edges][1] = pt_crit2;
	G_g[vd].crit_p[n_edges][2] = pt_crit3;
	//cout << pt_crit1 << endl;
	//cout << pt_crit2 << endl;
	//cout << pt_crit3 << endl;
	
}

void sg_map::critical_edge_points( SDG::Edge e, sg_edge_d ed ) {
	//cout << "   *** Examining crit pts for edge " 
	//     << int(source(ed,G_g)) << "<->" << int(target(ed,G_g)) << endl;
	SDG::Vertex_handle v[] = 
	     { e.first->vertex( voronoi_diagram.dual().ccw(e.second) ),
	       e.first->vertex( voronoi_diagram.dual().cw(e.second) ) };
	SDG::Point_2 p1, p2;
	
	//cout << " defining sites are " << endl;
	//cout << v[0]->site() << " " << v[1]->site() << endl;
	
	double dist = -1;
	
	if ( v[0]->site().is_segment() && v[1]->site().is_segment() ) {
		dist = distance_site_to_site( v[0]->site().segment(), 
		                              v[1]->site().segment(), p1,  p2);
	}
	else {
		if ( v[0]->site().is_point() ) {
			p1 = v[0]->site().point();
			dist = distance_to_site( v[1]->site(), p1, p2  );
		}
		else if ( v[1]->site().is_point()) {
			p2 = v[1]->site().point();
			dist = distance_to_site( v[0]->site(), p2, p1  );
		}
		else 
			cout << " ERROR: NO SITE IS A POINT NOR ARE BOTH SEGMENTS" << endl;
	}
	
	G_g[ed].w = number_of_robots(dist);
	G_g[ed].crit_p[0] = p1;
	G_g[ed].crit_p[1] = p2;
	//cout << " RESULT: G_g[ed].w " << G_g[ed].w << endl;
	//cout << p1 << "  " << p2 << endl;
}

int sg_map::number_of_robots( double d ) {
	// defines how many robots are needed to cover a line segment length d
	if ( d < minimum_gap )
		return 0;
	return int( ceil( d / ( sense_range_pix - sense_delta ) ) );
}

void sg_map::save_as_img_critical_points( SDG::Edge e ) {
	IplImage* img = cvCreateImage( img_map_size, 8, 3 );
	SDG::Vertex_handle v[] = 
	     { e.first->vertex( voronoi_diagram.dual().ccw(e.second) ),
	       e.first->vertex( voronoi_diagram.dual().cw(e.second) ),
	       e.first->vertex( e.second ),
	       voronoi_diagram.dual().tds().mirror_vertex(e.first, e.second) };
	for (int i = 0; i < 4; i++) {
		if ( voronoi_diagram.dual().is_infinite(v[i]) ) {
		  // This should never happen !
		} 
		else {
			//cout << vid[i] << ": " << v[i]->site() << endl;
			if ( v[i]->site().is_point() ) {
				SDG::Point_2 P1 = v[i]->site().point(); 
				cvLine(img, 
				  cvPoint( P1.y(),P1.x() ), 
				  cvPoint( P1.y(),P1.x() ), 
				  cvScalar(0,0,220), 1);
			}
			else if ( v[i]->site().is_segment() ) {
				SDG::Point_2 P1 = v[i]->site().source();
				SDG::Point_2 P2 = v[i]->site().target();
				cvLine(img, 
				  cvPoint( P1.y(),P1.x() ), 
				  cvPoint( P2.y(),P2.x() ), 
				  cvScalar(0,0,220), 1);
			}
		}
	}
	char filename[100];
	sprintf( filename, "%s%s", base_img_dir, "save_img_voronoi_edge.BMP");
	cvSaveImage( filename, img );
	cvReleaseImage(&img);
}

void sg_map::set_vertex_visited( VD_Vertex v ) {
	visited_vertices.insert(v);
	//if ( bound_check( v.point().x() , v.point().y() ) )
	//	vertex_visited( v.point().x(), v.point().y() ) = 1;
}

bool sg_map::is_vertex_visited( VD_Vertex v ) {
	//std::list<VD_Vertex>::iterator it =  ;
	return ( visited_vertices.find( v ) != visited_vertices.end() );
	//return bool( vertex_visited( v.point().x(), v.point().y() ) );
}

void sg_map::set_edge_visited( CGAL_VD::Halfedge e ) {
	//if ( e.is_segment() ) {
	//	VD_Point_2 VP1 = e.source()->point();
	//	VD_Point_2 VP2 = e.target()->point();
	//	int i = int ( (VP1.x() + VP2.x())/2 );
	//	int j = int ( (VP1.y() + VP2.y())/2 );
	//	if ( bound_check(i,j) )
	//		edge_visited( i, j ) = 1;
	//}
	visited_halfedges.insert(e);
}

bool sg_map::is_edge_visited( CGAL_VD::Halfedge e ) {
	//std::list< CGAL_VD::Halfedge >::iterator it = visited_halfedges.find( e) ;
	bool really_visited = false;
	if ( visited_halfedges.find( e ) != visited_halfedges.end() )
		really_visited = true;
	if ( visited_halfedges.find( *(e.twin()) ) != visited_halfedges.end())
		really_visited = true;
	return really_visited;
	//if ( e.is_segment() ) {
	//	VD_Point_2 VP1 = e.source()->point();
	//	VD_Point_2 VP2 = e.target()->point();
	//	int i = int ( (VP1.x() + VP2.x())/2 );
	//	int j = int ( (VP1.y() + VP2.y())/2 );
	//	if ( edge_visited( i, j ) == 1  ) {
	//		return true;
	//	}
	//}
	//return false;
}

bool sg_map::is_edge_valid( CGAL_VD::Halfedge e ) {
	if ( e.is_segment() ) {
		VD_Point_2 VP1 = e.source()->point();
		VD_Point_2 VP2 = e.target()->point();
		if ( occ( int(VP1.x()), int(VP1.y()) ) == 0 
		  && occ( int(VP2.x()), int(VP2.y()) ) == 0  ) {
			return true;
		}
	}
	return false;
}


double sg_map::distance_to_sites( VD_Site_2 s1, VD_Site_2 s2, SDG::Point_2 pt ) {
	double dis1 = distance_to_site( s1, pt );
	double dis2 = distance_to_site( s2, pt );
	return (dis1 + dis2);
}

double sg_map::distance_to_site( VD_Site_2 s, SDG::Point_2 pt ) {
	if ( s.is_segment() ) {
		VD_Segment_2 site_seg = s.segment();
		return sqrt(double(CGAL::squared_distance( site_seg, pt )));
	}
	else if ( s.is_point() ) {
		SDG::Point_2 site_pnt  = s.point();
		return sqrt(double(CGAL::squared_distance( site_pnt, pt )));
	}
	cout << "ERROR distance_to_site" << endl;
	return -1;
}

/* Author: Andreas Kolling
 * 
 * This one returns the closest point c_pt on s to the point pt and the distance
 */

double sg_map::distance_to_site( VD_Site_2 s, SDG::Point_2 pt, SDG::Point_2& c_pt  ) {
	if ( s.is_segment() ) {
		VD_Segment_2 site_seg = s.segment();
		SDG::Point_2 p1 = site_seg.source();
		SDG::Point_2 p2 = site_seg.target();
		
		double x,y, dist;
		dist = point_to_segment_distance( p1.x(), p1.y(), p2.x(), p2.y(), 
		                                  pt.x(), pt.y(), &x,      &y );
		SDG::Point_2 new_pt(x,y);
		c_pt = new_pt;
		return dist;
	}
	else if ( s.is_point() ) {
		SDG::Point_2 site_pnt  = s.point();
		c_pt = site_pnt;
		return sqrt(double( CGAL::squared_distance( site_pnt, pt ) ));
	}
	cout << "ERROR in distance_to_site" << endl;
	return -1;
}

double sg_map::distance_site_to_site( VD_Segment_2 s1, VD_Segment_2 s2, 
                                      SDG::Point_2& p1, SDG::Point_2& p2) 
{
	SDG::Point_2 s1p1 = s1.source();
	SDG::Point_2 s1p2 = s1.target();
	SDG::Point_2 s2p1 = s2.source();
	SDG::Point_2 s2p2 = s2.target();
	
	double x,y, dist, m_dist, m1_x,m1_y,m2_x,m2_y;
	m2_x = s2p1.x();
	m2_y = s2p1.y();
	m_dist = point_to_segment_distance( s1p1.x(), s1p1.y(), s1p2.x(), s1p2.y(),
	                                    m2_x, m2_y, &m1_x, &m1_y );
	
	dist = point_to_segment_distance( s1p1.x(), s1p1.y(), s1p2.x(), s1p2.y(),
	                                  s2p2.x(), s2p2.y(), &x,      &y );
	if ( dist < m_dist ) {
		m_dist = dist;
		m1_x = x; 
		m1_y = y;
		m2_x = s2p2.x();
		m2_y = s2p2.y();
	}
	
	dist = point_to_segment_distance( s2p1.x(), s2p1.y(), s2p2.x(), s2p2.y(),
	                                  s1p1.x(), s1p1.y(), &x,      &y );
	if ( dist < m_dist ) {
		m_dist = dist;
		m2_x = x; 
		m2_y = y;
		m1_x = s1p1.x();
		m1_y = s1p1.y();
	}
	dist = point_to_segment_distance( s2p1.x(), s2p1.y(), s2p2.x(), s2p2.y(),
	                                  s1p2.x(), s1p2.y(), &x,      &y );
	if ( dist < m_dist ) {
		m_dist = dist;
		m2_x = x; 
		m2_y = y;
		m1_x = s1p2.x();
		m1_y = s1p2.y();
	}
	
	SDG::Point_2 on_s1( m1_x, m1_y );
	SDG::Point_2 on_s2( m2_x, m2_y );
	p1 = on_s1;
	p2 = on_s2;
	return m_dist;
}

double sg_map::point_to_segment_distance(double nSx1, double nSy1, double nSx2,
                                    double nSy2, double nPx, double nPy,
                                    double *nCercaX,  double *nCercaY)
{
	// Y=m*X + b
	double m, b;
	bool HorizontalOVertical = false;
	// vertical
	if (nSx2 == nSx1){
	        *nCercaX = nSx1;
	        *nCercaY = nPy;
	        HorizontalOVertical = true;
	}
	// horizontal
	if (nSy2 == nSy1){
	        *nCercaY = nSy1;
	        *nCercaX = nPx;
	        HorizontalOVertical = true;
	}
	// AX + BY + C = 0 
	if (!HorizontalOVertical)
	{
	        m= (nSy2 - nSy1) / (nSx2 - nSx1);
	        b = nSy1 - (nSx1 * m);
	        // Y=m1*X + b1
	        double m1, b1;
	        m1 = -1 / m;
	        b1 = -m1 * nPx + nPy;
	        *nCercaX =  (b1 - b) / (m - m1);
	        *nCercaY =  m * *nCercaX + b;
	}
	if (*nCercaX < min(nSx1,nSx2)
	        ||  *nCercaX > max(nSx1,nSx2)
	        ||  *nCercaY < min(nSy1,nSy2)
	        ||  *nCercaY > max(nSy1,nSy2) ) 
	{
	        if (point_to_point_distance(nSx1, nSy1, nPx, nPy)
	        <= point_to_point_distance(nSx2, nSy2, nPx, nPy))
	        {
	                *nCercaX = nSx1;
	                *nCercaY = nSy1;
	        }
	        else
	        {
	                *nCercaX = nSx2;
	                *nCercaY = nSy2;
	        }
	}
	return point_to_point_distance(nPx, nPy, *nCercaX , *nCercaY);
}

double sg_map::point_to_point_distance(double lOldX, double lOldY,
                               double lNewX, double lNewY)
{
	double x = lNewX - lOldX;
	double y = lNewY - lOldY;
	return sqrt(pow(x,2) + pow(y,2));
}


void sg_map::show_voronoi_diagram() {
	//***** Get to know some stuff about the Voronoi Diagram
	cout << " DETAILS ON THE VORONOI DIAGRAM" << endl;
	CGAL_VD::Vertex_iterator v_it, v_it_end; 
	v_it     = voronoi_diagram.vertices_begin();
	v_it_end = voronoi_diagram.vertices_end();
	cout << " Go through vertices " << endl;
	for ( int i = 0 ; v_it != v_it_end; ++v_it, ++i) {
		cout << " ~~~~~~~~VERTEX~~~~~~~~ " << i << endl;
		cout << v_it->point().x() << ":" << v_it->point().y() << endl;
	}
	VD_Point_2 VP1;
	CGAL_VD::Edge_iterator edge_it, edge_it_end; 
	edge_it_end = voronoi_diagram.edges_end();
	edge_it     = voronoi_diagram.edges_begin();
	cout << " Go through edges " << endl;
	for ( int i = 0 ; edge_it != edge_it_end; ++edge_it, ++i) {
		cout << " ~~~~~~~~(HALF)-EDGE~~~~~~~~ " << i << endl;
		if ( edge_it->has_source() ) {
			VP1 = edge_it->source()->point();
			cout << VP1.x() << ":" << VP1.y() << endl;
		}
		else
			cout << " infity vertex " << endl;
		if ( edge_it->has_target() ) {
			VP1 = edge_it->target()->point();
			cout << VP1.x() << ":" << VP1.y() << endl;
		}
		else {
			cout << " infity vertex " << endl;
		}
	}
}

/* Author: Andreas Kolling
 * 
 * simplifies polygon poly_boundary[ poly_id ]
 */

void sg_map::simplify_polygon( int poly_id ) {
	std::vector<int> final_indices;
	
	dp_simplify( poly_boundary[ poly_id ], final_indices );
	sort( final_indices.begin(), final_indices.end() );
	
	cout << " Polygon " << poly_id << " simplified from " 
	     << poly_boundary[ poly_id ].size() << " "
	     << " to " 
	     << final_indices.size() << " points" << endl;
	
	Polygon new_poly;
	for( int i = 0; i < int(final_indices.size()) ; ++i ) {
		new_poly.push_back( poly_boundary[ poly_id ][final_indices[i]] );
	}
	poly_boundary_simple.push_back(new_poly);
}

void sg_map::find_split( int i, int j, int *split, double *dist, Polygon& P) {
	int k;
	HOMOG q;
	double tmp;
	*dist = -1;
	if (i + 1 < j)
	{
		/* out of loop portion of distance computation */ 
		CROSSPROD_2CCH(P[i], P[j], q); 
		for (k = i + 1; k < j; k++)
		{
			tmp = DOTPROD_2CH(P[k], q); /* distance computation */
			//cout << " Dist " << tmp << " of " << P[k].x() << " " << P[k].y() << endl;
			if (tmp < 0) tmp = - tmp;   /* calling fabs() slows us down */
			if (tmp > *dist) 
			{
				*dist  = tmp;	            /* record the maximum */
				*split = k;
			}
		}
		/* correction for segment */
		*dist *= *dist/(q[XX]*q[XX] + q[YY]*q[YY]); 
	} 
	/* length---should be redone if can == 0 */
}

//Douglas Peucker Line Simplification
void sg_map::dp_simplify( Polygon& P, std::vector<int>& final_indices ) 
{
	int n_points = P.size();
	// collect indices in stack
	stack<int> my_stack;
	double epsilon_sq = simplify_epsilon*simplify_epsilon;
	int i = 0, split_index; 
	double dist_sq;
	bool outFlag = true;
	//cout << "    Simplifying " << n_points << " points with epsilon " 
	//     << simplify_epsilon << endl;
	my_stack.push(n_points-2); //last point in stack now
	do {
		find_split(i, my_stack.top(), &split_index, &dist_sq, P );
		//cout << " SPLIT AT " << dist_sq << " ind= "<< split_index << endl;
		if ( dist_sq > epsilon_sq ) 
			my_stack.push(split_index);
		else
		{
			if ( outFlag ) { // add the first index
				outFlag = false;
				final_indices.push_back(i);
			}
			i = my_stack.top(); my_stack.pop();
			final_indices.push_back(i);
	  }
  } while (! (my_stack.empty()) );
	//cout << "     Simplified to " << final_indices.size() << " points " << endl;
}

//******* Segmenting the Voronoi Diagram ******* (fold)

int sg_map::f_v_number_non_segmented_neighbors( int i, int j) {
	int n_x, n_y, ii, jj, case_nr, n_neighbors = 0;
	for (case_nr = 0; case_nr < 8; case_nr++) {
		get_eight_case(case_nr, ii, jj); n_x = i + ii; n_y = j + jj;
		if ( bound_check(n_x,n_y) 
		       && fortune_voronoi(n_x,n_y) > 0 
		       && fortune_v_segment(n_x,n_y) == 0 ) {
			++n_neighbors;
		}
	}
	return n_neighbors;
}

void sg_map::f_v_parse_segments() {
	fortune_v_segment.set_elements(0);
	// find a first voronoi point
	int i,j;
	bool found = false;
	for ( i = 0; i < nrows	&& found == false; ++i ) {
		for ( j = 0; j < ncols && found == false; ++j ) {
			if ( this->fortune_voronoi(i,j) > 0 && f_v_number_non_segmented_neighbors( i, j) == 1 ) {
				if ( DEBUG_FO_PA >= 1 )
					cout << " FOUND " << i << "," << j << " as parsing starting point." << endl;
				found = true;
			}
		}
	}
	--j;--i;
	if ( DEBUG_FO_PA >= 1 )
		cout << " Starting to parse at " << i << "," << j << endl;	
	f_v_visit_point_mod( i, j, -1 );
}

/* Author: Andreas Kolling
 * 
 * Associate boundary points of obstacles with Voronoi edge segments
 * 1) Go through all voronoi points in voronoi_points
 */

void sg_map::f_v_associate_segments() {
  pointvector::iterator n_start = voronoi_points.begin();
	pointvector::iterator n_end		= voronoi_points.end();
	pointvector::iterator n;
	int i,j;
	map_point_list::iterator runner;
	
	for ( n = voronoi_points.begin(); n != voronoi_points.end() ; ++n ) {
  	i = (*n).x; j = (*n).y; // Voronoi Point (i,j)
		runner = (*closest_obstacle)[i][j].begin();
    // Go through all closest obstacles points from i,j
		for(; runner != (*closest_obstacle)[i][j].end(); runner++) {
			belongs_to_segment( (*runner).first, (*runner).second ) = 
			     fortune_v_segment( i, j );
	  }
	}
}

void sg_map::f_v_visit_point( int i, int j, int current_segment ) {
	bool new_segment = false;
	if ( current_segment == 0 ) {
		new_segment = true;
		current_segment = fortune_v_segment( i , j ); // load proper segment
	}
	else
		fortune_v_segment( i , j ) = current_segment; // assign current segment

	int n_x, n_y, ii, jj, case_nr;
	int n_non_seg_neighbors = f_v_number_non_segmented_neighbors( i, j );
	
	if ( DEBUG_FO_PA >= 2 ) {
		cout << "	 Visiting point " << i << "," << j << " in segment " 
		     << current_segment << " with " << n_non_seg_neighbors 
		     << " neighbors " << endl;
	}
	int n_neighbors; 
	switch( n_non_seg_neighbors ) {
		case 0:
			// End point at dead-end or hitting a loop
			int nei_bors[9][2];
			n_neighbors = get_voronoi_neighbors( nei_bors, i, j);
			if ( n_neighbors == 1 ) {
			  // Not a loop --> dead end segment
				if ( DEBUG_FO_PA >= 3 ) {
					cout << "		CASE 0: FOUND A DEADEND SEGMENT " << current_segment 
					     << endl;
				}
				dead_end_segments.push_back( current_segment );
			}
			break;
		case 1:
			// one neighbor unassigned --> continue through the segment
			if ( DEBUG_FO_PA >= 3 )
				cout << "		CASE 1: Just going along the segment " << endl;			 
			for (case_nr = 0; case_nr < 8; case_nr++) {
				get_eight_case(case_nr, ii, jj);
				n_x = i + ii; n_y = j + jj;
				if ( bound_check(n_x,n_y) 
							&& fortune_voronoi(n_x,n_y) > 0 
							&& fortune_v_segment(n_x,n_y) == 0 ) {
					f_v_visit_point(n_x, n_y, current_segment);
				}
			}
			break;
		case 2:
			// two unassigned neighbors means a split
			// --> finish this segment & create two new segments
			if ( DEBUG_FO_PA >= 3 )
				cout << "		CASE 2: Split into two " << endl;
				
			// first make sure that no neighbor will recognize 
			// the current neighbors as an unassigned point			 
			int neighbors[2][3]; // third entry is the number of non segmented neighbors
			bool revisiting;
			revisiting = false;
			int c_i; // current i for the neighbors
			c_i = 0;
			for (case_nr = 0; case_nr < 8; case_nr++) {
				get_eight_case(case_nr, ii, jj); n_x = i + ii; n_y = j + jj;
				if ( bound_check(n_x,n_y) 
							&& fortune_voronoi(n_x,n_y) > 0 
							&& fortune_v_segment(n_x,n_y) == 0 ) {
					++num_voronoi_segments;
					fortune_v_segment( n_x, n_y ) = num_voronoi_segments;
					neighbors[c_i][0] = n_x;
					neighbors[c_i][1] = n_y;
					c_i++;
				}
			} 
			
			// now find out how many new neighbors both have
			for ( c_i = 0; c_i < 2; c_i++) {
				neighbors[c_i][2] = f_v_number_non_segmented_neighbors( neighbors[c_i][0], neighbors[c_i][1] );
				if ( DEBUG_FO_PA >= 4 )
					cout << "		 neighbor has itself " << neighbors[c_i][2] << " neighbors " << endl;
				if ( neighbors[c_i][2] == 0 ) {
					// kill it since it has no neighbors
					fortune_voronoi( neighbors[c_i][0], neighbors[c_i][1]) = 0;
					revisiting = true;
				}
				else if ( neighbors[c_i][2] == 3 ) {
					bool skip = false;
					for (case_nr = 0; case_nr < 8; case_nr++) {
						get_eight_case(case_nr, ii, jj); n_x = neighbors[c_i][0] + ii; n_y = neighbors[c_i][1] + jj;
						if ( bound_check(n_x,n_y) && fortune_voronoi(n_x,n_y) > 0 && fortune_v_segment(n_x,n_y) == 0 ) {
							if ( skip == true ) {
								fortune_voronoi(n_x,n_y) = 0;
								skip = false;
							}
							else
								skip = true;
						}
					}
				}
			} 

			if ( revisiting == true ) {
				fortune_v_segment( neighbors[0][0], neighbors[0][1]) = 0;
				fortune_v_segment( neighbors[1][0], neighbors[1][1]) = 0;
				num_voronoi_segments = num_voronoi_segments - 2; 
				f_v_visit_point( i , j, current_segment );
			}
			else {
				// since we are not revisiting both have neighbors
				// we now check for overlapping neighbors
				bool stop;
				stop = false;
				for (case_nr = 0; case_nr < 8 && stop == false ; case_nr++) {
					get_eight_case(case_nr, ii, jj); 
					n_x = neighbors[0][0] + ii; 
					n_y = neighbors[0][1] + jj;
					if ( bound_check(n_x,n_y) 
								&& fortune_voronoi(n_x,n_y) > 0
								&& fortune_v_segment(n_x,n_y) == 0 ) {
						// check whether point is also a neighbor of neighbors[1][0] neighbors[1][1]
						if ( distance( neighbors[1][0] , neighbors[1][1], n_x, n_y ) < 1.5) {
							if ( DEBUG_FO_PA >= 4 )
								cout << "		 neighbors have joint neighbors " << n_x << "," << n_y << endl;
							// decide what to do with the joint neighbor 
							if ( neighbors[0][2] == 1 && neighbors[1][2] ) {
								if ( DEBUG_FO_PA >= 4 )
									cout << "		 BOTH NEED IT -- kill one neighbor " << endl;
								fortune_voronoi( neighbors[0][0], neighbors[0][1]) = 0;
								revisiting = true;
								stop = true;									
							}
							else if ( neighbors[0][2] == 1 ) {
								if ( DEBUG_FO_PA >= 4 )
									cout << "		 --> " << neighbors[0][0] << "," << neighbors[0][1] << " needs it" << endl;
								
								// this guy needs this neighbor 
								// we do this by assigning it the segment already
								fortune_v_segment( n_x, n_y ) = fortune_v_segment( neighbors[0][0], neighbors[0][1]);
								// now we switch to this piont
								neighbors[0][0] = n_x;
								neighbors[0][1] = n_y;
								stop = true;
							}
							else if ( neighbors[1][2] == 1 ) {
								if ( DEBUG_FO_PA >= 4 )
									cout << "		 --> " << neighbors[1][0] << "," << neighbors[1][1] << " needs it" << endl;
								
								fortune_v_segment( n_x, n_y ) = fortune_v_segment( neighbors[1][0], neighbors[1][1]);
								// now we switch to this piont
								neighbors[1][0] = n_x;
								neighbors[1][1] = n_y;
								stop = true;
							}
						}
					}
				}
				if ( revisiting == true ) {
					fortune_v_segment( neighbors[0][0], neighbors[0][1]) = 0;
					fortune_v_segment( neighbors[1][0], neighbors[1][1]) = 0;
					num_voronoi_segments = num_voronoi_segments - 2; 
					f_v_visit_point( i , j, current_segment );
				}
				else {				
					for ( c_i = 0; c_i < 2; c_i++) {
						f_v_visit_point( neighbors[c_i][0], neighbors[c_i][1], 0);
					}
				}
			}
			break;
		case 3:
			// three unassigned neighbors -- split and remove middle neighbor
			// finish segment & create new segments at unassigned points
			if ( DEBUG_FO_PA >= 3 )
				cout << "		CASE 3: Split into two, remove middle point " << endl;
			
			bool skip = false;
			for (case_nr = 0; case_nr < 8; case_nr++) {
				get_eight_case(case_nr, ii, jj); n_x = i + ii; n_y = j + jj;
				if ( bound_check(n_x,n_y) && fortune_voronoi(n_x,n_y) > 0 && fortune_v_segment(n_x,n_y) == 0 ) {
					if ( skip == true ) {
						fortune_voronoi(n_x,n_y) = 0;
						skip = false;
					}
					else
						skip = true;
				}
			}
			// visit this point again, now it has one neighbor less
			if ( DEBUG_FO_PA >= 3 )
				cout << "		--> and revisit " << endl;
			
			f_v_visit_point( i , j, current_segment );
			break;
	}
}

/* Author: Andreas Kolling
 * 
 * Modified version of f_v_visit_point. (PARTIAL - NOT FINISHED)
 * 
 */

void sg_map::f_v_visit_point_mod( int i, int j, int current_segment ) {
	if ( DEBUG_FO_PA >= 2 )
		cout << "		Voronoi Segments: visiting: " << i << ":" << j << endl;

	int n_x, n_y, ii, jj, case_nr, nei_bors[9][2];
	int n_non_seg_neighbors = f_v_number_non_segmented_neighbors( i, j );
	int n_neighbors         = get_voronoi_neighbors( nei_bors, i, j);
  
	switch( n_non_seg_neighbors ) {
		case 0: // End point at dead-end or hit a loop
			if ( n_neighbors == 1 ) {
			  if ( DEBUG_FO_PA >= 2 )
      		cout << "		+- Dead end or single " << current_segment << endl;        
			  if ( current_segment > 0 ) { // Dead end
			    map_point new_segment_point(i,j);
			    points_on_edge[current_segment-1].push_back( new_segment_point );
				  fortune_v_segment( i , j ) = current_segment;
          dead_end_segments.push_back( current_segment );
			  }
			}
			break;
		case 1: // One neighbor unassigned
			if ( n_neighbors == 2 || n_neighbors == 1 ) { // == 1 only for first pt.
        // segment point
        map_point new_segment_point(i,j);
        if ( current_segment == -1 ) {
          // start segment point - create a new segment
          map_point_list new_point_list;
          current_segment = points_on_edge.size() + 1;
          points_on_edge.push_back( new_point_list );
          
          if ( DEBUG_FO_PA >= 2 )
        		cout << "		++ Created new segment " << current_segment << endl;
          
          points_on_edge[current_segment-1].push_back( new_segment_point );
          fortune_v_segment( i , j ) = current_segment;
        }
        else { // mid segment point
          if ( DEBUG_FO_PA >= 2 )
        		cout << "		+ Add midpoint to segment " << current_segment << endl;
          points_on_edge[current_segment-1].push_back( new_segment_point );
          fortune_v_segment( i , j ) = current_segment;
        }
        // Continue by visiting the one unassigned neighbor
        for (case_nr = 0; case_nr < 8; case_nr++) {
  				get_eight_case(case_nr, ii, jj); n_x = i + ii; n_y = j + jj;
  				if ( bound_check(n_x,n_y) 
  							&& fortune_voronoi(n_x,n_y) > 0 
  							&& fortune_v_segment(n_x,n_y) == 0 ) {
  					f_v_visit_point_mod(n_x, n_y, current_segment);
  				}
  			}
      }
      else {
        if ( DEBUG_FO_PA >= 2 )
      		cout << "		- Vertex point - visiting neighbors " << endl;

        map_point new_vertex_point(i,j);
        vertex_points.push_back( new_vertex_point );
        // mark as not part of a segment & do not visit again
        fortune_v_segment(i,j) = -1;
        // visit other neighbors to create new segments
  			for (case_nr = 0; case_nr < 8; case_nr++) {
  				get_eight_case(case_nr, ii, jj); n_x = i + ii; n_y = j + jj;
  				if ( bound_check(n_x,n_y) 
  					    && fortune_voronoi(n_x,n_y) > 0 
  					    && fortune_v_segment(n_x,n_y) == 0 ) {
              f_v_visit_point_mod(n_x, n_y, -1 );
  				}
  			}
      }
			break;
		default: // 2+ unassigned neighb. --> 3+ neighbors --> vertex point			
			if ( DEBUG_FO_PA >= 2 )
    		cout << "		- Vertex point - visiting neighbors " << endl;
      	
      map_point new_vertex_point(i,j);
      vertex_points.push_back( new_vertex_point );
      // mark as not part of a segment & do not visit again
      fortune_v_segment(i,j) = -1;
      // visit other neighbors to create new segments
			for (case_nr = 0; case_nr < 8; case_nr++) {
				get_eight_case(case_nr, ii, jj); n_x = i + ii; n_y = j + jj;
				if ( bound_check(n_x,n_y) 
					    && fortune_voronoi(n_x,n_y) > 0 
					    && fortune_v_segment(n_x,n_y) == 0 ) {
            f_v_visit_point_mod(n_x, n_y, -1 );
				}
			}
      break;
	} 
}

void sg_map::f_v_parse_vertex_points() {
  // go through vertex points and cluster them (find neighbors)
  map_point_list::iterator it     = vertex_points.begin();
  map_point_list::iterator it_end = vertex_points.end();
  for ( ; it != it_end ; ++it ) {
    f_v_check_vertex_cluster( (*it).first, (*it).second  );
  }
}

void sg_map::f_v_check_vertex_cluster( int i, int j ) {
  
  // check neighbors
  int n_x, n_y, ii, jj, case_nr;
	//int n_neighbors         = get_voronoi_neighbors( nei_bors, i, j);
  for (case_nr = 0; case_nr < 8; case_nr++) {
		get_eight_case(case_nr, ii, jj); n_x = i + ii; n_y = j + jj;
		if ( bound_check(n_x,n_y) 
			    && fortune_voronoi(n_x,n_y) > 0 
			    && fortune_v_segment(n_x,n_y) == -1 ) {
        
		}
	}
  
  return;
}

bool sg_map::is_dead_end( int segment_id ) {
	std::vector<int>::iterator i;
	std::vector<int>::iterator s = dead_end_segments.begin();
	std::vector<int>::iterator e = dead_end_segments.end();
	
	for ( i = s ; i != e ; ++i ) {
		if ( *i == segment_id )
			return true;
	}
	return false;
}

//*** (end)

//******* Voronoi Minima Pipeline Processing ******* (fold)	

void sg_map::compute_voronoi_minima_p() {
	pointvector::iterator n_start = voronoi_points.begin();
	pointvector::iterator n_end		= voronoi_points.end();
	pointvector::iterator n;
	
	sort( n_start, n_end, sg_map_point_comparison_less );
	
	for ( n = voronoi_points.begin(); n != voronoi_points.end() ; ++n ) {
		voronoi_minimum_check( *n );
	}
}

void sg_map::voronoi_minimum_check( sg_map_point point ) {
	if ( blocked_for_minimum( point.x, point.y ) == 1 ) 
		return;
		
	if ( is_dead_end(fortune_v_segment(point.x, point.y)) )
		return;

	int n_neighbors = 0; int neighbors[9][2];
	n_neighbors = get_voronoi_neighbors( neighbors, point.x, point.y );
	
	if ( n_neighbors != 2 )
		return;

	int cost = clearance_value( point.x, point.y );
	
	visited_close_min.set_elements(0); 
	visited_close_min( point.x, point.y ) = 1;
	bool min_check1	 = is_increasing_direction( neighbors[0][0], neighbors[0][1], cost, 1 );
	
	visited_close_min.set_elements(0);
	visited_close_min( point.x, point.y ) = 1;
	bool min_check2 = is_increasing_direction( neighbors[1][0], neighbors[1][1], cost, 1 );
	
	// Cases: 1) both false 2) one true 3) both true
	if ( min_check1 == false && min_check2 == false )
		return;
	else if ( min_check1 == false || min_check2 == false	) {
		
		bool min_check3 = false;
		visited_close_min.set_elements(0); 
		visited_close_min( point.x, point.y ) = 1;
		min_check3 = is_not_decreasing( neighbors[min_check1][0], neighbors[min_check1][1], cost, 1 );
		
		if ( min_check3 == true )
			add_voronoi_minimum( point.x, point.y );
	}
	else 
		add_voronoi_minimum( point.x, point.y );
}

void sg_map::add_voronoi_minimum( int i, int j) {
	visited_close_min.set_elements(0);
	block_close_points_for_minima( i, j, 0);
	voronoi_minima_points.push_back( sg_map_point(i, j, 0, clearance_value(i,j)) );
	voronoi_minima(i,j) = 1;
}

int sg_map::get_voronoi_neighbors( int neighbors[][2], int i, int j) {
	int case_nr, ii, jj, n_x, n_y, n_neighbors = 0;
	for (case_nr = 0; case_nr < 8; case_nr++) {
		get_eight_case(case_nr, ii, jj);
		n_x = i + ii; n_y = j + jj;
		if ( bound_check(n_x,n_y) && fortune_voronoi(n_x,n_y) > 0 ) {
			neighbors[ n_neighbors ][0] = n_x;
			neighbors[ n_neighbors ][1] = n_y;
			n_neighbors++;
		}
	}
	return n_neighbors;
}

bool sg_map::is_increasing_direction(int i, int j, float best_minima, int dist_travelled) {
	visited_close_min( i, j ) = 1;
	if ( DEBUG_VOR_MIN >= 6 )
		cout << "			 Check for increasing_direction at distance " << dist_travelled << " around " << i << ":" << j << endl;
	int c_i, n_x, n_y, n_neighbors;
	int neighbors[9][2];
	float minima_diff, needed_gain;
	n_neighbors = get_voronoi_neighbors( neighbors, i, j);
	if ( dist_travelled > minima_travel_dist )	{
		for ( c_i = 0; c_i < n_neighbors; ++c_i ) {
			n_x = neighbors[c_i][0]; n_y = neighbors[c_i][1];
			if ( visited_close_min( n_x, n_y ) == 0 ) {
				minima_diff = clearance_value(n_x,n_y) - best_minima;
				needed_gain = dist_travelled * minima_increase_lvl;
				if ( DEBUG_VOR_MIN >= 6) {
					cout << "			 -> At " << i << ":" << j << " " << clearance_value(n_x,n_y) << " gained " << minima_diff << endl;
					cout << "			 -> Needed_gain " << needed_gain << endl;
				}
				if ( minima_diff < needed_gain) {
					if ( DEBUG_VOR_MIN >=5 )
						cout << "			Rejecting this direction " << endl;
					return false;
				}
			}
		}
	}
	// Continue to visit neighboring points
	bool next_best;
	if ( dist_travelled < minima_inc_corridor ) {
		for ( c_i = 0; c_i < n_neighbors; ++c_i ) {
			n_x = neighbors[c_i][0]; n_y = neighbors[c_i][1];
			if ( visited_close_min( n_x,n_y ) == 0 ) {
				next_best = is_increasing_direction( n_x, n_y, best_minima, dist_travelled+1 );
				if ( next_best == false )
					return false;
			}
		}
	}
	return true;
}

bool sg_map::is_not_decreasing(int i, int j, float best_minima, int dist_travelled) {
	visited_close_min( i, j ) = 1;
	if ( DEBUG_VOR_MIN >= 6 )
		cout << "			 Check for not_decreasing at distance " << dist_travelled << " " << i << ":" << j << endl;
	int c_i, n_x, n_y, n_neighbors;
	int neighbors[9][2];
	n_neighbors = get_voronoi_neighbors( neighbors, i, j);
	if ( dist_travelled > minima_non_dec_travel_dist )	{
		for ( c_i = 0; c_i < n_neighbors; ++c_i ) {
			n_x = neighbors[c_i][0]; n_y = neighbors[c_i][1];
			if ( visited_close_min(n_x, n_y) == 0 ) {
				if ( DEBUG_VOR_MIN >= 6 )
					cout << "			 Neighbor " << n_x << ":" << n_y << " has cost " << clearance_value(n_x,n_y) << endl;
				if ( clearance_value(n_x,n_y) < (best_minima - minima_margin) ) {
					if ( DEBUG_VOR_MIN >=5 )
						cout << "			Rejecting this direction" << endl;
					return false;
				}
			}
		}
	}
	// Continue to visit neighboring points
	bool next_best;
	if ( dist_travelled < minima_non_dec_corridor ) {
		for ( c_i = 0; c_i < n_neighbors; ++c_i ) {
			n_x = neighbors[c_i][0]; n_y = neighbors[c_i][1];
			if ( visited_close_min( n_x,n_y ) == 0 ) {
				next_best = is_not_decreasing( n_x, n_y, best_minima, dist_travelled+1 );
				if ( next_best == false )
					return false;
			}
		}
	}
	return true;
}

void sg_map::block_close_points_for_minima( int i, int j, int dist_travelled ) {
	blocked_for_minimum( i, j ) = 1;
	visited_close_min( i, j) = 1;
	int c_i, n_x, n_y, n_neighbors;
	int neighbors[9][2];
	n_neighbors = get_voronoi_neighbors( neighbors, i, j);
	if ( dist_travelled < minima_distance ) {
		for ( c_i = 0; c_i < n_neighbors; ++c_i ) {
			n_x = neighbors[c_i][0]; n_y = neighbors[c_i][1];
			if ( visited_close_min( n_x,n_y ) == 0 ) {
				block_close_points_for_minima( n_x, n_y, dist_travelled+1 );
			}
		}
	}
}

//**** (end)

//******* Graph Creation ******* (fold)

void sg_map::process_minima_to_vertices() {
	
	visited.set_elements(0);
	
	sg_map_point point = *(voronoi_minima_points.begin());
	
	visited( point.x, point.y ) = 1;
	
	int n_neighbors = 0, neighbors[9][2];
	n_neighbors = get_voronoi_neighbors( neighbors, point.x, point.y );
	
	sg_vertex_d vd = add_new_vertex( neighbors[0][0], neighbors[0][1] );
	add_vertex_membership( point.x, point.y, vd );
	
	process_minima_to_vertices_visit( neighbors[0][0], neighbors[0][1], vd );
}

void sg_map::process_minima_to_vertices_visit( int i, int j, sg_vertex_d vd ) {
	
	add_vertex_membership( i, j, vd );
	visited(i,j) = 1;
	
	int n_x, n_y, c_i, n_neighbors = 0, neighbors[9][2];
	n_neighbors = get_voronoi_neighbors( neighbors, i, j );
	
	if ( DEBUG_GR_CON >= 2 ) {
		cout << "	 visiting for vertex processing " << i << ":" << j << " vd=" << vd;
		cout << " n_neighbors " << n_neighbors << endl;
	}
	
	for ( c_i = 0 ; c_i < n_neighbors ; ++c_i ) {	 
		
		n_x = neighbors[c_i][0];
		n_y = neighbors[c_i][1];
		
		if ( visited(n_x,n_y) == 0 ) {			
			
			if ( voronoi_minima(n_x,n_y) == 1 ) {

				if ( DEBUG_GR_CON >= 2 )
					cout << "	 neighbor is minima " << n_x << ":" << n_y << endl;
				
				sg_vertex_d vd_new;
				int neighbors2[9][2];
				int n_neighbors2 = get_voronoi_neighbors( neighbors2, n_x, n_y );
				
				for ( int c_i2 = 0; c_i2 < n_neighbors2 ; ++c_i2 ) {
					if ( visited(neighbors2[c_i2][0],neighbors2[c_i2][1]) == 0 ) {
						visited(n_x,n_y) = 1;
						vd_new = add_new_vertex( neighbors2[c_i2][0], neighbors2[c_i2][1] );
						add_vertex_membership( n_x, n_y, vd_new );
						process_minima_to_vertices_visit( neighbors2[c_i2][0], neighbors2[c_i2][1], vd_new );
					}
				} 
			}
			else {
				process_minima_to_vertices_visit( n_x, n_y, vd );		 
			} 
		}
	}
}

sg_vertex_d sg_map::add_new_vertex(int i, int j) {
	if ( DEBUG_GR_CON >= 2 )
		cout << "	 +adding a new vertex " << i << ":" << j << endl;
	sg_vertex_d vert_desc = add_vertex(G_g);
	G_g[vert_desc].alive = 1;
	G_g[vert_desc].x = i;
	G_g[vert_desc].y = j;
	
	map_point_list new_list;
	points_for_vertex.push_back(new_list);
	
	return vert_desc;
}

void sg_map::process_minima_to_edges() {
	
	pointvector::iterator k = voronoi_minima_points.begin();
	int i, j, n_neighbors, neighbors[9][2];
	sg_vertex_d vd1, vd2;
	sg_edge_d ed;
	
	for (; k != voronoi_minima_points.end(); k++ ) {
		
		i = (*k).x; j = (*k).y;
		
		if ( DEBUG_GR_CON >= 2 )
			cout << "	 Creating an edge for minimum " << i << ":" << j << endl;
		
		n_neighbors = get_voronoi_neighbors( neighbors, i, j );
		
		vd1 = get_vertex_membership( neighbors[0][0] , neighbors[0][1] );
		vd2 = get_vertex_membership( neighbors[1][0] , neighbors[1][1] );
		
		ed = add_new_edge( vd1, vd2 );
		G_g[ed].x = i;
		G_g[ed].y = j;
	}
}

sg_edge_d sg_map::add_new_edge(sg_vertex_d vd1, sg_vertex_d vd2) {
	return add_edge(vd1, vd2, G_g).first;
}

//*** (end)

//******* Edge lines ******* (fold)
void sg_map::compute_edge_lines() {
	sg_edge_it edge_it, edge_it_end;
	tie(edge_it,edge_it_end) = edges(G_g);
	for (; edge_it != edge_it_end; ++edge_it)
		compute_edge_line( *edge_it );
}

void sg_map::compute_edge_line( sg_edge_d ed ) {
	// find two obstacle points that are closest
	// but have a significant distance from each other
	int x = G_g[ed].x;
	int y = G_g[ed].y;
	if ( DEBUG_GR_CON >= 2)
		cout << "	 Computing Edge line for " << ed << " at " << x << ":" << y << endl;
	
	float clearance = clearance_value(x,y);
	int offset = int ( ceil ( clearance ) );
	offset = offset + 2;
	int i, j;
	pointvector close_obstacles;
	if ( DEBUG_GR_CON >= 2)
		cout << "	 --> clearance " << clearance << " offset " << offset << endl;
	
	int obstacles_found = 0;
	for ( i = max(x - offset,0); i < min(x + offset, nrows); ++i ) {
		for ( j = max(y - offset,0); j < min(y + offset, ncols); ++j ) {
			if ( bound_check(i,j) && occ(i,j) == 1 ) {
				++obstacles_found;
				close_obstacles.push_back( sg_map_point(i,j, 0 , distance( i, j, x, y) ) );				 
			}
		}
	}
	
	if ( DEBUG_GR_CON >= 2)
		cout << "	 --> Obstacles found? " << obstacles_found << endl;
	
	pointvector::iterator beg_i = close_obstacles.begin();
	pointvector::iterator end_i = close_obstacles.end();
	sort( beg_i, end_i , sg_map_point_comparison_less );
	pointvector::iterator k = close_obstacles.begin();
	int closest_i = (*k).x;
	int closest_j = (*k).y;
	int second_i = -1;
	int second_j = -1;
	float minimum_distance = clearance * 1.8;
	++k;
	for ( ; k != close_obstacles.end(); ++k ) {
		if ( DEBUG_CLOSES_OBSTACLE >= 1 )
			cout << "			 looking at " << (*k).x << ":" << (*k).y << endl;
		if ( distance( closest_i, closest_j, (*k).x, (*k).y ) > minimum_distance ) {
			second_i = (*k).x;
			second_j = (*k).y;
			break;
		}
	}
	
	
	if ( DEBUG_GR_CON >= 2)
		cout << "	 --> Obstacles chosen " << closest_i << ":" << closest_j << " " << second_i << ":" << second_j << endl;
	
	reset_closest_obstacles(x,y);
	add_closest_obstacle(x,y,closest_i,closest_j);
	add_closest_obstacle(x,y,second_i,second_j);
}

void sg_map::get_edge_line( int& x_1, int& x_2, int& y_1, int& y_2, sg_edge_d edge) {
	int i = G_g[edge].x; 
	int j = G_g[edge].y;
	map_point_list::iterator runner = (*closest_obstacle)[i][j].begin();
	x_1 = (*runner).first;
	x_2 = (*runner).second;		
	runner++;
	y_1 = (*runner).first;
	y_2 = (*runner).second;
	return;
}

void sg_map::imprint_closest_obstacle() {
	pointvector::iterator k = voronoi_minima_points.begin();
	map_point_list::iterator runner;
	int i,j;
	for (; k != voronoi_minima_points.end(); k++ ) {
		i = (*k).x; j = (*k).y;
		if ( DEBUG_GR_CON >= 2)
			cout << "	 Imprinting obstacles for " << i << ":" << j << endl;
				
		runner = (*closest_obstacle)[i][j].begin();
		
		for(; runner != (*closest_obstacle)[i][j].end(); runner++)
			imprint_closest_obstacle( (*runner).first, (*runner).second, i, j);
	}
}

void sg_map::imprint_closest_obstacle(int x0, int y0, int x1, int y1 ) {
	if ( DEBUG_GR_CON >= 2)
		cout << "	 Imprinting line " << x0 << ":" << y0 << " " << x1 << ":" << y1 << endl;
	
	int dy = y1 - y0;
	int dx = x1 - x0;
	float t = (float) 0.5;											// offset for rounding

	occ(x0, y0) = 2;
	if ( abs(dx) > abs(dy) ) {					// slope < 1
		float m = (float) dy / (float) dx;			// compute slope
		t += y0;
		dx = (dx < 0) ? -1 : 1;
		m *= dx;
		while (x0 != x1) {
			x0 += dx;														// step to next x value
			t += m;															// add slope to y value
			occ(x0, (int) t) = 2;
		}
	} 
	else {																		// slope >= 1
		float m = (float) dx / (float) dy;			// compute slope
		t += x0;
		dy = (dy < 0) ? -1 : 1;
		m *= dy;
		while (y0 != y1) {
			y0 += dy;														// step to next y value
			t += m;															// add slope to x value
			occ((int) t, y0) = 2;
		}
	}
}

//*** (end)

//******** Process Vertex membership & positions**** (fold)

void sg_map::process_vertex_membership() {
	// Visited was last used with the voronoi point voronoi_minima search
	visited.set_elements(0);
	sg_vertex_it vert_it, vert_it_end;
	tie(vert_it, vert_it_end) = vertices(G_g);
	for (; vert_it != vert_it_end; ++vert_it) {
		if ( G_g[*vert_it].alive == 1 ) // do not post_process killed vertices
			process_vertex_membership_vertex(*vert_it);
	}
}

void sg_map::process_vertex_membership_vertex( sg_vertex_d vd ) {
	// we start of at the current position and visit all neighbors until we hit boundaries
	// defined in occ(i,j) > 0
	if ( DEBUG_MEMBER >= 3)
		cout << "		Processing membership of points for " << vd << endl;
	points_for_vertex[vd].clear();
	find_points_for_vertex( vd , G_g[vd].x, G_g[vd].y, true );
}

void sg_map::find_points_for_vertex( sg_vertex_d vd, int i, int j, bool starting ) {
	
	visited(i,j) = 1;
	
	(*vertex_membership)[i][j] = vd;
	map_point new_map_point(i,j);
	points_for_vertex[vd].push_back(new_map_point);
	
	int n_i, n_j, case_nr, ii, jj;
	bool boundary_break = false;
	
	for ( case_nr = 0 ; case_nr < 8 ; case_nr++ ) {
		get_eight_case(case_nr, ii, jj);
		n_i = i + ii; n_j = j + jj;
		if ( bound_check(n_i,n_j) && occ(n_i,n_j) == 2 )
			boundary_break = true;
	}
	if ( !boundary_break || starting ) {
		for ( case_nr = 0 ; case_nr < 8 ; ++case_nr ) {
			get_eight_case(case_nr, ii, jj);
			n_i = i + ii; n_j = j + jj;
			if ( bound_check(n_i,n_j) && get_occ(n_i,n_j) == 0 && visited(n_i,n_j) == 0 )
				find_points_for_vertex( vd, n_i, n_j, false );
		}
	}
}

void sg_map::compute_graph_vertex_positions() {
	sg_vertex_d vert_desc;
	sg_vertex_it vert_it, vert_it_end;
	tie(vert_it, vert_it_end) = vertices(G_g);
	for (; vert_it != vert_it_end; ++vert_it) {
		vert_desc = *vert_it;
		compute_graph_vertex_position(vert_desc);
	}
}

void sg_map::compute_graph_vertex_position(sg_vertex_d vd) {
	int count = 0;
	double x = 0, y = 0;
	map_point_list::iterator map_point_list_it;
	map_point_list_it = points_for_vertex[vd].begin();
	//compute the average of all points belonging to this vertex
	for (; map_point_list_it != points_for_vertex[vd].end(); map_point_list_it++) {
		x += (*map_point_list_it).first;
		y += (*map_point_list_it).second;
		count++;
	}
	G_g[vd].x = int(x / count);
	G_g[vd].y = int(y / count);
}

//*** (end)

//******* Weight Computation ******* (fold)

void sg_map::compute_graph_vertex_weights() {
	sg_vertex_d vert_desc;
	sg_vertex_it vert_it, vert_it_end;
	tie(vert_it, vert_it_end) = vertices(G_g);
	if ( DEBUG_MAP >= 2 )
		cout << "	 All vertex weight computations:" << endl;
	for (; vert_it != vert_it_end; ++vert_it) {
		vert_desc = *vert_it;
		if ( DEBUG_MAP >= 2 )
			cout << "	 Vert: " << vert_desc << endl;
		compute_graph_vertex_weight(vert_desc);
	}
}

void sg_map::compute_graph_vertex_weight(sg_vertex_d vd) {
	// search through the largest cost for all points within this 
	// vertex
	int current_max = 0;
	double x = 0, y = 0;
	map_point_list::iterator map_point_list_it, map_point_list_it_end;
	map_point_list_it = points_for_vertex[vd].begin();
	map_point_list_it_end = points_for_vertex[vd].end();
	G_g[vd].min_x = nrows+ncols;G_g[vd].max_x = 0;
	G_g[vd].min_y = nrows+ncols;G_g[vd].max_y = 0;
	
	//compute the average of all points belonging to this vertex
	if ( DEBUG_MAP >= 3 )
		cout << "		Max bbox for vertex:";
	for (; map_point_list_it != map_point_list_it_end; map_point_list_it++) {
		x = (*map_point_list_it).first;
		y = (*map_point_list_it).second;
		// Remember the lower left and upper right coordinates of a bounding box
		if ( x < G_g[vd].min_x )
			G_g[vd].min_x = x;
		else if ( x > G_g[vd].max_x )
			G_g[vd].max_x = x;
		if ( y < G_g[vd].min_y )
			G_g[vd].min_y = y;
		else if ( y > G_g[vd].max_y )
			G_g[vd].max_y = y;
			
		if ( current_max < clearance_value(x,y) ) 
			current_max = clearance_value(x,y);
	}
	if ( DEBUG_MAP >= 3 )
		cout << " (" << G_g[vd].min_x << "," << G_g[vd].min_y << ") | (" << G_g[vd].max_x << "," << G_g[vd].max_y << ")" << endl;
	
//	G_g[vd].bbox_cost = bounding_box_cost( min_x, min_y, max_x, max_y );
//	G_g[vd].w = int ( ceil(current_max / sense_range_pix) );
	G_g[vd].w = bounding_box_cost( G_g[vd].min_x, G_g[vd].min_y, G_g[vd].max_x, G_g[vd].max_y );;
	if ( DEBUG_MAP >= 3 )
		cout << "		Resulting w(" << vd << ") = " << G_g[vd].w << endl;
	G_g[vd].max_clearance = current_max;
}

int sg_map::bounding_box_cost( int ll_x, int ll_y, int ur_x, int ur_y ) {
	int width = abs(ll_x - ur_x);
	int height = abs(ll_y - ur_y);
	if ( DEBUG_MAP >= 4 ) {
		cout << "		 w=" << width << " h=" << height << " rang=" << sense_range_pix 
			<< " weight" << ceil( float(min( width, height )) / float(sense_range_pix))
			<< " rang=" << sense_range_pix << endl;
	}
	return int(ceil( float(min( width, height )) / float(sense_range_pix)));
}

void sg_map::compute_graph_edge_weights() {
	sg_edge_it e_i, e_i_end;
	tie( e_i , e_i_end ) = edges(G_g);
	for ( ; e_i != e_i_end ; ++e_i ) {
		compute_graph_edge_weight( *e_i );
	}
}

void sg_map::compute_graph_edge_weight( sg_edge_d ed ) {
	int x_1, y_1, x_2, y_2;
	get_edge_line( x_1, y_1, x_2, y_2, ed );
	float dist = distance( x_1, y_1, x_2, y_2);
	G_g[ed].w = int(ceil(dist/(sense_range_pix)));
}

//*** (end)

//******* Graph Improvements ******* (fold)


bool sg_map::easy_merge_leaves() {
	// Go through all leaves and attempt to easy_merge them
	bool return_value = false;
	sg_vertex_d vd;
	sg_vertex_it i, i_end;
	tie( i, i_end ) = vertices( G_g );
	if (DEBUG >= 1)
		cout << " Going through leaves for merging " << endl;
	for ( ; i != i_end ; ++i ) {
		vd = *i;
		if ( MST_DEGREE == 1 ) {
			if ( G_g[vd].mst_degree == 1 )
				if ( easy_merge_leaf( vd ) )
					return_value = true;
		}
		else {
			if ( degree(vd,G_g) == 1 )
				if ( easy_merge_leaf( vd ) )
					return_value = true;
		}
	}
	return return_value;
}

bool sg_map::easy_merge_leaf( sg_vertex_d leaf) {
	
	sg_o_edge_it	out_edge_it, out_edge_it_end;
	tie(out_edge_it, out_edge_it_end) = out_edges(leaf, G_g);
	sg_edge_d ed = *out_edge_it;
	sg_vertex_d vd2 = ( leaf == target( ed, G_g ) ) ? source( ed, G_g ) : target( ed, G_g );
	
	if ( DEBUG_MRG >= 2 )
		cout << "	 +Checking leaf merger at " << leaf << endl;
	
	if ( check_for_easy_leaf_merge( leaf, vd2, ed ) ) {
		leaf_mergers++;
		merge( leaf, vd2 );
		return true;
	}
	else
		return false;
}

bool sg_map::check_for_easy_leaf_merge( sg_vertex_d leaf, sg_vertex_d vd2, sg_edge_d ed) {
	
	int merged_weight = merged_vertex_weight( leaf, vd2 );
	int gain = ( merged_weight - G_g[vd2].w ) - G_g[ed].w;
	
	if ( DEBUG_MRG >= 3 ) {
		cout << "		gain= " << gain << " mergedweight= " << merged_weight	 << endl;
		cout << "		G_g[leaf].w = " << G_g[leaf].w << " G_g[vd2].w= " << G_g[vd2].w;
		cout << " edge_w=" << G_g[ed].w << endl;
	}
	
	if ( gain < 0 )
		return true; // merging may provide advantage
	else
		return false; // merging gives worse weights
}			

bool sg_map::easy_merge_twos() {
	// Go through all leaves and attempt to easy_merge them
	bool return_value = false;
	sg_vertex_d vd;
	sg_vertex_it i, i_end;
	tie( i, i_end ) = vertices( G_g );
	if (DEBUG >= 1)
		cout << " Going through degree 2 for merging " << endl;
	for ( ; i != i_end ; ++i ) {
		vd = *i;
		if ( MST_DEGREE == 1 ) {
			if ( G_g[vd].mst_degree == 2 )
				if ( easy_merge_two( vd ) )
					return_value = true;
		}
		else {
			if ( degree(vd,G_g) == 2 )
				if ( easy_merge_two( vd ) )
					return_value = true;
		}
	}
	return return_value;
}
										
bool sg_map::easy_merge_two( sg_vertex_d vd_c) {
	
	sg_o_edge_it	out_edge_it, out_edge_it_end;
	tie(out_edge_it, out_edge_it_end) = out_edges(vd_c, G_g);
	sg_edge_d ed1 = *out_edge_it;
	sg_vertex_d vd1 = ( vd_c == target( ed1, G_g ) ) ? source( ed1, G_g ) : target( ed1, G_g );
	++out_edge_it;
	sg_edge_d ed2 = *out_edge_it;
	sg_vertex_d vd2 = ( vd_c == target( ed2, G_g ) ) ? source( ed2, G_g ) : target( ed2, G_g );
	
	if (DEBUG_MRG >= 2)
		cout << "	 ++Checking degree two mergers around " << vd_c << endl;

	// check which side should be merged
	if ( G_g[ed1].w >= G_g[ed2].w ) { 
		// want to merge the larger edge first
		if ( check_for_easy_two_merge( vd_c, vd1, ed1, ed2 ) ) {
			two_mergers++;
			merge( vd_c, vd1 );
			return true;
		}
		else if ( G_g[ed1].w == G_g[ed2].w ) {
			if ( check_for_easy_two_merge( vd_c, vd2, ed2, ed1 ) ) {
				two_mergers++;
				merge( vd_c, vd2 );
				return true;
			}
		}
		return false;
	}
	else { 
		if ( check_for_easy_two_merge( vd_c, vd2, ed2, ed1 ) ) {
			two_mergers++;
			merge( vd_c, vd2 );
			return true;
		}
		return false;
	}
}														

bool sg_map::check_for_easy_two_merge( sg_vertex_d vd_c, sg_vertex_d vd, sg_edge_d ed, sg_edge_d ed2 ) {
	// here we know that G_g[ed].w <= G_g[ed2].w and vd_c gets merged into vd

	// New weight would be
	if ( DEBUG_MRG >= 3 )
		cout << "		Check " << vd_c << " into " << vd << endl;
	int merged_weight = merged_vertex_weight( vd_c, vd );
	int gain = ( merged_weight - G_g[vd].w ) - G_g[ed].w + G_g[ed2].w;
	if ( DEBUG_MRG >= 3 ) {
		cout << "		--> Gain is " << gain << " merged weight " << merged_weight << endl;
		cout << "		--> vd.w " << G_g[vd].w << " + vd_c.w " << G_g[vd_c].w << endl;
	}
	if ( gain <= 0 )
		return true; // merging may provide advantage
	else
		return false; // merging gives worse weights
}																								 

int sg_map::merged_vertex_weight( sg_vertex_d vd1, sg_vertex_d vd2 ) {
	int new_ll_x = min( G_g[vd1].min_x , G_g[vd2].min_x );
	int new_ll_y = min( G_g[vd1].min_y , G_g[vd2].min_y );
	int new_ur_x = max( G_g[vd1].max_x , G_g[vd2].max_x );
	int new_ur_y = max( G_g[vd1].max_y , G_g[vd2].max_y );
	return bounding_box_cost( new_ll_x, new_ll_y, new_ur_x, new_ur_y );
}

//tricky part! we don't want to to invalidate the point lists and other pointers within the graph
void sg_map::merge( sg_vertex_d vd_merged, sg_vertex_d vd2 ) {

	if ( DEBUG_MRG >= 2)
		cout << "__MERGING " << vd_merged << " and " << vd2 << endl;
		
	// collect the baggage from the other vertex. not done in merge_vertices
	// hence we need to new merge function
	G_g[vd2].w = merged_vertex_weight( vd_merged, vd2 );
	G_g[vd2].min_x = min( G_g[vd_merged].min_x , G_g[vd2].min_x );
	G_g[vd2].min_y = min( G_g[vd_merged].min_y , G_g[vd2].min_y );
	G_g[vd2].max_x = max( G_g[vd_merged].max_x , G_g[vd2].max_x );
	G_g[vd2].max_y = max( G_g[vd_merged].max_y , G_g[vd2].max_y );
	sg_o_edge_it	out_edge_it, out_edge_it_end;
	sg_edge_d ed;
	tie( out_edge_it, out_edge_it_end ) = out_edges( vd_merged, G_g );
	for( ; out_edge_it != out_edge_it_end ; out_edge_it++ ) {
		if ( source(*out_edge_it, G_g) == vd2 || target(*out_edge_it, G_g) == vd2 ) {
			ed = *out_edge_it;
			break;
		}
	}
	
	// Find corresponding min_point
	int i = G_g[ed].x, j = G_g[ed].y;	 
	pointvector::iterator k = voronoi_minima_points.begin();
	for (; k != voronoi_minima_points.end(); ++k ) {
		if ( i == (*k).x && j == (*k).y ) {
			if ( DEBUG_MAP >= 5 )
				cout << "			found and erasing min point " << i << "|" << j << endl;
			voronoi_minima_points.erase(k);
			break;
		}
	}
	
	merge_vertices(vd2, vd_merged); // kills the vd_merged	
	
}						 

void sg_map::merge_vertices(sg_vertex_d vd_merged, sg_vertex_d vd2) {
	
	if ( DEBUG_MRG >= 2 )
		cout << "	 f:merge_vertices(" << vd_merged << " , " << vd2 << ")" << endl;

	if (vd_merged == vd2)
		return;

	if ( DEBUG_MRG >= 5 )
		cout << "		- switching vertex membership and adding mappoints to " << vd_merged << endl;
	//set points in the vertex_membership from vd2 to vd_merged
	map_point_list::iterator map_point_list_it;
	map_point_list_it = points_for_vertex[vd2].begin();
	map_point new_map_point;
	for (; map_point_list_it != points_for_vertex[vd2].end(); map_point_list_it++) {
		new_map_point.first = (*map_point_list_it).first;
		new_map_point.second = (*map_point_list_it).second;
		points_for_vertex[vd_merged].push_back(new_map_point);
		add_vertex_membership((*map_point_list_it).first, (*map_point_list_it).second, vd_merged);
	}

	// insert points from vd2 into vd_merged
	// this is already done above;
	// points_for_vertex[vd_merged].insert(points_for_vertex[vd_merged].end(),
	// points_for_vertex[vd2].begin(),points_for_vertex[vd2].end());
	sg_o_edge_it	out_edge_it, out_edge_it_end;
	sg_edge_d ed;
	
	if ( DEBUG_MRG >= 5 )
		cout << "			clearing points_for_vertex for	" << vd2 << endl;
	points_for_vertex[vd2].clear();
	//points_for_vertex.erase(vd2); // problem with invalidation? 

	if ( DEBUG_MRG >= 5 )
		cout << "			adding edges from " << vd2 << " to " << vd_merged << endl;
	tie(out_edge_it, out_edge_it_end) = out_edges(vd2, G_g);
	for (;out_edge_it != out_edge_it_end; out_edge_it++) {
		if ( vd_merged != target(*out_edge_it, G_g) && vd_merged != source(*out_edge_it, G_g) ) {
			ed = add_edge(vd_merged, target(*out_edge_it, G_g), G_g).first;
			G_g.copy_edge_values( ed, *out_edge_it);
		}
	}
	if ( DEBUG_MRG >= 5 )
		cout << "			removing all edges to and from vertex " << vd2 << " from graph." << endl;
	clear_vertex(vd2, G_g);

	//remove vd2 from graph??? invalidating all stored vertex_descriptors
	//remove_vertex(vd2, G_g); --> instead let's kill it with a hack
	kill_vertex(vd2);
}
 
void sg_map::kill_vertex( sg_vertex_d vd ) {
	// defines what it means to get rid of a vertex
	// without invalidating pointers and vertex descriptors
	G_g[vd].alive = 0;					G_g[vd].w = 0;
	G_g[vd].x = 0;							G_g[vd].y = 0;
	G_g[vd].mst_degree = 0;			G_g[vd].s = 0;
	G_g[vd].max_clearance = 0;	G_g[vd].bbox_cost = 0;
	G_g[vd].min_x = 0;					G_g[vd].max_x = 0;
	G_g[vd].min_y = 0;					G_g[vd].max_y = 0;
}
							 
int sg_map::edge_cost( sg_vertex_d vd ) {
	sg_o_edge_it	i, i_end;
	tie( i, i_end ) = out_edges( vd, G_g );
	int total = 0;
	for ( ; i != i_end ; i++ ) { // FoR ALL EDGES? OR ONLY MST? ALL for now
		total += G_g[*i].w;
	}
	return total;
}

//*** (end)

//******* Sweeping task support function *******(fold)

br_pose_matrix*
sg_map::generate_pose_matrix( sg_vertex_d vd, sg_edge_d ed ) {
	// coming from ed and entering vd
	// we have to fill a pose matrix
	br_pose_matrix* pose_mat = new br_pose_matrix();
	// and fill it with pose_vec's
	cout << " ***** generate_pose_matrix ****" << endl;
	cout << " for " << int(vd) << " & " << int(target(ed,G_g)) 
	     << "<>" << int(target(ed,G_g)) << endl;
//	double sense_dis = sense_range_pix - sense_delta;

	// find out which index ed has for G_g[vd].crit_p[i][]
	sg_o_edge_it out_edge_it, out_edge_it_end;
	tie(out_edge_it, out_edge_it_end) = out_edges( vd, G_g);
	int dir_ind = 0;
	for (;out_edge_it != out_edge_it_end; out_edge_it++) {
		ed = *out_edge_it;
		if ( vd == source( ed, G_g ) || vd == target( ed, G_g ) )
			break;
		++dir_ind;
	}
	// from now on we can refer only to the right direction
	VD_Point_2* v_cri = G_g[vd].crit_p[dir_ind];
	VD_Point_2* e_cri = G_g[ed].crit_p;
	
	// Get the generating sites for ed 
	SDG::Edge e = G_g[ed].e.dual();
	SDG::Vertex_handle v[] = 
	     { e.first->vertex( voronoi_diagram.dual().ccw(e.second) ),
	       e.first->vertex( voronoi_diagram.dual().cw(e.second) ) };
	
	// find out to which sites the points 
	//G_g[vd].crit_p[0] and G_g[vd].crit_p[2] belong
	// could do more check here, but if everything is right
	// it falls into place
	double dist1, dist2;
	int cp[2];
	dist1 = distance_to_site( v[0]->site(), v_cri[0] );
	if ( dist1 < ERROR_THRESHOLD ) {
		 // associate G_g[vd].crit_p[dir_ind][0] with site 0
		cp[0] = 0;
		cp[1] = 1;
	}
	else {
		cp[0] = 1; 
		cp[1] = 0;
	}
	
	// Compute the distances on the respective sides between the current
	// point of the sweep line and the goal pt on the site
	// Recall that G_g[ed].crit_p[0] belongs to site 0
	// and G_g[ed].crit_p[2] belongs to site 1
	dist1 = sqrt(double( CGAL::squared_distance( 
	             e_cri[cp[0]], v_cri[0] ) ));
	dist2 = sqrt(double( CGAL::squared_distance( 
	             e_cri[cp[1]], v_cri[1] ) ));
	
	VD_Vector goal_v;
	VD_Vector star_v;
	int left_site = 0, righ_site = 1;
	
	std::vector< VD_Point_2 > goal_points;
	std::vector< VD_Point_2 > star_points;
	
	if ( dist1 > ERROR_THRESHOLD || dist2 > ERROR_THRESHOLD ) {
		// movement is needed either 1) 2-side movement or 2) 1-side move
		if ( v[0]->site().is_segment() && v[1]->site().is_segment() ) {
			cout << "    1) move to split " << endl;
			VD_Vector v0( v_cri[0].x()-e_cri[cp[0]].x(), 
			              v_cri[0].y() - e_cri[cp[0]].y() );
			VD_Vector ve( e_cri[cp[1]].x()-e_cri[cp[0]].x(), 
			              e_cri[cp[1]].y()-e_cri[cp[0]].y() );
			//check where ve is to the right of v0 or left?
			if ( ve.direction().counterclockwise_in_between( 
			                    v0.direction(),-(v0.direction()) ) ) {
				// v0 is on the left
				left_site = 0; 
				righ_site = 1;
			}
			else { // v0 is on the right
				left_site = 1; 
				righ_site = 0;
			}
			
			//*** from left side to right side
			//*** for GOAL POINTS on v[left_site]
			double d_x = v_cri[righ_site*2].x() - v_cri[left_site*2].x();
			double d_y = v_cri[righ_site*2].y() - v_cri[left_site*2].y();
			double dis = sqrt(double(CGAL::squared_distance( 
			                  v_cri[0],v_cri[2] )));
			d_x = d_x / dis;
			d_y = d_y / dis;
			int n_robots = number_of_robots( dis );
			for ( int i = 0; i < n_robots ; ++i ) {
				VD_Point_2 p1( 
					v_cri[left_site*2].x() + (2*i+1)* sense_delta * d_x,
					v_cri[left_site*2].y() + (2*i+1)* sense_delta * d_y);
				goal_points.push_back( p1 );
			}
			//*** for START POINTS
			d_x = e_cri[cp[righ_site]].x() - e_cri[cp[left_site]].x();
			d_y = e_cri[cp[righ_site]].y() - e_cri[cp[left_site]].y();
			dis = sqrt(double(CGAL::squared_distance( 
			                  e_cri[0],e_cri[1] )));
			d_x = d_x / dis;
			d_y = d_y / dis;
			int n_robots2 = number_of_robots( dis );
			for ( int i = 0; i < n_robots2 ; ++i ) {
				VD_Point_2 p1( 
					e_cri[cp[left_site]].x() + (2*i+1)* sense_delta * d_x,
					e_cri[cp[left_site]].y() + (2*i+1)* sense_delta * d_y);
				star_points.push_back( p1 );
			}
			// NOW connect points and record poses
			int last_i = 0, n_poses = 0;
			double last_ratio;
			for ( int i = 0; i < n_robots ; ++i ) {
				if ( i < n_robots2 ) {
					// connect start and goal point -- all these have same n_poses
					dis = sqrt(double(CGAL::squared_distance( 
						goal_points[i],star_points[i] )));
					if ( n_poses == 0 )
						n_poses = ceil(double(dis)/double(INTER_POSE_DISTANCE));
					add_poses( goal_points[i], star_points[i], n_poses, pose_mat);
					last_i = i;last_ratio = n_poses/dis; // stop points per pixel
				}
				else {
					// i is now equal or larger than n_robots2
					// create a shorter line intersecting the right site
					//*** Fill in start points
					double trans_v_x, trans_v_y;
					trans_v_x = goal_points[i].x()- goal_points[last_i].x();
					trans_v_y = goal_points[i].y()- goal_points[last_i].y();
					// now move the star points
					VD_Point_2 temp_seg_p1( star_points[last_i].x() + trans_v_x,
					                        star_points[last_i].y() + trans_v_y);
					VD_Segment_2 temp_seg( temp_seg_p1, goal_points[i] );
					CGAL::Object obj = intersection( temp_seg, v[cp[righ_site]]->site().segment() );
					if (const VD_Point_2 *start_pt = CGAL::object_cast<VD_Point_2>(&obj)) {
					    // take point and connect to 
						dis = sqrt(double(CGAL::squared_distance( goal_points[i], *start_pt )));
						n_poses = ceil( last_ratio * dis ) ;
						add_poses( goal_points[i], *start_pt, n_poses, pose_mat);
					} else if (const VD_Segment_2 *segment = CGAL::object_cast<VD_Segment_2>(&obj)) {
						cout << " ERROR intersection is a segment " << endl;
					}
				}
			}
		}
		else {
			// 1-side move since one site is a point
			cout << "    2) prep for split " << endl;
			int site_is_seg;
			if ( v[0]->site().is_segment() )
				site_is_seg = 0;
			else if ( v[1]->site().is_segment() )
				site_is_seg = 1;
			else 
				cout << " ERROR Neither side is segment" << endl;

			//v[site_is_seg]->site().segment();
			
			
		}
	}
	
	// AT THIS POINT BOTH END POINTS ARE ON THE TWO CRIT POINTS
	// 3) DO THE Split
	cout << "    3) do the split " << endl;
	// Check whether split is needed or we are already close
	return pose_mat;
}

void
sg_map::add_poses( VD_Point_2 goal_p, VD_Point_2 star_p, int n_poses, br_pose_matrix* pose_mat ) {
	double d_x = goal_p.x() - star_p.x();
	double d_y = goal_p.y() - star_p.y();
	d_x = d_x / double(n_poses);
	d_y = d_y / double(n_poses);
	// start at beginning and move on line
	br_pose_vector*  pose_vec = new br_pose_vector();
	for ( int j = 0; j <= n_poses ; ++j) {
		br_pose new_pose(4);
		new_pose.pose[0] = star_p.x() + d_x * j;
		new_pose.pose[1] = star_p.y() + d_y * j;
		pose_vec->push_back(new_pose);
	}
	pose_mat->push_back( pose_vec );
}

void
sg_map::write2img_pose_matrix( IplImage* img, br_pose_matrix* pose_mat ) {
	
	br_pose_vector* pose_vec;
	br_pose         pose;
	CvScalar        cv_scalar;
	int i,j;
	
	cv_scalar.val[0] = 250;
	cv_scalar.val[1] = 0;
	cv_scalar.val[2] = 0;
	int i_max = int(pose_mat->size());
	int j_max = int(pose_vec->size());
	for ( i = 0 ; i < i_max ; ++i ) {
		pose_vec = (*pose_mat)[i];
		for ( j = 0 ; j < j_max ; ++j   ) {
			pose = (*pose_vec)[j];
			cvSet2D(img,pose.pose[0],pose.pose[1],cv_scalar);
		}
	}
}

br_pose_vector sg_map::follow_main_voronoi_segments( sg_vertex_d vertex ) {
	if ( DEBUG_SWEEP_COMP >= 1 )
		cout << " Following the main voronoi segments in " << vertex << endl;
	
	// OPTIONS
	int recording_steps = 10;
	robot_cfg->lookupValue( "map.recording_steps", recording_steps ); // record positione very 10 steps

	sg_o_edge_it						e_begin, e_end;
	tie( e_begin, e_end ) = out_edges( vertex, G_g ); 
	sg_edge_d edge				= *e_begin;
	
	// get the voronoi point coordinates
	int voronoi_point_x = G_g[edge].x;
	int voronoi_point_y = G_g[edge].y;
	
	if ( DEBUG_SWEEP_COMP >= 1 )
		cout << " -> Getting the right direction ";
	int neighbors[9][2], n_neighbors, c_x, c_y, n_x, n_y, c_i;
	sg_vertex_d vd;
	n_neighbors = get_voronoi_neighbors( neighbors, voronoi_point_x, voronoi_point_y);
	for ( c_i = 0; c_i < n_neighbors; ++c_i ) {
		vd = get_vertex_membership( neighbors[c_i][0] , neighbors[c_i][1] );
		if ( vd == vertex ) {
			c_x = neighbors[c_i][0];
			c_y = neighbors[c_i][1];
		}
	}
	int current_segment =	 fortune_v_segment(c_x,c_y);
	if ( DEBUG_SWEEP_COMP >= 1 )
		cout << "into " << c_x << ":" << c_y << " segment " << current_segment << endl;
	
	// fill the temp_segment_vector with the list of the longest segment path
	temp_segment_vector.clear();
	visited.set_elements(0);
	visited(voronoi_point_x,voronoi_point_y) = 1;
	follow_segment( current_segment, c_x, c_y );	
	temp_segment_vector.push_back( current_segment );
	
	if ( DEBUG_SWEEP_COMP >= 1 )
		cout << " -> Going through computed segment vector	" << endl;
	segment_vector_status();
	visited.set_elements(0);
	visited(voronoi_point_x,voronoi_point_y) = 1;
	visited(c_x,c_y) = 1;
		
	br_pose_vector pose_vec;
	br_pose new_pose(4);	
	std::vector<int>::reverse_iterator it(temp_segment_vector.end()), it2;
	std::vector<int>::reverse_iterator it_end(temp_segment_vector.begin());
	bool next_segment_comes = false, current_segment_finished = true, last_segment = false;
	int next_segment = 0, step_counter = 0;
	for ( ; it != it_end; ++it ) {

		it2									= it; ++it2;
		current_segment			= *it;
		
		if ( it2 == it_end ) {
			if ( DEBUG_SWEEP_COMP >= 3 )
				cout << " LAST SEGMENT" << endl;
			last_segment = true;
		}
		next_segment				= *it2;
		next_segment_comes	= false;
		
		if ( DEBUG_SWEEP_COMP >= 3 )
			cout << "		Currently at segment " << current_segment << endl;
		if ( DEBUG_SWEEP_COMP >= 3 )
			cout << "		Expecting segment " << next_segment << endl;
	 
		
		while( next_segment_comes == false ) {
			++step_counter;
			
			current_segment_finished = true;
			
			// find next position
			n_neighbors = get_voronoi_neighbors( neighbors, c_x, c_y ); 
			for ( c_i = 0; c_i < n_neighbors; ++c_i ) {
				n_x = neighbors[c_i][0]; 
				n_y = neighbors[c_i][1];
				if ( visited( n_x, n_y ) == 0 && fortune_v_segment( n_x, n_y ) == current_segment ) {
					c_x = n_x;
					c_y = n_y;
					if ( DEBUG_SWEEP_COMP >= 4 )
						cout << " " << c_x << ":" << c_y;
					visited( n_x, n_y ) = 1;
					current_segment_finished = false;
				}
			} 
			
			if ( step_counter%recording_steps == 0 ) {
				cout << " RECORDING POSITION " << c_x << ":" << c_y << endl;
				new_pose.pose[0] = c_x;
				new_pose.pose[1] = c_y;
				pose_vec.push_back( new_pose );
			}
			//*** Checking for crossing over (fold)
			n_neighbors = get_voronoi_neighbors( neighbors, c_x, c_y ); 
			for ( c_i = 0; c_i < n_neighbors; ++c_i ) {
				n_x = neighbors[c_i][0]; 
				n_y = neighbors[c_i][1];			
				if ( visited(n_x,n_y) == 0 && fortune_v_segment(n_x,n_y) == next_segment )
					next_segment_comes = true;
			}	 
			if ( current_segment_finished == true && next_segment_comes == false ) {
				if ( DEBUG_SWEEP_COMP >= 3 )
					cout << "		--> current segment finished without new segment coming " << endl;
				
				// need to jump over to next segment 
				if ( last_segment == false ) {
					int neighbors2[9][2], n_neighbors2, c_i2, n_x2,n_y2;
					for ( c_i = 0; c_i < n_neighbors; ++c_i ) {
						n_x = neighbors[c_i][0]; 
						n_y = neighbors[c_i][1];			
						n_neighbors = get_voronoi_neighbors( neighbors2, n_x, n_y ); 
						for ( c_i2 = 0; c_i2 < n_neighbors2; ++c_i2 ) {
							n_x2 = neighbors2[c_i2][0]; 
							n_y2 = neighbors2[c_i2][1];			 
							if ( fortune_v_segment( n_x2, n_y2 ) == next_segment ) {
								c_x = n_x;
								c_y = n_y;
							}
						}
					}
				}
				next_segment_comes = true;
			}
			//*** (end)
		}
		
		
		
	}
	
	if ( DEBUG_SWEEP_COMP >= 1 )
		cout << " DONE following the main voronoi segments - returning pose vector " << endl;
	
	return pose_vec;
}

int sg_map::follow_segment( int segment_id,	 int current_x, int current_y ) {
	if ( DEBUG_SWEEP_COMP >= 1 )
		cout << " Continuing in segment " << segment_id << " at " << current_x << ":" << current_y << endl;
	
	segment_vector_status();
	visited( current_x, current_y ) = 1;
	
	int neighbors[9][2], n_neighbors, n_y, n_x, c_i;
	int steps_taken = 1, last_seg_index, last_seg_index2;

	n_neighbors = get_voronoi_neighbors( neighbors, current_x, current_y );
	
	//*** Exception check (fold)
	// need to handle the exception when a new segment opens right after we entered a segment
	// that also continues
	bool segment_continues = false;
	bool new_segment_arises = false;
	int new_segment_id = 0, new_segment_x = 0, new_segment_y = 0;
	for ( c_i = 0; c_i < n_neighbors; ++c_i ) {
		n_x = neighbors[c_i][0]; 
		n_y = neighbors[c_i][1];		
		if ( visited( n_x, n_y ) == 0 && fortune_v_segment( n_x, n_y ) == segment_id ) {
			segment_continues = true;
			current_x = n_x;
			current_y = n_y;
		}
		if ( visited( n_x, n_y ) == 0 && fortune_v_segment( n_x, n_y ) != segment_id ) {
			new_segment_arises = true;
			new_segment_x = n_x;
			new_segment_y = n_y;
			new_segment_id = fortune_v_segment( n_x, n_y );
		}
	}
	//*** (end)

	if ( n_neighbors == 3 && segment_continues == true && new_segment_arises == true ) {
		if ( DEBUG_SWEEP_COMP >= 2 )
			cout << "	 * at " << segment_id << " jumper case " << endl;
		
		last_seg_index = temp_segment_vector.size() - 1;
		int length1 = follow_segment( segment_id, current_x, current_y );
		
		last_seg_index2 = temp_segment_vector.size() - 1;
		int length2 = follow_segment( new_segment_id, new_segment_x, new_segment_y );
		
		if ( length1 < length2 ) {
			erase_from_segment_vector( last_seg_index+1, last_seg_index2);
			temp_segment_vector.push_back( new_segment_id );
			if ( DEBUG_SWEEP_COMP >= 2 )
				cout << "	 * at " << segment_id << " pushing " << new_segment_id << endl;
		}
		else {
			erase_from_segment_vector( last_seg_index2+1, temp_segment_vector.size()-1 );
			//temp_segment_vector.push_back( segment_id );
		}
		if ( DEBUG_SWEEP_COMP >= 2 )
			cout << "	 * at " << segment_id << " returning	" << max( length1, length2 )	<< endl;
		return max( length1, length2 );
	}
	else {	
		
		bool stop = false;
		bool new_unvisited_segment = false;
		
		while( stop == false ) {
		
			for ( c_i = 0; c_i < n_neighbors; ++c_i ) {
				n_x = neighbors[c_i][0]; 
				n_y = neighbors[c_i][1];
				if ( visited( n_x, n_y ) == 0 && fortune_v_segment( n_x, n_y ) == segment_id ) {
					current_x = neighbors[c_i][0];
					current_y = neighbors[c_i][1];
					visited( current_x, current_y ) = 1;
				}
			}
		
			n_neighbors = get_voronoi_neighbors( neighbors, current_x, current_y );
		
			// check for a crossing towards a new segment
			for ( c_i = 0; c_i < n_neighbors; ++c_i ) {
				n_x = neighbors[c_i][0]; 
				n_y = neighbors[c_i][1];			
				if ( visited(n_x,n_y) == 0 && fortune_v_segment(n_x,n_y) != segment_id )
					new_unvisited_segment = true;
			}
			if ( n_neighbors == 1 || new_unvisited_segment )
				stop = true;

			++steps_taken;
		}
	
		// if neighbors is 1 it can only be the already visited neighbor we came from, hence dead-end
		// otherwise there is a new line segment
		// we cannot have three neighbors and not a new_unvisited_segment by construction
		// we can have two neighbors and a new_unvisited_segment!!!
	
		if ( n_neighbors == 1 ) {
			if ( DEBUG_SWEEP_COMP >= 2 )
				cout << "	 -> at segment " << segment_id << " returning " << steps_taken << " at deadend " << current_x << ":" << current_y	 << endl;
			return steps_taken;
		}
		else {
			int max_length = 0, this_length = 0, longest_segment_id = 0;
			bool record_status = false;
			// neighbors is 2 or 3; unvisited neighbors is 1 or 2; every neighbor is a new segment
			// since we have a new_unvisited_segment and we did not fall into the first check
			for ( c_i = 0; c_i < n_neighbors; ++c_i ) {
				// first set unvisited to semi-visited == 2
				n_x = neighbors[c_i][0]; 
				n_y = neighbors[c_i][1];
				if ( visited( n_x, n_y ) == 0 ) 
					visited( n_x, n_y ) = 2;
			}
			
			if ( n_neighbors == 3 ) { // with 3 neighbors we need to keep track of the vector
				last_seg_index = temp_segment_vector.size() - 1;
				record_status = true;
			}
			for ( c_i = 0; c_i < n_neighbors; ++c_i ) {
				// now visit semi-visited
				n_x = neighbors[c_i][0]; 
				n_y = neighbors[c_i][1];
				if ( visited( n_x, n_y ) == 2 ) {
					this_length = follow_segment( fortune_v_segment(n_x,n_y), n_x, n_y );
					if ( this_length >= max_length ) {
						longest_segment_id = fortune_v_segment(n_x,n_y);
						max_length = this_length;
					}
					if ( record_status == true ) {
						last_seg_index2 = temp_segment_vector.size() - 1;
						record_status = false;
					}	 
				}			 
			}
			if ( this_length == max_length && n_neighbors == 3) {
				// second one was better - erase from last_seg_index++ to last_seg_index2 inclusive
				erase_from_segment_vector( last_seg_index+1, last_seg_index2);
			}
			else if ( n_neighbors == 3 ){
				// first one waas better
				erase_from_segment_vector( last_seg_index2+1, temp_segment_vector.size()-1 );
			}
			 
			if ( DEBUG_SWEEP_COMP >= 2 )
				cout << "	 -> from segment " << segment_id << " returning " << max_length << " & pushing " << longest_segment_id << endl;		 
			temp_segment_vector.push_back( longest_segment_id );
			return max_length + steps_taken;
		}
	}
	// just to remove the compiler warning - logically this can never happen
	return 0;
}

br_pose_vector sg_map::follow_main_voronoi_segment_path( sg_vertex_d vertex, sg_edge_d edge1, sg_edge_d edge2 ) {
	if ( DEBUG_SWEEP_COMP >= 1 )
		cout << " Finding path from edge to edge for vertex " << vertex << " from " << edge1 << " to " << edge2 << endl;
	
	int recording_steps = 10; // record positione very 10 steps
	robot_cfg->lookupValue( "map.recording_steps", recording_steps );	 

	// get the voronoi point coordinates
	int voronoi_point_1[2], voronoi_point_2[2];
	voronoi_point_1[0] = G_g[edge1].x; voronoi_point_1[1] = G_g[edge1].y;
	voronoi_point_2[0] = G_g[edge2].x; voronoi_point_2[1] = G_g[edge2].y;
	
	if ( DEBUG_SWEEP_COMP >= 1 ) cout << " -> Getting the right direction ";
	int neighbors[9][2], n_neighbors, c_x, c_y, n_x, n_y, c_i; sg_vertex_d vd;
	n_neighbors = get_voronoi_neighbors( neighbors, voronoi_point_1[0], voronoi_point_1[1]);
	for ( c_i = 0; c_i < n_neighbors; ++c_i ) {
		vd = get_vertex_membership( neighbors[c_i][0] , neighbors[c_i][1] );
		if ( vd == vertex ) {
			c_x = neighbors[c_i][0]; c_y = neighbors[c_i][1];
		}
	}
	int current_segment =	 fortune_v_segment(c_x,c_y);
	if ( DEBUG_SWEEP_COMP >= 1 )
		cout << "into " << c_x << ":" << c_y << " segment " << current_segment << endl;
	
	// fill the temp_segment_vector with the list of the longest segment path
	temp_segment_vector.clear();
	visited.set_elements(0);
	visited(voronoi_point_1[0],voronoi_point_1[1]) = 1;
	visited(voronoi_point_2[0],voronoi_point_2[1]) = 1;
	
	follow_segment_path( current_segment, c_x, c_y, voronoi_point_2[0], voronoi_point_2[1] );	 
	temp_segment_vector.push_back( current_segment );
	
	if ( DEBUG_SWEEP_COMP >= 1 )
		cout << " -> Going through computed segment vector	" << endl;
	segment_vector_status();
	visited.set_elements(0);
	visited(voronoi_point_1[0],voronoi_point_1[1]) = 1;
	visited(voronoi_point_2[0],voronoi_point_2[1]) = 1;
	visited(c_x,c_y) = 1;
	path_points.set_elements(0);
		
	br_pose_vector pose_vec;
	br_pose new_pose(4);	
	std::vector<int>::reverse_iterator it = temp_segment_vector.rbegin(), it2;
	std::vector<int>::reverse_iterator it_end = temp_segment_vector.rend();
	bool next_segment_comes = false, current_segment_finished = true, last_segment = false;
	int next_segment = 0, step_counter = 0;
	for ( ; it != it_end && !last_segment; ++it ) {

		it2									= it; ++it2;
		current_segment			= *it;
		
		if ( it2 == it_end ) {
			cout << " LAST SEGMENT" << endl;
			last_segment = true;
		}
		next_segment				= *it2;
		next_segment_comes	= false;
		
		
		if ( DEBUG_SWEEP_COMP >= 3 )
			cout << "		Currently at segment " << current_segment << endl;
		if ( DEBUG_SWEEP_COMP >= 3 )
			cout << "		Expecting segment " << next_segment << endl;

		
		while( next_segment_comes == false ) {
			++step_counter;
			
			current_segment_finished = true;
			
			// find next position
			n_neighbors = get_voronoi_neighbors( neighbors, c_x, c_y ); 
			for ( c_i = 0; c_i < n_neighbors; ++c_i ) {
				n_x = neighbors[c_i][0]; 
				n_y = neighbors[c_i][1];
				if ( visited( n_x, n_y ) == 0 && fortune_v_segment( n_x, n_y ) == current_segment ) {
					c_x = n_x;
					c_y = n_y;
					if ( DEBUG_SWEEP_COMP >= 4 )
						cout << " " << c_x << ":" << c_y;
					visited( n_x, n_y ) = 1;
					path_points( n_x, n_y ) = 1;
					current_segment_finished = false;
				}
			} 
			
			if ( step_counter%recording_steps == 0 ) {
				cout << " RECORDING POSITION " << c_x << ":" << c_y << endl;
				new_pose.pose[0] = c_x;
				new_pose.pose[1] = c_y;
				pose_vec.push_back( new_pose );
			}
			//*** Checking for crossing over (fold)
			n_neighbors = get_voronoi_neighbors( neighbors, c_x, c_y ); 
			for ( c_i = 0; c_i < n_neighbors; ++c_i ) {
				n_x = neighbors[c_i][0]; 
				n_y = neighbors[c_i][1];			
				if ( visited(n_x,n_y) == 0 && fortune_v_segment(n_x,n_y) == next_segment )
					next_segment_comes = true;
			}	 
			if ( current_segment_finished == true && next_segment_comes == false ) {
				if ( DEBUG_SWEEP_COMP >= 3 )
					cout << "		--> current segment finished without new segment coming " << endl;
				
				// need to jump over to next segment 
				if ( last_segment == false ) {
					int neighbors2[9][2], n_neighbors2, c_i2, n_x2,n_y2;
					for ( c_i = 0; c_i < n_neighbors; ++c_i ) {
						n_x = neighbors[c_i][0]; 
						n_y = neighbors[c_i][1];			
						n_neighbors = get_voronoi_neighbors( neighbors2, n_x, n_y ); 
						for ( c_i2 = 0; c_i2 < n_neighbors2; ++c_i2 ) {
							n_x2 = neighbors2[c_i2][0]; 
							n_y2 = neighbors2[c_i2][1];			 
							if ( fortune_v_segment( n_x2, n_y2 ) == next_segment ) {
								c_x = n_x;
								c_y = n_y;
							}
						}
					}
				}
				next_segment_comes = true;
			}
			//*** (end)
		}
	}
	
	if ( DEBUG_SWEEP_COMP >= 1 )
		cout << " DONE finding path from edge to edge - returning pose vector " << endl;
	
	return pose_vec;
}

/* Author: Andreas Kolling
 * 
 * Major difference to follow_segment is that we disgard dead ends 
 * since we are looking for a path from edge to edge
 */

int sg_map::follow_segment_path( int segment_id,	int current_x, int current_y, int goal_x, int goal_y ) {
	if ( DEBUG_SWEEP_COMP >= 1 )
		cout << " Continuing PATH in segment " << segment_id << " at " << current_x << ":" << current_y << endl;
	
	segment_vector_status();
	visited( current_x, current_y ) = 1;
	
	int neighbors[9][2], n_neighbors, n_y, n_x, c_i;
	int steps_taken = 1, last_seg_index, last_seg_index2;


	if ( is_dead_end(segment_id) ) {
		if ( DEBUG_SWEEP_COMP >= 1 )
			cout << " Segment is a DEADEND - returning false " << endl;
		return false;
	}
	
	n_neighbors = get_voronoi_neighbors( neighbors, current_x, current_y );
	
	//*** Exception check (fold)
	// need to handle the exception when a new segment opens right after we entered a segment
	// that also continues
	bool segment_continues = false;
	bool new_segment_arises = false;
	int new_segment_id = 0, new_segment_x = 0, new_segment_y = 0;
	for ( c_i = 0; c_i < n_neighbors; ++c_i ) {
		n_x = neighbors[c_i][0]; 
		n_y = neighbors[c_i][1];		
		if ( visited( n_x, n_y ) == 0 && fortune_v_segment( n_x, n_y ) == segment_id ) {
			segment_continues = true;
			current_x = n_x;
			current_y = n_y;
		}
		if ( visited( n_x, n_y ) == 0 && fortune_v_segment( n_x, n_y ) != segment_id ) {
			new_segment_arises = true;
			new_segment_x = n_x;
			new_segment_y = n_y;
			new_segment_id = fortune_v_segment( n_x, n_y );
		}
	}
	//*** (end)

	if ( n_neighbors == 3 && segment_continues == true && new_segment_arises == true ) {
		if ( DEBUG_SWEEP_COMP >= 2 )
			cout << "	 * at " << segment_id << " jumper case " << endl;
		
		last_seg_index = temp_segment_vector.size() - 1;
		bool arrives1 = follow_segment_path( segment_id, current_x, current_y, goal_x, goal_y );
		
		last_seg_index2 = temp_segment_vector.size() - 1;
		bool arrives2 = follow_segment_path( new_segment_id, new_segment_x, new_segment_y, goal_x, goal_y );
		
		if ( arrives2 ) {
			erase_from_segment_vector( last_seg_index+1, last_seg_index2);
			temp_segment_vector.push_back( new_segment_id );
		}
		else {
			erase_from_segment_vector( last_seg_index2+1, temp_segment_vector.size()-1 );
		}
		return ( arrives2 || arrives1 );
	}
	else {	
		
		bool stop = false, found_goal = false, new_unvisited_segment = false;
		
		while( stop == false ) {
		
			for ( c_i = 0; c_i < n_neighbors; ++c_i ) {
				n_x = neighbors[c_i][0]; 
				n_y = neighbors[c_i][1];
				if ( visited( n_x, n_y ) == 0 && fortune_v_segment( n_x, n_y ) == segment_id ) {
					current_x = neighbors[c_i][0];
					current_y = neighbors[c_i][1];
					visited( current_x, current_y ) = 1;
				}
			}
		
			n_neighbors = get_voronoi_neighbors( neighbors, current_x, current_y );
		
			// check for a crossing towards a new segment
			for ( c_i = 0; c_i < n_neighbors; ++c_i ) {
				n_x = neighbors[c_i][0]; 
				n_y = neighbors[c_i][1];			
				if ( n_x == goal_x && n_y == goal_y )
					found_goal = true;
				if ( visited(n_x,n_y) == 0 && fortune_v_segment(n_x,n_y) != segment_id )
					new_unvisited_segment = true;
			}
			if ( n_neighbors == 1 || new_unvisited_segment || found_goal)
				stop = true;

			++steps_taken;
		}
	
		// if neighbors is 1 it can only be the already visited neighbor we came from, hence dead-end
		// otherwise there is a new line segment
		// we cannot have three neighbors and not a new_unvisited_segment by construction
		// we can have two neighbors and a new_unvisited_segment!!!
	
		if ( n_neighbors == 1 ) // can never be a path towards the goal
			return false;
		else if ( found_goal == true )
			return true;
		else {
			int reach_goal_segment_id = 0, n_count = 0;
			bool record_status = false, reaches_goal[9], goal_reached = false;
			// neighbors is 2 or 3; unvisited neighbors is 1 or 2; every neighbor is a new segment
			// since we have a new_unvisited_segment and we did not fall into the first check
			for ( c_i = 0; c_i < n_neighbors; ++c_i ) {
				// first set unvisited to semi-visited == 2
				n_x = neighbors[c_i][0]; 
				n_y = neighbors[c_i][1];
				if ( visited( n_x, n_y ) == 0 ) 
					visited( n_x, n_y ) = 2;
			}
			
			if ( n_neighbors == 3 ) { // with 3 neighbors we need to keep track of the vector
				last_seg_index = temp_segment_vector.size() - 1;
				record_status = true;
			}
			
			for ( c_i = 0; c_i < n_neighbors; ++c_i ) {
				// now visit semi-visited
				n_x = neighbors[c_i][0]; 
				n_y = neighbors[c_i][1];
				if ( visited( n_x, n_y ) == 2 ) {
					reaches_goal[n_count] = follow_segment_path( fortune_v_segment(n_x,n_y), n_x, n_y, goal_x, goal_y );
					if ( reaches_goal[n_count] ) {
						reach_goal_segment_id = fortune_v_segment(n_x,n_y);
						goal_reached = true;
					}
					if ( record_status == true ) {
						last_seg_index2 = temp_segment_vector.size() - 1;
						record_status = false;
					}
					n_count++;	
				}			 
			}
			if ( reaches_goal[0] && n_neighbors == 3) {
				// first one reaches goal
				erase_from_segment_vector( last_seg_index2+1, temp_segment_vector.size()-1 );
			}
			else if ( reaches_goal[1] && n_neighbors == 3 ){
				// second one reaches goal - erase from last_seg_index++ to last_seg_index2 inclusive
				erase_from_segment_vector( last_seg_index+1, last_seg_index2);
			}
			 
			temp_segment_vector.push_back( reach_goal_segment_id );
			return goal_reached;
		}
	}
	// just to remove the compiler warning - logically this can never happen
	return 0;
}

br_pose_vector sg_map::follow_main_voronoi_segment_path2( sg_vertex_d vertex, sg_edge_d edge1, int& reached_x, int& reached_y ) {
	if ( DEBUG_SWEEP_COMP >= 1 )
		cout << " Finding path from edge to path point for vertex " << vertex << " from " << edge1 << endl;
	
	int recording_steps = 10; // record positione very 10 steps
	robot_cfg->lookupValue( "map.recording_steps", recording_steps );	 

	// get the voronoi point coordinates
	int voronoi_point_1[2];
	voronoi_point_1[0] = G_g[edge1].x; voronoi_point_1[1] = G_g[edge1].y;
	
	if ( DEBUG_SWEEP_COMP >= 1 ) cout << " -> Getting the right direction ";
	int neighbors[9][2], n_neighbors, c_x, c_y, n_x, n_y, c_i; sg_vertex_d vd;
	n_neighbors = get_voronoi_neighbors( neighbors, voronoi_point_1[0], voronoi_point_1[1]);
	for ( c_i = 0; c_i < n_neighbors; ++c_i ) {
		vd = get_vertex_membership( neighbors[c_i][0] , neighbors[c_i][1] );
		if ( vd == vertex ) {
			c_x = neighbors[c_i][0]; c_y = neighbors[c_i][1];
		}
	}
	int current_segment =	 fortune_v_segment(c_x,c_y);
	if ( DEBUG_SWEEP_COMP >= 1 )
		cout << "into " << c_x << ":" << c_y << " segment " << current_segment << endl;
	
	// fill the temp_segment_vector with the list of the longest segment path
	temp_segment_vector.clear();
	visited.set_elements(0);
	visited(voronoi_point_1[0],voronoi_point_1[1]) = 1;
	
	follow_segment_path2( current_segment, c_x, c_y);	 
	temp_segment_vector.push_back( current_segment );
	
	if ( DEBUG_SWEEP_COMP >= 1 )
		cout << " -> Going through computed segment vector	" << endl;
	segment_vector_status();
	visited.set_elements(0);
	visited(voronoi_point_1[0],voronoi_point_1[1]) = 1;
	visited(c_x,c_y) = 1;
	path_points.set_elements(0);
		
	br_pose_vector pose_vec;
	br_pose new_pose(4);	
	std::vector<int>::reverse_iterator it(temp_segment_vector.end()), it2;
	std::vector<int>::reverse_iterator it_end(temp_segment_vector.begin());
	bool next_segment_comes = false, current_segment_finished = true, last_segment = false;
	int next_segment = 0, step_counter = 0;
	for ( ; it != it_end && !last_segment ; ++it ) {

		it2									= it; ++it2;
		current_segment			= *it;
		if ( it2 == it_end ) {
			cout << " LAST SEGMENT" << endl;
			last_segment = true;
		}
		next_segment				= *it2;
		next_segment_comes	= false;
		
		if ( DEBUG_SWEEP_COMP >= 3 )
			cout << "		Currently at segment " << current_segment << endl;
		if ( DEBUG_SWEEP_COMP >= 3 )
			cout << "		Expecting segment " << next_segment << endl;

		
		while( next_segment_comes == false ) {
			++step_counter;
			
			current_segment_finished = true;
			
			// find next position
			n_neighbors = get_voronoi_neighbors( neighbors, c_x, c_y ); 
			for ( c_i = 0; c_i < n_neighbors; ++c_i ) {
				n_x = neighbors[c_i][0]; 
				n_y = neighbors[c_i][1];
				if ( visited( n_x, n_y ) == 0 && fortune_v_segment( n_x, n_y ) == current_segment ) {
					c_x = n_x;
					c_y = n_y;
					if ( DEBUG_SWEEP_COMP >= 4 )
						cout << " " << c_x << ":" << c_y;
					visited( n_x, n_y ) = 1;
					path_points( n_x, n_y ) = 1;
					current_segment_finished = false;
				}
			} 
			
			if ( step_counter%recording_steps == 0 ) {
				cout << " RECORDING POSITION " << c_x << ":" << c_y << endl;
				new_pose.pose[0] = c_x;
				new_pose.pose[1] = c_y;
				pose_vec.push_back( new_pose );
			}
			//*** Checking for crossing over (fold)
			n_neighbors = get_voronoi_neighbors( neighbors, c_x, c_y ); 
			for ( c_i = 0; c_i < n_neighbors; ++c_i ) {
				n_x = neighbors[c_i][0]; 
				n_y = neighbors[c_i][1];			
				if ( visited(n_x,n_y) == 0 && fortune_v_segment(n_x,n_y) == next_segment )
					next_segment_comes = true;
			}	 
			if ( current_segment_finished == true && next_segment_comes == false ) {
				if ( DEBUG_SWEEP_COMP >= 3 )
					cout << "		--> current segment finished without new segment coming " << endl;
				
				if ( last_segment == false ) {
					// need to jump over to next segment 
					int neighbors2[9][2], n_neighbors2, c_i2, n_x2,n_y2;
					for ( c_i = 0; c_i < n_neighbors; ++c_i ) {
						n_x = neighbors[c_i][0]; 
						n_y = neighbors[c_i][1];			
						n_neighbors = get_voronoi_neighbors( neighbors2, n_x, n_y ); 
						for ( c_i2 = 0; c_i2 < n_neighbors2; ++c_i2 ) {
							n_x2 = neighbors2[c_i2][0]; 
							n_y2 = neighbors2[c_i2][1];			 
							if ( fortune_v_segment( n_x2, n_y2 ) == next_segment ) {
								c_x = n_x;
								c_y = n_y;
							}
						}
					}
				}
				next_segment_comes = true;
			}
			//*** (end)
		}
	}
	
	reached_x = partial_goal_point_x;
	reached_y = partial_goal_point_y;
	
	if ( DEBUG_SWEEP_COMP >= 1 )
		cout << " DONE finding path from edge to edge - returning pose vector " << endl;
	
	return pose_vec;
}

/* Author: Andreas Kolling
 * 
 * Major difference to follow_segment is that we disgard dead ends 
 * since we are looking for a path from edge to edge
 */

int sg_map::follow_segment_path2( int segment_id,	 int current_x, int current_y ) {
	if ( DEBUG_SWEEP_COMP >= 1 )
		cout << " Continuing PATH in segment " << segment_id << " at " << current_x << ":" << current_y << endl;
	
	segment_vector_status();
	visited( current_x, current_y ) = 1;
	
	int neighbors[9][2], n_neighbors, n_y, n_x, c_i;
	int steps_taken = 1, last_seg_index, last_seg_index2;

	if ( is_dead_end(segment_id)) {
		if ( DEBUG_SWEEP_COMP >= 1 )
			cout << " Segment is a DEADEND - returning false " << endl;
		return false;
	}

	n_neighbors = get_voronoi_neighbors( neighbors, current_x, current_y );
	
	//*** Exception check (fold)
	// need to handle the exception when a new segment opens right after we entered a segment
	// that also continues
	bool segment_continues = false;
	bool new_segment_arises = false;
	int new_segment_id = 0, new_segment_x = 0, new_segment_y = 0;
	for ( c_i = 0; c_i < n_neighbors; ++c_i ) {
		n_x = neighbors[c_i][0]; 
		n_y = neighbors[c_i][1];		
		if ( visited( n_x, n_y ) == 0 && fortune_v_segment( n_x, n_y ) == segment_id ) {
			segment_continues = true;
			current_x = n_x;
			current_y = n_y;
		}
		if ( visited( n_x, n_y ) == 0 && fortune_v_segment( n_x, n_y ) != segment_id ) {
			new_segment_arises = true;
			new_segment_x = n_x;
			new_segment_y = n_y;
			new_segment_id = fortune_v_segment( n_x, n_y );
		}
	}
	//*** (end)

	if ( n_neighbors == 3 && segment_continues == true && new_segment_arises == true ) {
		if ( DEBUG_SWEEP_COMP >= 2 )
			cout << "	 * at " << segment_id << " jumper case " << endl;
		
		last_seg_index = temp_segment_vector.size() - 1;
		bool arrives1 = follow_segment_path2( segment_id, current_x, current_y );
		
		last_seg_index2 = temp_segment_vector.size() - 1;
		bool arrives2 = follow_segment_path2( new_segment_id, new_segment_x, new_segment_y );
		
		if ( arrives2 ) {
			erase_from_segment_vector( last_seg_index+1, last_seg_index2);
			temp_segment_vector.push_back( new_segment_id );
		}
		else {
			erase_from_segment_vector( last_seg_index2+1, temp_segment_vector.size()-1 );
		}
		return ( arrives2 || arrives1 );
	}
	else {	
		
		bool stop = false, found_goal = false, new_unvisited_segment = false;
		
		while( stop == false ) {
		
			for ( c_i = 0; c_i < n_neighbors; ++c_i ) {
				n_x = neighbors[c_i][0]; 
				n_y = neighbors[c_i][1];
				if ( visited( n_x, n_y ) == 0 && fortune_v_segment( n_x, n_y ) == segment_id ) {
					current_x = neighbors[c_i][0];
					current_y = neighbors[c_i][1];
					visited( current_x, current_y ) = 1;
				}
			}
		
			n_neighbors = get_voronoi_neighbors( neighbors, current_x, current_y );
		
			// check for a crossing towards a new segment
			for ( c_i = 0; c_i < n_neighbors; ++c_i ) {
				n_x = neighbors[c_i][0]; 
				n_y = neighbors[c_i][1];			
				if ( path_points(n_x,n_y) == 1 ) {
					found_goal = true;
					partial_goal_point_x = n_x;
					partial_goal_point_y = n_y;
				}
				if ( visited(n_x,n_y) == 0 && fortune_v_segment(n_x,n_y) != segment_id )
					new_unvisited_segment = true;
			}
			if ( n_neighbors == 1 || new_unvisited_segment || found_goal)
				stop = true;

			++steps_taken;
		}
	
		// if neighbors is 1 it can only be the already visited neighbor we came from, hence dead-end
		// otherwise there is a new line segment
		// we cannot have three neighbors and not a new_unvisited_segment by construction
		// we can have two neighbors and a new_unvisited_segment!!!
	
		if ( n_neighbors == 1 ) // can never be a path towards the goal
			return false;
		else if ( found_goal == true )
			return true;
		else {
			int reach_goal_segment_id = 0, n_count = 0;
			bool record_status = false, reaches_goal[9], goal_reached = false;
			// neighbors is 2 or 3; unvisited neighbors is 1 or 2; every neighbor is a new segment
			// since we have a new_unvisited_segment and we did not fall into the first check
			for ( c_i = 0; c_i < n_neighbors; ++c_i ) {
				// first set unvisited to semi-visited == 2
				n_x = neighbors[c_i][0]; 
				n_y = neighbors[c_i][1];
				if ( visited( n_x, n_y ) == 0 ) 
					visited( n_x, n_y ) = 2;
			}
			
			if ( n_neighbors == 3 ) { // with 3 neighbors we need to keep track of the vector
				last_seg_index = temp_segment_vector.size() - 1;
				record_status = true;
			}
			
			for ( c_i = 0; c_i < n_neighbors; ++c_i ) {
				// now visit semi-visited
				n_x = neighbors[c_i][0]; 
				n_y = neighbors[c_i][1];
				if ( visited( n_x, n_y ) == 2 ) {
					reaches_goal[n_count] = follow_segment_path2( fortune_v_segment(n_x,n_y), n_x, n_y );
					if ( reaches_goal[n_count] ) {
						reach_goal_segment_id = fortune_v_segment(n_x,n_y);
						goal_reached = true;
					}
					if ( record_status == true ) {
						last_seg_index2 = temp_segment_vector.size() - 1;
						record_status = false;
					}
					n_count++;	
				}			 
			}
			if ( reaches_goal[0] && n_neighbors == 3) {
				// first one reaches goal
				erase_from_segment_vector( last_seg_index2+1, temp_segment_vector.size()-1 );
			}
			else if ( reaches_goal[1] && n_neighbors == 3 ){
				// second one reaches goal - erase from last_seg_index++ to last_seg_index2 inclusive
				erase_from_segment_vector( last_seg_index+1, last_seg_index2);
			}
			 
			temp_segment_vector.push_back( reach_goal_segment_id );
			return goal_reached;
		}
	}
	// just to remove the compiler warning - logically this can never happen
	return 0;
}



void sg_map::erase_from_segment_vector( int first_index, int last_index) {
	// delete first_index up to last_index inclusively
	std::vector<int>::iterator i = temp_segment_vector.begin();
	std::vector<int>::iterator j = i + first_index;
	std::vector<int>::iterator k = i + last_index + 1;
	if ( DEBUG_SWEEP_COMP >= 2 )
		cout << "	 --- REMOVING indices " << first_index << " to " << last_index << endl;
	temp_segment_vector.erase(j,k);
}

void sg_map::segment_vector_status( ) {
	std::vector<int>::iterator i = temp_segment_vector.begin();
	cout << " STATUS segment_vector: ";
	for ( ; i != temp_segment_vector.end(); ++i) {
		cout << " " << *i;
	}
	cout << endl;
}

//*** (end)

//******* Public Display / Print Functions ******* (fold)

void sg_map::save_img_posevector( const char* filename, br_pose_vector poses ) {
	IplImage* img = cvCreateImage( img_map_size, 8, 3);//IPL_DEPTH_8U
	this->write2img_occupancy( img );
	this->write2img_posevector( img , poses);
	cvSaveImage( filename, img );
	cvReleaseImage(&img);
} 

void sg_map::save_img_posevector_real( const char* filename, br_pose_vector poses ) {
	IplImage* img = cvCreateImage( img_map_size, 8, 3);//IPL_DEPTH_8U
	this->write2img_occupancy( img );
	this->write2img_posevector_real( img , poses);
	cvSaveImage( filename, img );
	cvReleaseImage(&img);
} 

void sg_map::save_img_occupancy( const char* filename ) {
	IplImage* img = cvCreateImage( img_map_size, 8, 3 );
	this->write2img_occupancy( img );
	if ( show_maps == 1 ) {
		cvNamedWindow( "Occupancy with causation lines", CV_WINDOW_AUTOSIZE );
		cvShowImage( "Occupancy with causation lines", img);
	}
	if ( save_imgs == 1 )
		cvSaveImage( filename , img );
	cvReleaseImage(&img);
}

void sg_map::save_img_boundary( const char* filename )  {
	IplImage* img = cvCreateImage( img_map_size, 8, 3 );
	this->write2img_boundary( img );
	if ( show_maps == 1 ) {
		cvNamedWindow( "boundary lines", CV_WINDOW_AUTOSIZE );
		cvShowImage( "boundary lines", img);
	}
	if ( save_imgs == 1 )
		cvSaveImage( filename , img );
	cvReleaseImage(&img);
}


void sg_map::save_img_clearance_value( const char* filename ) {
	IplImage* img = cvCreateImage( img_map_size, 8, 3 );
	this->write2img_clearance_value( img );
	if ( show_maps == 1 ) {
		cvNamedWindow( "Display of Costs of Wavepoints", CV_WINDOW_AUTOSIZE );
		cvShowImage( "Display of Costs of Wavepoints", img);
	}
	if ( save_imgs == 1 )
		cvSaveImage( filename, img );
	cvReleaseImage(&img); 
}

void sg_map::save_img_graph( const char* filename ) {		
	IplImage* img = cvCreateImage( img_map_size, 8, 3);//IPL_DEPTH_8U
	write2img_sg_vertices( img, 1 , 1 );
	write2img_sg_edges( img, 1 );
	for ( int i = 0; i < n_polygons; ++i )
		write2img_polygon( img, poly_boundary_simple[i] );
	cvSaveImage( filename, img );
	cvReleaseImage(&img);
	img = cvCreateImage( img_map_size, 8, 3);//IPL_DEPTH_8U
	write2img_sg_vertices( img, 0 , 1 );
	write2img_sg_edges( img, 1 );
	for ( int i = 0; i < n_polygons; ++i )
		write2img_polygon( img, poly_boundary_simple[i] );
	char filename2[100];
	sprintf( filename2, "%s%s", filename, "2.BMP");
	cvSaveImage( filename2, img );
	cvReleaseImage(&img);
}

void sg_map::save_img_voronoi_diagram( const char* filename ) {
	IplImage* img = cvCreateImage( img_map_size, 8, 3);
	this->write2img_occupancy( img );
	this->write2img_voronoi_diagram( img );
	cvSaveImage( filename, img );
	cvReleaseImage(&img);	 
}

void sg_map::save_img_voronoi_segments( const char* filename ) {
	IplImage* img = cvCreateImage( img_map_size, 8, 3);
	this->write2img_occupancy( img );
	this->write2img_voronoi_diagram_segments( img );
	cvSaveImage( filename, img );
	cvReleaseImage(&img);	 
}

void sg_map::save_img_voronoi_segments_alive( const char* filename ) {
	IplImage* img = cvCreateImage( img_map_size, 8, 3);
	this->write2img_occupancy( img );
	this->write2img_voronoi_diagram_segments_alive( img );
	cvSaveImage( filename, img );
	cvReleaseImage(&img);	 
}

void sg_map::save_img_voronoi_minima( const char* filename ) {
	IplImage* img = cvCreateImage( img_map_size, 8, 3);
	this->write2img_occupancy( img );
	this->write2img_voronoi_minima( img );
	cvSaveImage( filename, img );
	cvReleaseImage(&img);
}

void sg_map::save_img_segment_association( const char* filename ) {
	IplImage* img = cvCreateImage( img_map_size, 8, 3);
	this->write2img_occupancy( img );
	this->write2img_voronoi_diagram_segments( img );
	this->write2img_segment_association( img );
	cvSaveImage( filename, img );
	cvReleaseImage(&img);
}

void sg_map::save_img_diagram_cost_minima( const char* filename ) {
	IplImage* img = cvCreateImage( img_map_size, 8, 3);
	this->write2img_clearance_value( img );
	this->write2img_voronoi_diagram( img );
	this->write2img_voronoi_minima( img );
	cvSaveImage( filename, img );
	cvReleaseImage(&img);
}

void sg_map::save_img_polygon( const char* filename, Polygon& poly ) {
	IplImage* img = cvCreateImage( img_map_size, 8, 3);
	this->write2img_polygon( img, poly );
	cvSaveImage( filename, img );
	cvReleaseImage(&img);
}

void sg_map::save_img_poly_boundary_simple( const char* filename ) {
	IplImage* img = cvCreateImage( img_map_size, 8, 3);
	for ( int i = 0; i < n_polygons; ++i )
		write2img_polygon( img, poly_boundary_simple[i] );
	cvSaveImage( filename, img );
	cvReleaseImage(&img);
}


void sg_map::save_img_CGAL_voronoi_diagram( const char* filename ) {
	IplImage* img = cvCreateImage( img_map_size, 8, 3 );
	for ( int i = 0; i < n_polygons; ++i )
		write2img_polygon( img, poly_boundary_simple[i] );
	write2img_CGAL_voronoi_vertices( img );
	cvSaveImage( filename, img );
	cvReleaseImage(&img);
}

void sg_map::save_img_sg_vertex_all_crits( const char* filename) {
	IplImage* img = cvCreateImage( img_map_size, 8, 3 );
	for ( int i = 0; i < n_polygons; ++i )
		write2img_polygon( img, poly_boundary_simple[i] );
	for ( int i = 0; i < 3; ++i )
		write2img_sg_vertex_crits( img, i );
	cvSaveImage( filename, img );
	cvReleaseImage(&img);
}


void sg_map::save_img_sg_vertex_crits( const char* filename, int from_dir) {
	IplImage* img = cvCreateImage( img_map_size, 8, 3 );
	for ( int i = 0; i < n_polygons; ++i )
		write2img_polygon( img, poly_boundary_simple[i] );
	write2img_sg_vertices( img, true, false );
	write2img_sg_vertex_crits( img, from_dir );
	cvSaveImage( filename, img );
	cvReleaseImage(&img);
}




void sg_map::display_graph(bool display_text, bool display_boxes, char* filename, char* filename2) {
	CvScalar cv_scalar;
	IplImage* graph_bare_img = NULL;
//	IplImage* graph_bare_img_bw = NULL;
	IplImage* graph_img = NULL;
//	IplImage* graph_img_bw = NULL;
	graph_bare_img = cvCreateImage( img_map_size, 8, 3);
	graph_img = cvCreateImage( img_map_size, 8, 3);//IPL_DEPTH_8U
//	graph_bare_img_bw = cvCreateImage( img_map_size, 8, 1);
//	graph_img_bw = cvCreateImage( img_map_size, 8, 1);
	
	//cv_scalar = cvGet2D(graph_bare_img,i,j); 
	int i, j;
	for ( i = 0; i < graph_bare_img->height; i++ ) { //nrows
		for ( j = 0; j < graph_bare_img->width; j++ ) { //ncols
			if (this->get_occ(i,j) == 1) {
				cv_scalar.val[0] = 0;
				cv_scalar.val[1] = 0;
				cv_scalar.val[2] = 0;
			}
			else if (this->voronoi_minima(i,j) == 1) {
				cv_scalar.val[0] = 0;
				cv_scalar.val[1] = 0;
				cv_scalar.val[2] = 250;
			}
			//else if (this->get_wave_count(i,j) > 0) {
			//	cv_scalar.val[0] = 50;
			//	cv_scalar.val[1] = 0;
			//	cv_scalar.val[2] = 0;
			//}
			else {
				cv_scalar.val[0] = 250;
				cv_scalar.val[1] = 250;
				cv_scalar.val[2] = 250;
			}
			cvSet2D(graph_bare_img,i,j,cv_scalar);
			cvSet2D(graph_img,i,j,cv_scalar);
		}		
	}

	this->write2img_sg_vertices(graph_bare_img, display_text, display_boxes);
	this->write2img_sg_edges(graph_bare_img, display_text);

	this->write2img_causes_for_minima(graph_img);
	this->write2img_sg_vertices(graph_img, display_text, display_boxes);
	this->write2img_sg_edges(graph_img, display_text);

	//override the voronoi_minima points in red
	for ( i = 0; i < graph_bare_img->height; i++ ) { //nrows
		for ( j = 0; j < graph_bare_img->width; j++ ) { //ncols
			if (this->voronoi_minima(i,j) == 1) {
				cv_scalar.val[0] = 0;
				cv_scalar.val[1] = 250;
				cv_scalar.val[2] = 0;
				cvSet2D(graph_bare_img,i,j,cv_scalar);
			}
		}
	}
	
	if ( show_maps == 1) { 
		cvNamedWindow( "Graph clean", CV_WINDOW_AUTOSIZE ); 
		cvShowImage( "Graph clean", graph_bare_img);
		cvNamedWindow( "Graph", CV_WINDOW_AUTOSIZE ); 
		cvShowImage( "Graph", graph_img);
	}
	if ( save_imgs == 1 ) {
		cvSaveImage( filename2 , graph_bare_img );
		cvSaveImage( filename, graph_img );
	} 
	cvReleaseImage(&graph_bare_img);
	cvReleaseImage(&graph_img);
}							 

void sg_map::cout_wavepoint_cost() {
	cout << "Wavepoints_cost (matrix) " << endl;
	int i, j;
	for ( i = 0; i < this->nrows; i++ ) { //nrows
		cout << "ROW " << i << " \n";
		for ( j = 0; j < this->ncols; j++ ) //ncols
			cout << this->clearance_value(i,j) << "\t";
		cout << "\n \n";
	}
}

void sg_map::cout_cause(int i, int j) {
	map_point_list::iterator runner, end;
	runner = (*closest_obstacle)[i][j].begin();
	cout << "Causations: ";
	for(runner = (*closest_obstacle)[i][j].begin(); runner != (*closest_obstacle)[i][j].end(); runner++)
	{
		cout << " " << (*runner).first << " " << (*runner).second << " " << endl;
	}
}

void sg_map::cout_all_causes() {
	int i,j;
	for (i = 0; i < nrows ; i++) {
		for ( j = 0; j < ncols; j++) {
			cout << "------ Point " << i << ":" << j << endl;
			cout_cause(i,j);
			cout << "------" << endl;
		}
	}
}

void sg_map::cout_graph_diagnostic(int l_id) {
	cout << "	 ++++++	 MAP / GRAPH DIAGNOSTICS +++++ " << endl;
	sg_o_edge_it	out_edge_it, out_edge_it_end;
	sg_adj_it		adj_it, adj_it_end;
	sg_vertex_d vert_desc;
	sg_vertex_it	vert_it, vert_it_end;
	tie(vert_it, vert_it_end) = vertices(G_g);
	for (; vert_it != vert_it_end ; ++vert_it ) {
		tie(out_edge_it, out_edge_it_end)	 = out_edges(*vert_it, G_g);
		tie(adj_it, adj_it_end)				 = adjacent_vertices(*vert_it, G_g);
		vert_desc = vertex(*vert_it, G_g); //get the vertex descriptor
		cout_vertex(vert_desc);
	}

	// Traverse Edges
	sg_edge_it tree_edge_it,tree_edge_it_end;
	sg_edge_d ed;
	tie( tree_edge_it, tree_edge_it_end ) = edges( G_g );
	int counted_edge = 0;
	for (; tree_edge_it != tree_edge_it_end; tree_edge_it++) {
			counted_edge++;
			ed = *tree_edge_it;
			std::cout << "E: " << ed << " w=" << G_g[ed].w;
			std::cout << " l" << l_id << ",0=" << G_g[ed].label[l_id][0];
			std::cout << " l" << l_id << ",1=" << G_g[ed].label[l_id][1];
			std::cout << " mst?=" << G_g[ed].is_in_mst;
			std::cout << " V2,d0?=" << G_g[ed].is_in_V_2_perm[0];
			std::cout << " V2,d1?=" << G_g[ed].is_in_V_2_perm[1];
			std::cout << " x=" << G_g[ed].x;
			std::cout << " y=" << G_g[ed].y;
			std::cout << endl;
	}
}

void sg_map::cout_alive_vertices() {
	cout << "	 ++++++	 ALIVE VERTICES +++++ " << endl;
	sg_vertex_d vert_desc;
	sg_vertex_it	vert_it, vert_it_end;
	tie(vert_it, vert_it_end) = vertices(G_g);
	for (; vert_it != vert_it_end ; ++vert_it ) {
		vert_desc = vertex(*vert_it, G_g); //get the vertex descriptor
		if (G_g[vert_desc].alive == 1) {
			cout_vertex(vert_desc);
		}
	}
}

void sg_map::cout_vertex( sg_vertex_d vd ) {
	std::cout << "V: " << vd << " w=" << G_g[vd].w 
		<< " s=" << G_g[vd].s 
		<< " deg=" << G_g[vd].degree 
		<< " mstdeg=" << G_g[vd].mst_degree 
		<< " x=" << G_g[vd].x
		<< " y=" << G_g[vd].y
		<< " alive?=" << G_g[vd].alive
	<< endl;
}

void sg_map::cout_graph_summary() {
	cout << " leaf mergers " << leaf_mergers << endl;
	cout << " two mergers " << two_mergers << endl;
	
	int mst_edges = 0, mst_edge_weight = 0; 
	sg_edge_it e_i, e_i_end;
	tie( e_i , e_i_end ) = edges(G_g);
	for ( ; e_i != e_i_end ; ++e_i ) {
		if ( G_g[*e_i].is_in_mst == false ) {
			mst_edge_weight += G_g[*e_i].w;
			++mst_edges;
		}
	}
	cout << " mst edges " << mst_edges << endl;
	cout << " mst edge weight " << mst_edge_weight << endl;
}

//*** (end)

//******* write2img functions ******* (fold)

void sg_map::write2img_posevector( IplImage* img, br_pose_vector poses) {
	br_pose_vector::iterator it = poses.begin();
	int i,j, count = 0; float yaw;
	CvFont font;
	double hScale = 0.30;
	double vScale = 0.30;
	int		 lineWidth = 2;
	cvInitFont(&font,CV_FONT_HERSHEY_PLAIN,
				 hScale,vScale,0,lineWidth);
	char text_string[100];
	
	for( ; it != poses.end() ; ++it) {
		i = it->pose[0];
		j = it->pose[1];
		((uchar *)(img->imageData + i*img->widthStep))[j*img->nChannels + 0] = 0;// B
		((uchar *)(img->imageData + i*img->widthStep))[j*img->nChannels + 1] = 0; // G
		((uchar *)(img->imageData + i*img->widthStep))[j*img->nChannels + 2] = 250; // R
		
		sprintf( text_string,"%d", count);
		cvPutText ( img, text_string, cvPoint(j+5,i+5), &font, cvScalar(0,255,250) );
		
		if ( it->pose_dim > 2 ) {
			yaw = it->pose[2];		
			cvLine(img,
				cvPoint( j - 3 * cos(yaw),i - 3 * sin(yaw)),
				cvPoint( j - 6 * cos(yaw), i - 6 * sin(yaw) ),
				cvScalar(200,0,200), 1);		
		}
		++count;
	}
}

void sg_map::write2img_posevector_real( IplImage* img, br_pose_vector poses) {
	br_pose_vector::iterator it = poses.begin();
	int i,j, count = 0; float yaw;
	CvFont font;
	double hScale = 0.30;
	double vScale = 0.30;
	int		 lineWidth = 2;
	cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC,
				 hScale,vScale,0,lineWidth);
	char text_string[100];
	int max_pix[2];
	max_pix[0] = nrows;
	max_pix[1] = ncols;
	br_pose pix_pos;
	for( ; it != poses.end() ; ++it) {
		pix_pos = it->real_to_pixel( resolution, max_pix);
		i = pix_pos.pose[0];
		j = pix_pos.pose[1];
		if ( it->pose_dim > 2 ) {
			yaw = it->pose[2];		
			cvLine(img,
				cvPoint( i , j ),
				cvPoint( i + 6 * cos(yaw), j - 6 * sin(yaw) ),
				cvScalar(200,0,200), 1);		
		}
		((uchar *)(img->imageData + j*img->widthStep))[i*img->nChannels + 0] = 0;// B
		((uchar *)(img->imageData + j*img->widthStep))[i*img->nChannels + 1] = 0; // G
		((uchar *)(img->imageData + j*img->widthStep))[i*img->nChannels + 2] = 250; // R
		
		sprintf( text_string,"%d", count);
		cvPutText ( img, text_string, cvPoint(i+5,j+8), &font, cvScalar(0,255,250) );
		
		
		++count;
	}
}

void sg_map::write2img_clearance_value(IplImage* img) {
	int i, j;
	float max_cost = 0;
	for (i = 0; i < img->height; i++ ) { //nrows
		for (j = 0; j < img->width; j++ ) { //ncols
			if ( this->clearance_value(i,j) > max_cost ) 
				max_cost = this->clearance_value(i,j);
		}		
	}
	float factor = 255/max_cost;
	char value;
	for (i = 0; i < img->height; i++ ) { //nrows
		for (j = 0; j < img->width; j++ ) { //ncols
			value = ( char ) ( min( int((this->clearance_value(i,j) * factor)),255) );
			((uchar *)(img->imageData + i*img->widthStep))[j*img->nChannels + 0] = value;// B
			((uchar *)(img->imageData + i*img->widthStep))[j*img->nChannels + 1] = value; // G
			((uchar *)(img->imageData + i*img->widthStep))[j*img->nChannels + 2] = value; // R
		}		
	}
}

void sg_map::write2img_voronoi_diagram(IplImage* img) {
	int i, j;
	for ( i = 0; i < img->height; i++ ) { //nrows
		for ( j = 0; j < img->width; j++ ) { //ncols
			if ( this->fortune_voronoi(i,j) > 0) {
				((uchar *)(img->imageData + i*img->widthStep))[j*img->nChannels + 0] = 0; // B
				((uchar *)(img->imageData + i*img->widthStep))[j*img->nChannels + 1] = 200; // G
				((uchar *)(img->imageData + i*img->widthStep))[j*img->nChannels + 2] = 200; // R
			}
		}		
	}
}

void sg_map::write2img_voronoi_diagram_segments(IplImage* img) {
	int i, j, color_index;
	CvFont font;
	double hScale = 0.30;
	double vScale = 0.30;
	int		 lineWidth = 2;
	cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC,
				 hScale,vScale,0,lineWidth);

	for ( i = 0; i < img->height; i++ ) { //nrows
		for ( j = 0; j < img->width; j++ ) { //ncols
			if ( this->fortune_v_segment(i,j) > 0) {
				color_index = int ( fortune_v_segment(i,j) );
				((uchar *)(img->imageData + i*img->widthStep))[j*img->nChannels + 0] 
					= 70 * ( color_index%4 ) ; // B
				((uchar *)(img->imageData + i*img->widthStep))[j*img->nChannels + 1] 
					= 70 * ( (color_index)%3 ); // G
				((uchar *)(img->imageData + i*img->widthStep))[j*img->nChannels + 2] 
					= 120 * ( (color_index)%2 ); // G; // R
				
				//if ( i%30 == 0 || j%30 == 0) {
				//	char text_string[100];
				//	sprintf( text_string,"S%d", int ( fortune_v_segment(i,j) ) );
				//	cvPutText ( img, text_string, cvPoint(j+5,i+5), &font, cvScalar(0,255,250) );
				//}
			}
		}		
	}
}

void sg_map::write2img_voronoi_diagram_segments_alive(IplImage* img) {
	int i, j, color_index;
	for ( i = 0; i < img->height; i++ ) { //nrows
		for ( j = 0; j < img->width; j++ ) { //ncols
			if ( this->fortune_v_segment(i,j) > 0 && !is_dead_end(this->fortune_v_segment(i,j)) ) {
				color_index = int ( fortune_v_segment(i,j) );
				((uchar *)(img->imageData + i*img->widthStep))[j*img->nChannels + 0] 
					= 70 * ( color_index%4 ) ; // B
				((uchar *)(img->imageData + i*img->widthStep))[j*img->nChannels + 1] 
					= 70 * ( (color_index)%3 ); // G
				((uchar *)(img->imageData + i*img->widthStep))[j*img->nChannels + 2] 
					= 120 * ( (color_index)%2 ); // G; // R
			}
		}		
	}
}

void sg_map::write2img_voronoi_minima(IplImage* img) {
	int i, j;
	for ( i = 0; i < img->height; i++ ) { //nrows
		for ( j = 0; j < img->width; j++ ) { //ncols
			if ( this->voronoi_minima(i,j) > 0) {
				((uchar *)(img->imageData + i*img->widthStep))[j*img->nChannels + 0]	= 0; // B
				 ((uchar *)(img->imageData + i*img->widthStep))[j*img->nChannels + 1] = 0; // G
				 ((uchar *)(img->imageData + i*img->widthStep))[j*img->nChannels + 2] = 250; // R
			}
		}		
	} 
}

void sg_map::write2img_segment_association(IplImage* img) {
	int i, j, color_index;
	for ( i = 0; i < img->height; i++ ) { //nrows
		for ( j = 0; j < img->width; j++ ) { //ncols
			if ( this->belongs_to_segment(i,j) > 0) {
				color_index = int ( belongs_to_segment(i,j) );
				((uchar *)(img->imageData + i*img->widthStep))[j*img->nChannels + 0] 
					= 70 * ( color_index%4 ) ; // B
				((uchar *)(img->imageData + i*img->widthStep))[j*img->nChannels + 1] 
					= 70 * ( (color_index)%3 ); // G
				((uchar *)(img->imageData + i*img->widthStep))[j*img->nChannels + 2] 
					= 120 * ( (color_index)%2 ); // G; // R
			}
		}		
	} 
}

void sg_map::write2img_graph(IplImage* img) {
	int i, j;
	for ( i = 0; i < img->height; i++ ) { //nrows
		for ( j = 0; j < img->width; j++ ) { //ncols
			if (this->get_occ(i,j) == 0) {
				((uchar*)(img->imageData + img->widthStep*i))[j] 
					= ( char ) ( min( int((this->voronoi_minima(i,j) * 100)),255) );
			}
			else
				((uchar*)(img->imageData + img->widthStep*i))[j] = ( char ) ( 255 );
		}		
	}
}

void sg_map::write2img_causes_for_minima(IplImage* img) {
	pointvector::iterator k = voronoi_minima_points.begin();
	map_point_list::iterator runner;
	int i,j;
	for (; k != voronoi_minima_points.end(); k++ ) {
		i = (*k).x; j = (*k).y;
		runner = (*closest_obstacle)[i][j].begin();
		for(; runner != (*closest_obstacle)[i][j].end(); runner++) {
			cvLine(img,
				cvPoint((*runner).second, (*runner).first),
				cvPoint(j, i),
				cvScalar(0,0,100), 1);
		}
	}
}

void sg_map::write2img_sg_vertex_crits( IplImage* img, int from_dir ) {
	// from_dir is 0,1,2
	CvFont font;
	double hScale = 1.0;
	double vScale = 1.0;
	int		 lineWidth = 2;
	cvInitFont(&font,CV_FONT_HERSHEY_PLAIN,
				 hScale,vScale,0,lineWidth);
	char text_string[100];
	if ( from_dir < SG_GRAPH_MAX_VERTEX_DEGREE ) {
		sg_vertex_d  vd;
		sg_vertex_it vert_it, vert_it_end;
		tie(vert_it, vert_it_end) = vertices(G_g);
		for (; vert_it != vert_it_end; ++vert_it) {
			vd = *vert_it;
			if ( G_g[vd].x > 0 && ( from_dir < int(degree(vd,G_g)) ) ) {
				SDG::Point_2 P1 = G_g[vd].crit_p[from_dir][0];
				SDG::Point_2 P2 = G_g[vd].crit_p[from_dir][1];
				SDG::Point_2 P3 = G_g[vd].crit_p[from_dir][2];
				cvLine( img,
				        cvPoint( int(P1.y()), P1.x() ),
				        cvPoint( P2.y(), P2.x() ),
				        cvScalar(125*from_dir,125*((from_dir+1)%3),125*((from_dir+2)%3)), 4);
				cvLine( img,
				        cvPoint( P2.y(), P2.x() ),
				        cvPoint( P3.y(), P3.x() ),
				        cvScalar(125*from_dir,125*((from_dir+1)%3),125*((from_dir+2)%3)), 4);
				sprintf( text_string,"%d", int(vd) );
				cvPutText ( img, text_string, cvPoint( P1.y(), P1.x() ), &font, cvScalar(0,255,0) );
				cvPutText ( img, text_string, cvPoint( P2.y(), P2.x() ), &font, cvScalar(0,255,0) );
				cvPutText ( img, text_string, cvPoint( P3.y(), P3.x() ), &font, cvScalar(0,255,0) );
			}
		}
	}
	else {
		cout << " ERROR : SG_GRAPH_MAX_VERTEX_DEGREE EXCEEDED!" << endl;
		//cout << " OCCURRED AT sg_ver " << vd1 << " & sg_ver " << vd2 << endl;
	}
}

void sg_map::write2img_sg_vertex_crits( IplImage* img, sg_vertex_d vd, int from_dir ) {
	// from_dir is 0,1,2
	CvFont font;
	double hScale = 1.0;
	double vScale = 1.0;
	int		 lineWidth = 2;
	cvInitFont(&font,CV_FONT_HERSHEY_PLAIN,
				 hScale,vScale,0,lineWidth);
	char text_string[100];
	if ( from_dir < SG_GRAPH_MAX_VERTEX_DEGREE ) {
		if ( G_g[vd].x > 0 && ( from_dir < int(degree(vd,G_g)) )) {
			SDG::Point_2 P1 = G_g[vd].crit_p[from_dir][0];
			SDG::Point_2 P2 = G_g[vd].crit_p[from_dir][1];
			SDG::Point_2 P3 = G_g[vd].crit_p[from_dir][2];
			cvLine( img,
			        cvPoint( int(P1.y()), P1.x() ),
			        cvPoint( P2.y(), P2.x() ),
			        cvScalar(125*from_dir,125*((from_dir+1)%3),125*((from_dir+2)%3)), 4);
			cvLine( img,
			        cvPoint( P2.y(), P2.x() ),
			        cvPoint( P3.y(), P3.x() ),
			        cvScalar(125*from_dir,125*((from_dir+1)%3),125*((from_dir+2)%3)), 4);
			sprintf( text_string,"%d", int(vd) );
			cvPutText ( img, text_string, cvPoint( P1.y(), P1.x() ), &font, cvScalar(0,255,0) );
			cvPutText ( img, text_string, cvPoint( P2.y(), P2.x() ), &font, cvScalar(0,255,0) );
			cvPutText ( img, text_string, cvPoint( P3.y(), P3.x() ), &font, cvScalar(0,255,0) );
		}
	}
	else {
		cout << " ERROR : SG_GRAPH_MAX_VERTEX_DEGREE EXCEEDED!" << endl;
		//cout << " OCCURRED AT sg_ver " << vd1 << " & sg_ver " << vd2 << endl;
	}
}



void 
sg_map::write2img_vertex_crit_pts( IplImage* img,  sg_vertex_d vd1, sg_vertex_d vd2) {
	// find the proper n_edges number for coming-from-edge of vd2
	sg_edge_d ed;
	sg_o_edge_it out_edge_it, out_edge_it_end;
	tie(out_edge_it, out_edge_it_end) = out_edges( vd2, G_g);
	int n_edges = 0;
	//cout << " Sweeping weight for " << vd2 << " coming from " << vd1 << endl;
	for (;out_edge_it != out_edge_it_end; out_edge_it++) {
		ed = *out_edge_it;
		if ( vd1 == source( ed, G_g ) || vd1 == target( ed, G_g ) )
			break;
		++n_edges;
	}
	//cout << " is at " << n_edges << " and is " << G_g[vd2].ww[n_edges] << endl;
	CvFont font;
	double hScale = 1.0, vScale = 1.0;
	int    lineWidth = 2;
	cvInitFont( &font, CV_FONT_HERSHEY_PLAIN, hScale, vScale, 0, lineWidth );
	char text_string[100];
	if ( n_edges < SG_GRAPH_MAX_VERTEX_DEGREE ) {
		if ( G_g[vd2].x > 0 ) {
			SDG::Point_2 P1 = G_g[vd2].crit_p[n_edges][0];
			SDG::Point_2 P2 = G_g[vd2].crit_p[n_edges][1];
			SDG::Point_2 P3 = G_g[vd2].crit_p[n_edges][2];
			cvLine( img, cvPoint( int(P1.y()), P1.x() ), cvPoint( P2.y(), P2.x() ),
			        cvScalar(125,250,0), 4);
			cvLine( img, cvPoint( P2.y(), P2.x() ), cvPoint( P3.y(), P3.x() ),
			        cvScalar(125,250,0), 4);
			//sprintf( text_string,"%d", int(vd2) );
			//cvPutText ( img, text_string, cvPoint( P1.y(), P1.x() ), &font, cvScalar(0,255,0) );
			//cvPutText ( img, text_string, cvPoint( P2.y(), P2.x() ), &font, cvScalar(0,255,0) );
			//cvPutText ( img, text_string, cvPoint( P3.y(), P3.x() ), &font, cvScalar(0,255,0) );
			if ( G_g[vd2].ww[n_edges] > 0 ) {
				sprintf( text_string,"%d ", G_g[vd2].ww[n_edges] );
				cvPutText ( img, text_string, 
				            cvPoint( P2.y()/2 + P3.y()/2, P2.x()/2+P3.x()/2 ), 
				            &font, cvScalar(0,255,0) );
			}
		}
	}
	else {
		cout << " ERROR : SG_GRAPH_MAX_VERTEX_DEGREE EXCEEDED!" << endl;
		cout << " OCCURRED AT sg_ver " << vd1 << " & sg_ver " << vd2 << endl;
	}
}

void 
sg_map::write2img_edge_crit_pts( IplImage* img,  sg_edge_d ed) {
	SDG::Point_2 P1 = G_g[ed].crit_p[0];
	SDG::Point_2 P2 = G_g[ed].crit_p[1];
	
	cvLine( img,
	        cvPoint( P1.y(), P1.x() ),
	        cvPoint( P2.y(), P2.x() ),
	        cvScalar(0,0,250), 4);
	CvFont font;
	double hScale = 1.0, vScale = 1.0;
	int    lineWidth = 2;
	cvInitFont( &font, CV_FONT_HERSHEY_PLAIN, hScale, vScale, 0, lineWidth );
	char text_string[100];
	if ( G_g[ed].w > 0 ) {
		sprintf( text_string,"%d ", G_g[ed].w );
		if ( G_g[ed].is_in_mst == false ) {
			cvPutText ( img, text_string, 
			            cvPoint( P1.y()/2 + P2.y()/2, P1.x()/2+P2.x()/2 ), 
			            &font, cvScalar(250,100,0) );			
		}
		else {
			cvPutText ( img, text_string, 
			            cvPoint( P1.y()/2 + P2.y()/2, P1.x()/2+P2.x()/2 ), 
			            &font, cvScalar(0,0,250) );
		}
	}
	
}



void sg_map::write2img_sg_vertices(IplImage* img, bool display_text, bool display_boxes) {
	CvFont font;
	double hScale = 1.0;
	double vScale = 1.0;
	int		 lineWidth = 2;
	cvInitFont(&font,CV_FONT_HERSHEY_PLAIN,
				 hScale,vScale,0,lineWidth);
	char text_string[100]; 
	
	sg_vertex_d vd;
	sg_vertex_it vert_it, vert_it_end;
	tie(vert_it, vert_it_end) = vertices(G_g);
	for (; vert_it != vert_it_end; ++vert_it) {
		vd = *vert_it;
		if ( G_g[vd].x > 0 ) {
			cvCircle(img, cvPoint(G_g[vd].y,G_g[vd].x), 2, cvScalar(0,255,0), 1);
			if ( display_text == true ) {
				//sprintf( text_string,"V%d-%d-%d", int(vd), G_g[vd].w,G_g[vd].s);
				sprintf( text_string,"%d", int(vd) );
				cvPutText ( img, text_string, cvPoint(G_g[vd].y,G_g[vd].x+14), &font, cvScalar(0,255,0) );
			}
		}
	}
}

void sg_map::write2img_sg_edges(IplImage* img, bool display_text) {
	
	CvFont font;
	double hScale = 0.40;
	double vScale = 0.40;
	int		 lineWidth = 2;
	cvInitFont(&font,CV_FONT_HERSHEY_PLAIN, hScale,vScale,0,lineWidth);
	// char text_string[100]; 
	int red, green;
	sg_edge_it edge_it, edge_it_end;
	tie(edge_it,edge_it_end) = edges(G_g);
	sg_vertex_d vd1, vd2;
	for (; edge_it != edge_it_end; ++edge_it) {
		vd1 = source(*edge_it, G_g);
		vd2 = target(*edge_it, G_g);
		green = 255;
		red = 0;
		if (G_g[*edge_it].label[G_g.label_id][0] > 0 && G_g[*edge_it].label[G_g.label_id][1] > 0) {
			green = 0;
			red = 255; // red is std edge
		}
		cvLine(img,
			cvPoint(G_g[vd1].y, G_g[vd1].x),
			cvPoint(G_g[vd2].y, G_g[vd2].x),
			cvScalar(0,green,red), 1);	// green is mst edge
		if ( display_text == true ) {
			//sprintf( text_string,"%d,%d,L0=%d,L1=%d", G_g[*edge_it].w, G_g[*edge_it].is_in_mst, G_g[*edge_it].label[G_g.label_id][0], G_g[*edge_it].label[G_g.label_id][1]);
			//sprintf( text_string,"%d", G_g[*edge_it].w);
			//cvPutText ( img, text_string, cvPoint((G_g[vd1].y + G_g[vd2].y)/2, (G_g[vd1].x + G_g[vd2].x)/2), &font, cvScalar(255,0,0) );	
		}
	}
} 

void sg_map::write2img_occupancy( IplImage* img ) {
	CvScalar cv_scalar;
	int i, j;
	for ( i = 0; i < img->height; i++ ) { //nrows
		for ( j = 0; j < img->width; j++ ) { //ncols
			if (this->get_occ(i,j) == 1) {
				cv_scalar.val[0] = 0;
				cv_scalar.val[1] = 0;
				cv_scalar.val[2] = 0;
			}
			else {
				cv_scalar.val[0] = 250;
				cv_scalar.val[1] = 250;
				cv_scalar.val[2] = 250;
			}
			cvSet2D(img,i,j,cv_scalar);
		}		
	}
}

void sg_map::write2img_boundary( IplImage* img ) {
	CvScalar cv_scalar;
	int i, j;
	for ( i = 0; i < img->height; i++ ) { //nrows
		for ( j = 0; j < img->width; j++ ) { //ncols
			if (this->boundary(i,j) == 1) {
				cv_scalar.val[0] = 0;
				cv_scalar.val[1] = 0;
				cv_scalar.val[2] = 0;
			}
			else {
				cv_scalar.val[0] = 250;
				cv_scalar.val[1] = 250;
				cv_scalar.val[2] = 250;
			}
			cvSet2D(img,i,j,cv_scalar);
		}		
	}
}

void sg_map::write2img_visited(IplImage* img) {
	int i, j;
	for (i = 0; i < img->height; i++ ) { //nrows
		for (j = 0; j < img->width; j++ ) { //ncols
			if ( visited(i,j) == 1 )
			((uchar*) (img->imageData + img->widthStep*i))[j] 
				 = ( char ) ( 255 );
			else
				((uchar*) (img->imageData + img->widthStep*i))[j] 
					 = ( char ) ( 0);
		}		
	}
}

void sg_map::write2img_polygon_boundary_simple( IplImage* img ) {
	for ( int i = 0; i < n_polygons; ++i )
		write2img_polygon( img, poly_boundary_simple[i] );
}

void sg_map::write2img_polygon( IplImage* img, Polygon& poly ) {
	int cvline_width = 1;
	CGAL_Point P1 = poly[0];
	int i = 0;
	for ( i = 1; i < poly.size() ; ++i ) {
		cvLine(img, 
		  cvPoint( P1.y(),P1.x() ), 
		  cvPoint( poly[i].y(),poly[i].x() ), 
		  cvScalar(250,250,250), cvline_width );
		//cout << " P " << P1.y() << P1.x() << endl;
		cvSet2D( img, P1.x(), P1.y(), cvScalar(0,0,255) );
		P1 = poly[i];
	}
	cvLine(img, 
	  cvPoint( P1.y(), P1.x() ), 
	  cvPoint( poly[0].y(),poly[0].x() ), 
	  cvScalar(250,250,250), cvline_width );
}

void sg_map::write2img_CGAL_voronoi_vertices( IplImage* img ) {
	VD_Point_2 P1,P2;
	CGAL_VD::Edge_iterator edge_it, edge_it_end; 
	edge_it_end = voronoi_diagram.edges_end();
	edge_it     = voronoi_diagram.edges_begin();
	// Go through all edges
	//for ( int i = 0 ; edge_it != edge_it_end; ++edge_it, ++i) {
	//	if ( edge_it->is_segment() ) {
	//		P1 = edge_it->source()->point();
	//		P2 = edge_it->target()->point();
	//		//cout << (*edge_it) << endl;
	//		cvLine(img, 
	//		  cvPoint( P1.y(),P1.x() ), 
	//		  cvPoint( P2.y(),P2.x() ), 
	//		  cvScalar(220,0,0), 1);
	//	}
	//}
	// Go through all vertices
	CGAL_VD::Vertex_iterator v_it, v_it_end; 
	v_it     = voronoi_diagram.vertices_begin();
	v_it_end = voronoi_diagram.vertices_end();
	for ( int i = 0 ; v_it != v_it_end; ++v_it, ++i) {
		P1 = v_it->point();		
		cvLine(img, 
		  cvPoint( P1.y(),P1.x() ), 
		  cvPoint( P1.y(),P1.x() ), 
		  cvScalar(0,0,220), 1);
	}
}

//*** (end)


//******* Helper functions ******* (fold)

sg_graph* sg_map::get_graph() {
	return &(G_g);
}

/* Gives the distance of the vector a,b to the origin */
float sg_map::dis_qui(int a, int b) {
	return float(sqrt(pow(double(a),2) + pow(double(b),2)));
	//return float(abs(a) + abs(b));
}

/* Gives the distance of two int locations */
float sg_map::distance(int a, int b, int c, int d) {
	return float(sqrt(pow(double(a-c),2) + pow(double(b-d),2)));
}

/* 
 * Sets ii,jj to the value for the given direction of a case, counted
 * clockwise beginning at north
 */
void sg_map::get_eight_case(int case_nr, int& ii, int& jj) {
	switch(case_nr) {
		case 0:
			ii = -1; jj = 0; //up			| center
			break;
		case 1:
			ii = -1; jj = 1; //up			| right
			break;
		case 2:
			ii = 0; jj = 1;	 //center		| right
			break;
		case 3:
			ii = 1; jj = 1;	 //bottom		| right
			break;
		case 4:
				ii = 1; jj = 0;	 //bottom		| center
			break;
		case 5:
			ii = 1; jj = -1;	//bottom	 | left
			break;
		case 6:
			ii = 0; jj = -1; //center			| left
			break;
		case 7:
			ii = -1; jj = -1; //up		| left
			break;
	}
}

void sg_map::get_sixteen_case(int case_nr, int& ii, int& jj) {
	if ( case_nr <= 4) {
		ii = -2; 
		jj = -2 + case_nr;
	}
	else if ( case_nr <= 9 ) {
		ii = 2;
		jj = -2 + case_nr - 5;
	}
	else if ( case_nr <= 12 ) {
		ii = -1 + case_nr - 10;
		jj = -2;
	}
	else if ( case_nr <= 16 ) {
		ii = -1 + case_nr - 13;
		jj = 2;
	}
}

/* helper to check bounds of indices */
bool sg_map::bound_check(int n_x, int n_y) {
	return (n_x >= 0 && n_x < nrows && n_y >= 0 && n_y < ncols);
}

void sg_map::add_vertex_membership(int i, int j, sg_vertex_d vd) {
		(*vertex_membership)[i][j] = vd;
}

sg_vertex_d sg_map::get_vertex_membership(int i, int j) {
	return (*vertex_membership)[i][j];
}

int sg_map::count_vertices() {
	int vertex_count = 0;
	sg_vertex_d vd;
	sg_vertex_it vert_it, vert_it_end;
	tie(vert_it, vert_it_end) = vertices(G_g);
	for (; vert_it != vert_it_end; ++vert_it) {
		vd = *vert_it;
		if ( G_g[vd].x > 0 ) {
			vertex_count++;
		}
	}
	return vertex_count;
}

//*** (end)

//******** Occupancy ******** (fold)

void sg_map::set_occ(int row, int col, int value) {
	occ(row,col) = float(value);
}

int sg_map::get_occ(int row, int col) {
	return int(this->occ(row,col));
}

void sg_map::add_occ_pt(int i, int j) {
	occ_points.push_back( sg_map_point(i,j) );
}

sg_map_point sg_map::get_occ_pt(int i) {
	return occ_points[i];
}

//*** (end)


/*
 * Add a cause point (int pair) to the 2-d vector of lists of causes at position
 * i,j
 */

void sg_map::add_closest_obstacle(int i, int j, int cause_x, int cause_y) {
	std::pair<int,int>* new_p = new std::pair<int,int>(cause_x,cause_y);
		(*closest_obstacle)[i][j].push_back(*new_p);
}

/* Clear the list at position i,j */
void sg_map::reset_closest_obstacles(int i, int j) {
	(*closest_obstacle)[i][j].clear();
}

//*** (end)
