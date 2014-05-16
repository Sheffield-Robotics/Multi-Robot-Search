#include "heightmap/perimeter.h"

#include <list>
#include <map>
#include <deque>
#include <set>
#include "heightmap/heightmap.h"
#include "heightmap/heightcell.h"
#include "utilities/misc.h"
#include "utilities/math/bresenham.h"
#include "heightmap/visibility_helpers.h"

using namespace std;

// Compare Cells for ordering them
bool cellComp(const Perimeter::Cell c1, const Perimeter::Cell c2)
{
	return (c1.value < c2.value);
}

void Perimeter::computePerimeterSchedule() 
{
	computePerimeterRegions();		
	printf("Have %d vertices\n", (int) vertices.size());

	if (vertices.size() == 0)
		return;

	// Initialize Vertices
	for (unsigned int vert=0; vert<vertices.size(); vert++) 
	{
		printf("ID is %d    Cell is (%d,%d)\n",vertices[vert].id, vertices[vert].pos.x, vertices[vert].pos.y);
		vertices[vert].border.clear();
		vertices[vert].perimeter.clear();		
		vertices[vert].percentage = drand48();

		// SEARCH PERIMETER AND BORDER CELLS		
		list<Cell> search;
		set<Cell,Cell> closed;
		Cell c(vertices[vert].pos.x,vertices[vert].pos.y);
		search.push_back(c);
		closed.insert(c);
		int size = 1;
		while (!search.empty()) 
		{
			// Take out next element in queue
			Cell cP = search.front();
			search.pop_front();

			// Initialize distance with inf
			cP.value = HUGE_VAL; 

			// Check if cell is of perimeter type
			if (hmap->getCellsMM()[cP.x][cP.y].isPerimeter()) {
				vertices[vert].perimeter.push_back(cP);
			}

			// Check if cell is of border type
			if (hmap->getCellsMM()[cP.x][cP.y].getBorders().size() != 0) {
				// Add pos to result
				vertices[vert].border.push_back(cP);
				continue;
			}

			// Expand 
			for (int xx=-1; xx<=1; xx++) {
				for (int yy=-1; yy<=1; yy++) {
					if (verify(cP.x+xx,cP.y+yy) && closed.find(Cell(cP.x+xx,cP.y+yy)) == closed.end())  {
						search.push_back(Cell(cP.x+xx,cP.y+yy));
						closed.insert(Cell(cP.x+xx, cP.y+yy));
						size++;
					}
				}
			}
		}
		printf("Size: %d  Found %d perimeter cells and %d border cells\n",
			size, (int) vertices[vert].perimeter.size(), (int) vertices[vert].border.size());

		// Compute distances
		////////////////////////////////////////////////////////////////
		search.clear();
		closed.clear();
		// Initialize all perimeter cells with distance 1
		printf("Adding perimeter cells: \n");
		for (unsigned int ii=0; ii<vertices[vert].perimeter.size(); ii++) {
			Cell cell = vertices[vert].perimeter[ii];
			search.push_back(cell);
			closed.insert(cell);
			//hmap->getCellsMM()[cell.x][cell.y].setDistance(1.0); 
			printf("%d %d\n",cell.x, cell.y);
			value[cell.x][cell.y] = 0.0;
			cell.value = 0.0;
			cell.print();
			fflush(stdout);
		} printf("\n");
	
		// Perform Bushfire Algorithm
		// for computing minimal distances from border to perimeter
		////////////////////////////////////////////////////////////
		int expCount=0;
		while (!search.empty()) {
			// Sorting list so that smallest elements are on top
			search.sort(cellComp);

			// Expand
			Cell c = search.front();
			search.pop_front();
			for (int xx=-1; xx<=1; xx++) {
				for (int yy=-1; yy<=1; yy++) {
					Cell cp = Cell(c.x+xx, c.y+yy);

					if (!verify(cp.x, cp.y) || closed.find(Cell(cp.x,cp.y)) != closed.end())
						continue;		

					closed.insert(cp);
					expCount++;

					double D_old = value[cp.x][cp.y];					
					double minDist = HUGE_VAL;
					for (int xx2=-1; xx2<=1; xx2++) {
						for (int yy2=-1; yy2<=1; yy2++) {
							Cell a = Cell(cp.x+xx2, cp.y+yy2);
							if (!verify(a.x,a.y)) continue;							
							double d = cp.dist(a) + value[a.x][a.y];
							if (d<minDist) minDist = d;			        
						}
					}
					//printf("%d %d --> D_old: %lf, MinDist: %lf\n",cp.x, cp.y, D_old, minDist);
					value[cp.x][cp.y] = minDist;
					cp.value = minDist; 				
					if (minDist<D_old && (hmap->getCellsMM()[cp.x][cp.y].getBorders().size() == 0)) 
						search.push_back(cp);
				}
			}
		}
		//printf("Expanded %d cells\n",expCount);		

	// Print out the minimal dist through the border cells
	printf("Minimal distances of border: ");
	double min = HUGE_VAL;
	for (unsigned int kk=0; kk<vertices[vert].border.size(); kk++) {
		 Cell p = vertices[vert].border[kk];
		 double d = value[p.x][p.y];
		 if (d<min) min = d;
		
		// TODO: Handle special cases
		// 1st Border is protected by obstacle
		
		//2nd Border is end of map

/*		for (int xx2=-1; xx2<=1; xx2++) {
			for (int yy2=-1; yy2<=1; yy2++) {
				Cell c = Cell(p.x+xx, p.y+yy);
				
				if (c part of patch) ?
					if (c part of border) ?
						if (c part of obstacle) ?
				
				
			}
		}
*/		
	}
	printf("%lf \n",min);
	}

}


// Check if position is within map
bool Perimeter::verify(int x, int y) 
{
   if (hmap==NULL || x<0 || y<0 || x >= hmap->sizeX() || y >= hmap->sizeZ()) {
      return false;
   }
   return true;
}

bool Perimeter::load(const string &fname) 
{
	printf("Loading from file %s\n",fname.c_str());
	return true;
};


void Perimeter::computePerimeterRegions()
{
	// Initialize value array
	if (value != NULL) delete [] value;
	value = new double* [hmap->sizeX()];
	for (int x=0; x<hmap->sizeX(); x++) {
		value[x] = new double [hmap->sizeZ()];
		for (int y=0; y<hmap->sizeZ(); y++) {
			value[x][y] = HUGE_VAL;		
			}					
	}
	
	// Clear all old stuff
	vertices.clear();
	for (int xx=0; xx<hmap->sizeX(); xx++) {
		for (int yy=0; yy<hmap->sizeZ(); yy++) { 
			hmap->getCellsMM()[xx][yy].getRegions().clear();
			hmap->getCellsMM()[xx][yy].getBorders().clear();
			hmap->getCellsMM()[xx][yy].setRGB(0.5,0.5,0.5);
//			hmap->getCellsMM()[xx][yy].togglePerimeterOff(); 			
		}
	}

	// Generate Set
	deque<Perimeter::Cell> cells;
	for (int xx=0; xx<hmap->sizeX(); xx++) {
		for (int yy=0; yy<hmap->sizeZ(); yy++) {
			if ((hmap->getCellsMM()[xx][yy].getClass() == HeightCell::ELC_FLAT_GROUND))  {
			// only add perimeter pixels in this case
				if (hmap->getCellsMM()[xx][yy].isPerimeter()) {
					cells.push_back(Cell(xx,yy));
					}
			}
		}
	}
	printf("Cells has the size %d\n",(int) cells.size());
	computePerimeterRegions(cells);
}

void Perimeter::computePerimeterRegionsBIAS(const deque<Perimeter::Cell>& cells,
    const vector<double>& cells_probability, std::list<Visibility::Pos>& pos_list)
{
	// Generate P
	P.clear();
    vertices.clear();
    
	for (unsigned int i=0; i<cells.size(); i++) {
		P.insert(cells[i]);
	}		
	//printf("P has the size %d\n",(int) P.size());

	int count = 0;
    int min_x = hmap->sizeX(),max_x = 0;
    int min_y = hmap->sizeZ(),max_y = 0;
    // Cover P with regions
	while (!P.empty()) {
		Cell p = drawRandomFromP();
		Visibility::VisSet vset; vis->computeVisibilitySet(p.x, p.y, vset);

		int removedCells = 0;
		if (Params::g_use_only_one_region_per_pixel) {
			if (!Params::g_generate_regular_instead_of_sparse_edges)
				removedCells = vis->filterOutAssignedCells( vset );
			filterVisibilitySet(vset, true, p.x,p.y);
		}
		double r = drand48();
		double g = drand48();
		double b = drand48();

		bool added = false;
		if ( (vset.size() + removedCells) >= Params::g_imporved_sampling_min_area_size ) {
			vertices.push_back(Perimeter::Vertex(count, p));
            Visibility::Pos vis_pos(p.x,p.y);
            pos_list.push_back(vis_pos);
			//printf("Add --> ");
			added=true;
		}

		// Remove vis set from P
		//printf("Removing %d\n",(int)vset.size());
		for (Visibility::VisSet::iterator it=vset.begin(); it != vset.end(); )  {
			Visibility::Pos p_vis = it->first;
			it++;
			if (added) {
				// color cells according to region color
				hmap->getCellsMM()[p_vis.x][p_vis.y].setRGB(r,g,b);
				hmap->getCellsMM()[p_vis.x][p_vis.y].getRegions().push_back(count);
			}
			P.erase(Cell(p_vis.x,p_vis.y));
			if ( p_vis.x > max_x ) 
                max_x = p_vis.x;
            if ( p_vis.y > max_y ) 
                max_y = p_vis.y;
            if ( p_vis.x < min_x ) 
                min_x = p_vis.x;
            if ( p_vis.y < min_y ) 
                min_y = p_vis.y;
		}
		if (added)
			count++;
	}
	//printf("Created %d vertexes\n",count);

	// Compute border cells and mark them white
	for (int xx=0; xx<hmap->sizeX(); xx++) 
	for (int yy=0; yy<hmap->sizeZ(); yy++) {
		vis->detectRegionBorder(xx, yy, true);
		if (hmap->getCellsMM()[xx][yy].isBorderCell()) {
			//hmap->getCellsMM()[xx][yy].setRGB(1.0,1.0,1.0);
		}
        hmap->getCellsMM()[xx][yy].getRegions().clear();
	}
	//M_INFO2("Computed border cells\n");
}



void Perimeter::computePerimeterRegions(const deque<Perimeter::Cell>& cells)
{
	// Generate P
	P.clear();
    vertices.clear();
    
	for (unsigned int i=0; i<cells.size(); i++) {
		P.insert(cells[i]);
	}		
	//printf("P has the size %d\n",(int) P.size());

	int count = 0;
    int min_x = hmap->sizeX(),max_x = 0;
    int min_y = hmap->sizeZ(),max_y = 0;
    // Cover P with regions
	while (!P.empty()) {
		Cell p = drawRandomFromP();
		Visibility::VisSet vset; vis->computeVisibilitySet(p.x, p.y, vset);

		int removedCells = 0;
		if (Params::g_use_only_one_region_per_pixel) {
			if (!Params::g_generate_regular_instead_of_sparse_edges)
				removedCells = vis->filterOutAssignedCells( vset );
			filterVisibilitySet(vset, true, p.x,p.y);
		}
		double r = drand48();
		double g = drand48();
		double b = drand48();

		bool added = false;
		if ( (vset.size() + removedCells) >= Params::g_imporved_sampling_min_area_size ) {
			vertices.push_back(Perimeter::Vertex(count, p));
			//printf("Add --> ");
			added=true;
		}

		// Remove vis set from P
		//printf("Removing %d\n",(int)vset.size());
		for (Visibility::VisSet::iterator it=vset.begin(); it != vset.end(); )  {
			Visibility::Pos p_vis = it->first;
			it++;
			if (added) {
				// color cells according to region color
				hmap->getCellsMM()[p_vis.x][p_vis.y].setRGB(r,g,b);
				hmap->getCellsMM()[p_vis.x][p_vis.y].getRegions().push_back(count);
			}
			P.erase(Cell(p_vis.x,p_vis.y));
			if ( p_vis.x > max_x ) 
                max_x = p_vis.x;
            if ( p_vis.y > max_y ) 
                max_y = p_vis.y;
            if ( p_vis.x < min_x ) 
                min_x = p_vis.x;
            if ( p_vis.y < min_y ) 
                min_y = p_vis.y;
		}
		if (added)
			count++;
	}
	//printf("Created %d vertexes\n",count);

	// Compute border cells and mark them white
	for (int xx=0; xx<hmap->sizeX(); xx++) 
	for (int yy=0; yy<hmap->sizeZ(); yy++) {
		vis->detectRegionBorder(xx, yy, true);
		if (hmap->getCellsMM()[xx][yy].isBorderCell()) {
			//hmap->getCellsMM()[xx][yy].setRGB(1.0,1.0,1.0);
		}
        hmap->getCellsMM()[xx][yy].getRegions().clear();
	}
	//M_INFO2("Computed border cells\n");

#if 0

	vector<Cell> dummy1;
	vector< vector <Cell> > dummy2( vertices.size()  , dummy1);
	vector< vector < vector <Cell> > > borders_region_intersect( vertices.size(), dummy2);
	// borders_region_intersect[j][jj] === \delta D(v_j) \intersect D(v_{jj}) 

	// fill borders_region_intersect
	for (int xx=0; xx<hmap->sizeX(); xx++) 
	{
		for (int yy=0; yy<hmap->sizeZ(); yy++) 
		{
			vector<int> borders = hmap->getCellsMM()[xx][yy].getBorders();
			for (unsigned j=0; j<borders.size(); j++) 
			{
			// xx,yy is a border pixel for D(v_j)
			// check vicinity (new, previously we only checked xx,yy)
				vector<Cell> vicinity;
				for (int xxx=-1; xxx<=1; xxx++) {
					for (int yyy=-1; yyy<=1; yyy++) {
						if (verify(xx+xxx,yy+yyy,true)) {
							vicinity.push_back(Cell(xx+xxx,yy+yyy));
						}
					}
				}
				for (unsigned int i=0; i<vicinity.size(); i++) 
				{
					Cell v = vicinity[i];
					vector<int> regions = hmap->getCellsMM()[v.x][v.y].getRegions();
					for ( unsigned jj=0; jj<regions.size(); jj++ )
					{
					// D(v_jj) also sees this pixel xx,yy
						if ( borders[j] != regions[jj] )
							borders_region_intersect[ borders[j] ][ regions[jj] ].push_back( Cell(xx,yy) );
					}
				}               
			}
		}
	}

	vector< int > dummy3( vertices.size() );
	vector< vector<int> > edge_matrix( vertices.size(), dummy3 );
	for ( unsigned j=0; j<vertices.size(); j++) 
	{  
		for ( unsigned jj=0; jj<vertices.size(); jj++ )
		{
			edge_matrix[j][jj] = 0;
		}
	}

	unsigned counter = 0;

	for ( unsigned j=0; j<vertices.size(); j++) 
	{
		for ( unsigned jj=0; jj<vertices.size(); jj++ )
		{
			if ( jj == j )
				continue;
			if ( borders_region_intersect[j][jj].size() == 0 )
				continue;
			// check whether the intersection of \delta D(v_j) and D(v_{jj})
			// is covered entirely by another region
			vector<int> covering_regions;
			bool fill_first = true;
			// go through all points of the intersection of \delta D(v_j) and D(v_{jj})
			for ( unsigned i=0; i < borders_region_intersect[j][jj].size(); ++i )
			{
			// going through all points of the intersection of 
			// \delta D(v_j) and D(v_{jj})
				if ( !fill_first && covering_regions.size() == 0)
					break;
				Cell pose = borders_region_intersect[j][jj][i];
				vector<int> regions = hmap->getCellsMM()[pose.x][pose.y].getRegions();
				if ( fill_first )
				{
					covering_regions = regions; 
					fill_first = false;
				}
			// go through all covering regions and see whether coverage is broken
				vector<int> new_covering_regions;
				for ( unsigned ii=0; ii < covering_regions.size(); ++ii )
				{
				// is covering_regions[ii] in regions?
					if ( find( regions.begin(), regions.end(), covering_regions[ii] ) != regions.end() )
					{
						if ( covering_regions[ii] != int(j) && covering_regions[ii] != int(jj) )
							new_covering_regions.push_back( covering_regions[ii] );
					}               
				}
				covering_regions = new_covering_regions;
			}

			if ( covering_regions.size() > 0 )
			{
				edge_matrix[j][jj] = 2;
			}
			else
			{
				edge_matrix[j][jj] = 1;
			}
			++counter;
			for ( unsigned i=0; i < borders_region_intersect[j][jj].size(); ++i )
			{
				Cell pose = borders_region_intersect[j][jj][i];
				hmap->getCellsMM()[pose.x][pose.y].setDistance( counter );
			}
		}
	}

	// Clean up matrix and make symmetric
	for ( unsigned j=0; j<vertices.size(); j++) 
	{
		for ( unsigned jj=0; jj<vertices.size(); jj++ )
		{
			edge_matrix[j][jj] = std::min( edge_matrix[j][jj], edge_matrix[jj][j] );
		}
	}

	// Graph construction
	// Vertexes
}


endGraphT = getCurrentTime();

// Write statistics from experiment
if (Params::g_run_experiment_mode) {
	static bool first = true;
	if (first) {
		writeExperimentInfo("PERI_EXPERIMENT_STATS.info");
		first=false;
	}
	ofstream outfile;
	outfile.open("PERI_EXPERIMENT_STATS.txt",ios_base::app);
	outfile 
		<< base_filename.c_str() << " "
		<< Params::g_pursuer_height << " "
		<< Params::g_pursuer_range << " "
		<< Params::g_target_height << " "
		<< Params::g_use_improved_sampling << " "
		<< Params::g_imporved_sampling_min_area_size << " "
		<< Params::g_use_only_one_region_per_pixel << " "
		<< Params::g_min_size_for_polygon_at_vertex << " "
		<< Params::g_generate_regular_instead_of_sparse_edges << " "
		<< Params::g_compute_shady_edges << " "
		<< endGraphT-startT << " "
		<< endStrategyT - endGraphT 
		<< endl;
	outfile.close();
	
#endif
}

Perimeter::Cell Perimeter::drawRandomFromP() 
{
   int pos = (int) (drand48() * (P.size()-1));
	set<Cell,Cell>::iterator it = P.begin();
   while (pos-- > 0)
      it++;
   Cell p = *it;
   P.erase(it);
   return p;
}
