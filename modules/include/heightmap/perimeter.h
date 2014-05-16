#ifndef PERIMETER_H
#define PERIMETER_H

#include <map>
#include <deque>
#include <set>
#include <list>
#include "heightmap/heightmap.h"
#include "heightmap/heightcell.h"
#include "utilities/misc.h"
#include "visibility.h"
#include "utilities/math/bresenham.h"

//class HeightMap;
//class Visibility::Vertex;

class Perimeter 
{
	friend class Viewer;

  public:
	struct Cell{
		Cell(int _x, int _y, double _value=0.0) : x(_x), y(_y), value(_value){};
		Cell() : x(0), y(0) {};

        bool operator()(const Cell p1, const Cell p2) const
        {
           if (p1.x > p2.x)
              return true;
           else if (p1.x < p2.x)
              return false;
           else if (p1.y > p2.y)
              return true;
           else
              return false;
        }
		void print() {printf("(%d,%d,%lf)",x,y,value);}
        double sqrdist(Cell& o) {return (o.x-x)*(o.x-x)+(o.y-y)*(o.y-y);};
        double dist(Cell& o) {return sqrt(sqrdist(o));};

        int x,y;
		double value;
      };

     struct Vertex {
         Vertex(int _id, Cell _pos, double _height=0.0, double _range = 0.0) : 
				id(_id), pos(_pos), height(_height), range(_range), percentage(0), minDistToBorder(0) {};
         int id;
         Cell pos;
         double height;
         double range;
		 double percentage;
		 double minDistToBorder;
		 vector<Cell> border;
		 vector<Cell> perimeter;
      };

public:
	Perimeter(HeightMap* _map, Visibility * _vis, double, double, double) {value=NULL;hmap=_map;vis=_vis;};
	~Perimeter() {if (value != NULL) delete [] value;};
	
	bool load(const string &fname);

	// Computes observation regions covering the perimeter	
	void computePerimeterRegions(); 
    void computePerimeterRegionsBIAS(const std::deque<Perimeter::Cell>& cells, const vector<double>& cells_probability, std::list<Visibility::Pos>& pos_list);
    
	// Computes observation regions covering the perimeter given by the cells
	void computePerimeterRegions(const std::deque<Perimeter::Cell>& cells);
	
	
	
	void computePerimeterSchedule();
	bool verify(int x, int y);
	Cell drawRandomFromP(); 
	double getValue(int x, int y) {if (value !=NULL) return value[x][y];else return -1;};	
	int getNumVertices() {return vertices.size();};

private:
	      HeightMap* hmap;
		  Visibility* vis;
		  std::set<Cell,Cell> P; // perimeter region
		  std::deque<Vertex> vertices;
	      double **value;
	
};

#endif