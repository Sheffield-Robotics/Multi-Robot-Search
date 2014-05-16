#ifndef POLYGONS_H
#define POLYGONS_H

#include "heightmap/visibility.h"

void writeVisSetToFile(string filename, const Visibility::VisSet& vset );

//void writePolygonFromVisSet(ofstream& s, Visibility::VisSet vset, int num );
std::deque< std::deque<Visibility::Pos> > writePolygonFromVisSet(std::ofstream& outstream, const Visibility::VisSet& vset, int num );

void filterVisibilitySet(Visibility::VisSet& vset,  bool onlyOnePoly=false, int x=0, int y=0);


//bool importantFizzelArea(Visibility::VisSet& vset);
#endif
