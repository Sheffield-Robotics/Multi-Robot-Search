#define TERRAIN_ERROR_INVALID_PARAM		-5
#define TERRAIN_ERROR_LOADING_IMAGE		-4
#define TERRAIN_ERROR_MEMORY_PROBLEM	-3
#define	TERRAIN_ERROR_NOT_SAVED			-2
#define TERRAIN_ERROR_NOT_INITIALISED	-1
#define TERRAIN_OK						 0
#include "heightmap/heightmap.h"

enum HMAP_COLOR_MODE {
   HMAP_COLOR_NONE,
   HMAP_COLOR_PLAIN,
   HMAP_COLOR_VARIANCE,
   HMAP_COLOR_CLASS,
   HMAP_COLOR_VISIBILITY,
   HMAP_COLOR_REGION_SET,
};

int terrainLoadFromHeightMap(HeightMap* map, bool drawSmoothedCells, HMAP_COLOR_MODE colorMode);
void terrainDraw(bool redraw, float xOffset, float yOffset);
void terrainDrawWithNames(float xOffset, float yOffset);
void terrainDestroy();
int terrainScale(float min,float max);
float terrainGetHeight(int x, int z);
int terrainSimulateLighting(int sim);
void terrainLightPosition(float x, float y, float z,float w);
void terrainDiffuseColor(float r, float g, float b);
void terrainAmbientColor(float r, float g, float b);
int terrainDim(float stepWidth, float stepLength);


void terrainRemoveColoredCell(long cellNum, bool all=false);
void terrainAddColoredCell(long cellNum, double r, double g, double b);
void  terrainAddColoredCell(int cx, int cy, double r, double g, double b);
void terrainDrawColoredCells(float xOffset, float yOffset);
void terrainGetBoundingBox(double &x1,double &y1,double &x2,double &y2);
