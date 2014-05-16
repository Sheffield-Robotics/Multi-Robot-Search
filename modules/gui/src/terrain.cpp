#include <math.h>
#ifdef __APPLE__
#include <GLUT/glut.h>
#include <OpenGL/glu.h>
#include <OpenGL/gl.h>
#else
#include <GL/glut.h>
#include <GL/glu.h>
#include <GL/gl.h>
#endif

#include <stdlib.h>
#include <stdio.h>
#include "gui/terrain.h"
#include <vector>
#include <string.h>
#include "utilities/paramfile.h"

using std::vector;

static int terrainGridWidth = 0;
static int terrainGridLength = 0;
static float *terrainHeights = NULL;
static float *terrainColors = NULL;
static float *terrainNormals = NULL;
static float terrainStepLength = 1.0;
static float terrainStepWidth = 1.0;


static float terrainLightPos[4] = {0.0,0.1,0.1,0.0};
static float terrainDiffuseCol[3] = {1.0,1.0,1.0};
static float terrainAmbientCol[3] = {0.04,0.04,0.04};
static int terrainSimLight = 1;

static void terrainComputeNormals();
static void terrainNormalize(float *v);

static int terrainDL;

struct ColoredCell 
{
   int num;
   int cx;
   int cy;
   double r,g,b;
};

static vector<ColoredCell> coloredCells;


void terrainGetBoundingBox(double &x1,double &y1,double &x2,double &y2)
{
   x1 = 0.0;
   y1 = 0.0;
   x2 = terrainGridWidth * terrainStepWidth;
   y2 = terrainGridLength * terrainStepLength;
}

void terrainLightPosition(float x, float y, float z,float w) {

   terrainLightPos[0] = x;
   terrainLightPos[1] = y;
   terrainLightPos[2] = z;
   terrainLightPos[3] = w;

   /* normalise this vector to save time later */
   if (terrainLightPos[3] == 0.0)
      terrainNormalize(terrainLightPos);
}

void terrainDiffuseColor(float r, float g, float b) {

   terrainDiffuseCol[0] = r ;
   terrainDiffuseCol[1] = g;
   terrainDiffuseCol[2] = b;

}

void terrainAmbientColor(float r, float g, float b) {

   terrainAmbientCol[0] = r;
   terrainAmbientCol[1] = g;
   terrainAmbientCol[2] = b;
}



int terrainSimulateLighting(int sim) {

   terrainSimLight = sim;

   /* just in case we don't have normals already */
   if (terrainNormals == NULL) {
      terrainNormals = (float *)malloc(terrainGridWidth * terrainGridLength * sizeof(float) * 3);
      terrainComputeNormals();
   }
   if (terrainNormals == NULL) 
      return(TERRAIN_ERROR_MEMORY_PROBLEM);
   else
      return(TERRAIN_OK);

}


static float *terrainCrossProduct(int x1,int z1,int x2,int z2,int x3,int z3) {

   float *auxNormal,v1[3],v2[3];

   v1[0] = (x2-x1) * terrainStepWidth; 
   v1[1] = -terrainHeights[z1 * terrainGridWidth + x1] 
      + terrainHeights[z2 * terrainGridWidth + x2];
   v1[2] = (z2-z1) * terrainStepLength; 


   v2[0] = (x3-x1) * terrainStepWidth; 
   v2[1] = -terrainHeights[z1 * terrainGridWidth + x1] 
      + terrainHeights[z3 * terrainGridWidth + x3];
   v2[2] = (z3-z1) * terrainStepLength; 

   auxNormal = (float *)malloc(sizeof(float)*3);

   auxNormal[2] = v1[0] * v2[1] - v1[1] * v2[0];
   auxNormal[0] = v1[1] * v2[2] - v1[2] * v2[1];
   auxNormal[1] = v1[2] * v2[0] - v1[0] * v2[2];

   return(auxNormal);
}

static void terrainNormalize(float *v) {

   double d;

   d = sqrt((v[0]*v[0]) + (v[1]*v[1]) + (v[2]*v[2]));

   v[0] = v[0] / d;
   v[1] = v[1] / d;
   v[2] = v[2] / d;
}

static void terrainAddVector(float *a, float *b) {

   a[0] += b[0];
   a[1] += b[1];
   a[2] += b[2];
}
void terrainComputeNormals() {

   float *norm1,*norm2,*norm3,*norm4; 
   int i,j,k;

   if (terrainNormals == NULL)
      return;


   for(i = 0; i < terrainGridLength; i++)
      for(j = 0; j < terrainGridWidth; j++) {
         norm1 = NULL;
         norm2 = NULL;
         norm3 = NULL;
         norm4 = NULL;

         /* normals for the four corners */
         if (i == 0 && j == 0) {
            norm1 = terrainCrossProduct(0,0, 0,1, 1,0);	
            terrainNormalize(norm1);				
         }
         else if (j == terrainGridWidth-1 && i == terrainGridLength-1) {
            norm1 = terrainCrossProduct(j,i, j,i-1, j-1,i);	
            terrainNormalize(norm1);				
         }
         else if (j == 0 && i == terrainGridLength-1) {
            norm1 = terrainCrossProduct(j,i, j,i-1, j+1,i);	
            terrainNormalize(norm1);				
         }
         else if (j == terrainGridWidth-1 && i == 0) {
            norm1 = terrainCrossProduct(j,i, j,i+1, j-1,i);	
            terrainNormalize(norm1);				
         }

         /* normals for the borders */
         else if (i == 0) {
            norm1 = terrainCrossProduct(j,0, j-1,0, j,1);
            terrainNormalize(norm1);
            norm2 = terrainCrossProduct(j,0,j,1,j+1,0);
            terrainNormalize(norm2);
            terrainAddVector(norm1,norm2);
            free(norm2);
         }
         else if (j == 0) {
            norm1 = terrainCrossProduct(0,i, 1,i, 0,i-1);
            terrainNormalize(norm1);
            norm2 
               = terrainCrossProduct(0,i, 0,i+1, 1,i);
            terrainNormalize(norm2);
            terrainAddVector(norm1,norm2);
            free(norm2);
         }
         else if (i == terrainGridLength-1) {
            norm1 = terrainCrossProduct(j,i, j,i-1, j+1,i);
            terrainNormalize(norm1);
            norm2 = terrainCrossProduct(j,i, j+1,i, j,i-1);
            terrainNormalize(norm2);
            terrainAddVector(norm1,norm2);
            free(norm2);
         }
         else if (j == terrainGridWidth-1) {
            norm1 = terrainCrossProduct(j,i, j,i-1, j-1,i);
            terrainNormalize(norm1);
            norm2 = terrainCrossProduct(j,i, j-1,i, j,i+1);
            terrainNormalize(norm2);
            terrainAddVector(norm1,norm2);
            free(norm2);
         }

         /* normals for the inner vertices using 8 neighbours */
         else {
            norm1 = terrainCrossProduct(j,i, j-1,i, j-1,i+1);
            terrainNormalize(norm1);
            norm2 = terrainCrossProduct(j,i, j-1,i+1, j,i+1);
            terrainNormalize(norm2);
            terrainAddVector(norm1,norm2);
            free(norm2);
            norm2 = terrainCrossProduct(j,i, j,i+1, j+1,i+1);
            terrainNormalize(norm2);
            terrainAddVector(norm1,norm2);
            free(norm2);
            norm2 = terrainCrossProduct(j,i, j+1,i+1, j+1,i);
            terrainNormalize(norm2);
            terrainAddVector(norm1,norm2);
            free(norm2);
            norm2 = terrainCrossProduct(j,i, j+1,i, j+1,i-1);
            terrainNormalize(norm2);
            terrainAddVector(norm1,norm2);
            free(norm2);
            norm2 = terrainCrossProduct(j,i, j+1,i-1, j,i-1);
            terrainNormalize(norm2);
            terrainAddVector(norm1,norm2);
            free(norm2);
            norm2 = terrainCrossProduct(j,i, j,i-1, j-1,i-1);
            terrainNormalize(norm2);
            terrainAddVector(norm1,norm2);
            free(norm2);
            norm2 = terrainCrossProduct(j,i, j-1,i-1, j-1,i);
            terrainNormalize(norm2);
            terrainAddVector(norm1,norm2);
            free(norm2);
         }

         terrainNormalize(norm1);
         norm1[2] = - norm1[2];
         for (k = 0; k< 3; k++) 
            terrainNormals[3*(i*terrainGridWidth + j) + k] = norm1[k];

         free(norm1);


      }
}


int terrainLoadFromHeightMap(HeightMap* map, bool drawSmoothedCells, HMAP_COLOR_MODE colorMode)
{
   // if a terrain already exists, destroy it.
   if (terrainHeights != NULL)
      terrainDestroy();

   // set the width and height of the terrain 
   terrainGridWidth = map->sizeX();
   terrainGridLength = map->sizeZ();
   terrainStepWidth = map->resolution();
   terrainStepLength = map->resolution();

   // allocate memory for the terrain, and check for errors
   terrainHeights = (float *)malloc(terrainGridWidth * terrainGridLength * sizeof(float));
   if (terrainHeights == NULL)
      return(TERRAIN_ERROR_MEMORY_PROBLEM);
   memset(terrainHeights,0, terrainGridWidth * terrainGridLength * sizeof(float));

   // allocate memory for the normals, and check for errors
   terrainNormals = (float *)malloc(terrainGridWidth * terrainGridLength * sizeof(float) * 3);
   if (terrainNormals == NULL)
      return(TERRAIN_ERROR_MEMORY_PROBLEM);
   memset(terrainNormals,0, terrainGridWidth * terrainGridLength * sizeof(float) * 3);

   // if color mode then allocate memory for colors, and check for errors 
   if (colorMode != HMAP_COLOR_PLAIN) {
      terrainColors = (float *)malloc(terrainGridWidth * terrainGridLength * sizeof(float)*3);
      if (terrainColors == NULL)
         return(TERRAIN_ERROR_MEMORY_PROBLEM);
      memset(terrainColors,0, terrainGridWidth * terrainGridLength * sizeof(float) * 3);
   }
   else
      terrainColors = NULL;

   // fill arrays 
   for (int y = 0; y < terrainGridLength; y++) {
      for (int x = 0; x < terrainGridWidth; x++) {
         double height = 0.0;
         if (drawSmoothedCells)
            height = map->getSmoothedCellsMM()[x][y].getHeight() / 1000.0;
         else
            height = map->getCellsMM()[x][y].getHeight() / 1000.0;

         //pointHeight = info->imageData[mode*(i*terrainGridWidth + j)+(mode-1)] / 255.0;
         terrainHeights[y*terrainGridWidth + x] = height;
         // if color mode then fill the colors array as well 
         if (colorMode == HMAP_COLOR_CLASS) {
            HeightCell::ELEVATION_CLASSES c = map->getCellsMM()[x][y].getClass();
            double r,g,b;
            if (map->getCellsMM()[x][y].isPerimeter()) {
               r=1.0;
               g=1.0;
               b=1.0;
            }
            else switch (c) {
              case HeightCell::ELC_NOT_INITIALIZED:
                  r = 0.5;
                  g = 0.5;
                  b = 0.5;
                  break;
               case HeightCell::ELC_FLAT_GROUND:
                  r = 0.0;
                  g = 0.8;
                  b = 0.0;
                  break;
               case HeightCell::ELC_RAMPED_GROUND:
                  r = 0.9;
                  g = 0.8;
                  b = 0.0;
                  break;
               case HeightCell::ELC_DRIVABLE_OBSTACLE:
                  r = 0.6;
                  g = 0.4;
                  b = 0.1;
                  break;
               case HeightCell::ELC_WALL:
                  r = 0.8;
                  g = 0.0;
                  b = 0.0;
                  break;
               case HeightCell::ELC_STAIRS:
                  r = 0.0;
                  g = 0.8;
                  b = 0.8;
                  break;
               default:
               case HeightCell::ELC_UNCLASSIFIED:
                  r = 0.0;
                  g = 0.0;
                  b = 0.9;
                  break;
            }
            terrainColors[3*(y*terrainGridWidth + x)]   = r;
            terrainColors[3*(y*terrainGridWidth + x)+1] = g;
            terrainColors[3*(y*terrainGridWidth + x)+2] = b;
         }
         else if (colorMode == HMAP_COLOR_REGION_SET) {
            double r,g,b;
            map->getCellsMM()[x][y].getRGB(r,g,b);
            terrainColors[3*(y*terrainGridWidth + x)]   = r;
            terrainColors[3*(y*terrainGridWidth + x)+1] = g;
            terrainColors[3*(y*terrainGridWidth + x)+2] = b;

            bool hasSchrub = (map->getCellsMM()[x][y].getVegetation() > 0);
            
            if (map->getCellsMM()[x][y].isPerimeter()) {
              terrainColors[3*(y*terrainGridWidth + x)]   = 1.0;
              terrainColors[3*(y*terrainGridWidth + x)+1] = 1.0;
              terrainColors[3*(y*terrainGridWidth + x)+2] = 1.0;
            }
			else if (hasSchrub && Params::g_use_shrubs) {
              terrainColors[3*(y*terrainGridWidth + x)]   = 0.0;
              terrainColors[3*(y*terrainGridWidth + x)+1] = 1.0;
              terrainColors[3*(y*terrainGridWidth + x)+2] = 0.0;
            }
            else if (map->getCellsMM()[x][y].getClass() != HeightCell::ELC_FLAT_GROUND) {
               terrainColors[3*(y*terrainGridWidth + x)]   = 1.0;
               terrainColors[3*(y*terrainGridWidth + x)+1] = 0.;
               terrainColors[3*(y*terrainGridWidth + x)+2] = 0.;
            }  
         }
         else if (colorMode == HMAP_COLOR_VISIBILITY) {
            bool isVisible = map->getCellsMM()[x][y].isVisible();
            bool isPerimeter  = map->getCellsMM()[x][y].isPerimeter();
            bool isCleared = map->getCellsMM()[x][y].isCleared();
            bool hasSchrub = (map->getCellsMM()[x][y].getVegetation() > 0);
           if (isVisible) {
               terrainColors[3*(y*terrainGridWidth + x)]   = 1.0;
               terrainColors[3*(y*terrainGridWidth + x)+1] = 0.0;
               terrainColors[3*(y*terrainGridWidth + x)+2] = 0.0;
            }
           else if (isCleared) {
               terrainColors[3*(y*terrainGridWidth + x)]   = 0.0;
               terrainColors[3*(y*terrainGridWidth + x)+1] = 0.0;
               terrainColors[3*(y*terrainGridWidth + x)+2] = 1.0;
            }
           else if (isPerimeter) {
               terrainColors[3*(y*terrainGridWidth + x)]   = 1.0;
               terrainColors[3*(y*terrainGridWidth + x)+1] = 1.0;
               terrainColors[3*(y*terrainGridWidth + x)+2] = 1.0;
            }
           else if (hasSchrub && Params::g_use_shrubs) {
              terrainColors[3*(y*terrainGridWidth + x)]   = 0.0;
              terrainColors[3*(y*terrainGridWidth + x)+1] = 1.0;
              terrainColors[3*(y*terrainGridWidth + x)+2] = 0.0;
            }
            else if (map->getCellsMM()[x][y].getClass() == HeightCell::ELC_FLAT_GROUND) {
               terrainColors[3*(y*terrainGridWidth + x)]   = 0.5;
               terrainColors[3*(y*terrainGridWidth + x)+1] = 0.5;
               terrainColors[3*(y*terrainGridWidth + x)+2] = 0.0;
            }
            else {
               terrainColors[3*(y*terrainGridWidth + x)]   = 0.5;
               terrainColors[3*(y*terrainGridWidth + x)+1] = 0.5;
               terrainColors[3*(y*terrainGridWidth + x)+2] = 0.5;
            }
         }
      }
   }

   // compute normals
   terrainComputeNormals();

   return(TERRAIN_OK); 
}


int terrainScale(float min,float max) {

   float amp,aux,min1,max1,height;
   int total,i;

   if (terrainHeights == NULL)
      return(TERRAIN_ERROR_NOT_INITIALISED);

   if (min > max) {
      aux = min;
      min = max;
      max = aux;
   }

   amp = max - min;
   total = terrainGridWidth * terrainGridLength;

   min1 = terrainHeights[0];
   max1 = terrainHeights[0];
   for(i=1;i < total ; i++) {
      if (terrainHeights[i] > max1)
         max1 = terrainHeights[i];
      if (terrainHeights[i] < min1)
         min1 = terrainHeights[i];
   }
   for(i=0;i < total; i++) {
      height = (terrainHeights[i] - min1) / (max1-min1);
      terrainHeights[i] = height * amp - min;
   }
   if (terrainNormals != NULL)
      terrainComputeNormals();
   return(TERRAIN_OK);
}


int terrainDim(float stepWidth, float stepLength) {

   if (stepWidth > 0 && stepLength > 0) {
      terrainStepWidth = stepWidth;
      terrainStepLength = stepLength;

      if (terrainNormals != NULL)
         terrainComputeNormals();

      return(TERRAIN_OK);
   }
   else
      return(TERRAIN_ERROR_INVALID_PARAM);
}


#if 0
static float terrainComputeLightFactor(int i,int j,int offseti, int offsetj) {

   float factor,v[3];

   if (terrainLightPos[3] == 0.0) /* directional light */
      factor = terrainNormals[3*(i * terrainGridWidth + j)] * terrainLightPos[0] +
         terrainNormals[3*(i * terrainGridWidth + j) +1] * terrainLightPos[1] +
         terrainNormals[3*(i * terrainGridWidth + j) +2] * terrainLightPos[2];
   else { /* positional light */
      v[0] = terrainLightPos[0] - (j + offsetj)*terrainStepWidth;
      v[1] = terrainLightPos[1] - terrainHeights[i*terrainGridWidth + j];
      v[2] = terrainLightPos[2] - (offseti -i) * terrainStepLength;
      terrainNormalize(v);
      factor = terrainNormals[3*(i * terrainGridWidth + j)] * v[0] +
         terrainNormals[3*(i * terrainGridWidth + j) +1] * v[1] +
         terrainNormals[3*(i * terrainGridWidth + j) +2] * v[2];
   }	
   if (factor < 0)
      factor = 0;
   return(factor);
}
#endif

void terrainRemoveColoredCell(long cellNum, bool all)
{
   if (all)
      coloredCells.clear();
   else
      for (unsigned int i=0; i<coloredCells.size(); i++) {
         if (coloredCells[i].num == cellNum) {
            coloredCells.erase(coloredCells.begin() + i);
            i=0;
         }
      }
}

void terrainAddColoredCell(long cellNum, double r, double g, double b)
{
   if (terrainGridWidth == 0 || terrainGridLength == 0)
      return;

   ColoredCell cell;
   cell.num = cellNum;
   cell.cx = (int) (cellNum / terrainGridWidth);
   cell.cy = (int) (cellNum % terrainGridWidth);
   cell.r = r;
   cell.g = g;
   cell.b = b;

   coloredCells.push_back(cell);
}

void terrainAddColoredCell(int cx, int cy, double r, double g, double b)
{
   if (terrainGridWidth == 0 || terrainGridLength == 0)
      return;

   ColoredCell cell;
   cell.num = cy * terrainGridWidth + cx;
   cell.cx = cx;
   cell.cy = cy;
   cell.r = r;
   cell.g = g;
   cell.b = b;

   coloredCells.push_back(cell);
}

void terrainDrawColoredCells(float xOffset, float yOffset)
{
   if (terrainGridWidth == 0 || terrainGridLength == 0)
      return;

   float startW = xOffset;
   float startL = yOffset;

   static const double z = 0.1;

   glBegin(GL_QUADS);

   double h;
   for (unsigned int ii=0; ii<coloredCells.size(); ii++) 
   {
      ColoredCell cell = coloredCells[ii];
      int i = cell.cx;
      int j = cell.cy;
 
      int pi = i;
      int pj = j;

      // Set color
      glColor3f(cell.r, cell.g, cell.b);

      // Set vertex
      h = z;
      if (pj>=0 && pj<terrainGridWidth && pi>0 && pi<terrainGridLength)
         h += terrainHeights[(pi)*terrainGridWidth + (pj)];
      glVertex3f(
            startW + ((pj)*terrainStepWidth),
            startL - ((pi)*terrainStepLength),				
            h);

      pi = i;
      pj = j + 1;

      // Set color
      glColor3f(cell.r, cell.g, cell.b);

      // Set vertex
      h = z;
      if (pj>=0 && pj<terrainGridWidth && pi>0 && pi<terrainGridLength)
         h += terrainHeights[(pi)*terrainGridWidth + (pj)];
      glVertex3f(
            startW + ((pj)*terrainStepWidth),
            startL - ((pi)*terrainStepLength),				
            h);

      pi = i + 1;
      pj = j + 1;

      // Set color
      glColor3f(cell.r, cell.g, cell.b);

      // Set vertex
      h = z;
      if (pj>=0 && pj<terrainGridWidth && pi>0 && pi<terrainGridLength)
         h += terrainHeights[(pi)*terrainGridWidth + (pj)];
      glVertex3f(
            startW + ((pj)*terrainStepWidth),
            startL - ((pi)*terrainStepLength),				
            h);

      pi = i + 1;
      pj = j;

      // Set color
      glColor3f(cell.r, cell.g, cell.b);

      // Set vertex
      h = z;
      if (pj>=0 && pj<terrainGridWidth && pi>0 && pi<terrainGridLength)
         h += terrainHeights[(pi)*terrainGridWidth + (pj)];
      glVertex3f(
            startW + ((pj)*terrainStepWidth),
            startL - ((pi)*terrainStepLength),				
            h);
   }
   glEnd();
}

void terrainDraw(bool redraw, float xOffset, float yOffset) 
{
   float startW = xOffset;
   float startL = yOffset;

   if (redraw) 
   {
      if (terrainDL != 0)
         glDeleteLists(terrainDL,1);
      terrainDL = glGenLists(1);
      glNewList(terrainDL,GL_COMPILE);

      glEnable(GL_COLOR_MATERIAL);
      //glEnable( GL_CULL_FACE ); // Enable Cull Facings
      glShadeModel(GL_SMOOTH);
      //glCullFace( GL_FRONT ); // Telling OpenGL We Want To Draw The Fronts Of The Objects
      //glPolygonMode( GL_FRONT_AND_BACK, GL_FILL ); // Drawing And Filling Only The Front Of The Polygons


      // Somewhere in the initialization part of your program…
      glEnable(GL_LIGHTING);
      glDisable(GL_LIGHT0);
      glDisable(GL_LIGHT1);
      glDisable(GL_LIGHT2);
      glDisable(GL_LIGHT3);
       
      glEnable(GL_LIGHT0);

      // Create light components
      GLfloat ambientLight[] = { 0.2f, 0.2f, 0.2f, 1.0f };
      GLfloat diffuseLight[] = { 0.8f, 0.8f, 0.8, 1.0f };
      GLfloat specularLight[] = { 0.5f, 0.5f, 0.5f, 1.0f };
      //GLfloat position[] = { -1.5f, 1.0f, 1000400.0f, 1.0f };
      GLfloat position[] = { 0.0f, 0.0f, 850.0f, 1.0f };
       
      //GLfloat ambientLight[] = { 0.1f, 0.1f, 0.1f, 1.0f };
      //GLfloat diffuseLight[] = { 0.2f, 0.2f, 0.2, 1.0f };

      // Assign created components to GL_LIGHT0
      glLightfv(GL_LIGHT0, GL_AMBIENT, ambientLight);
      glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuseLight);
      glLightfv(GL_LIGHT0, GL_SPECULAR, specularLight);
      glLightfv(GL_LIGHT0, GL_POSITION, position);

      float mcolor[] = { 0.0f, 0.0f, 0.0f, 0.0f };
      glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, mcolor);

      for (int i = 0 ; i < terrainGridLength-1; i++) 
      {
         glBegin(GL_QUAD_STRIP);
         for(int j=0; j < terrainGridWidth; j++) 
         {

            int pi = i+1;
            int pj = j;

            // Set color
            if (terrainColors != NULL) 
               glColor3f(terrainColors[3*((pi)*terrainGridWidth + pj)],
                     terrainColors[3*((pi)*terrainGridWidth + pj)+1],
                     terrainColors[3*((pi)*terrainGridWidth + pj)+2]);
            else
               glColor3f(1.0,1.0,1.0);
            // Set normal
            if (terrainNormals != NULL)
               glNormal3f(terrainNormals[3*((pi)*terrainGridWidth + pj)],
                     terrainNormals[3*((pi)*terrainGridWidth + pj)+2],
                     terrainNormals[3*((pi)*terrainGridWidth + pj)+1]);
            // Set vertex
            glVertex3f(
                  startW + ((pj)*terrainStepWidth),
                  startL - ((pi)*terrainStepLength),				
                  terrainHeights[(pi)*terrainGridWidth + (pj)]);

            pi = i;
            pj = j;

            // Set color
            if (terrainColors != NULL) 
               glColor3f(terrainColors[3*((pi)*terrainGridWidth + pj)],
                     terrainColors[3*((pi)*terrainGridWidth + pj)+1],
                     terrainColors[3*((pi)*terrainGridWidth + pj)+2]);
            else
               glColor3f(1.0,1.0,1.0);
            // Set normal
            if (terrainNormals != NULL)
               glNormal3f(terrainNormals[3*((pi)*terrainGridWidth + pj)],
                     terrainNormals[3*((pi)*terrainGridWidth + pj)+2],
                     terrainNormals[3*((pi)*terrainGridWidth + pj)+1]);
            // Set vertex
            glVertex3f(
                  startW + ((pj)*terrainStepWidth),
                  startL - ((pi)*terrainStepLength),				
                  terrainHeights[(pi)*terrainGridWidth + (pj)]);
         }
         glEnd();
      }
      glEndList();
   }
   glCallList(terrainDL);
}

// Give names (ids) to the cells
void terrainDrawWithNames(float xOffset, float yOffset) 
{
   float startW = xOffset;
   float startL = yOffset;

   int count = 0;
   for (int i=0 ; i<terrainGridLength-1; i++) 
   {
      for(int j=0; j<terrainGridWidth; j++) 
      {
         glPushName(count++);
         glBegin(GL_QUADS);
         glVertex3f(
               startW + ((float)(j)*terrainStepWidth),
               startL - ((float)(i)*terrainStepLength),				
               terrainHeights[(i)*terrainGridWidth + (j)]);

         glVertex3f(
               startW + ((float)(j+1)*terrainStepWidth),
               startL - ((float)(i)*terrainStepLength),				
               terrainHeights[(i)*terrainGridWidth + (j+1)]);

         glVertex3f(
               startW + ((float)(j+1)*terrainStepWidth),
               startL - ((float)(i+1)*terrainStepLength),				
               terrainHeights[(i+1)*terrainGridWidth + (j+1)]);

         glVertex3f(
               startW + ((float)(j)*terrainStepWidth),
               startL - ((float)(i+1)*terrainStepLength),
               terrainHeights[(i+1)*terrainGridWidth + (j)]);
         glEnd();
         glPopName();
      }
   }
}


float terrainGetHeight(int x, int z) {

   int xt,zt;

   if (terrainHeights == NULL) 
      return(0.0);

   xt = x + terrainGridWidth /2;
   zt = terrainGridLength - (z + terrainGridLength /2);

   if ((xt > terrainGridWidth) || (zt > terrainGridLength) || (xt < 0) || (zt < 0))
      return(0.0);

   return(terrainHeights[zt * terrainGridWidth + xt]);
}


void terrainDestroy() {

   if (terrainHeights != NULL)
      free(terrainHeights);

   if (terrainColors != NULL)
      free(terrainColors);

   if (terrainNormals != NULL)
      free(terrainNormals);
}
