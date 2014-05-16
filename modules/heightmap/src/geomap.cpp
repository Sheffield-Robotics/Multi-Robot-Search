#include <stdlib.h>
#include <math.h>
#include <sstream>
//#include <QGLViewer/qglviewer.h>
#include "utilities/misc.h"
#include "heightmap/geomap.h"
#include "utilities/gpsconvert.h"

#ifdef __APPLE__
#include <OpenGL/glu.h>
#else
#include <glu.h>
#endif

using namespace std;
//using namespace qglviewer;

GeoMap::GeoMap() 
{
   _geoMapDL = 0;
   _rasterWidth = 0;
   _rasterHeight = 0;
   _rasterImage = NULL;
   _myTif = NULL;
   _myGeoInfo = NULL;
   _scaleX = 1.0;
   _scaleY = 1.0;
   _worldWidth=0.0;
   _worldHeight=0.0;
   _resolution = 0.0;
   _heightResolution = 0.0;
}

GeoMap::~GeoMap()
{
   printf("DELETE GEOMAP skipped!\n");
   return;
   if (_myGeoInfo)
      GTIFFree(_myGeoInfo);
   if (_myTif)
      XTIFFClose(_myTif);
   if (_geoMapDL)
      glDeleteLists(_geoMapDL,1);
   if (_rasterImage)
      free(_rasterImage);
}


bool GeoMap::loadGeoTiff(std::string mapName)
{
   // Load Tiff image
   TIFFRGBAImage* img = new TIFFRGBAImage;
   char emsg[1024];
   size_t npixels;

   // Open tiff file
   _myTif = XTIFFOpen(mapName.c_str(), "r");
   if (_myTif == NULL) {
      M_ERR("Problem loading geo map %s\n", mapName.c_str());
      return false;
   }

   // Read Tags
   _myGeoInfo = GTIFNew(_myTif);
   if (_myGeoInfo == NULL) {
      M_ERR("Problem loading geo info from %s\n", mapName.c_str());
      return false;
   }

   int width, height;
   width=height=0;

   // Get the attributes of this file
   TIFFGetField(_myTif, TIFFTAG_IMAGEWIDTH,  &width);                  // get the width of the lImage
   TIFFGetField(_myTif, TIFFTAG_IMAGELENGTH, &height);                 // get the height of the lImage
   //TIFFGetField(_myTif, TIFFTAG_ORIENTATION, &orientation);            // get the origin of the lImage.
   //TIFFGetField(_myTif, TIFFTAG_XRESOLUTION, &xres);                   // get the x resolution.
   //TIFFGetField(_myTif, TIFFTAG_YRESOLUTION, &yres);                   // get the y resolution.

   // Geospecific Tags
   short tiepointsize, pixscalesize;
   double* tiepoints;//[6];
   double* pixscale;//[3];
   TIFFGetField(_myTif, TIFFTAG_GEOTIEPOINTS,  &tiepointsize, &tiepoints);
   if (TIFFGetField(_myTif, TIFFTAG_GEOPIXELSCALE, &pixscalesize, &pixscale) != 1) {
      M_ERR("--> Either this is not a proper geoTIFF or the important field TIFFTAG_GEOPIXELSCALE is not set!\n");
      M_ERR("--> Please use the tools 'listgeo' and 'geotifcp' to encode the resolution information into the image file\n");
      XTIFFClose(_myTif);
      return false;
   }

   M_INFO1("Geo Tie Points: %f  %f  %f  %f %f  %f \n",
         tiepoints[0],tiepoints[1],tiepoints[2],tiepoints[3],tiepoints[4],tiepoints[5]);

   M_INFO1("Geo Pixel Scale: %f  %f  %f\n",pixscale[0],pixscale[1],pixscale[2]);

   double xres =  pixscale[0];
   double yres =  pixscale[1];
   double zres =  pixscale[2];

   if (xres != yres) {
      M_ERR("GeoTiff images should have same resolution in x and y!\n");
   }
   _resolution = xres;

   if (zres <= 0.0)
      zres = 1.0;
   _heightResolution = zres;

   double ul_x = 0.0;
   double ul_y = 0.0;
   double ll_x = 0.0;
   double ll_y = height;
   double ur_x = width;
   double ur_y = 0.0;
   double lr_x = width;
   double lr_y = height;
   double cx = width / 2.0;
   double cy = height / 2.0;

   image2world(&ul_x, &ul_y);
   image2world(&ll_x, &ll_y);
   image2world(&ur_x, &ur_y);
   image2world(&lr_x, &lr_y);
   image2world(&cx, &cy);

   char zone[] = GLOBAL_UTM_ZONE;
   double ul_latitude, ul_longitude, lr_latitude, lr_longitude, ur_latitude, ur_longitude, ll_latitude, ll_longitude;
   gps_UTMtoLL(ul_y, ul_x, zone, &ul_latitude,  &ul_longitude);
   gps_UTMtoLL(lr_y, lr_x, zone, &lr_latitude,  &lr_longitude);
   gps_UTMtoLL(ur_y, ur_x, zone, &ur_latitude,  &ur_longitude);
   gps_UTMtoLL(ll_y, ll_x, zone, &ll_latitude,  &ll_longitude);


   _worldWidth = ur_x - ul_x;
   _worldHeight = ul_y - ll_y;

   double center_longitude;
   double center_latitude;
   
   gps_UTMtoLL(cy, cx, zone, &center_latitude,  &center_longitude);

   printf("Corner Coordinates (lat/long):\n");
   printf("Upper Left          %lf %lf\n",ul_latitude, ul_longitude);
   printf("Lower Left          %lf %lf\n",ll_latitude, ll_longitude);
   printf("Upper Right         %lf %lf\n",ur_latitude, ur_longitude);
   printf("Lower Right         %lf %lf\n",lr_latitude, lr_longitude);
   printf("Center              %lf m,      %lf m    latitude=%lf    longitude=%lf\n",cx, cy, center_latitude, center_longitude);
   printf("World Size:         %lf m,      %lf m\n", _worldWidth, _worldHeight);
   printf("Resolution x/y:         %lf meter / pixel \n", _resolution);
   printf("Resolution height:         %lf meter / pixel \n", _heightResolution);

   if (TIFFRGBAImageBegin(img, _myTif, 0, emsg)) {
      npixels = img->width * img->height;
      _rasterImage = (uint32 *) _TIFFmalloc(npixels * sizeof(uint32));
      if (_rasterImage != NULL) {
         if (TIFFRGBAImageGet(img, _rasterImage, img->width, img->height) == 0) {
            M_ERR("Error initializing tiff\n");
            XTIFFClose(_myTif);
            return false;
         }
      }
      TIFFRGBAImageEnd(img);
   } else {
      M_ERR("Error initializing tiff\n");
      XTIFFClose(_myTif);
      return false;
   }
   //XTIFFClose(_myTif);

   _rasterWidth = img->width;
   _rasterHeight = img->height;

   _scaleX = _worldWidth / (2.0);
   _scaleY = _worldHeight / (2.0);

   M_INFO2("Loaded image of size %d X %d\n",_rasterWidth, _rasterHeight);

   delete img;
   return true;
}

void GeoMap::init()
{
   if (_rasterImage == NULL) {
      M_WARN("Raster Image not loaded!\n");
      return;
   }
   // Begin draw list
   _geoMapDL = glGenLists(1);
   glNewList(_geoMapDL, GL_COMPILE);

   // Set stuff for texture map
   glEnable( GL_TEXTURE_2D );
   glDisable(GL_LIGHTING);
   
   //glClearColor( 0.0f, 0.0f, 0.0f, 0.0f );
   //glClear( GL_COLOR_BUFFER_BIT );
   // OLD OLD
   //glEnable(GL_DEPTH_TEST);
   //glDisable(GL_LIGHTING);
   //glCullFace(GL_BACK);
   //glEnable(GL_CULL_FACE);
   //glShadeModel(GL_SMOOTH);              // Enable Smooth Shading
   //glClearDepth(1.0f);                   // Depth Buffer Setup
   //glDepthFunc(GL_LEQUAL);               // The Type Of Depth Testing To Do
   //glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST); // Really Nice Perspective Calculations

   glColor3f(1.0, 1.0, 1.0);

   GLuint texture;

   // allocate a texture name
   glGenTextures( 1, &texture );

   // select our current texture
   glBindTexture( GL_TEXTURE_2D, texture );

   // select modulate to mix texture with color for shading
   glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE );

   // when texture area is small, bilinear filter the closest MIP map
   glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
         GL_LINEAR_MIPMAP_NEAREST );

   // when texture area is large, bilinear filter the first MIP map
   glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );

   // if wrap is true, the texture wraps over at the edges (repeat)
   //       ... false, the texture ends at the edges (clamp)
   glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S,
         true ? GL_REPEAT : GL_CLAMP );
   glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T,
         true ? GL_REPEAT : GL_CLAMP );

   // build our texture MIP maps
   gluBuild2DMipmaps( GL_TEXTURE_2D, 3, _rasterWidth, _rasterHeight, GL_RGBA, GL_UNSIGNED_BYTE, _rasterImage);

   glBindTexture( GL_TEXTURE_2D, texture );

   glBegin( GL_QUADS );
    glTexCoord2d(0.0,0.0); glVertex2d(-1.0,-1.0);
    glTexCoord2d(1.0,0.0); glVertex2d(+1.0,-1.0);
    glTexCoord2d(1.0,1.0); glVertex2d(+1.0,+1.0);
    glTexCoord2d(0.0,1.0); glVertex2d(-1.0,+1.0);
   glEnd();

   glDisable( GL_TEXTURE_2D );
   glEnable(GL_LIGHTING);

   glEndList();
   glDeleteTextures( 1, &texture );
   M_INFO2("Loaded texture\n");
}

void GeoMap::world2image(double* x, double* y)
{
   int ret = GTIFPCSToImage(_myGeoInfo, x, y);
   if (!ret)
      M_ERR("Error converting world2image\n");
   *x *= _resolution;
   *y *= _resolution;
}

void GeoMap::image2world(double* x, double* y)
{
   *x /= _resolution;
   *y /= _resolution;

   int ret = GTIFImageToPCS(_myGeoInfo, x, y);
   if (!ret)
      M_ERR("Error converting image2world\n");
}

void GeoMap::adjustSize(double fac)
{
   _scaleX *= fac;
   _scaleY *= fac;
}

void GeoMap::draw(double offsX, double offsY)
{
   if (!_geoMapDL)
      return;

   glPushMatrix();
   glTranslatef(offsX, offsY, 0.0);
   glScalef(_scaleX, - _scaleY, 1.0); // mirror y to be correct with terrain map
   glCallList(_geoMapDL);
   glPopMatrix();
}
