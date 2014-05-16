#ifndef GEO_MAP_H
#define GEO_MAP_H

#include <string>
#include <tiffio.h>

#ifdef __APPLE__
#include "xtiffio.h"  /* for TIFF */
#include "geotiffio.h" /* for GeoTIFF */
#else
#include "geotiff/xtiffio.h"  /* for TIFF */
#include "geotiff/geotiffio.h" /* for GeoTIFF */
#endif
class GeoMap {


   public:
      GeoMap();
      ~GeoMap();

      // Load GeoTiff from file
      bool loadGeoTiff(std::string mapName);

      // initialize for drawing. Should be called in the viewer's init() function
      void init();

      // Draw the map. Should be called in the viewer's draw() function
      void draw(double offsX, double offsY);

      // Conversion between pixels and real world
      void world2image(double* x, double* y);
      void image2world(double* x, double* y);

      // Can be used to play around with the scale factor
      void adjustSize(double fac);

      // Return the image buffer
      void getData(uint32* &buffer) {buffer=_rasterImage;}

      // Return the size
      int width()  {return _rasterWidth;}
      int height()  {return _rasterHeight;}
      int widthW()  {return _worldWidth;}
      int heightW()  {return _worldHeight;}

      // Return resolution. Length of each pixel in meters.
      double resolution()  {return _resolution;}

      // Return resolution. Length of each pixel in meters.
      double heightResolution()  {return _heightResolution;}

   private:
      int _geoMapDL;
      int _rasterWidth, _rasterHeight;
      uint32 *_rasterImage;
      TIFF *_myTif;
      GTIF CPL_DLL* _myGeoInfo;
      double _scaleX, _scaleY;
      double _worldWidth, _worldHeight;
      double _resolution;
      double _heightResolution;
};
#endif
