#ifndef JPEG_IMAGE_H
#define JPEG_IMAGE_H

#include <string>

class JpegImage
{
   public:
      JpegImage();
      ~JpegImage();

      bool load(const std::string & filename);
      bool save(const std::string & filename);

      static JpegImage fromRGB(int width, int height, unsigned char* data, int quality = 75);

      // toRGB (i.e. rgb data*)
   
   protected:
      int width, height;
      unsigned char* data;    ///< raw jpeg data
      size_t data_size;
};

#endif

