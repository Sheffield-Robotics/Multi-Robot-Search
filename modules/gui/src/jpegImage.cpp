#include "gui/jpegImage.h"
#include "gui/jpeg_codec.h"

JpegImage::JpegImage() : width(0), height(0), data(NULL), data_size(0)
{
}

JpegImage::~JpegImage()
{

   delete [] data;
}

bool JpegImage::load(const std::string & filename)
{
   (void) filename;
   if(data_size == 0 || data == NULL)
      return false;

   return false;
}

bool JpegImage::save(const std::string & filename)
{
   if(data_size == 0 || data == NULL)
      return false;

   FILE* f = fopen(filename.c_str(), "w");
   if(f == NULL)
      return false;

   bool ret =(fwrite(data, 1, data_size, f) == data_size);
   fclose(f);
   return ret;
}

JpegImage JpegImage::fromRGB(int width, int height, unsigned char* data, int quality)
{
   JpegImage ji;
   ji.width = width;
   ji.height = height;
   
   // now encode data/data_size from RGB to jpeg
   JpegEncoder je(width, height, quality, 3, JCS_RGB);

   ji.data = je.encode(data, ji.data_size);
   return ji;
}

