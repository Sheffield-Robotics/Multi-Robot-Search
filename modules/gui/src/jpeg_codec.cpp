//
// File: jpeg_codec.cc
// Author: Kolja Glogowski <kolja@pixie.de>
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307, USA

#include "gui/jpeg_codec.h"
#include "gui/memdata_mgr.h"

#include <cassert>


JpegEncoder::JpegEncoder(int width, int height, int quality, int in_components,
	J_COLOR_SPACE in_color_space)
	: destmgr_(0)
{
	cinfo_.err = jpeg_std_error(&jerr_);
	jpeg_create_compress(&cinfo_);
	
	destmgr_ = new mem_dst_mgr(&cinfo_);
	
	cinfo_.image_width = width;
	cinfo_.image_height = height;
	cinfo_.input_components = in_components;
	cinfo_.in_color_space = in_color_space;
	
	jpeg_set_defaults(&cinfo_);
	jpeg_set_quality(&cinfo_, quality, TRUE);
}


JpegEncoder::~JpegEncoder()
{
	jpeg_destroy_compress(&cinfo_);
	delete destmgr_;
}


JSAMPLE* JpegEncoder::encode(JSAMPLE *raw_image, size_t &jpeg_size)
{
	JSAMPROW row_ptr;
	int row_length = cinfo_.image_width * 3;
	
	jpeg_start_compress(&cinfo_, TRUE);
	
	while (cinfo_.next_scanline < cinfo_.image_height)
	{
		row_ptr = &raw_image[cinfo_.next_scanline * row_length];
		jpeg_write_scanlines(&cinfo_, &row_ptr, 1);
	}

	jpeg_finish_compress(&cinfo_);
	
	octet_vector &v = destmgr_->imgvec_;
	
	jpeg_size = v.size();
	JSAMPLE *jpeg_buf = new JSAMPLE[jpeg_size];
	
	std::copy(v.begin(), v.end(), jpeg_buf);
	
	return jpeg_buf;
}


JpegDecoder::JpegDecoder(J_COLOR_SPACE out_color_space)
	: srcmgr_(0), out_color_space_(out_color_space)
{
	cinfo_.err = jpeg_std_error(&jerr_);
	jpeg_create_decompress(&cinfo_);
	
	srcmgr_ = new mem_src_mgr(&cinfo_);
}

JpegDecoder::~JpegDecoder()
{
	jpeg_destroy_decompress(&cinfo_);
	delete srcmgr_;
}

JSAMPLE* JpegDecoder::decode(JOCTET *jpeg_image, int jpeg_size,
	int &width, int &height, int &color_components)
{
	assert(jpeg_image != 0);
	assert(jpeg_size != 0);
	
	srcmgr_->set_jpeg_image(jpeg_image, jpeg_size);
	
	jpeg_read_header(&cinfo_, TRUE);
	cinfo_.out_color_space = out_color_space_;
	
	jpeg_start_decompress(&cinfo_);

	width = cinfo_.output_width;
	height = cinfo_.output_height;
	color_components = cinfo_.out_color_components;
	
	int row_length = width * color_components;
	JSAMPLE *raw_image = new JSAMPLE[row_length * height];
	
	JSAMPROW p = raw_image;
	while (cinfo_.output_scanline < cinfo_.output_height)
	{
		jpeg_read_scanlines(&cinfo_, &p, 1);
		p += row_length;
	}
	
	jpeg_finish_decompress(&cinfo_);
	
	return raw_image;
}

/*
 * JpegYuv420pEncoder
 */
JpegYuv420pEncoder::JpegYuv420pEncoder(int width, int height, int quality, int in_components,
	J_COLOR_SPACE in_color_space)
	: destmgr_(0)
{
	cinfo_.err = jpeg_std_error(&jerr_);
	jpeg_create_compress(&cinfo_);
	
	destmgr_ = new mem_dst_mgr(&cinfo_);
	
	cinfo_.image_width = width;
	cinfo_.image_height = height;
	cinfo_.input_components = in_components;
	cinfo_.in_color_space = in_color_space;
	
	jpeg_set_defaults(&cinfo_);
	jpeg_set_quality(&cinfo_, quality, TRUE);
	
	cinfo_.raw_data_in = TRUE;
	cinfo_.in_color_space = JCS_YCbCr;
	cinfo_.dct_method = JDCT_IFAST;
	
   // see format Description at fourcc (URL in header file)
	cinfo_.comp_info[0].h_samp_factor = 2;
	cinfo_.comp_info[0].v_samp_factor = 2;
	cinfo_.comp_info[1].h_samp_factor = 1;
	cinfo_.comp_info[1].v_samp_factor = 1;
	cinfo_.comp_info[2].h_samp_factor = 1;
	cinfo_.comp_info[2].v_samp_factor = 1;

	m_data[0] = m_y;
	m_data[1] = m_cb;
	m_data[2] = m_cr;
}


JpegYuv420pEncoder::~JpegYuv420pEncoder()
{
	jpeg_destroy_compress(&cinfo_);
	delete destmgr_;
}


JSAMPLE* JpegYuv420pEncoder::encode(JSAMPLE *yuv420praw_image, size_t &jpeg_size)
{
	jpeg_start_compress(&cinfo_, TRUE);	
	
	int width = cinfo_.image_width;
	int height = cinfo_.image_height;
	int tmpOffset;
	int pixelCount = width*height;
	int pixelCountandQuarter = width*height + width*height/4;
	
	for (int j=0;j<height;j+=16) {
		for (int i=0;i<16;i++) {
			tmpOffset = width*(i+j);
			m_y[i] = yuv420praw_image + tmpOffset;
			tmpOffset /= 4;
			if ((i & 1) == 0) {
				m_cb[i/2] = yuv420praw_image + pixelCount + tmpOffset;
				m_cr[i/2] = yuv420praw_image + pixelCountandQuarter + tmpOffset;
			}
		}
		jpeg_write_raw_data (&cinfo_, m_data, 16);
  	}
	
	jpeg_finish_compress(&cinfo_);
	
	octet_vector &v = destmgr_->imgvec_;
	
	jpeg_size = v.size();
	JSAMPLE *jpeg_buf = new JSAMPLE[jpeg_size];
	
	std::copy(v.begin(), v.end(), jpeg_buf);
	
	return jpeg_buf;
}

/*
 * JpegYuv422Encoder
 */
JpegYuv422Encoder::JpegYuv422Encoder(int width, int height, int quality, int in_components,
	J_COLOR_SPACE in_color_space)
	: destmgr_(0)
{
	cinfo_.err = jpeg_std_error(&jerr_);
	jpeg_create_compress(&cinfo_);
	
	destmgr_ = new mem_dst_mgr(&cinfo_);
	
	cinfo_.image_width = width;
	cinfo_.image_height = height;
	cinfo_.input_components = in_components;
	cinfo_.in_color_space = in_color_space;
	
	jpeg_set_defaults(&cinfo_);
	jpeg_set_quality(&cinfo_, quality, TRUE);
	
	cinfo_.raw_data_in = TRUE;
	cinfo_.in_color_space = JCS_YCbCr;
	cinfo_.dct_method = JDCT_IFAST;
	
   // see format Description at fourcc (URL in header file)
	cinfo_.comp_info[0].h_samp_factor = 2;
	cinfo_.comp_info[0].v_samp_factor = 2;
	cinfo_.comp_info[1].h_samp_factor = 1;
	cinfo_.comp_info[1].v_samp_factor = 2;
	cinfo_.comp_info[2].h_samp_factor = 1;
	cinfo_.comp_info[2].v_samp_factor = 2;

	m_data[0] = m_y;
	m_data[1] = m_cb;
	m_data[2] = m_cr;

   m_yBytes = new unsigned char[width*height];
   m_cbBytes = new unsigned char[width*height/2];
   m_crBytes = new unsigned char[width*height/2];
}


JpegYuv422Encoder::~JpegYuv422Encoder()
{
	jpeg_destroy_compress(&cinfo_);
	delete destmgr_;

   delete[] m_yBytes;
   delete[] m_cbBytes;
   delete[] m_crBytes;
}



/*void planar422p_to_yuv(  unsigned char *in, unsigned char *out, int width, int height)
{
    unsigned char *pY;
    unsigned char *pYEND;
    unsigned char *pCb;
    unsigned char *pCr;
    unsigned long  size = height * width;
    unsigned long i;
    pY = (unsigned char *) in;
    pCr =(unsigned char *) in + (height * width);
    pCb = (unsigned char *) in + (height * width) +  (height * width /2);
    pYEND = pCr;
    for( i=0; pY<pYEND; i+=6) {
        out[i] = *pY++;
        out[i+1] = *pCr;
        out[i+2] = *pCb;
        out[i+3] = *pY++;        
        out[i+4] = *pCr++;
        out[i+5] = *pCb++;        

    }

    return;
}
*/

JSAMPLE* JpegYuv422Encoder::encode(JSAMPLE *yuv422raw_image, size_t &jpeg_size)
{
	jpeg_start_compress(&cinfo_, TRUE);	

   // fill temporary Buffers
   unsigned int i = 0, j = 0;
   unsigned int yCount = 0, cbCount = 0, crCount = 0;
   register int u, y1, y2, v;
   static unsigned int dest_size = (unsigned int)(cinfo_.image_height * cinfo_.image_width / 2);

   while (j < dest_size)
   {
      u = yuv422raw_image[i++];
      y1 = yuv422raw_image[i++];
      v = yuv422raw_image[i++];
      y2 = yuv422raw_image[i++];

      m_yBytes[yCount++] = y1;
      m_yBytes[yCount++] = y2;
      m_cbBytes[cbCount++] = u;
      m_crBytes[crCount++] = v;

      j++;
   } 

	for (unsigned int j=0; j < cinfo_.image_height; j+=16) {
		for (int i=0; i < 16; i++) {
			int tmpOffset = cinfo_.image_width*(i+j);
			m_y[i] = m_yBytes + tmpOffset;
			tmpOffset /= 2;
         m_cb[i] = m_cbBytes + tmpOffset;
         m_cr[i] = m_crBytes + tmpOffset;
		}
		jpeg_write_raw_data (&cinfo_, m_data, 16);
  	}
	
	jpeg_finish_compress(&cinfo_);
	
	octet_vector &ov = destmgr_->imgvec_;
	
	jpeg_size = ov.size();
	JSAMPLE *jpeg_buf = new JSAMPLE[jpeg_size];
	
	std::copy(ov.begin(), ov.end(), jpeg_buf);
	
	return jpeg_buf;
}
