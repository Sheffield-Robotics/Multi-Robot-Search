//
// File: gui/jpeg_codec.h
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


#ifndef _JPEG_CODEC_H_
#define _JPEG_CODEC_H_

#include <cstdio>
extern "C" {
#include <jpeglib.h>
}


/**
 * \defgroup libjpegcodec JPEG Memory Codec
 *
 * @{
 */


//! \cond 0  // doxygen should ignore this

// forward decl
struct mem_dst_mgr;
struct mem_src_mgr;

//! \endcond


/**
 * \brief JPEG encoder class for encoding JPEGs to a memory location.
 */
class JpegEncoder
{
public:
	/**
	 * \brief The contructor.
	 */
	JpegEncoder(int width, int height, int quality = 75, int in_components = 3,
		J_COLOR_SPACE in_color_space = JCS_YCbCr);
	
	/**
	 * \brief The destructor.
	 */
	virtual ~JpegEncoder();

public:
	/**
	 * \brief Encodes a raw image pointer.
	 */
	JOCTET* encode(JSAMPLE *raw_image, size_t &jpeg_size);

private:
	jpeg_compress_struct cinfo_;
	jpeg_error_mgr jerr_;
	mem_dst_mgr *destmgr_;
};


/**
 * \brief JPEG decoder class for decoding JPEGs from a memory location.
 */
class JpegDecoder
{
public:
	/**
	 * \brief The contructor.
	 */
	JpegDecoder(J_COLOR_SPACE out_color_space = JCS_RGB);

	/**
	 * \brief The destructor.
	 */
	virtual ~JpegDecoder();
	
public:
	/**
	 * \brief Decodes a Jpeg from a location in memory.
	 */
	JSAMPLE* decode(JOCTET *jpeg_image, int jpeg_size,
		int &width, int &height, int &color_components);

private:
	jpeg_decompress_struct cinfo_;
	jpeg_error_mgr jerr_;
	mem_src_mgr *srcmgr_;
	J_COLOR_SPACE out_color_space_;
};

/*
 * Formatbeschreibung unter
 * http://www.fourcc.org/yuv.php#IYUV
*/
class JpegYuv420pEncoder
{
public:
	/**
	 * \brief The contructor.
	 */
	JpegYuv420pEncoder(int width, int height, int quality = 75, int in_components = 3,
		J_COLOR_SPACE in_color_space = JCS_YCbCr);
	
	/**
	 * \brief The destructor.
	 */
	virtual ~JpegYuv420pEncoder();

public:
	/**
	 * \brief Encodes a yuv420p image pointer.
	 */
	JOCTET* encode(JSAMPLE *raw_image, size_t &jpeg_size);

private:
	jpeg_compress_struct cinfo_;
	jpeg_error_mgr jerr_;
	mem_dst_mgr *destmgr_;

	JSAMPROW m_y[16],m_cb[16],m_cr[16];
	JSAMPARRAY m_data[3];
};


/*
 * Formatbeschreibung unter
 * http://www.fourcc.org/yuv.php#YUYV
*/
class JpegYuv422Encoder
{
public:
	/**
	 * \brief The contructor.
	 */
	JpegYuv422Encoder(int width, int height, int quality = 75, int in_components = 3,
		J_COLOR_SPACE in_color_space = JCS_YCbCr);
	
	/**
	 * \brief The destructor.
	 */
	virtual ~JpegYuv422Encoder();

public:
	/**
	 * \brief Encodes a yuv422 image pointer.
	 */
	JOCTET* encode(JSAMPLE *raw_image, size_t &jpeg_size);

private:
	jpeg_compress_struct cinfo_;
	jpeg_error_mgr jerr_;
	mem_dst_mgr *destmgr_;

	JSAMPROW m_y[16],m_cb[16],m_cr[16];
	JSAMPARRAY m_data[3];

   unsigned char* m_yBytes;
   unsigned char* m_cbBytes;
   unsigned char* m_crBytes;
};


/** @}  // end of group libjpegcodec */


#endif  // _JPEG_CODEC_H_
