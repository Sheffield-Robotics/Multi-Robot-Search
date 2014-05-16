//
// File: gui/memdata_mgr.h
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

#ifndef _MEMDATA_MGR_
#define _MEMDATA_MGR_

#include <cstdio>
#include <vector>
extern "C" {
#include <jpeglib.h>
}


/**
 * \addtogroup libjpegcodec
 *
 * @{
 */


/**
 * \brief A STL vector for JOCTETs (bytes)
 */
typedef std::vector<JOCTET> octet_vector;


/**
 * \brief A custom destination manager for encoding JPEGs into RAM.
 */
struct mem_dst_mgr : public jpeg_destination_mgr
{
	// public interface
	mem_dst_mgr(j_compress_ptr cinfo, size_t buffersize = 4096);
	virtual ~mem_dst_mgr();
	
	JOCTET* get_image() const;
	
	// private stuff
	static void _init_destination(j_compress_ptr cinfo);
	static boolean _empty_output_buffer(j_compress_ptr cinfo);
	static void _term_destination(j_compress_ptr cinfo);

	j_compress_ptr cinfo_;
	size_t bufsize_;
	JOCTET *buffer_;
	octet_vector imgvec_;
};


/**
 * \brief A custom source manager for decoding JPEGs from RAM.
 */
struct mem_src_mgr : public jpeg_source_mgr
{
	// public interface
	mem_src_mgr(j_decompress_ptr cinfo);
	virtual ~mem_src_mgr();
	
	void set_jpeg_image(JOCTET *jpeg_image, size_t jpeg_size);
	
	// private stuff
	static void _init_source(j_decompress_ptr cinfo);
	static boolean _fill_input_buffer(j_decompress_ptr cinfo);
	static void _skip_input_data(j_decompress_ptr cinfo, long num_bytes);
	static void _term_source(j_decompress_ptr cinfo);
	
	j_decompress_ptr cinfo_;
	size_t jpeg_size_;
	JOCTET *jpeg_image_;
	boolean first_time_;
	
	static JOCTET FAKE_EOF[];
};


/** @}  // end of group libjpegcodec */


#endif  // _MEMDATA_MGR_
