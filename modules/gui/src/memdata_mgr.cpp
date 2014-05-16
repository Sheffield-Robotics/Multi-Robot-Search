//
// File: memdata_mgr.cc
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

#include "gui/memdata_mgr.h"
#include <jerror.h>


mem_dst_mgr::mem_dst_mgr(j_compress_ptr cinfo, size_t bufsize)
	: cinfo_(cinfo), bufsize_(bufsize), buffer_(0)
{
	cinfo->dest = this;
	
	init_destination = _init_destination;
	empty_output_buffer = _empty_output_buffer;
	term_destination = _term_destination;
	
	buffer_ = new JOCTET[bufsize_];
}

mem_dst_mgr::~mem_dst_mgr()
{
	cinfo_->dest = 0;
	delete [] buffer_;
}


void mem_dst_mgr::_init_destination(j_compress_ptr cinfo)
{
	mem_dst_mgr *dest = static_cast<mem_dst_mgr*>(cinfo->dest);
	dest->free_in_buffer = dest->bufsize_;
	dest->next_output_byte = dest->buffer_;
	dest->imgvec_.clear();
}

boolean mem_dst_mgr::_empty_output_buffer(j_compress_ptr cinfo)
{
	mem_dst_mgr *dest = static_cast<mem_dst_mgr*>(cinfo->dest);
	
	octet_vector &v = dest->imgvec_;
	v.insert(v.end(), dest->buffer_, dest->buffer_ + dest->bufsize_);
	
	dest->free_in_buffer = dest->bufsize_;
	dest->next_output_byte = dest->buffer_;
	
	return TRUE;
}

void mem_dst_mgr::_term_destination(j_compress_ptr cinfo)
{
	mem_dst_mgr *dest = static_cast<mem_dst_mgr*>(cinfo->dest);

	size_t count = dest->bufsize_ - dest->free_in_buffer;

	if (count > 0)
	{
		octet_vector &v = dest->imgvec_;
		v.insert(v.end(), dest->buffer_, dest->buffer_ + count);
	}
}


mem_src_mgr::mem_src_mgr(j_decompress_ptr cinfo)
	: cinfo_(cinfo), jpeg_size_(0), jpeg_image_(0), first_time_(TRUE)
{
	cinfo->src = this;

	init_source = _init_source;
	fill_input_buffer = _fill_input_buffer;
	skip_input_data = _skip_input_data;
	resync_to_restart = jpeg_resync_to_restart;
	term_source = _term_source;

	bytes_in_buffer = 0;
	next_input_byte = 0;
}

mem_src_mgr::~mem_src_mgr()
{
	cinfo_->src = 0;
}

void mem_src_mgr::_init_source(j_decompress_ptr cinfo)
{
	mem_src_mgr *src = static_cast<mem_src_mgr*>(cinfo->src);
	src->first_time_ = TRUE;
}

JOCTET mem_src_mgr::FAKE_EOF[] = { JOCTET(0xFF), JOCTET(JPEG_EOI) };

boolean mem_src_mgr::_fill_input_buffer(j_decompress_ptr cinfo)
{
	mem_src_mgr *src = static_cast<mem_src_mgr*>(cinfo->src);
	
	if (src->first_time_)
	{
		src->first_time_ = FALSE;
		src->bytes_in_buffer = src->jpeg_size_;
		src->next_input_byte = src->jpeg_image_;
	}
	else
	{
		WARNMS(cinfo, JWRN_JPEG_EOF);
		src->bytes_in_buffer = 2;
		src->next_input_byte = FAKE_EOF;
	}
	
	return TRUE;
}

void mem_src_mgr::_skip_input_data(j_decompress_ptr cinfo, long num_bytes)
{
	mem_src_mgr *src = static_cast<mem_src_mgr*>(cinfo->src);

	if (num_bytes > 0)
	{
		src->next_input_byte += size_t(num_bytes);
    	src->bytes_in_buffer -= size_t(num_bytes);
	}
}

void mem_src_mgr::_term_source(j_decompress_ptr /* cinfo */)
{
	// noting to be done
}

void mem_src_mgr::set_jpeg_image(JOCTET *jpeg_image, size_t jpeg_size)
{
	jpeg_image_ = jpeg_image;
	jpeg_size_ = jpeg_size;
}
