#ifndef GRAPHCONSTRUCTION_IMG_STREAM_H
#define GRAPHCONSTRUCTION_IMG_STREAM_H

#include <opencv/cv.h>
#include <opencv/cvaux.h>
#include <opencv/highgui.h>
#include <CGAL/Kernel/global_functions_internal_2.h> 
#include <CGAL/basic.h>


namespace graphconstruction
{

class img_stream {
 public:
	img_stream( IplImage* img ) { stream_img = img; };
	~img_stream( ) { };
	img_stream& operator= (const img_stream& o) 
		{ stream_img = o.stream_img; return *this;};
	IplImage* stream_img;
};

template < class R >
img_stream&
operator<<(img_stream &gv, const CGAL::Point_2<R> &p)
{
	return gv;
}

template < class R >
img_stream&
operator<<(img_stream &gv, const CGAL::Segment_2<R> &s)
{
	IplImage* img = gv.stream_img;
	int x1,x2,y1,y2;
	x1 = int(CGAL::to_double(s.source().x())); 
    y1 = int(CGAL::to_double(s.source().y()));
	x2 = int(CGAL::to_double(s.target().x())); 
    y2 = int(CGAL::to_double(s.target().y()));
    
    if ( !(0 < x1 && x1 < img->height) 
        || !(0 < x2 && x2 < img->height)
        || !(0 < y1 && y1 < img->width) 
        || !(0 < y2 && y2 < img->width) )
    {
        return gv;
    }   
	CvScalar s1 = cvGet2D( img, x1, y1 );
	CvScalar s2 = cvGet2D( img, x2, y2 );
	if ( s1.val[0] == 0 || s2.val[0] == 0 ) {
		// black is inside obstacle
	}
	else {
		// draw something close to black
		cvLine( img,
		        cvPoint( y1, x1 ),
		        cvPoint( y2, x2 ),
		        cvScalar( 1, 1, 1 ), 1);
	}
	return gv;
}

template < class R >
img_stream&
operator<<(img_stream &gv, const CGAL::Line_2<R> &r)
{
	//cout << " called line << operator " << endl;
	return gv;
}

template < class R >
img_stream&
operator<<(img_stream &gv, const CGAL::Ray_2<R> &r)
{
	//cout << " called ray << operator " << endl;
 	return gv;
}

} /* graphconstruction */

#endif