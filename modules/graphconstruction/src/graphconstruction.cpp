#include "graphconstruction/graphconstruction.h"
#include "graphconstruction/img_stream.h"
#include <unistd.h>

#include <string>
#include <iostream>

using namespace std;
using namespace graphconstruction;

int main (int argc, char **argv)
{
    double lower_thres = 3.0;
    std::string img_filename = "";
    char c;
    while((c = getopt(argc, argv, "m:c:")) != EOF)
    {
       switch(c)
       {
          case 'm':
             img_filename = optarg;
             break;
          default:
          case 'h':
          case 'H':
             printf("\nOptions:\n");
             printf("--------------\n");
             printf("-m <filename> map img file.\n");
             printf("\n");
             exit(0);
             break;
       }
    }
    if (img_filename == "") {
       printf("You need to provide a img file for the map!\n");
       exit(0);
    }
    
    // load an image and create an occ vector
    IplImage* img_map = cvLoadImage( img_filename.c_str(), -1);
    CvSize img_map_size;
	img_map_size.width    = img_map->width;
	img_map_size.height   = img_map->height;
	IplImage* thresholded = cvCreateImage( img_map_size, 8, 1 );
	int threshold = 230, pixel = 0;
	cvThreshold( img_map, thresholded, lower_thres, 
	             double(threshold), CV_THRESH_BINARY );
	// build the occupancy set - reset occupancy variables/information
    int n_occupied_cells = 0, n_free_cells = 0;
    bool** occ = new bool*[thresholded->height];
    int max_x = thresholded->height;
    int max_y = thresholded->width;
	for ( int i = 0; i < max_x; i++ ) {
        occ[i] = new bool[max_y];
		for ( int j = 0; j < max_y; j++ ) {
			pixel = int (((uchar*)(thresholded->imageData + thresholded->widthStep*i))[j]);
			if (pixel == threshold) {
			    occ[i][j] = false;
                n_free_cells++;
			} else {
				occ[i][j] = true;
                n_occupied_cells++;
			}
		}
	}
    cout << " Occupied pixels " << n_occupied_cells << endl;
	cout << " Free pixels " << n_free_cells << endl;
	cvReleaseImage( &img_map ); cvReleaseImage( &thresholded );
    
    float a  = 5; // alphashape parameter
    float e  = 3; // simplify epsilon
    int gx = 100;
    int gy = 100;
    polygonization::Polygon_Environment *pol 
        = polygonization::polygonize_img( occ, max_x,max_y,
            a, e, gx, gy, "somefile" );
            
    polygonization::Voronoi_Diagram::Vertex_iterator 
        it = pol->VD->vertices_begin();

	IplImage* my_img = cvCreateImage( img_map_size, 8, 3 );
    cvSet(my_img, cvScalar(255,255,255));
	img_stream imgstream( my_img );
	
	pol->VD->dual().draw_dual( imgstream );
    char filename[200];
	sprintf( filename, "%s%s", "./", "save_img_CGAL_Real_Voronoi.BMP");
	cvSaveImage( filename, my_img );
	cvReleaseImage(&my_img);
    return 0;
}


