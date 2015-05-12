#include "heightmap/visibility_helpers.h"

#ifdef __APPLE__
#include <opencv/cv.h>
#include <opencv/highgui.h>
#else
#include <cv.h>
#include <highgui.h>
#endif

#include <iostream>
#include <sstream>
#include <iomanip>
#include <string.h>

#include "gui/viewer.h"
#include "heightmap/heightmap.h"

using std::endl;
using std::ostringstream;
using std::ios;
using std::setprecision;
using std::setiosflags;
using namespace std;

#define DEBUG_POLY (0)

#define MIN_OUTER_AREA (10)
#define MIN_INNER_AREA (100)

#define POLYGIN_APPORIMATION_PARAM (5.0)

// Filtering params
#define DEBUG_FILTERING (0)

#define DEBUG_FIZZEL_DETECTOR (0)
extern Viewer *viewer;

static char UTMZone[100] = GLOBAL_UTM_ZONE;

// Converts to gps
void convert(double x, double y, double& longitute, double& latitude)
{
   if (!viewer || !viewer->_geo) {
      longitute = 0;
      latitude = 0;
      printf("ERROR conversion\n");
      return;
   }
   HeightMap* hmap = viewer->getMap();
   if (hmap == NULL) {
      longitute = 0;
      latitude = 0;
      printf("ERROR conversion\n");
      return;
   }
 
   double worldX=0, worldY=0;
   hmap->grid2world(worldX, worldY, x, y);
   double easting = worldX;
   double northing = -worldY;
   if (viewer && viewer->_geo)
      viewer->_geo->image2world(&easting, &northing);
   gps_UTMtoLL(northing, easting, UTMZone, &latitude, &longitute);
}

//void convert2(double x, double y, double& worldX, double& worldY)
//{
//   if (!viewer || !viewer->_geo) {
//      worldX = worldY = 0;
//      printf("ERROR conversion\n");
//      return;
//   }
//   HeightMap* hmap = viewer->getMap();
//   if (hmap == NULL) {
//      worldX = worldY = 0;
//      printf("ERROR conversion\n");
//      return;
//   }
// 
//   hmap->grid2world(worldX, worldY, x, y);
//   worldY *= -1.0;
//   if (viewer && viewer->_geo)
//      viewer->_geo->image2world(&worldX, &worldY);
//   //gps_UTMtoLL(northing, easting, UTMZone, &latitude, &longitute);
//}


void writeVisSetToFile(string filename, const Visibility::VisSet& vset ) 
{
   CvScalar RED = CV_RGB(250,0,0);
   CvScalar BLUE = CV_RGB(0,0,255);
   CvScalar GREEN = CV_RGB(0,255,0);
   CvScalar WHITE = CV_RGB(255,255,255);

   CvPoint * cvpoints = new CvPoint[vset.size()];
   int count=0, maxX=0, maxY=0,minX=INT_MAX,minY=INT_MAX;
   for (Visibility::VisSet::const_iterator it=vset.begin(); it != vset.end(); it++)  {
      Visibility::Pos vp = it->first;
      cvpoints[count].x = vp.x;
      cvpoints[count].y = vp.y;
      if (vp.x>maxX) maxX=vp.x;
      if (vp.y>maxY) maxY=vp.y;
      if (vp.x<minX) minX=vp.x;
      if (vp.y<minY) minY=vp.y;
      count++;
   }
   int width = maxX-minX;
   int height = maxY-minY;
   //printf("# pts %d sizeX=%d sizeY=%d\n",count, width, height);

   //Draw points
   IplImage *ptimg = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
   cvZero( ptimg );
   for (int i=0; i<count; i++)
      cvLine(ptimg, cvPoint(cvpoints[i].x-minX, cvpoints[i].y-minY), cvPoint(cvpoints[i].x-minX, cvpoints[i].y-minY), WHITE);  

   // Write image
   cvSaveImage(filename.c_str(),ptimg);
   delete[] cvpoints;
}


deque< deque<Visibility::Pos> > writePolygonFromVisSet(ofstream& outstream, const Visibility::VisSet& vset, int num ) 
{
   deque< deque<Visibility::Pos> > polyset;
   CvScalar RED = CV_RGB(250,0,0);
   CvScalar BLUE = CV_RGB(0,0,255);
   CvScalar GREEN = CV_RGB(0,255,0);
   CvScalar WHITE = CV_RGB(255,255,255);

   CvPoint * cvpoints = new CvPoint[vset.size()];
   int count=0, maxX=0, maxY=0,minX=INT_MAX,minY=INT_MAX;
   for (Visibility::VisSet::const_iterator it=vset.begin(); it != vset.end(); it++)  {
      Visibility::Pos vp = it->first;
      cvpoints[count].x = vp.x;
      cvpoints[count].y = vp.y;
      if (vp.x>maxX) maxX=vp.x;
      if (vp.y>maxY) maxY=vp.y;
      if (vp.x<minX) minX=vp.x;
      if (vp.y<minY) minY=vp.y;
      count++;
   }
   int width = maxX-minX;
   int height = maxY-minY;
   //printf("# pts %d sizeX=%d sizeY=%d\n",count, width, height);

   //Draw points
   IplImage *ptimg = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
   cvZero( ptimg );
   for (int i=0; i<count; i++)
      cvLine(ptimg, cvPoint(cvpoints[i].x-minX, cvpoints[i].y-minY), cvPoint(cvpoints[i].x-minX, cvpoints[i].y-minY), WHITE);  

   // Write image
   if (DEBUG_POLY) {
      char fname[100];
      sprintf(fname,"/tmp/input%d.bmp",num);
      cvSaveImage(fname,ptimg);
   }

   // Find contours on image
   CvMemStorage* storage = cvCreateMemStorage(0);
   CvSeq* contours = 0;
   IplImage* gray = cvCreateImage( cvGetSize( ptimg ), 8, 1 );
   cvZero( gray );
   cvCvtColor( ptimg, gray, CV_BGR2GRAY );
   cvThreshold( gray, gray, 10, 255, CV_THRESH_BINARY );
   int n = cvFindContours( gray, storage, &contours, sizeof(CvContour), CV_RETR_CCOMP,  CV_CHAIN_APPROX_SIMPLE );
   printf("Found %d img_contours\n",n);

   IplImage *resimg = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
   cvZero( resimg );

   // Get contour points
   CvSeq* pCur = contours;

   ostringstream totalBuffer;

   ostringstream outerPolyBuffer;
   int outerPolyCount = 0;
   while(pCur != 0L)
   {
      //CvSeq* result = cvApproxPoly(pCur, sizeof(CvContour), storage, CV_POLY_APPROX_DP, cvContourPerimeter(pCur)*0.02, 0);
      CvSeq* result = cvApproxPoly(pCur, sizeof(CvContour), storage, CV_POLY_APPROX_DP, POLYGIN_APPORIMATION_PARAM, 1);

      // For debugging
      cvDrawContours(resimg, result, WHITE, WHITE, 100 );

      // Get contour points
      CvPoint point;
      CvSeq* pCur2 = result;
      CvSeqReader reader;

      while(pCur2 != 0L)
      {
         // Extract outer boundary

         double area = fabs(cvContourArea(pCur2, CV_WHOLE_SEQ));
         if (area < MIN_OUTER_AREA) {
            pCur2 = pCur2->h_next;
            continue;
         }
         cvStartReadSeq(pCur2, &reader);
         ostringstream buffer;
         deque<Visibility::Pos> poly;
         for( int i = 0; i < pCur2->total; i++ )
         {
            CV_READ_SEQ_ELEM( point, reader );
            //printf(" (%d %d) ",point.x, point.y);
            cvCircle(resimg, cvPoint(point.x, point.y), 1, GREEN, 1, 8, 0);  
            double longitute, latitude;
            convert(minX + point.x, minY + point.y, longitute, latitude);
            buffer << setiosflags(ios::fixed) << setprecision(10);
            buffer << latitude << " " << longitute << " ";
            poly.push_back(Visibility::Pos(minX + point.x, minY + point.y));
         }
         //printf("\n\n");
         polyset.push_back(poly);

         outerPolyBuffer << "Outer " << pCur2->total << " " << buffer.str() << " ";
         outerPolyCount++;

         // Extract inner boundaries
         ostringstream innerPolyBuffer;
         int innerPolyCount = 0;
         if (pCur2->v_next) {
            CvSeq* inner = pCur2->v_next;
            while (inner != NULL) {
               double area = fabs(cvContourArea(inner, CV_WHOLE_SEQ));
               if (area < MIN_INNER_AREA) {
                  inner = inner->h_next;
                  continue;
               }
               cvStartReadSeq(inner, &reader);
               ostringstream buffer;
               deque<Visibility::Pos> poly;
               for( int i = 0; i < inner->total; i++ )
               {
                  CV_READ_SEQ_ELEM( point, reader );
                  //printf(" (%d %d) ",point.x, point.y);
                  cvCircle(resimg, cvPoint(point.x, point.y), 1, BLUE, 1, 8, 0);  
                  double longitute, latitude;
                  convert(minX + point.x, minY + point.y, longitute, latitude);
                  buffer << setiosflags(ios::fixed) << setprecision(10);
                  buffer << latitude << " " << longitute << " ";
                  poly.push_back(Visibility::Pos(minX + point.x, minY + point.y));
               }
               polyset.push_back(poly);
               innerPolyBuffer << "Inner " << inner->total << " " << buffer.str() << " ";
               innerPolyCount++;
               inner = inner->h_next;
            }

         }
         outerPolyBuffer << innerPolyCount << " " << innerPolyBuffer.str() << " ";

         // Next outer contour
         pCur2 = pCur2->h_next;
      }

      pCur=pCur->h_next;
   }

   outstream << num << " " << outerPolyCount << " " << outerPolyBuffer.str() << endl;

   if (DEBUG_POLY) {
      char fname3[100];
      sprintf(fname3,"/tmp/contours%d.bmp",num);
      cvSaveImage(fname3,resimg);
   }

   // Clean up
   cvClearMemStorage(storage);
   cvReleaseImage(&gray);
   cvReleaseImage(&ptimg);
   cvReleaseImage(&resimg);
   delete [] cvpoints;

   return polyset;
}

void filterVisibilitySet(Visibility::VisSet& vset, bool onlyOnePoly, int x, int y) 
{
   (void) onlyOnePoly;
   if (vset.size() < Params::g_min_size_for_polygon_at_vertex)
      return;

   CvScalar GREEN = CV_RGB(0,255,0);
   CvScalar WHITE = CV_RGB(255,255,255);
   
   // Create points and bounding box from visibility set
   CvPoint * cvpoints = new CvPoint[vset.size()];
   int count=0, maxX=0, maxY=0,minX=INT_MAX,minY=INT_MAX;
   for (Visibility::VisSet::const_iterator it=vset.begin(); it != vset.end(); it++)  {
      Visibility::Pos vp = it->first;
      cvpoints[count].x = vp.x;
      cvpoints[count].y = vp.y;
      if (vp.x>maxX) maxX=vp.x;
      if (vp.y>maxY) maxY=vp.y;
      if (vp.x<minX) minX=vp.x;
      if (vp.y<minY) minY=vp.y;
      count++;
   }
   int width = maxX-minX;
   int height = maxY-minY;

   // Draw points into image
   IplImage *ptimg = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
   cvZero( ptimg );
   for (int i=0; i<count; i++)
      cvLine(ptimg, cvPoint(cvpoints[i].x-minX, cvpoints[i].y-minY), cvPoint(cvpoints[i].x-minX, cvpoints[i].y-minY), WHITE);  

   // Find contours on image
   CvMemStorage* storage = cvCreateMemStorage(0);
   CvSeq* contours = 0;
   IplImage* gray = cvCreateImage( cvGetSize( ptimg ), 8, 1 );
   cvZero( gray );
   cvCvtColor( ptimg, gray, CV_BGR2GRAY );
   cvThreshold( gray, gray, 10, 255, CV_THRESH_BINARY );
   cvFindContours( gray, storage, &contours, sizeof(CvContour), CV_RETR_EXTERNAL,  CV_CHAIN_APPROX_SIMPLE );
   if (contours == NULL) {
      cvReleaseImage(&gray);
      cvReleaseImage(&ptimg);
      cvClearMemStorage(storage);
      delete [] cvpoints;
      return;
   }

   // Remove points not in any polygon found from visibility set
   //double minPolyArea = Params::g_min_size_for_polygon_at_vertex;
   double maxPolyArea = 0;
   CvSeq* pCur = cvApproxPoly(contours, sizeof(CvContour), storage, CV_POLY_APPROX_DP, 1.0, 1);
   
   CvSeq* pPoly = pCur;
   CvSeq* maxPoly = NULL;
   while(pPoly != NULL) {
      double area = fabs(cvContourArea(pPoly, CV_WHOLE_SEQ));
      double ret = cvPointPolygonTest(pPoly, cvPointTo32f( cvPoint(x, y)), 1);
      if (ret >= -1 ) {
         maxPolyArea = area;
         maxPoly = pPoly;
      }
      pPoly=pPoly->h_next;
   }
   if ( maxPoly == NULL )
      return;
   
   for (Visibility::VisSet::iterator it=vset.begin(); it != vset.end(); )  
   {
      Visibility::Pos p = it->first;
      it++;
      int testx = (int)p.x-(int)minX;
      int testy = (int)p.y-(int)minY;

      double ret = cvPointPolygonTest(maxPoly, cvPointTo32f( cvPoint(testx, testy)), 1);
      if(ret < -1.0) {
         vset.erase(p);
      }
   }

   if (DEBUG_FILTERING) 
   {
      // Draw all polygons into input image
      while(pCur != NULL)
      {
         CvSeqReader reader;
         CvPoint point;
         cvStartReadSeq(pCur, &reader);
         for( int i = 0; i < pCur->total; i++ )
         {
            CV_READ_SEQ_ELEM( point, reader );
            cvCircle(ptimg, cvPoint(point.x, point.y), 1, GREEN, 1, 8, 0);  
         }
         pCur=pCur->h_next;
      }
 
      // Write input image
      static int cnt = 0;
      char fname[100];
      sprintf(fname,"/tmp/input%d.bmp",++cnt);
      cvSaveImage(fname,ptimg);

      // Write output image
      IplImage *ptimg2 = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
      cvZero( ptimg2 );
      for (Visibility::VisSet::const_iterator it=vset.begin(); it != vset.end(); it++)  {
         Visibility::Pos vp = it->first;
         cvLine(ptimg2, cvPoint(vp.x-minX, vp.y-minY), cvPoint(vp.x-minX, vp.y-minY), WHITE);  
      }
      sprintf(fname,"/tmp/output%d.bmp",0);
      cvSaveImage(fname,ptimg2);
      cvReleaseImage(&ptimg2);
   }

   // Clean up
   cvClearMemStorage(storage);
   cvReleaseImage(&gray);
   cvReleaseImage(&ptimg);
   delete [] cvpoints;
}

#if 0
bool importantFizzelArea(Visibility::VisSet& vset)
{
   if (vset.size() >  Params::g_imporved_sampling_min_area_size) 
      return true;
   if (vset.size() < 10)
      return false;

   HeightMap* hmap = viewer->getMap();
   if (hmap == NULL) {
      M_ERR("ERROR: Cannot access height map\n");
      return false;
   }
 
   CvScalar RED = CV_RGB(250,0,0);
   CvScalar BLUE = CV_RGB(0,0,255);
   CvScalar GREEN = CV_RGB(0,255,0);
   CvScalar WHITE = CV_RGB(255,255,255);
   CvScalar BLACK = CV_RGB(0,0,0);
   CvScalar ORANGE = CV_RGB(255,140,0);
   CvScalar VIOLET = CV_RGB(153,50,204);
   // Create points and bounding box from visibility set
   int vsize = 20;
   int arraysize = 9*vsize*vsize*vset.size();
   CvPoint * points = new CvPoint[arraysize];
   CvScalar * colors = new CvScalar[arraysize];
   int count=0, maxX=0, maxY=0,minX=INT_MAX,minY=INT_MAX;
   for (Visibility::VisSet::const_iterator it=vset.begin(); it != vset.end(); it++)  {
      Visibility::Pos point = it->first;

      for (int xx=-vsize; xx<=vsize; xx++) {
         for (int yy=-vsize; yy<=vsize; yy++) {
            Visibility::Pos vp = point;
            vp.x+=xx; vp.y+=yy;
 
            if (vp.x<0 || vp.y<0 || vp.x >= hmap->sizeX() || vp.y >= hmap->sizeZ()) 
               continue;

            // Do not overwrite set points
            if ((xx!=0 || yy!=0) && vset.find(vp) != vset.end())
               continue;

           if (Params::g_use_shrubs &&  hmap->getCellsMM()[vp.x][vp.y].getVegetation() > 0)
              colors[count] = GREEN;
           else if ((hmap->getCellsMM()[vp.x][vp.y].getClass() != HeightCell::ELC_FLAT_GROUND))
              colors[count] = RED;
           else if (xx==0 && yy==0)
              colors[count] = WHITE;
           else
              colors[count] = BLUE;

            points[count].x = vp.x;
            points[count].y = vp.y;
            if (vp.x>maxX) maxX=vp.x;
            if (vp.y>maxY) maxY=vp.y;
            if (vp.x<minX) minX=vp.x;
            if (vp.y<minY) minY=vp.y;
            count++;
            if (count>=(arraysize-1)) {
               M_ERR("Out of array bounds : %d\n",count/vset.size());
               break;
            }
         }
      }
   }
   int width = maxX-minX;
   int height = maxY-minY;

   // Draw points into image
   IplImage *ptimg = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
   cvZero( ptimg );
   for (int i=0; i<count; i++)
      cvLine(ptimg, cvPoint(points[i].x-minX, points[i].y-minY), cvPoint(points[i].x-minX, points[i].y-minY), colors[i]);  
   delete [] points;
   delete [] colors;

   // Find blue contours on image
   CvMemStorage* storage = cvCreateMemStorage(0);
   CvSeq* contours = 0;
   IplImage* gray = cvCreateImage( cvGetSize( ptimg ), 8, 1 );
   cvZero( gray );
   cvCvtColor( ptimg, gray, CV_BGR2GRAY );
   cvThreshold( gray, gray, 254, 255, CV_THRESH_BINARY );
   cvFindContours( gray, storage, &contours, sizeof(CvContour), CV_RETR_EXTERNAL,  CV_CHAIN_APPROX_NONE );
 
   cvDrawContours(ptimg, contours, BLACK, ORANGE, 1 );

   // Evaluate fizzle contour
   CvSeq* curr = contours;
   int freeCount=0;
   int blockedCount=0;
   while(curr != NULL)
   {
      double area = fabs(cvContourArea(curr, CV_WHOLE_SEQ));
      if (area < 2) {
         curr = curr->h_next;
         continue;
      }
      CvSeqReader reader;
      cvStartReadSeq(curr, &reader);
      for( int i=0; i<curr->total; i++)
      {
         CvPoint point;
         CV_READ_SEQ_ELEM( point, reader );
         
         // drawContour manual
         //cvSet2D(ptimg, point.y, point.x,BLACK);

         // Check all neighbors
         int vsize=1;
         Visibility::Pos vp(point.y, point.x);
         for (int xx=-vsize; xx<=vsize; xx++) {
            for (int yy=-vsize; yy<=vsize; yy++) {
               vp.x+=xx; vp.y+=yy;

               if (vp.x<0 || vp.y<0 || vp.x>=ptimg->width || vp.y>=ptimg->height)
                  continue;
 
               CvScalar s=cvGet2D(ptimg,vp.x,vp.y); 

               bool red = (s.val[0]==RED.val[0] && s.val[1]==RED.val[1] && s.val[2]==RED.val[2]);
               bool green = (s.val[0]==GREEN.val[0] && s.val[1]==GREEN.val[1] && s.val[2]==GREEN.val[2]);
               bool blue = (s.val[0]==BLUE.val[0] && s.val[1]==BLUE.val[1] && s.val[2]==BLUE.val[2]);
               bool black = (s.val[0]==BLACK.val[0] && s.val[1]==BLACK.val[1] && s.val[2]==BLACK.val[2]);
               bool white = (s.val[0]==WHITE.val[0] && s.val[1]==WHITE.val[1] && s.val[2]==WHITE.val[2]);

               if (black || white) {
                  continue;
               }

               if (blue) 
                  freeCount++;
               else if (red || green)
                  blockedCount++;
            }
         }
      }
      curr = curr->h_next;
   }

   bool important=false;
   if ((double) blockedCount / (double) freeCount  > 1.5)
      important = true;

   // Write debug image
   if (important)
      cvCircle(ptimg, cvPoint(5, 5), 5, ORANGE, 3, 8, 0);  
   else
      cvCircle(ptimg, cvPoint(5, 5), 5, VIOLET, 3, 8, 0);  

   if (DEBUG_FIZZEL_DETECTOR) {
      CvFont fontSmall;
      cvInitFont(&fontSmall,CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5, 0,1);
      char txt[100];
      sprintf(txt,"%d/%d",blockedCount, freeCount);
      cvPutText (ptimg,txt,cvPoint(20,15), &fontSmall, WHITE);

      char fname[100];
      static int cnt=0;
      sprintf(fname,"/tmp/fizzel_input%d.bmp",cnt);
      cnt++;
      cvSaveImage(fname,ptimg);
   }

   cvReleaseImage(&ptimg);
   cvReleaseImage(&gray);
   return important;
}

#endif
