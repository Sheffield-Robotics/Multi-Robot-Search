#include "gui/glHelpers.h"
#include "utilities/misc.h"
#include <sstream>

#define SCAN_LINE_WIDTH 2
#define POSE_RADIUS 0.05
#define NUM_PIXELS_PER_ARC 500

using std::deque;

static GLUquadricObj* g_quadratic;

void initGLHelpers()
{
   g_quadratic = gluNewQuadric();              // Create A Pointer To The Quadric Object ( NEW )
   gluQuadricNormals(g_quadratic, GLU_SMOOTH); // Create Smooth Normals ( NEW )
}

#if 0
void drawGpsSatellites(MatI satellites, sixd_robot_pos_message robot)
{
   if(satellites.empty())
      return;

   glDisable(GL_LIGHTING);
   glColor3f(0.6,0.2,1);
   glBegin(GL_LINE_LOOP);
   glVertex3f(robot.pose.x+100,robot.pose.y,robot.pose.z);
   glVertex3f(robot.pose.x,robot.pose.y,robot.pose.z);
   glEnd();
   glBegin(GL_LINE_LOOP);
   glColor3f(0.2,0.6,1);
   glVertex3f(robot.pose.x,robot.pose.y+100,robot.pose.z);
   glVertex3f(robot.pose.x,robot.pose.y,robot.pose.z);
   glEnd();
   glEnable(GL_LIGHTING);
   double alpha,beta,r,x,y,z,a,b,c;
   r = 70;
   FTGLPixmapFont fnt("/usr/share/fonts/truetype/msttcorefonts/arial.ttf");
   for(unsigned int ch = 0; ch < satellites.size(); ch++)
      if(satellites[ch][4] == 1)
      {
         alpha = -satellites[ch][0]*M_PI/180 + M_PI/2;
         beta  = satellites[ch][1]*M_PI/180;
         z = sin(beta)*r;
         x = cos(beta)*r*cos(alpha);
         y = cos(beta)*r*sin(alpha);
         a = robot.pose.x + x; //odo coordinate system
         b = robot.pose.y + y; // x = north, y = west
         c = robot.pose.z + z;
         glDisable(GL_LIGHTING);
         if(satellites[ch][5]==1) //visible
            glColor3f(0.7,1,0);
         else
            glColor3f(1,0.7,0.0); //not visible
         // drawCircle(a,b,z,1.5,false);

         drawSphere(1.5,a,b,c);
         glRasterPos3f( a,b, c);
         std::stringstream out;
         out << satellites[ch][2];
         fnt.FaceSize(16);
         fnt.Render(out.str().c_str());

         //glEnable(GL_LINE_STIPPLE);
         glLineStipple(1, 0x3F07);
         glLineStipple(1, 0x0101);

         glBegin(GL_LINE_LOOP);
         glVertex3f(a,b,c);
         glVertex3f(robot.pose.x,robot.pose.y,robot.pose.z);
         glEnd();
         //glDisable(GL_LINE_STIPPLE);

         glEnable(GL_LIGHTING);
      }
}
void drawTrajectory(vector <sixd_robot_pos_message> &poses, double r, double g, double b)
{
   glColor3f(r, g, b); 
   glDisable(GL_LIGHTING);
   glPushMatrix();
   glLineWidth(5.0);
   glBegin(GL_LINE_STRIP);
   for (unsigned i=0; i<poses.size(); i++)  {
      glVertex3f(poses[i].pose.x,poses[i].pose.y,poses[i].pose.z);
   }
   glEnd();
   glPopMatrix();

   glEnable(GL_LIGHTING);
   //drawSphere(POSE_RADIUS, poses[i].pose.x, poses[i].pose.y, poses[i].pose.z);
}


void drawPointCloud(bool allignToPose, const point_cloud_message& msg, const sixd_robot_pos_message lastPose, double r, double g, double b)
{
   double x,y,z,phi,theta,psi;
   if (!allignToPose) {
      x = msg.reference.x;
      y = msg.reference.y;
      z = msg.reference.z;
      phi = msg.reference.phi;
      theta = msg.reference.theta;
      psi = msg.reference.psi;
   }
   else {
      x = lastPose.pose.x;
      y = lastPose.pose.y;
      z = lastPose.pose.z;
      phi = lastPose.pose.phi;
      theta = lastPose.pose.theta;
      psi = lastPose.pose.psi;
   }

   // Disable lightning
   glDisable(GL_LIGHTING);

   // Set color 
   glColor3f(r,g,b); 
   //glLineWidth(2.0);
   glPointSize(1.5);

   // Draw the points
   glPushMatrix();
   //printf("Drawing at %lf,%lf yaw=%lf\n", x,y,RAD2DEG(phi));

   glTranslatef(x,y,z);
   glRotatef(RAD2DEG(phi),1,0,0);
   glRotatef(RAD2DEG(theta),0,1,0);
   glRotatef(RAD2DEG(psi),0,0,1);

   glBegin(GL_POINTS);
   unsigned int num_points = 0;
   for (unsigned int i = 0; i < msg.num_scanner_data; ++i)
      for (unsigned int j = 0; j < msg.scanner_data[i].num_scan_meas; ++j) {
         scan_meas_t& meas = msg.scanner_data[i].scans[j];
         for (unsigned int k = 0; k < meas.num_points; k++) {
            glVertex3f(meas.points[k].x, meas.points[k].y, meas.points[k].z);
            ++num_points;
         }
      }
   glEnd();
   glPopMatrix();
   //printf("draw point cloud with %d points\n", num_points);

   // Enable lightning
   glEnable(GL_LIGHTING); 
}


void drawLineScan(carmen_laser_laser_message &scan, sixd_robot_pos_message &p)
{
   // Disable lightning
   glDisable(GL_LIGHTING);
   glColor3f(1.0, 0.0, 0.0); 
   glPointSize(1.5);

   // Draw the points
   glPushMatrix();

   glTranslatef(p.pose.x, p.pose.y, p.pose.z);
   glRotatef(RAD2DEG(p.pose.phi),1,0,0);
   glRotatef(RAD2DEG(p.pose.theta),0,1,0);
   glRotatef(RAD2DEG(p.pose.psi),0,0,1);

   glLineWidth(SCAN_LINE_WIDTH);
   glBegin(GL_LINES);
   glColor3f(1.0, 0.5, 0.0); 
   double a = scan.config.start_angle;
   for (int j=0; j<scan.num_readings; j++) 
   {
      double r = scan.range[j];
      if (r >= scan.config.maximum_range)
         continue;

      glVertex3f(0,0,0); 
      glVertex3f(cos(a) * r, sin(a) * r, 0);
      a += scan.config.angular_resolution;
   }
   glEnd();
   glPopMatrix();

   // Enable lightning
   glEnable(GL_LIGHTING);
}

void drawOdoPose(carmen_base_odometry_message &odo)
{
   double xc = odo.x;
   double yc = odo.y;
   double rad = POSE_RADIUS;
   glDisable(GL_LIGHTING);
   glColor3f(0.0, 1.0, 1.0); 
   glLineWidth(2.0);
   drawCircle (xc,yc, 0.0, rad, false);
   glEnable(GL_LIGHTING); 
}

void drawGpsPose(rescue_gps_sensor_message &gps, double height)
{
   static bool _firstGps = true;
   static double _gpsStartX = 0;
   static double _gpsStartY = 0;

   if (_firstGps) {
      _gpsStartX = gps.UTMEasting;
      _gpsStartY = gps.UTMNorthing;
      _firstGps = false;
   }
   double xc = (int) (1.0 * (_gpsStartX - gps.UTMEasting));
   double yc = (int) (1.0 * (_gpsStartY - gps.UTMNorthing));
   double rad = POSE_RADIUS;
   glColor3f(0.0, 0.0, 1.0); 
   drawSphere(rad,xc,yc,height + 0.5);
}

void drawPose(sixd_robot_pos_message &pose, GLfloat r, GLfloat g, GLfloat b)
{
   double xc = pose.pose.x;
   double yc = pose.pose.y;
   double zc = pose.pose.z;
   double rad = POSE_RADIUS;
   glColor3f(r, g, b); 
   drawSphere(rad,xc,yc,zc);
}

void drawBearing(QGLViewer* v, sixd_robot_pos_message &pose, double bearing, double dist)
{
   (void) dist;
   glColor3f(1,0,1); 
   qglviewer::Vec p1(0.0, 0.0, 0.5);
   qglviewer::Vec p2(1.0, 0.0, 0.5);

   glPushMatrix();
   glTranslatef(pose.pose.x,pose.pose.y,pose.pose.z);
   glRotatef(RAD2DEG(bearing),0,0,1);

   v->drawArrow(p1, p2,0.03);
   glPopMatrix();
}

void drawOrientation(QGLViewer* v, sixd_robot_pos_message &robotPose)
{
   glColor3f(0,1,0); 
   qglviewer::Vec p1(0.0, 0.0, 0.5);
   qglviewer::Vec p2(1.0, 0.0, 0.5);

   double g_tx = robotPose.pose.x;
   double g_ty = robotPose.pose.y; 
   double g_tz = robotPose.pose.z;
   double g_rx = robotPose.pose.phi; // roll 
   double g_ry = robotPose.pose.theta; // pitch
   double g_rz = robotPose.pose.psi; // yaw 

   glPushMatrix();
   glTranslatef(g_tx,g_ty,g_tz);
   glRotatef(RAD2DEG(g_rx),1,0,0);
   glRotatef(RAD2DEG(g_ry),0,1,0);
   glRotatef(RAD2DEG(g_rz),0,0,1);
   v->drawArrow(p1, p2,0.03);
   glPopMatrix();
}


#endif


void drawSphere(GLfloat radius, GLfloat x, GLfloat y, GLfloat z)
{
   glPushMatrix();
   glTranslatef(x,y,z);
   drawSphere(radius);
   glPopMatrix();
}

void drawSphere(GLfloat radius)
{
   gluSphere(g_quadratic, radius, 32, 32);
}

void drawBox(GLfloat l, GLfloat w, GLfloat h)
{
  GLfloat sx = l*0.5f;
  GLfloat sy = w*0.5f;
  GLfloat sz = h*0.5f;

  glBegin(GL_QUADS);
  // bottom
  glNormal3f( 0.0f, 0.0f,-1.0f);
  glVertex3f(-sx, -sy, -sz);
  glVertex3f(-sx, sy, -sz);
  glVertex3f(sx, sy, -sz);
  glVertex3f(sx, -sy, -sz);
  // top
  glNormal3f( 0.0f, 0.0f,1.0f);
  glVertex3f(-sx, -sy, sz);
  glVertex3f(-sx, sy, sz);
  glVertex3f(sx, sy, sz);
  glVertex3f(sx, -sy, sz);
  // back
  glNormal3f(-1.0f, 0.0f, 0.0f);
  glVertex3f(-sx, -sy, -sz);
  glVertex3f(-sx, sy, -sz);
  glVertex3f(-sx, sy, sz);
  glVertex3f(-sx, -sy, sz);
  // front
  glNormal3f( 1.0f, 0.0f, 0.0f);
  glVertex3f(sx, -sy, -sz);
  glVertex3f(sx, sy, -sz);
  glVertex3f(sx, sy, sz);
  glVertex3f(sx, -sy, sz);
  // left
  glNormal3f( 0.0f, -1.0f, 0.0f);
  glVertex3f(-sx, -sy, -sz);
  glVertex3f(sx, -sy, -sz);
  glVertex3f(sx, -sy, sz);
  glVertex3f(-sx, -sy, sz);
  //right
  glNormal3f( 0.0f, 1.0f, 0.0f);
  glVertex3f(-sx, sy, -sz);
  glVertex3f(sx, sy, -sz);
  glVertex3f(sx, sy, sz);
  glVertex3f(-sx, sy, sz);
  glEnd();
}

void drawCircle(double xc, double yc, double zc, double rad, bool text)
{
   // draw a circle centered at (xc,yc) with radius rad
   glBegin(GL_LINE_LOOP);
   for (int angle=0; angle<365; angle=angle+5)
   {
      double angle_radians = DEG2RAD(angle);
      double x = xc + rad * cos(angle_radians);
      double y = yc + rad * sin(angle_radians);
      glVertex3f(x,y,zc);
   }
   glEnd();

   if (text) {
      //FTGLPixmapFont fnt("/usr/share/fonts/truetype/msttcorefonts/arial.ttf");
      //fnt.FaceSize(10);
      glColor3f(0.0, 0.0, 0.0); 
      char dist[10];
      sprintf(dist,"%lf",rad);
      glRasterPos3f( rad , 0, 1.0);
      //fnt.Render(dist);
   }
}


void drawGoalMarker(double x, double y, double height, double size)
{
   drawCircle(x,y,height+0.1,size,false);
   drawCircle(x,y,height+0.1,size/2.0,false);
   drawCircle(x,y,height+0.1,size/10.0,false);
}

void drawArc(double x0, double y0, double angle, double radius, double length, double alpha, bool backwards, int color)
{
   glPushMatrix();

   double dx = x0;
   double dy = y0;
   double z = 0.1;
   glTranslatef(dx, dy,0);

   if (backwards)
      glRotatef(180.0 + RAD2DEG(angle),0,0,1);
   else
      glRotatef(RAD2DEG(angle),0,0,1);


   glDisable(GL_LIGHTING);

   // Special case: straight
   if (radius == 0.0 && length != 0.0) {
      double x = length;
      if (backwards) 
         x *= -1;
      // Draw end point
      glPointSize(5);
      glColor3f(1.0,0.1,0);
      glBegin(GL_POINTS);
      glVertex3f(x, 0, z);
      glEnd();

      // Draw points
      glLineWidth(2);
      if (color == 0)
         glColor3f(1.0,1.0,0);
      else
         glColor3f(0.0, 0.0, 1.0);
      glBegin(GL_LINES);
      glVertex3f(0, 0, z);
      glVertex3f(x, 0, z);
      glEnd();
   }
   // Special case: point turn
   else if(radius == 0.0 && length == 0.0) {
      double r = 0.4;
      double d = r;
      if (alpha>0)
         alpha *= -1;
      drawArc(0, d, angle, r, M_PI, 5*alpha, backwards, color);
   }
   else { // arc
      // Resolution of the arc
      int r = NUM_PIXELS_PER_ARC;
      int f = 1 - r;
      int ddF_x = 0;
      int ddF_y = -2 * r;
      int x = 0;
      int y = r;
      double resolution = radius / (double) r;

      // Generate points
      deque<int> x1;
      deque<int> y1;
      deque<int> x2;
      deque<int> y2;
      deque<int> x3;
      deque<int> y3;
      deque<int> x4;
      deque<int> y4;
      while(x < y) 
      {
         if(f >= 0) 
         {
            y--;
            ddF_y += 2;
            f += ddF_y;
         }
         x++;
         ddF_x += 2;
         f += ddF_x + 1;

         if (alpha > 0) {
            x1.push_back(x);
            y1.push_back(-y+r);
            x2.push_front(y);
            y2.push_front(-x+r);

            x4.push_front(x);
            y4.push_front(y+r);
            x3.push_back(y);
            y3.push_back(x+r);
         }
         else {
            x1.push_back(x);
            y1.push_back(y-r);
            x2.push_front(y);
            y2.push_front(x-r);

            x4.push_front(x);
            y4.push_front(-y-r);
            x3.push_back(y);
            y3.push_back(-x-r);
         }
      }

      // Draw Points
      deque<double> X;
      deque<double> Y;

      double endX = radius * sin(alpha) / resolution;
      double endY = radius * (1.0 - cos(alpha)) / resolution;
      double d = hypot(endX,endY);

      // First quarter
      for (unsigned i=0; i<x1.size(); i++) {
         if (hypot((double)x1[i],(double)y1[i]) >= d) break;
         X.push_back((double)(x1[i])*resolution);
         Y.push_back((double)(y1[i])*resolution);
      }

      for (unsigned i=0; i<x2.size(); i++) {
         if (hypot((double)x2[i],(double)y2[i]) >= d) break;
         X.push_back((double)(x2[i])*resolution);
         Y.push_back((double)(y2[i])*resolution);
      }

      // Second Quarter
      for (unsigned i=0; i<x3.size(); i++) {
         if (hypot((double)x3[i],(double)y3[i]) >= d) break;
         X.push_back((double)(x3[i])*resolution);
         Y.push_back((double)(y3[i])*resolution);
      }
      for (unsigned i=0; i<x4.size(); i++) {
         if (hypot((double)x4[i],(double)y4[i]) >= d) break;
         X.push_back((double)(x4[i])*resolution);
         Y.push_back((double)(y4[i])*resolution);
      }

      // Draw end point
      glPointSize(5);
      glColor3f(1.0,0.1,0);
      glBegin(GL_POINTS);
      if (X.size()>0) {
         glVertex3f(X.back(),Y.back(),z);
         //printf("END %1.2lf %1.2lf LENGTH=%1.2lf\n",X.back(), Y.back(),length);
      }
      glEnd();

      // Draw points
      glPointSize(2);
      if (color == 0)
         glColor3f(1.0,1.0,0);
      else
         glColor3f(0.0, 0.0, 1.0);
      glBegin(GL_POINTS);
      for (unsigned int i=0; i<X.size(); i++) 
         glVertex3f(X[i],Y[i],z);
      glEnd();
   }

   glPopMatrix();
   glEnable(GL_LIGHTING);
}

void drawPlan(HeightMap* map, deque<PlanNode>* plan, double offset)
{
   if (plan != NULL && plan->size() > 0) {
      //printf("Path length: %d\n",pursuer.plan->size());
      glLineWidth(3.0);
      glDisable(GL_LIGHTING);
      glBegin(GL_LINES);
      for (unsigned int i=1; i<plan->size(); i++) {
         PlanNode n1 = (*plan)[i-1];
         PlanNode n2 = (*plan)[i];
         int x1= (int) n1.x;
         int x2= (int) n2.x;
         int y1= (int) n1.z;
         int y2= (int) n2.z;
         map->grid2world(n1.x,n1.z, x1, y1);
         map->grid2world(n2.x,n2.z, x2, y2);
         double h1 = offset + map->getCellsMM()[x1][y1].getHeight()/ 1000.0;
         double h2 = offset + map->getCellsMM()[x2][y2].getHeight()/ 1000.0;
         glVertex3f(n1.x,n1.z,h1);
         glVertex3f(n2.x,n2.z,h2);
      }
      glEnd();
      glEnable(GL_LIGHTING);
   }
}


