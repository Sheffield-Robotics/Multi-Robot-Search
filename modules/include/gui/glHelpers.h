#ifndef GL_HELPERS_H
#define GL_HELPERS_H

#include <vector>
#include <deque>
#include <QGLViewer/qglviewer.h>
#include "heightmap/heightmap.h"
#include "planner/planningMap.h"
#include "planner/aStarSearch.h"
//#include <glut.h>
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

using std::vector;

typedef vector<int> VecI;
typedef vector<VecI> MatI;
void initGLHelpers();
void drawArc(double x0, double y0, double angle, double radius, double length, double alpha, bool backwards, int color);
void drawCircle(double xc, double yc, double zc, double rad, bool text);
void drawSphere(GLfloat radius, GLfloat x, GLfloat y, GLfloat z);
void drawSphere(GLfloat radius);
void drawBox(GLfloat l, GLfloat w, GLfloat h);
void drawGoalMarker(double x, double y, double height, double size);
void drawPlan(HeightMap* map, deque<PlanNode>* plan, double offset);

#if 0
void drawGpsSatellites(MatI satellites, sixd_robot_pos_message pose);
void drawOrientation(QGLViewer* v, sixd_robot_pos_message &robotPose);
void drawBearing(QGLViewer* v, sixd_robot_pos_message &pose, double bearing, double dist);
void drawTrajectory(vector <sixd_robot_pos_message> &poses, double r, double g, double b);
void drawLineScan(carmen_laser_laser_message &scan, sixd_robot_pos_message &p);
void drawOdoPose(carmen_base_odometry_message &odo);
void drawGpsPose(rescue_gps_sensor_message &gps, double height);
void drawPose(sixd_robot_pos_message &pose, GLfloat r, GLfloat g, GLfloat b);
void drawPointCloud(bool allignToPose, const point_cloud_message &msg, const sixd_robot_pos_message lastPose, double r, double g, double b);
#endif

#endif

