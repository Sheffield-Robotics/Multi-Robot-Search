#ifndef GL_POSEGUI_H
#define GL_POSEGUI_H

#include <QGLViewer/qglviewer.h>
#include <QMouseEvent>
#include <QWidget>  
#include <vector>
#include <string>
#include "utilities/angles.h"
#include "heightmap/heightmap.h"
#include "gui/terrain.h"
#include "agents/agentstate.h"
#include "heightmap/visibility.h"
#include "gui/animator.h"
#include "heightmap/geomap.h"
#include "utilities/gpsconvert.h"
#include "planner/aStarSearch.h"
#include "utilities/timing.h"
#include "heightmap/perimeter.h"
#include "polygonization/polygonization.h"
#include "polygonization/voronoi_diagram.h"
#include "lineclear/ChoiceTree.h"
#include "hungarian/hungarian2.h"
#include "utilities/Yaml_Config.h"
#include <qmessagebox.h>
#include <qspinbox.h>
#include <qcombobox.h>
#include <qradiobutton.h>
#include <qfiledialog.h>
#include <qlineedit.h>
#include <qpushbutton.h>
#include <qlabel.h>
#include <qpoint.h>


#define DEBUG_HUNGARIAN 1

extern bool redraw;

class HeightMap;
class MyMouseGrabber;
namespace Ui {
    class ViewerInterface;
    class OptionWidget;
}


class Viewer : public QGLViewer
{
    friend class Ui::ViewerInterface;
    friend class Ui::OptionWidget;
   public:
      Viewer(QWidget* parent=NULL, const QGLWidget* shareWidget=0, Qt::WFlags flags=0);
 
      void setPointers(HeightMap* map, Visibility* vis, Animator* anim, Perimeter* per);

      bool loadHeightmapFromTIFF(const string &filename);
      bool loadImageMapFromTIFF(const string &filename);

      // Switching draw modes
      void setDrawHeightmapRegions(bool redraw=false);
      void setDrawVisibility(bool redraw=false);
      void setDrawHeightmapPlain();
      void setDrawHeightmapClasses();
      void setDrawPoses(bool on);
      void setDrawWireFrame(bool on); 
      void setDrawGeoMapDEMImage();
      void setDrawGeoMapImage();
      void drawGraphStructure();
      void drawVisibilityGraph();
      void drawVisibilityGraphVertex(Segment_Visibility_Graph::mygraph_t::vertex_iterator v_it);
      void drawVisibilityPolygon();
      void drawPolygonEnvironment();
      void drawAllTrajectories();
      void drawAllTrajectoriesLines();
      void draw_uav_test_poses();
      void drawFrequinPoses();
      void drawSweepState();
      
      void build_graph_from_sequence();
      
      void apply_hungarian();
      
      void toggle_draw_up_to();
      void toggle_sweep_state();
      void toggle_current_step();
      void toggle_visi_poly(bool backwards = false);
      void toggle_visi_graph();
      void toggle_visi_graph_vertex();
      void drawPolygonRawEnvironment();
      void drawPoly( polygonization::Polygon *poly );
      void drawVoronoiDiagram();
      void drawStrategyStep();
      void drawSegment(lineclear::Segment s, double add_ground=0);
      void draw_sphere_at(int x, int y, double h, double size);
      void draw_line_from_to(int gx,int gy, int g2x, int g2y, double h, double h2);
      void drawText3D(double p1x,double p1y, double ground, std::string txt);
      void drawLine(double p1x,double p1y, double p2x, double p2y, double ground = 0);
      void drawPolygonizationSegment(polygonization::Segment s);
      void draw_UAV_at(int x, int y);
      void draw_shortest_split();
      void ask_for_ijk();
      void compute_lineclear_strategy();
      //void computing_adapted_cost();
      void save_choice_tree();
      void load_choice_tree();
      void load_strategy();
      void test_visibility_line_cost();
      void next_step();
      void setPursuerPosFromUTM(double easting, double northing);
      void triggerSetRobotPose();
      
      void print_help();
      void draw_gui_text();
      
      void init_pol();
      void init_random_pol();
      void init_env();
      
      std::vector<int> artificials;

      HeightMap* getMap() {return _map;}
      Visibility* getVisibility() const {return _vis;} 
      Animator* getAnimator() const {return _anim;} 
      
      void save_polygon();

      deque<PlanNode>* testPlan;
      
      // test stuff for lineclear strategies
      int current_i;
      int current_k;
      int current_cost;
      int current_blocking_cost;
      int artifical_cost;
      int max_cost;
      int draw_up_to;
      int sweep_state_i;
      int current_step_i;
      int updated_cost;
      int visi_poly_index;
      Segment_Visibility_Graph::mygraph_t::vertex_iterator v_it, v_end;
      int segment_plan_to_vertex_id;
      int segment_plan_to_vertex_id2;
      std::list<int> obstacle_sequence;
      std::list<int>::iterator obstacle_sequence_it;
      std::list<int> cleared_obstacles;
      lineclear::Segment l1,l2,l3,l4; 
      std::map< std::pair<int,int>,lineclear::Segment> blocking_lines_map;
      std::map< std::pair<int,int>,std::list<polygonization::KERNEL::Segment_2> > blocking_lines_map2;
      std::map<int,int> artifical_to_cost;
      std::vector< std::vector<NavPoint> > all_uav_poses;
      std::list<Visibility::Pos> uav_test_poses;
      std::list<polygonization::KERNEL::Segment_2> shortest_path;
      std::list<polygonization::KERNEL::Segment_2> shortest_split;
      std::list<polygonization::KERNEL::Segment_2> split_point_list;

   public:
      bool showHeightMap;
      bool showGeoMap;
      bool drawGraph;
      bool drawPolygonEnvironmentFlag;
      bool drawFrequinPosesFlag;
      bool drawAllTrajectoriesFlag;
      bool drawAllTrajectoriesLinesFlag;
      bool drawUAVtestposesFlag;
      bool drawSweepStateFlag;
      bool drawPolygonRawEnvironmentFlag;
      bool drawVoronoiDiagramFlag;
      bool drawStrategyStepFlag;
      bool drawPursuerExample;
      bool drawAgentSize;
      bool drawVisiPoli;
      bool drawVisiGraph;
      bool drawVisiGraphVertex;
      bool drawShortestSplit;
      bool triggerDumpScreenShot;
      bool only_plot_enabled;
      Segment_Visibility_Graph::mygraph_t::vertex_iterator 
          only_plot_vis_seg_graph_vertex;
      Segment_Visibility_Graph::mygraph_t::vertex_iterator 
          only_plot_vis_seg_graph_vertex_end;

   protected :
      virtual void init();
      virtual void draw();
      virtual void keyPressEvent(QKeyEvent *e);
      void mouseMoveEvent( QMouseEvent* const e);
      virtual QString helpString() const;

      void loadImage();
      void drawHeightMap();

      void prepareHelp();

      // Object selection
      virtual void drawWithNames();
      virtual void postSelection(const QPoint& point);
      void dumpScreenShot();
      void drawAgentSizes();
      double get_max_height_for_draw();
      double get_max_height();
      void openInterfaceWindow();
      void closeInterfaceWindow();
      const Viewer* qglviewer() const { 
          const Viewer* v = this;
          return v; };
      
   public:
      GeoMap* _geo;

   protected :
      HeightMap* _map;
      Visibility* _vis;
      polygonization::Polygon_Environment* _pol;
	  Perimeter* _per;
      Animator* _anim;
      GeoMap* _geoImg;
      lineclear::Environment *_env;
      lineclear::ChoiceTree *_ct;
      // Object selection
      long lastCellSelect;
      HMAP_COLOR_MODE currentHeightMapMode;
      bool setRobotPose;
      bool setTargetPose;
      bool drawWireFrame;
      int arcDL;
      int pursuerX,pursuerY;
      double sceneRadius;
      bool classifiedMap;
      bool showDEMimage;
      bool geoInit, geoImageInit;
      std::string _lastMapPureFileName;
      int _currentPoly;
      double max_height;
      bool line_visibility_test_flag;
      int line_visibility_test_grid_x1;
      int line_visibility_test_grid_y1;
      int line_visibility_test_grid_x2;
      int line_visibility_test_grid_y2;
      
      int vis_graph_i,vis_graph_j,vis_graph_k;
      
      Ui::ViewerInterface* viewer_interface_;
      Ui::OptionWidget* option_widget_;
};


inline void Viewer::mouseMoveEvent( QMouseEvent* const e) 
{ 
   static int oldMouseX = e->x();
   //std::cout << "Mouse at " << e->x() << ":" << e->y() << std::endl;
   if ((e->modifiers() & Qt::ShiftModifier) && (e->modifiers() & Qt::ControlModifier)) { 
      bool found = false;
      qglviewer::Vec point = camera()->pointUnderPixel(QPoint(e->x(),e->y()), found);
      if (found) {
         double ox=point.x;
         double oy=-point.y;
         double lon,lat;
         char UTMZone[100] = GLOBAL_UTM_ZONE;
         if (_geo || _geoImg) {
            int gridX=0, gridY=0;
            _map->world2grid(point.x, point.y, gridX, gridY);
            if (_geo)
               _geo->image2world(&ox, &oy);
            else if (_geoImg)
               _geoImg->image2world(&ox, &oy);
            gps_UTMtoLL(oy, ox, UTMZone, &lat, &lon);
			printf("Deactivating Perimeter pixel (x,y,z)->(%1.2lfm %1.2lfm %1.2lfm) grid_cells (x,y)->(%d,%d) world_UTM (x,y)->(%lf %lf) (lat/lon)->(%lf %lf) \n"
                  ,point.x, point.y, point.z, gridX, gridY, ox, oy, lat,lon);
			if (_map->pointInMap(gridX, gridY))
				_map->getCellsMM()[gridX][gridY].togglePerimeterOff(); 
			redraw=true;
			terrainLoadFromHeightMap(_map, false, HMAP_COLOR_CLASS);
			terrainDraw(true, _map->worldOffsetX(),-_map->worldOffsetZ());
			updateGL();
         }
      }
   } else if ((e->modifiers() & Qt::ShiftModifier) && (e->modifiers() & Qt::AltModifier)) { 
      bool found = false;
      qglviewer::Vec point = camera()->pointUnderPixel(QPoint(e->x(),e->y()), found);
      if (found) {
         double ox=point.x;
         double oy=-point.y;
         double lon,lat;
         char UTMZone[100] = GLOBAL_UTM_ZONE;
         if (_geo || _geoImg) {
            int gridX=0, gridY=0;
            _map->world2grid(point.x, point.y, gridX, gridY);
            if (_geo)
               _geo->image2world(&ox, &oy);
            else if (_geoImg)
               _geoImg->image2world(&ox, &oy);
            gps_UTMtoLL(oy, ox, UTMZone, &lat, &lon);
            printf("Activating Perimeter pixel (x,y,z)->(%1.2lfm %1.2lfm %1.2lfm) grid_cells (x,y)->(%d,%d) world_UTM (x,y)->(%lf %lf) (lat/lon)->(%lf %lf) \n"
                  ,point.x, point.y, point.z, gridX, gridY, ox, oy, lat,lon);
			if (_map->pointInMap(gridX, gridY))
				_map->getCellsMM()[gridX][gridY].togglePerimeterOn(); 
			redraw=true;
			//terrainLoadFromHeightMap(_map, false, HMAP_COLOR_CLASS);
			//terrainDraw(true, _map->worldOffsetX(),-_map->worldOffsetZ());
			updateGL();
         }
      }
   } 
   // Write position to file
/*   else if (e->modifiers() ==  Qt::ControlModifier) 
   {
      bool found = false;
      qglviewer::Vec point = camera()->pointUnderPixel(QPoint(e->x(),e->y()), found); 
      if (found) 
      {
         FILE* f=fopen("markedPoints.dat","a+");
         if (!f)
            return;
         double ox=point.x;
         double oy=-point.y;
         double lon,lat;
         char UTMZone[100] = GLOBAL_UTM_ZONE;
         if (_geo || _geoImg) {
            int gridX=0, gridY=0;
            _map->world2grid(point.x, point.y, gridX, gridY);
            if (_geo)
               _geo->image2world(&ox, &oy);
            else if (_geoImg)
               _geoImg->image2world(&ox, &oy);
            gps_UTMtoLL(oy, ox, UTMZone, &lat, &lon);

            double timeNow = getCurrentTime();
            printf("Adding pointed location to file\n");
            fprintf(f,"time:%lf, Point under pixel (x,y,z)->(%1.2lfm %1.2lfm %1.2lfm) grid_cells (x,y)->(%d,%d) world_UTM (x,y)->(%lf %lf) (lat/lon)->(%lf %lf) \n",
                  timeNow,point.x, point.y, point.z, gridX, gridY, ox, oy, lat,lon);
            fclose(f);
         }
      }
   }
*/
   // Test planner
   else if (e->modifiers() ==  Qt::AltModifier) 
   {
      bool found = false;
      qglviewer::Vec point = camera()->pointUnderPixel(QPoint(e->x(),e->y()), found); 
      if (found) 
      {
         if (_map == NULL) return;

         int gridX=0, gridY=0;
         _map->world2grid(point.x, point.y, gridX, gridY);
 
         M_INFO3("Planning from (%d, %d) to (%d %d)\n",pursuerX, pursuerY, gridX, gridY);
         Timing timing("Planning");
         PlanningMap* _pmap = new PlanningMap();
         _pmap->createFromHeightMap(_map);
         AStarSearch astar;
         testPlan = astar.search(_pmap, pursuerX, pursuerY, gridX, gridY);
         delete _pmap;
         timing.printInfo(true);
         if (testPlan == NULL)
            M_ERR("No plan found!!\n");
         else
            M_INFO3("Found plan of size %d\n",testPlan->size());
         updateGL();
      }
   }
  else if ((e->modifiers() & Qt::ShiftModifier)) { 
		bool found = false;
      	qglviewer::Vec point = camera()->pointUnderPixel(QPoint(e->x(),e->y()), found);
      	if (found) {
         	if (_geo || _geoImg) {
            	int gridX=0, gridY=0;
            	_map->world2grid(point.x, point.y, gridX, gridY);
				if (_map->pointInMap(gridX, gridY)) {
					double value = _per->getValue(gridX, gridY);
            		printf("Grid Cells (x,y)->(%d,%d) Pixel (x,y,z)->(%1.2lfm %1.2lfm %1.2lfm) Value: %lf \n",
                  		gridX, gridY,
						point.x, point.y, point.z,
						value
					);
				redraw=true;
			   }
         }
      }
   }
   //else if ((e->modifiers() & Qt::ControlModifier)) { 
   //    bool found = false;
   //    qglviewer::Vec point = camera()->pointUnderPixel(QPoint(e->x(),e->y()), found);
   //    if (found) {
   //        if (_geo || _geoImg) {
   //            int gridX=0, gridY=0;
   //            _map->world2grid(point.x, point.y, gridX, gridY);
   //            if (_map->pointInMap(gridX, gridY)) {
   //                double value = _per->getValue(gridX, gridY);
   //                if ( line_visibility_test_flag ) {
   //                    line_visibility_test_grid_x1 = gridX;
   //                    line_visibility_test_grid_y1 = gridY;
   //                } else {
   //                    line_visibility_test_grid_x2 = gridX;
   //                    line_visibility_test_grid_y2 = gridY;
   //                }
   //                line_visibility_test_flag = !line_visibility_test_flag;
   //            }
   //        }
   //    }
   //    if ( found && line_visibility_test_flag ) {
   //        redraw=true;
   //    }
   //}
   else {
    QGLViewer::mouseMoveEvent(e);
   }
   oldMouseX = e->x();
}


#endif
