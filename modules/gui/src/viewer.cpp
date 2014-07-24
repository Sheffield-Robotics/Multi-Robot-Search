#include <sstream>
#include "gui/viewer.h"
#include "gui/glHelpers.h"
#include "utilities/misc.h"
#include "utilities/math/bresenham.h"
#include "heightmap/visibility.h"
#include "gui/jpegImage.h"
#include <map>
#include <vector>
#include "utilities/filesysTools.h"

using namespace std;


using std::map;

extern bool quit_signal;
extern bool redraw;
extern bool recreate;
extern string confFile;

Viewer::Viewer(QWidget* parent, const QGLWidget* shareWidget, Qt::WFlags flags) : QGLViewer(parent, shareWidget, flags)
{
    _map = NULL;
    _vis = NULL;
    _anim = NULL;
    _geo = NULL;
    _pol = NULL;
    _env = NULL;
    _ct = NULL;
    current_cost = 0;
    max_cost = 0;
    artifical_cost = 0;
    current_blocking_cost = 0;
    draw_up_to = 0;
    visi_poly_index = -1;
    sweep_state_i = 0;
    current_step_i = 0;
    max_height = 0;
    line_visibility_test_flag = true;
    // disable auto saved state
    setStateFileName(QString::null);
    setRobotPose = false;
    drawWireFrame = false;
    drawVisiPoli = false;
    drawVisiGraphVertex = false;
    drawStrategyStepFlag = false;
    drawFrequinPosesFlag = false;
    drawSweepStateFlag = false;
    setMouseTracking(true);
    showHeightMap = true;
    drawGraph = false;
    drawAllTrajectoriesFlag = false;
    drawAllTrajectoriesLinesFlag = false;
    drawUAVtestposesFlag = false;
    drawPolygonEnvironmentFlag = false;
    drawPolygonRawEnvironmentFlag = false;
    drawVoronoiDiagramFlag = false;
    pursuerX = pursuerY = 0;
    drawAgentSize=true;
    triggerDumpScreenShot = false;
    currentHeightMapMode = HMAP_COLOR_NONE;
    sceneRadius = 100;
    classifiedMap = false;
    showDEMimage = true;
    geoInit = false;
    geoImageInit = false;
    testPlan = NULL;
    _lastMapPureFileName = "";
    _currentPoly = -1;
}

void Viewer::setPointers(HeightMap* map, Visibility* vis, Animator* anim, Perimeter* per)
{
    if (map==NULL) {
        M_ERR("Got empty map pointer!\n");
        return;
    }
    _map = map;
    _vis = vis;
    _anim = anim;
	_per = per;
    // Initialize height map
    terrainLoadFromHeightMap(_map, false, currentHeightMapMode);
    terrainDraw(true, _map->worldOffsetX(),-_map->worldOffsetZ());
}

void Viewer::init()
{
    initGLHelpers();

    glLoadIdentity();

    // Set color for text messages
    setForegroundColor(QColor(255,255,0));

    // Set light for height map
    glEnable(GL_DEPTH_TEST);
    terrainSimulateLighting(0);
    terrainDiffuseColor(1.0, 1.0, 1.0);
    terrainAmbientColor(0.04, 0.04, 0.04);
    GLfloat lAmbient[] = {0.1,0.1,0.1,0.1};
    GLfloat lDiffuse[] = {0.5,0.5,0.5,0.5};
    glLightfv(GL_LIGHT0,GL_AMBIENT,lAmbient);
    glLightfv(GL_LIGHT0,GL_DIFFUSE,lDiffuse);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);

    // Set background color
    glClearColor(0.3, 0.3, 0.5, 1.0);

    prepareHelp();

    setGridIsDrawn(false);
    setAxisIsDrawn(false);

    lastCellSelect = -1;

    // Activate mouse tracking
    setMouseTracking(true);

    //resize(800,600);
    resize(1024,768);
}


void Viewer::draw()
{
    // Clear the color buffer
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (_map==NULL || _vis==NULL || _anim==NULL)
        return;

    // Center map on screen
    static bool first = true;
    if (first) {
        double w=100; double h=100;
        setDrawHeightmapPlain();
        if (_geo) {
            w = _map->resolution() * _geo->width()/2;
            h = _map->resolution() * _geo->height()/2;
        }
        else if (_geoImg) {
            w = _map->resolution() * _geoImg->width()/2;
            h = _map->resolution() * _geoImg->height()/2;
        }
        sceneRadius = std::max(w,h);
        setSceneRadius(sceneRadius);
        setSceneCenter(qglviewer::Vec(w,-h, 0.0));
        showEntireScene();
        first=false;
    }

    double offsX=0;double offsY=0;
    /*if (_map != NULL && _geo != NULL) {
      offsX = _map->resolution() * _geo->width()/2;
      offsY = - _map->resolution() * _geo->height()/2;
   }*/
    if ( _geo != NULL) {
        offsX = _geo->resolution() * _geo->width()/2;
        offsY = - _geo->resolution() * _geo->height()/2;
    }
    else if ( _geoImg != NULL) {
        offsX = _geoImg->resolution() * _geoImg->width()/2;
        offsY = - _geoImg->resolution() * _geoImg->height()/2;
    }

    // Draw geo image or dem image
    if (!showHeightMap) {
        if (showDEMimage && _geo)
        {
            // Initialize geo map
            if (geoInit == false) {
                _geo->init();
                geoInit = true;
            }
            _geo->draw(offsX,offsY);
        }
        else if (_geoImg != NULL)
        {
            if (geoImageInit == false)  {
                _geoImg->init();
                geoImageInit = true;
            }
            _geoImg->draw(offsX,offsY);
        }
    }

    if (showHeightMap)
        drawHeightMap();

    // Draw Agents
    _anim->draw();

    // Draw size indicators
    if (drawAgentSize)
        drawAgentSizes();

    // Draw PE-Graph
    if (drawGraph)
        drawGraphStructure();

    if (drawVisiPoli)
        drawVisibilityPolygon();
    
    if (drawVisiGraphVertex)
        drawVisibilityGraphVertex(v_it);
    
    if (drawVisiGraph)
        drawVisibilityGraph();
    
    if (drawPolygonEnvironmentFlag)
        drawPolygonEnvironment();

    if (drawAllTrajectoriesFlag)
        drawAllTrajectories();
    
    if (drawAllTrajectoriesLinesFlag)
        drawAllTrajectoriesLines();

    if ( drawUAVtestposesFlag )
        draw_uav_test_poses();

    if (drawFrequinPosesFlag)
        drawFrequinPoses();
    
    if (drawSweepStateFlag)
        drawSweepState();

    if (drawPolygonRawEnvironmentFlag)
        drawPolygonRawEnvironment();
    
    if (drawVoronoiDiagramFlag)
        drawVoronoiDiagram();

    if (drawStrategyStepFlag) 
        drawStrategyStep();
    
    if (triggerDumpScreenShot) {
        dumpScreenShot();
        triggerDumpScreenShot=false;
    }

    if (testPlan != NULL) {
        glColor3f(0, 0, 1.0);
        drawPlan(_map,testPlan, 1.0);
    }


    redraw = false;
    //printf(".");
    //fflush(stdout);
}

void Viewer::keyPressEvent(QKeyEvent *e)
{
    bool shift = false;
    bool ctrl = false;
    bool alt = false;

    if ( e->key() & Qt::Key_Shift)
        shift = true;
    if ( e->key() & Qt::Key_Control)
        ctrl = true;
    if ( e->key() & Qt::Key_Alt)
        alt = true;

    switch (e->key())
    {
    case Qt::Key_Q :
        quit_signal = true;
        break;
    case Qt::Key_W :
        drawWireFrame = !drawWireFrame;
        setDrawWireFrame(drawWireFrame);
        break;
    case Qt::Key_Y :
        Params::readConfFile(confFile.c_str());
        M_INFO3("RELOADED CONFIGURATION FILE!\n");
        displayMessage("RELOADED CONFIGURATION FILE");
        break;
    case Qt::Key_G :
        drawGraph = !drawGraph;
        redraw=true;
        break;
    case Qt::Key_L :
        _map->classifyMap();
        classifiedMap = true;
        break;
      case Qt::Key_R:
         {
            if (!classifiedMap) {
               _map->classifyMap();
               classifiedMap = true;
            }
            // Compute map 
            bool graphCreated = _vis->computeRegionSet(_lastMapPureFileName, 
                  Params::g_num_spanning_trees, Params::g_bias_spanning_trees);

            if (graphCreated) 
               _anim->init("",false);
            else {
               // Reload schedule in animator
               if (!Params::g_use_compressed_strategy) {
                  string tt = _lastMapPureFileName + ".sch";
                  _anim->init(tt, true);
               }
               else {
                  string tt2 = _lastMapPureFileName + ".cst";
                  _anim->init(tt2, true);
               }
            }
            setDrawHeightmapRegions(true);
         }
         break;
    case Qt::Key_A:
        if ( _ct != NULL ) {
            updated_cost = _ct->update_costs();
            _ct->_sg->compute();
            _ct->_sg->print();
            this->apply_hungarian();
            _ct->_sg->print();
            int par_cost = _ct->_sg->get_parallel_cost();
            std::cout << " get_parallel_cost " << par_cost << std::endl;
            _ct->_sg->compute_depth_first(updated_cost,par_cost+1);
            //drawFrequinPosesFlag = true;
            drawSweepStateFlag = true;
        }
        break;
    case Qt::Key_T:        
        init_pol();
        break;
    case Qt::Key_F:
        M_INFO3("Toggle option: drawing polygon environment into gui \n");
        drawPolygonEnvironmentFlag = !drawPolygonEnvironmentFlag;
        redraw=true;
        break;
    case Qt::Key_J:
        M_INFO3("Toggle option: drawing raw polygon environment into gui \n");
        drawPolygonRawEnvironmentFlag = !drawPolygonRawEnvironmentFlag;
        redraw=true;
        break;
    case Qt::Key_Z:
        this->load_choice_tree();
        break;
    case Qt::Key_U:
        M_INFO3("Toggle option: drawing Voronoi Diagram \n");
        drawVoronoiDiagramFlag = !drawVoronoiDiagramFlag;
        redraw=true;
        break;
    case Qt::Key_I:
        M_INFO3("Toggle option: drawVisiGraphVertex \n");
        if ( drawVisiGraphVertex == false) {
            drawVisiGraphVertex = true;
            tie(v_it, v_end) = boost::vertices(*(_pol->seg_vis_graph->g));
            redraw = true;
        } else {
            this->toggle_visi_graph_vertex();
        }
        break;
    case Qt::Key_D:
        M_INFO3("Compute line-clear strategy on master polygon \n");
        this->compute_lineclear_strategy();
        break;
    case Qt::Key_C:
        M_INFO3("Next step in strategy \n");
        drawStrategyStepFlag = true;
        redraw=true;
        this->next_step();
        break;
    case Qt::Key_X:
        M_INFO3("Loading strategy \n");
        this->load_strategy();
        break;
    case Qt::Key_K:
        if (_vis->saveGraph(string("/tmp/graph.dot")))
            M_INFO3("Saved graph under %s\n","/tmp/graph.dot");
        break;
	case Qt::Key_P:
			if (!classifiedMap) {
       			_map->classifyMap();
       			classifiedMap = true;
    		}
			_per->computePerimeterSchedule();
			setDrawHeightmapRegions(true);    
			displayMessage("Perimeter Computation Done!");	                
		break;
    case Qt::Key_S :
        triggerDumpScreenShot=true;
        updateGL();
        break;
    case Qt::Key_N :
        this->toggle_visi_poly(true);
        //if (!classifiedMap) {
        //    _map->classifyMap();
        //    classifiedMap = true;
        //}
        //_anim->update();
        //setDrawVisibility(true);
        //updateGL();
        break;
    case Qt::Key_E :
    {
        if (!classifiedMap) {
            _map->classifyMap();
            classifiedMap = true;
        }
        if (!Params::g_use_compressed_strategy) {
            string tt = _lastMapPureFileName + ".sch";
            _anim->init(tt, true);
        }
        else {
            string tt2 = _lastMapPureFileName + ".cst";
            _anim->init(tt2, true);
        }
        string tt3 = _lastMapPureFileName + ".exec";
        _anim->setGeoMapPointer(_geo);
        for (int i=0; i<_anim->getMaxSteps(); i++)
            _anim->update(tt3);
        M_INFO3("Saved exec file under %s\n",tt3.c_str());
    }
        break;
    case Qt::Key_V :
    {
        Timing t("Visibility");
        Visibility::VisSet vset;
        int c = _vis->computeVisibilitySet(pursuerX, pursuerY, vset);
        _vis->setVisiblilityMarkings(vset, true);
        M_INFO3("Visibility size: %d\n",c);
        t.printInfo(true);
        setDrawVisibility(true);
        updateGL();
    }
        break;
    case Qt::Key_Minus :
    {
        if ((sceneRadius*2.0)<=10000.0)
            sceneRadius *= 2.0;
        else
            sceneRadius = 10000.0;
        setSceneRadius(sceneRadius);
        showEntireScene();
        char msg[100];
        sprintf(msg,"Zooming in! New scene radius: %1.2lf",sceneRadius);
        displayMessage(msg);
    }
        break;
    case Qt::Key_Plus:
    {
        if ((sceneRadius/2.0)>=2.0)
            sceneRadius /= 2.0;
        else
            sceneRadius = 2.0;
        setSceneRadius(sceneRadius);
        showEntireScene();
        char msg[100];
        sprintf(msg,"Zooming out! New scene radius: %1.2lf",sceneRadius);
        displayMessage(msg);
    }
        break;
    case Qt::Key_1 :
        setDrawHeightmapPlain();
        break;
    case Qt::Key_2 :
        setDrawHeightmapClasses();
        break;
    case Qt::Key_3 :
        setDrawHeightmapRegions();
        break;
    case Qt::Key_4 :
        setDrawVisibility();
        break;
    case Qt::Key_5 :
        setDrawGeoMapDEMImage();
        break;
    case Qt::Key_6 :
        setDrawGeoMapImage();
        break;
    case Qt::Key_7 :
        drawAllTrajectoriesFlag = !drawAllTrajectoriesFlag;
        break;
    case Qt::Key_8 :
        drawAllTrajectoriesLinesFlag = !drawAllTrajectoriesLinesFlag;
        break;
    case Qt::Key_9 :
        this->toggle_visi_graph();
        break;
    case Qt::Key_M :
        //toggle_draw_up_to();
        //toggle_sweep_state();
        //toggle_current_step();
        //test_visibility_line_cost();
        this->toggle_visi_poly();
        break;
    case Qt::Key_O :
        this->apply_hungarian();
        break;
    case Qt::Key_H :
        M_INFO1("Simple Help:\n");
        M_INFO1("- W - Toggle draw wire frame\n");
        M_INFO1("- S - Dummp screen shot (automatically numbered)\n");
        M_INFO1("- R - Compute RegionSet\n");
        M_INFO1("- K - Save graph from RegionSet\n");
        M_INFO1("- M - Increment current time step by 1 - go trough pose list\n");
        M_INFO1("- A - Build pose list from strategy\n");
        M_INFO1("- Z - Load choice tree\n");
        M_INFO1("- C - Play through strategy with lines\n");
        M_INFO1("- E - Save exec file (for real world experiments)\n");
        M_INFO1("- G - Draw Graph\n");
        M_INFO1("- N - Proceed animation of schedule\n");
        M_INFO1("- V - Compute visibility for current location\n");
        M_INFO1("- Y - Reload configuration file.\n");
        M_INFO1("- A - Compute abstraction\n");
        M_INFO1("- L - Classify the map\n");
        M_INFO1("- T - Build polygonization and simply-connected master polygon\n");
        M_INFO1("- J - Draw polygonization\n");
        M_INFO1("- F - Draw master polygon\n");
        M_INFO1("- D - Compute line-clear strategy (builds choice tree if you do not have one already) \n");
        M_INFO1("- X - Loads a computed strategy \n");
        M_INFO1("- Z - Load choice tree (then compute strategy with D) \n");
        M_INFO1("- U - Draw Voronoi Diagram \n");
        M_INFO1("- 1 - Draw plain map\n");
        M_INFO1("- 2 - Draw classified map\n");
        M_INFO1("- 3 - Draw variances in map\n");
        M_INFO1("- 4 - Draw visibility\n");
        M_INFO1("- 5 - Draw geotiff height map image\n");
        M_INFO1("- 6 - Draw geotiff color image\n");
        M_INFO1("- 7 - Draw drawAllTrajectoriesFlag - M increments pose list\n");
        M_INFO1("- + - Zoom IN\n");
        M_INFO1("- - - Zoom OUT\n");
        M_INFO1("- Cursor - Move left, right, up, down\n");
        M_INFO1("- q - Quit\n");
        break;
    default:
        QGLViewer::keyPressEvent(e);
        break;
    }
}

void Viewer::load_choice_tree() {
    if ( _ct == NULL ) {
        init_env();
        _ct = new lineclear::ChoiceTree( _env, _vis);
        string fname = _lastMapPureFileName + ".ct";
        M_INFO1("Loading choice tree from file %s \n",fname.c_str());
        _ct->load_from_file(fname);
    }
}

void Viewer::init_pol() {
    if (_pol == NULL ) {
        if (!classifiedMap) {
           _map->classifyMap();
           classifiedMap = true;
        }
        M_INFO3("Initializing Polygonization\n");
        // TODO: insert parameter here for a and epsilon 
        double a = Params::g_alpha;
        double e = Params::g_epsilon;
        int gx = Params::g_startx, gy = Params::g_starty;
        _pol = polygonization::polygonize_heightmap( _map, a, e, gx, gy, 
            _lastMapPureFileName );
    }
}

void Viewer::init_env() {
    if ( _env == NULL ) {
        init_pol();
        M_INFO3("Initializing Line-Clear Environment\n");
        _env = new lineclear::Environment( _pol->master_polygon );
    }
}


void Viewer::compute_lineclear_strategy() {
    // check preconditions
    if ( _ct == NULL ) {
        init_env();
        M_INFO1("Using %s.ct as file\n",_lastMapPureFileName.c_str());
        M_INFO1("Creating and initializing first choice tree\n");
        _ct = new lineclear::ChoiceTree( _env, _vis );
        M_INFO1("Initializing choice tree (takes a long time)\n");
        _ct->init_choice_tree();
        save_choice_tree();
    }
    
    M_INFO1("Fetching optimal obstacle sequence\n");
    int first_obstacle;
    obstacle_sequence = _ct->get_optimal_obstacle_sequence(first_obstacle);    
    obstacle_sequence.push_front(first_obstacle);
    obstacle_sequence_it = obstacle_sequence.begin();
    current_cost = 0;
    current_blocking_cost = 0;
    
    M_INFO1("OPTIMAL COST %d \n", _ct->get_optimal_cost());
    
    string fname = _lastMapPureFileName + ".ose";
    M_INFO2("Fetched optimal obstacle sequence and storing it in file %s.\n",fname.c_str());
    ofstream outFile;
    outFile.open(fname.c_str(),ios::out);
    std::list<int>::iterator i = obstacle_sequence.begin();
    M_INFO2("Optimal sequence is:\n");
    while ( i != obstacle_sequence.end() ) {
        std::cout << *i << ",";
        outFile << *i << " ";
        i++;
    }
    std::cout << "final mu"
        << _ct->get_optimal_cut_sequence(first_obstacle)->get_final_cost();
    std::cout << std::endl;
    outFile.close();
}

void Viewer::save_choice_tree() {
    M_INFO1("Saving Choice Tree to file %s.ct\n",_lastMapPureFileName.c_str());
    _ct->save_to_file( _lastMapPureFileName + ".ct" );
}

void Viewer::load_strategy() {
    string filename = _lastMapPureFileName + ".ose";
    ifstream file (filename.c_str(), ios::in);
    if (!file.is_open()) {
        M_ERR("Error: could not open file: %s for reading",filename.c_str());
        return;
    }
    M_INFO2("Reading strategy from file '%s'\n",filename.c_str());

    string line;
    istringstream inStream;

    getline(file, line);
    if ( line.size() < 1) {
        M_ERR("No obstacle sequence found in line: '%s'\n",line.c_str());
        file.close();
        return;
    }
    else {
        inStream.str(line);
        int obstacle_id;
        while (inStream >> obstacle_id) {
            obstacle_sequence.push_back(obstacle_id);
            std::cout << " read in " << obstacle_id << std::endl;
        }
        obstacle_sequence_it = obstacle_sequence.begin();
    }
    
}

void Viewer::test_visibility_line_cost() {
    int o1,o2,o3,o4;
    int map_size = std::min(_map->sizeX(), _map->sizeZ());
    o1 = rand() % map_size;
    o2 = rand() % map_size;
    o3 = rand() % map_size;
    o4 = rand() % map_size;
    //int c = _vis->get_visibility_line_cost(o1,o2,o3,o4);
}

void Viewer::build_graph_from_sequence() 
{
    
}

void Viewer::next_step() {
    // check preconditions
    if ( obstacle_sequence.size() == 0 ) {
        M_INFO2("First computing the strategy");
        this->compute_lineclear_strategy();
    }
    if ( _env == NULL ) {
        init_env();
    }
    
    if ( obstacle_sequence_it == obstacle_sequence.end()) {
        // TODO: restart the animation
        std::cout << " WE ARE DONE " << std::endl;
        return;
    }
    // display split between current obstacles and the new one 
    int new_obstacle = *obstacle_sequence_it;

    if ( cleared_obstacles.size() > 1 ) {
        
        lineclear::Segment o_seg = _env->get_edge( new_obstacle );

        if ( _pol->is_artificial(o_seg) ) {
            std::cout << " obstacle " << new_obstacle << " is artificial. "
                << " o_seg " << o_seg << std::endl;
            int a = _pol->is_artificial2(o_seg.source() );
            //int b = _pol->is_artificial2(o_seg.target() );
            bool have_same = false;
            int up_to_i = int(artificials.size());
            for ( int i = 0; i < up_to_i; i++ ) {
                if ( _pol->is_same_articifial(a,artificials[i]) ) {
                    //is_same_articifial
                    have_same = true;
                    artifical_cost -= artifical_to_cost[artificials[i]];
                    artifical_to_cost.erase(artificials[i]);
                    artificials.erase(artificials.begin() + i);
                    
                    break;
                }
            }
            if ( ! have_same ) {
                // add the cost
                std::cout << " adding cost " << std::endl;
                double d = sqrt(CGAL::to_double(o_seg.squared_length()));
                int art_cost = ceil ( d );
                if ( Params::g_use_new_sensing_range ) {
                    art_cost = ceil ( d / (2*_vis->get_max_steps()) );
                }
                artifical_to_cost[a] = art_cost;
                std::cout << " added " << art_cost;
                artifical_cost += art_cost;
                artificials.push_back(a);
            }
        }
        
        // add a split between the two indices between new_obstacle
        std::list<int>::iterator i = cleared_obstacles.begin();
        std::cout << "Cleared obstacles " << std::endl;
        while ( i != cleared_obstacles.end() ) { 
            std::cout << *i << " ";
            i++;
        }
        std::cout << std::endl;
        
        i = cleared_obstacles.begin();
        int left_obstacle = 0, right_obstacle = 0;
        while ( new_obstacle > *i && i != cleared_obstacles.end() ) { 
            left_obstacle = *i;
            i++;
        }
        bool there_is_no_smaller = false;
        if ( left_obstacle == 0 ) {
            //std::cout << " there is no smaller " << std::endl;
            there_is_no_smaller = true;
            left_obstacle = cleared_obstacles.back();
        } 
        bool there_is_no_larger = false;
        if ( i == cleared_obstacles.end() ) {
            there_is_no_larger = true;
            //std::cout << " there is no larger " << std::endl;
            right_obstacle = cleared_obstacles.front();
        } else {
            right_obstacle = *i;
        }

        // we have smaller and larger obstacle
        std::cout << " extending left= " << left_obstacle
            << " and right=" << right_obstacle << " onto "
            << new_obstacle << std::endl;
        int ext_cost = _env->get_shortest_extension(left_obstacle,
            right_obstacle, new_obstacle,l1,l2);
        
        if ( Params::g_use_new_sensing_range )
            ext_cost = ceil ( ext_cost / (2*_vis->get_max_steps()) );
        
        
        M_INFO2("Extension costs %d", ext_cost);
        //What's the choice set we are extending?
        // i = left_obstacle + 1; k =
        // i-1 = left_obstacle and i+k = right_obstacle
        // i = left_obstacle+1 and k = right_obstacle - left_obstacle-1
        //
        int cs_i = left_obstacle+1;
        int cs_k;
        
        if ( cleared_obstacles.size() == 2 ) {
            cs_k = _env->get_obstacle_number() - 2;
        } else {
            if ( there_is_no_smaller || there_is_no_larger ) {
                // NOTE: that left_obstacle > right_obstacle
                // DUE TO CIRCULAR ORDERING
                cs_k = right_obstacle-1 
                    + _env->get_obstacle_number() - left_obstacle;
            } else {
                //// properly ordered smaller 
                cs_k = right_obstacle - left_obstacle-1;
            }
        }
        std::cout << " Getting choice set i=" << cs_i << " k=" << cs_k 
            << std::endl;
        if ( _ct != NULL ) {
            ChoiceSet* current_cs = _ct->get_choice_set_at(cs_i,cs_k);
            for ( int i = 0; i < current_cs->cut_sequences_size(); i++) {
                std::cout << " CUT SEQ " << i << std::endl;
                int oo;
                CutSequence* cut_seq = current_cs->get_cut_sequence(i, oo);
                std::cout << " CUT SEQ obstacle choice " << oo << std::endl;
                cut_seq->print();
            }
        }
        // remove old block
        std::cout << " removing old block " << std::endl;
        std::pair<int,int> b_p(left_obstacle, right_obstacle);
        blocking_lines_map.erase(b_p);
        int b_removed = _env->get_shortest_line_inside_cost(
            left_obstacle, right_obstacle);
        if ( b_removed != -1 ) {
            if ( Params::g_use_new_sensing_range )
                b_removed = ceil ( b_removed / (2*_vis->get_max_steps()) );
            current_blocking_cost -= b_removed ;
        }

        current_cost = ext_cost + current_blocking_cost + artifical_cost;
        if ( max_cost < current_cost ) {
            max_cost = current_cost;
        }
        std::cout << " COST= " << current_cost << std::endl;
        std::cout << " MAX COST= " << max_cost << std::endl;
        std::cout << " ext_cost= " << ext_cost << std::endl;
        std::cout << " current_blocking_cost= " << current_blocking_cost << std::endl;
        std::cout << " artifical_cost= " << artifical_cost << std::endl;
        std::cout << " just removed block= " << b_removed << std::endl;
        
        
        l3 = _env->get_shortest_line_inside(new_obstacle,right_obstacle);
        if ( l3.target() != l3.source() ) {
            std::pair<int,int> block_between(new_obstacle,right_obstacle);
            blocking_lines_map[block_between] = l3;
            int b_right = _env->get_line_cost(l3);
            if ( Params::g_use_new_sensing_range )
                b_right = ceil ( b_right / (2*_vis->get_max_steps()) );
            
            if ( b_right > 0 ) {
                current_blocking_cost += b_right;
            }
            std::cout << " right_obstacle new block= " << b_right << std::endl;
        }
        l4 = _env->get_shortest_line_inside(left_obstacle,new_obstacle);
        if ( l4.target() != l4.source() ) {
            std::pair<int,int> block_between(left_obstacle, 
                new_obstacle);
            int b_left = _env->get_line_cost(l4);
            if ( Params::g_use_new_sensing_range )
                b_left = ceil ( b_left / (2*_vis->get_max_steps()) );
            
            blocking_lines_map[block_between] = l4;
            if ( b_left > 0 ) {
                current_blocking_cost += b_left;
            }
            std::cout << " left_obstacle new block= " << b_left << std::endl;
        }
        std::cout << " new blocking = " << current_blocking_cost << std::endl;
    } else {
        // we do not yet have enough obstacle indices to draw any lines
    }
    std::cout << " cleared obstacle " << new_obstacle << std::endl;
    cleared_obstacles.push_back(new_obstacle);
    cleared_obstacles.sort();
    obstacle_sequence_it++;
}

void Viewer::prepareHelp()
{
    setKeyDescription(Qt::Key_S, "(And left mouse) Select planning target manually");
    setKeyDescription(Qt::Key_Q, "Quit");
}


QString Viewer::helpString() const
{
    QString text("<h2>Height Map Viewer </h2>");
    text += "This is the viewer help<br><br>";
    return text;
}

void Viewer::drawHeightMap()
{
    // Set color
    glColor3f(1.0, 1.0, 1.0);
    glPointSize(0.5);
    terrainDraw(false, _map->worldOffsetX(),-_map->worldOffsetZ());
}

void Viewer::drawWithNames()
{   
    if (_map != NULL && showHeightMap)
        terrainDrawWithNames(_map->worldOffsetX(),-_map->worldOffsetZ());
}

bool Viewer::loadHeightmapFromTIFF(const string &filename)
{
    if (filename == "")
        return false;

    //if (_geo != NULL)
    //   delete _geo;
    _geo = new GeoMap();

    uint32* dataBuf=NULL;
    uint32* maskDataBuf = NULL;

    _lastMapPureFileName = getPureFilename(filename);

    // Load geotiff file
    if(!_geo->loadGeoTiff(filename)) {
        M_ERR("Error loading geo map\n");
        return false;
    }

    // reload param file
    string fname = _lastMapPureFileName + ".ini";
    Params::readConfFile(fname.c_str());
    string fname_yaml = _lastMapPureFileName + ".yaml";
    Yaml_Config::load_yaml_file_into_param(fname_yaml.c_str());
    M_INFO1("Re-loading configuration file .ini for map");
    //M_INFO1("Loaded yaml_param %f",Yaml_Config::yaml_param["max_ramp_angle"].as<float>());

        
    int w = _geo->width();
    int h = _geo->height();
    double res = _geo->resolution();
    double hres = _geo->heightResolution();
    _geo->getData(dataBuf);

    // Handle map masking
    string maskFile = _lastMapPureFileName;
    maskFile += MASK_FILE_EXTENSIOM;
    GeoMap* mask = new GeoMap();
    if (fileExists(maskFile) && mask->loadGeoTiff(maskFile)) {
        if (mask->width() != w || mask->height() != h || mask->resolution() != res) {
            M_ERR("Mask file %s is not fitting for height map %s\n",maskFile.c_str(), filename.c_str());
        }
        else {
            mask->getData(maskDataBuf);
            M_INFO3("Read mask data from %s\n",maskFile.c_str());
        }
    }
    else {
        M_ERR("ERROR: Cannot find mask file %s. Activating classifier computation.\n",maskFile.c_str());
        Params::g_classify_from_mask_only = 0;
    }

    // Load into height map
    bool inverse = false;
    if (!_map->loadFromTIFF(dataBuf, w, h, res, hres, inverse, maskDataBuf)) {
        M_ERR("Error while initializing height map\n");
        return false;
    }
    //delete mask;
    classifiedMap = false;

    if (_anim != NULL) {
        _map->classifyMap();
        classifiedMap = true;
    }

    return true;
}

double Viewer::get_max_height() 
{
    if ( max_height == 0 ) {
        double height = 0;
        int size_x = _map->sizeX();
        int size_y = _map->sizeZ();
        for (int x=0; x < size_x; x++) {
            for (int y=0; y < size_y; y++) {
                height = _map->getCellsMM()[x][y].getHeight();
                if ( height > max_height ) 
                    max_height = height;
            }
        }
        M_INFO3("Max height computed: %f", max_height);
    }
    return max_height;
}

bool Viewer::loadImageMapFromTIFF(const string &filename)
{
    if (filename == "")
        return false;

    //if (_geoImg != NULL)
    //   delete _geoImg;
    _geoImg = new GeoMap();

    bool ret = _geoImg->loadGeoTiff(filename);
    if (!ret) {
        M_ERR("Error while initializing image map from file %s\n",filename.c_str());
        return false;
    }
    return true;
}


void Viewer::postSelection(const QPoint& point)
{
    (void) point;

    int selected = selectedName();
    if (selected == -1) {
        displayMessage("No object found under mouse cursor\n");
        return;
    }

    printf("selectedName: %d Point: %d %d \n", selectedName(), point.x(), point.y());

    if (lastCellSelect > 0)
        terrainRemoveColoredCell(lastCellSelect);

    terrainAddColoredCell(selected, 1.0, 1.0, 0.0);
    lastCellSelect = selected;

    int gridY = (int) (selected / _map->sizeX());
    int gridX = (int) (selected % _map->sizeX());
    double height = _map->getCellsMM()[gridX][gridY].getHeight() / 1000.0;
    double dist = -1;
    double worldX,worldY;
    _map->grid2world(worldX, worldY, gridX, gridY);

    // Debug output
    if (1) {
        M_INFO1("Selected cell: %d\n",selected);
        M_INFO1("Selected Cell: wx=%lf m, wy=%lf m, cx=%d, xy=%d\n",worldX, worldY, gridX, gridY);
        HeightCell cell = _map->getCellsMM()[gridX][gridY];
        M_INFO1("    height=%lf, sigma = %lf, variance = %lf, class = %d dist=%lf\n",
                height, cell.getSigma(), cell.getVariance(), (int) cell.getClass(), cell.getDistance());
    }

    // Display on screen
    char dummy[1000];
    sprintf(dummy,"Selection: grid --> (%d,%d) world --> (%1.2lf, %1.2lf) dist=%1.2lf height=%1.2lf",gridX,gridY,worldX,worldY,dist,height);
    displayMessage(dummy,5000);

    if (!classifiedMap) {
        _map->classifyMap();
        classifiedMap = true;
    }
    
    if ( line_visibility_test_flag ) {
       line_visibility_test_grid_x1 = gridX;
       line_visibility_test_grid_y1 = gridY;
    } else {
       line_visibility_test_grid_x2 = gridX;
       line_visibility_test_grid_y2 = gridY;
    }
    line_visibility_test_flag = !line_visibility_test_flag;
    if ( !line_visibility_test_flag ) {
        M_INFO1(" no get_max_coverage_pose_on_line\n");
    } else {
        int x_uav_max, y_uav_max;
        Visibility::VisSet covered_poses;
        M_INFO1("get_max_coverage_pose_on_line %d,%d %d,%d \n",
            line_visibility_test_grid_x1,line_visibility_test_grid_y1,
            line_visibility_test_grid_x2,line_visibility_test_grid_y2);
        // gets max uav location
        bool first = true;
        _vis->get_max_coverage_pose_on_line(
                line_visibility_test_grid_x1,line_visibility_test_grid_y1,
                line_visibility_test_grid_x2,line_visibility_test_grid_y2,
                x_uav_max, y_uav_max,
                covered_poses,first );
        // gets the covered_poses set, but changes
        // line_visibility_test_grid_x1,line_visibility_test_grid_y1
        int buffer1 = line_visibility_test_grid_x1;
        int buffer2 = line_visibility_test_grid_y1;
        _vis->get_coverage_poses(x_uav_max, y_uav_max,
                line_visibility_test_grid_x1,line_visibility_test_grid_y1,
                line_visibility_test_grid_x2,line_visibility_test_grid_y2,
                covered_poses );    
        line_visibility_test_grid_x1 = buffer1;
        line_visibility_test_grid_y1 = buffer2;
        _vis->setVisiblilityMarkings(covered_poses,true);
        M_INFO1(" best uav pose size %d,%d\n",x_uav_max, y_uav_max);
        //draw_UAV_at(x_uav_max, y_uav_max);
        M_INFO1(" covered_poses size %d\n",covered_poses.size());
        setDrawVisibility(true);
        uav_test_poses.clear();
        _vis->get_visibility_line_cost(line_visibility_test_grid_x1,
            line_visibility_test_grid_y1,
            line_visibility_test_grid_x2,
            line_visibility_test_grid_y2,
            uav_test_poses);
        drawUAVtestposesFlag = true;
        redraw=true;
    }
    
    //Timing t("Visibility");
    //Visibility::VisSet set;
    //int c = _vis->computeVisibilitySet(gridX, gridY,set);
    //_vis->setVisiblilityMarkings(set, true);
    //M_INFO3("Visibility size: %d\n",c);
    //pursuerX = gridX;
    //pursuerY = gridY;
    //t.printInfo(true);
    //setDrawVisibility(true);
    updateGL();

    // TESTING CIRCLE
    //_vis->tesCircle(100,gridX, gridY);
    //setDrawVisibility(true);

    if (setRobotPose)
    {
        AgentState::robotX = worldX;
        AgentState::robotY = worldY;
        AgentState::robotZ = height;
        setRobotPose = false;
    }
}

void Viewer::setDrawGeoMapDEMImage()
{
    showHeightMap = false;
    showDEMimage = true;
    currentHeightMapMode = HMAP_COLOR_NONE;
    updateGL();
}

void Viewer::setDrawGeoMapImage()
{
    showHeightMap = false;
    showDEMimage = false;
    currentHeightMapMode = HMAP_COLOR_NONE;
    updateGL();
}

void Viewer::setDrawHeightmapClasses()
{
    if (currentHeightMapMode == HMAP_COLOR_CLASS)
        return;
    if (!classifiedMap) {
        _map->classifyMap();
        classifiedMap = true;
    }
    showHeightMap = true;
    terrainLoadFromHeightMap(_map, false, HMAP_COLOR_CLASS);
    terrainDraw(true, _map->worldOffsetX(),-_map->worldOffsetZ());
    displayMessage("Drawing height map CLASSES");
    currentHeightMapMode = HMAP_COLOR_CLASS;
}

void Viewer::setDrawHeightmapPlain()
{
    if (currentHeightMapMode == HMAP_COLOR_PLAIN)
        return;
    showHeightMap = true;
    terrainLoadFromHeightMap(_map, false, HMAP_COLOR_PLAIN);
    terrainDraw(true, _map->worldOffsetX(),-_map->worldOffsetZ());
    displayMessage("Drawing height map PLAIN");
    currentHeightMapMode = HMAP_COLOR_PLAIN;
}

void Viewer::setDrawHeightmapRegions(bool redraw)
{
    if (!redraw && currentHeightMapMode == HMAP_COLOR_REGION_SET)
        return;
    showHeightMap = true;
    terrainLoadFromHeightMap(_map, false, HMAP_COLOR_REGION_SET);
    terrainDraw(true, _map->worldOffsetX(),-_map->worldOffsetZ());
    displayMessage("Drawing height map PARTITIONS");
    currentHeightMapMode = HMAP_COLOR_REGION_SET;
}

void Viewer::setDrawVisibility(bool redraw)
{
    if (!redraw && currentHeightMapMode == HMAP_COLOR_VISIBILITY)
        return;
    showHeightMap = true;
    terrainLoadFromHeightMap(_map, false, HMAP_COLOR_VISIBILITY);
    terrainDraw(true, _map->worldOffsetX(),-_map->worldOffsetZ());
    displayMessage("Drawing height map VISIBILITY");
    currentHeightMapMode = HMAP_COLOR_VISIBILITY;
}

void Viewer::toggle_sweep_state() {
    sweep_state_i++;
    std::cout << " sweep state " << sweep_state_i << std::endl;
    if ( _ct == NULL ) {
        return;
    }
    if ( sweep_state_i == int(_ct->all_lines_in_time.size()) ) {
        sweep_state_i = 0;
    }
    redraw = true;
}

void Viewer::toggle_current_step() {
    current_step_i++;
    std::cout << " current_step_i " << current_step_i << std::endl;
    if ( all_uav_poses.size() == 0 ) {
        return;
    }
    if ( current_step_i == int(all_uav_poses.size()) ) {
        current_step_i = 0;
    }
    redraw = true;
}

void Viewer::toggle_draw_up_to() {
    draw_up_to++;
    if ( _ct == NULL ) {
        return;
    }
    if ( draw_up_to == int(_ct->all_frequin_poses.size()) ) {
        draw_up_to = 0;
    }
    redraw = true;
}

void Viewer::toggle_visi_poly(bool backwards) {
    drawVisiPoli = true;
    if ( backwards )
        visi_poly_index--;
    else
        visi_poly_index++;
    if ( visi_poly_index == -1 ) {
        drawVisiPoli = false;
    }
    if ( visi_poly_index == int( _pol->visi_polies.size() ) ) {
        visi_poly_index = -1;
        drawVisiPoli = false;
    } 
    redraw = true;
}

void Viewer::toggle_visi_graph() {
    drawVisiGraph = !drawVisiGraph;
    redraw = true;
}

void Viewer::toggle_visi_graph_vertex()
{
    v_it++;
    if ( v_it == v_end ) {
        tie(v_it, v_end) = boost::vertices(*(_pol->seg_vis_graph->g));
    }
    
    Segment_Visibility_Graph::vertex v,w;
    v = *v_it;
    int vertex_id = rand() % _pol->seg_vis_graph_type1_vertices.size();
    Segment_Visibility_Graph::mygraph_t* g = _pol->seg_vis_graph->g;
    M_INFO3(" Going from vertex %d to vertex %d\n",(*g)[v].segment_index,vertex_id);
    w = _pol->get_segment_visibility_vertex( vertex_id, 1 );
    shortest_path = _pol->seg_vis_graph->get_shortest_path(v,w);
    
    redraw = true;
}

void Viewer::drawFrequinPoses() {
    if ( _ct == NULL ) 
        return;
    if ( _ct->all_frequin_poses.size() == 0 ) 
        return;
        
    double min_size = 1.0;
    double max_size = 2.0;
    double size_inc = (max_size-min_size) / _ct->all_frequin_poses.size();
    double c_size = min_size;
    for (int i = 0; i < int(_ct->all_frequin_poses.size()); i++) {
        lineclear::Pos_list::iterator it = _ct->all_frequin_poses[i].begin();
        if (draw_up_to != 0 ) {
            if ( i > draw_up_to ) {
                return;
            }
        }
        while ( it != _ct->all_frequin_poses[i].end()) {
            double wx,wy;
            int gx = (*it).x,gy = (*it).y;
            _map->grid2world(wx,wy,gx,gy);
            double ground = 0;
            if ( _map->pointInMap(gx,gy)) {
                ground = _map->getCellsMM()[gx][gy].getHeight()/ 1000.0;
            }
            double h = _vis->getPursuerHeight();
            drawSphere(c_size,wx,wy,ground + h);
            it++;
        }
        c_size += size_inc;
    }
}

void Viewer::draw_uav_test_poses () 
{
    lineclear::Pos_list::iterator it = uav_test_poses.begin();
    while ( it != uav_test_poses.end()) {
        int gx = (*it).x,gy = (*it).y;
        draw_UAV_at(gx,gy);
        it++;
    }
    if ( uav_test_poses.size() > 0 && line_visibility_test_flag ) {
        lineclear::Point p1(line_visibility_test_grid_x1,line_visibility_test_grid_y1);
        lineclear::Point p2(line_visibility_test_grid_x2,line_visibility_test_grid_y2);
        lineclear::Segment seg(p1,p2);
        drawSegment(seg);
    }
}

void Viewer::drawAllTrajectories() {
    int n_steps = all_uav_poses.size();
    int start_at = 1;
    if ( current_step_i != 0 ) {
        n_steps = current_step_i;
        start_at = current_step_i-1;
        if ( start_at == 0 ) 
            start_at = 1;
    }
    int n_uavs = all_uav_poses[0].size();
    //double scale = 
    //    std::min(_map->sizeX(), _map->sizeZ()) * _map->resolution() / 700.0;
    glEnable(GL_LIGHTING);

    for ( int t=start_at; t < n_steps; t++) {
        for (int uav=0; uav < n_uavs; uav++) {
            double wx,wy,w2x,w2y;
            int gx,gy,gx2,gy2;
            gx = all_uav_poses[t-1][uav].x;
            gy = all_uav_poses[t-1][uav].y;
            gx2 = all_uav_poses[t][uav].x;
            gy2 = all_uav_poses[t][uav].y;
            _map->grid2world(wx,wy,gx,gy);
            _map->grid2world(w2x,w2y,gx2,gy2);
            double height1 = 0;
            double height2 = 0;
            if ( _map->pointInMap(gx,gy) && _map->pointInMap(gx2,gy2)) {
                height1 = _map->getCellsMM()[gx][gy].getHeight()/ 1000.0;
                height2 = _map->getCellsMM()[gx2][gy2].getHeight()/ 1000.0;
            }
            if ( !all_uav_poses[t][uav].locked ) {
                //std::cout << " uav " << uav << " not locked " << std::endl;
                glLineWidth(1.0);
                // free roamer
                int tt = t+1;
                while ( tt < int(all_uav_poses.size())
                    && !all_uav_poses[tt][uav].locked ) {
                    tt++;
                }
                if ( tt < int(all_uav_poses.size()) ) {
                    gx2 = all_uav_poses[tt][uav].x;
                    gy2 = all_uav_poses[tt][uav].y;
                }
                _map->grid2world(w2x,w2y,gx2,gy2);
                glColor3f(0.0, 1.0, 0.0);
            } else {
                glLineWidth(2.0);
                glColor3f(1.0, 0.0, 0.0);
            }
            double th_inv = all_uav_poses[t-1][uav].th;
            if ( Params::g_motion_cost_computation == 2 ) {
                double L,r_c,phi_g;
                all_uav_poses[t-1][uav].getArcParams(all_uav_poses[t][uav], L, r_c, phi_g);
                drawArc(wx, wy, th_inv , r_c, L, phi_g, false, 1);
            }
            double h = _vis->getPursuerHeight();
            //glBegin(GL_LINES);
            //  glVertex3f(wx,wy, height1 + h);
            //  glVertex3f(w2x,w2y, height2 + h);
            //glEnd();
            glLineWidth(2.0);
            glBegin(GL_LINES);
              glVertex3f(wx,wy, height1 + h);
              glVertex3f(wx + 0.5*cos(th_inv) ,wy - 0.5*sin(th_inv), height1 + h);
            glEnd();
            double col = ((double) uav)/(((double) n_uavs)-1);
            glColor3f(1.0-col, 0.0, col);
            drawSphere(0.3,wx,wy,height1 + h);
        }
    }
}

void Viewer::drawAllTrajectoriesLines() {
    int n_steps = all_uav_poses.size();
    int start_at = 1;
    if ( current_step_i != 0 ) {
        n_steps = current_step_i;
        start_at = current_step_i-1;
        if ( start_at == 0 ) 
            start_at = 1;
    }
    int n_uavs = all_uav_poses[0].size();
    //double scale = 
    //    std::min(_map->sizeX(), _map->sizeZ()) * _map->resolution() / 700.0;
    glEnable(GL_LIGHTING);

    for ( int t=start_at; t < n_steps-1; t++) {
        for (int uav=0; uav < n_uavs; uav++) {
            double wx,wy,w2x,w2y;
            int gx,gy,gx2,gy2;
            gx = all_uav_poses[t-1][uav].x;
            gy = all_uav_poses[t-1][uav].y;
            gx2 = all_uav_poses[t][uav].x;
            gy2 = all_uav_poses[t][uav].y;
            _map->grid2world(wx,wy,gx,gy);
            _map->grid2world(w2x,w2y,gx2,gy2);
            double height1 = 0;
            double height2 = 0;
            glLineWidth(4.0);
            if ( _map->pointInMap(gx,gy) && _map->pointInMap(gx2,gy2)) {
                height1 = _map->getCellsMM()[gx][gy].getHeight()/ 1000.0;
                height2 = _map->getCellsMM()[gx2][gy2].getHeight()/ 1000.0;
            }
            if ( !all_uav_poses[t][uav].locked ) {
                glLineWidth(2.0);
                int tt = t+1;
                while ( tt < int(all_uav_poses.size())
                    && !all_uav_poses[tt][uav].locked ) {
                    tt++;
                }
                if ( tt < int(all_uav_poses.size()) ) {
                    gx2 = all_uav_poses[tt][uav].x;
                    gy2 = all_uav_poses[tt][uav].y;
                }
                _map->grid2world(w2x,w2y,gx2,gy2);
            } else {
                
            }
            double th_inv = all_uav_poses[t-1][uav].th;
            double h = _vis->getPursuerHeight();
            glBegin(GL_LINES);
              glVertex3f(wx,wy, height1 + h);
              glVertex3f(w2x,w2y, height2 + h);
            glEnd();
            double col = ((double) uav)/(((double) n_uavs)-1);
            glColor3f(1.0 - col, 0.0, col);
            drawSphere(0.01,wx,wy,height1 + h);
        }
    }
}

void Viewer::draw_sphere_at(int x, int y, double h ) {
    if ( !_map->pointInMap(x,y) )
        return;
    double wx,wy; _map->grid2world(wx,wy,x,y);
    double height = _map->getCellsMM()[x][y].getHeight()/ 1000.0;
    glColor3f(0.0, 0.0, 1.0);
    glLineWidth(3.0);
    drawSphere(0.3,wx,wy,height + h);
}

void Viewer::draw_line_from_to(int gx,int gy, int g2x, int g2y, double h)
{
    double wx, wy, w2x,w2y;
    _map->grid2world(wx,wy,gx,gy);
    _map->grid2world(w2x,w2y,g2x,g2y);
    //double height = max(
    //    _map->getCellsMM()[gx][gy].getHeight() / 1000.0,
    //    _map->getCellsMM()[g2x][g2y].getHeight() / 1000.0);
    glBegin(GL_LINES);
     glVertex3f(wx,wy, h);
     glVertex3f(w2x,w2y,  h);
    glEnd();
}

void Viewer::draw_UAV_at(int x, int y) {
    if ( !_map->pointInMap(x,y) )
        return;
    //M_INFO1(" draw uav at %d,%d\n",x, y);
    double wx,wy;
    _map->grid2world(wx,wy,x,y);
    double height = _map->getCellsMM()[x][y].getHeight()/ 1000.0;
    double h = _vis->getPursuerHeight();
    glColor3f(0.0, 0.0, 1.0);
    glLineWidth(3.0);
    drawSphere(0.3,wx,wy,height + h);
}

void Viewer::drawSweepState() {
    if ( _ct == NULL ) 
        return;
    if ( _ct->all_lines_in_time.size() == 0 ) 
        return;
    
    //
    //lineclear::Sweep_state s = _ct->all_lines_in_time[sweep_state_i];
    //
    //// draw all lines of this sweep state;
    //lineclear::Sweep_state::iterator it = s.begin();
    //std::cout << " obstacle pairs with lines between: ";
    //while ( it != s.end() ) {
    //    lineclear::Pos_list::iterator it2 = it->second.begin();
    //    std::cout << " " << it->first.first << ":" << it->first.second;
    //    //draw uav on one line between obstacles  it->first.first
    //    // and obstacle it->first.second
    //    
    //    while ( it2 != it->second.end() ) {
    //        double wx,wy;
    //        int gx = (*it2).x,gy = (*it2).y;
    //        _map->grid2world(wx,wy,gx,gy);
    //        double ground = 0;
    //        if ( _map->pointInMap(gx,gy)) {
    //            ground = _map->getCellsMM()[gx][gy].getHeight()/ 1000.0;
    //        }
    //        drawSphere(2.0,wx,wy,ground);
    //        it2++;
    //    }
    //    it++;
    //}
    
    //int n_steps = all_uav_poses.size();
    int n_uavs = all_uav_poses[sweep_state_i+1].size();
    double scale = 
        std::min(_map->sizeX(), _map->sizeZ()) * _map->resolution() / 1000.0;
    double height = scale * scale * 60;
    
    glEnable(GL_LIGHTING);
    for ( int t=sweep_state_i+1; t < sweep_state_i+2; t++) {
        for (int uav=0; uav < n_uavs; uav++) {
            double wx,wy,w2x,w2y;
            int gx,gy,gx2,gy2;
            gx = all_uav_poses[t-1][uav].x;
            gy = all_uav_poses[t-1][uav].y;
            gx2 = all_uav_poses[t][uav].x;
            gy2 = all_uav_poses[t][uav].y;
            _map->grid2world(wx,wy,gx,gy);
            _map->grid2world(w2x,w2y,gx2,gy2);
            double height1 = 0;
            double height2 = 0;
            if ( _map->pointInMap(gx,gy) && _map->pointInMap(gx2,gy2)) {
                height1 = _map->getCellsMM()[gx][gy].getHeight()/ 1000.0;
                height2 = _map->getCellsMM()[gx2][gy2].getHeight()/ 1000.0;
            }
            drawSphere(0.2,wx,wy,height);
            if ( pow(double(gx-gx2),2) + pow(double(gy-gy2),2) > 1000 ) {
                glColor3f(1.0, 1.0, 0.0);
                glLineWidth(0.5);
            } else {
                glColor3f(1.0, 0.0, 0.0);
                glLineWidth(3.0);
            }
            glBegin(GL_LINES);
             glVertex3f(wx,wy, height);
             glVertex3f(w2x,w2y, height);
            glEnd();
        }
    }
}

void Viewer::apply_hungarian() {
    int numAgents = updated_cost;    
    int numSteps = _ct->all_lines_in_time.size();
    vector<int> cost_correction(numAgents);
    for (int p=0; p<numAgents; p++) {
        cost_correction[p] = 0;
    }
    vector<NavPoint> current_poses(numAgents);
    all_uav_poses.resize(numSteps+1,current_poses);
    
    M_INFO1_D(DEBUG_HUNGARIAN,1,"ASSIG. %d STEPS %d UAVS\n",numSteps,numAgents);
    std::pair<int,int> last_line_at, line_at;
    vector<bool> obstacle_appeared(_env->get_obstacle_number()+1);
    for ( int i = 0; i < obstacle_appeared.size(); i++ ) {
        obstacle_appeared[i] = false;
    }
    
    //for ( int n_ob = 1 ; n_ob <= _env->get_obstacle_number() ; n_ob++ ) {
    //    M_INFO1_D(DEBUG_HUNGARIAN,1,"START OF obst %d \n",n_ob);
    //    M_INFO1_D(DEBUG_HUNGARIAN,1,"START AT step %d \n",_ct->start_step_for_split_obstacle[n_ob]);
    //}
    
    float time_since_new_split = 0;
    int last_split = 0;
    for ( int i = 0; i < numSteps; i++ ) {
        M_INFO1_D(DEBUG_HUNGARIAN,1,"Next sweep state - counting targets \n");
        lineclear::Sweep_state s = _ct->all_lines_in_time[i];
        int numTargets = 0;
        lineclear::Sweep_state::iterator it = s.begin();
        
        for ( int n_ob = 1 ; n_ob <= _env->get_obstacle_number() ; n_ob++ ) {
            if ( i == _ct->start_step_for_split_obstacle[n_ob] ) {
                M_INFO1_D(DEBUG_HUNGARIAN,1,"time_since_new_split %f \n",time_since_new_split);
                M_INFO1_D(DEBUG_HUNGARIAN,1,"FOUND START OF obst %d \n",n_ob);
                M_INFO1_D(DEBUG_HUNGARIAN,1,"FOUND START AT step %d \n",i);
                if ( last_split > 0 ) {
                    _ct->_sg->nodes[last_split].time = time_since_new_split;
                }
                last_split = n_ob;
                time_since_new_split = 0;
                break;
            }
        }
        
        while ( it != s.end() ) {
            M_INFO1_D(DEBUG_HUNGARIAN,2,"Next line \n");
            line_at = it->first;
            // first time the obstacle appeared is the time of the split
            if ( obstacle_appeared[line_at.first] == false ) {
                obstacle_appeared[line_at.first] = true;
            }
            if ( obstacle_appeared[line_at.second] == false ) {
                obstacle_appeared[line_at.second] = true;
            }
            
            lineclear::Pos_list::iterator it2 = it->second.begin();
            while ( it2 != it->second.end() ) {
                numTargets++;
                it2++;
            }
            it++;
        }
        
        bool poses_are_on_new_line = false;
        if ( last_line_at != line_at ) {
            M_INFO1_D(DEBUG_HUNGARIAN,1,"NEW LINE IN ALL POSES\n");
            M_INFO1_D(DEBUG_HUNGARIAN,1,"Last Line %d-%d Line %d-%d \n",last_line_at.first,last_line_at.second,line_at.first,line_at.second);
            poses_are_on_new_line = true;
        }
        
        M_INFO1_D(DEBUG_HUNGARIAN,1," Targets %d \n",numTargets);
        if ( numTargets == 0 )
            continue;
        
        M_INFO1_D(DEBUG_HUNGARIAN,1,"Time %d with %d targets\n",i,numTargets);
        M_INFO1_D(DEBUG_HUNGARIAN,1," Initializing costs and assignment\n");
        vector<double> dummy(numTargets);
        vector< vector <double> > costs(numAgents, dummy);
        vector<bool> dummy2(numTargets);
        vector< vector <bool> > assignment(numAgents, dummy2);
        for (int d=0; d<numTargets; d++) {
           for (int p=0; p<numAgents; p++) {
               costs[p][d] = 0;
               assignment[p][d] = false;
           }
        }
        
        M_INFO1_D(DEBUG_HUNGARIAN,1," Initializing uav poses\n");
        for ( int p = 0; p < numAgents; p++ ) {
            all_uav_poses[i][p].tv = Params::g_UAV_max_velocity;
        }
        
        M_INFO1_D(DEBUG_HUNGARIAN,1," Computing cost values\n");
        vector<NavPoint> new_poses;
        it = s.begin();
        int d = 0;
        while ( it != s.end() ) {
            lineclear::Pos_list::iterator it2 = it->second.begin();
            while ( it2 != it->second.end() ) {
                double wx,wy;
                int gx = (*it2).x,gy = (*it2).y;
                _map->grid2world(wx,wy,gx,gy);
                double ground = 0;
                if ( _map->pointInMap(gx,gy) ) {
                    ground = _map->getCellsMM()[gx][gy].getHeight()/ 1000.0;
                }
                NavPoint sample_point(double(gx), double(gy), ground, 0);
                new_poses.push_back(sample_point);
                for ( int p = 0; p < numAgents; p++ ) {
                    double c = 0;
                    if ( Params::g_motion_cost_computation == 1 ) {
                        c = sample_point.getDistToPoint(all_uav_poses[i][p]);
                    } else if ( Params::g_motion_cost_computation == 2 ) {
                        c = all_uav_poses[i][p].getCostToPoint( sample_point );
                    }
                    //TODO: how do we make sure that only new locations
                    // are using new robots that join the lines
                    if ( poses_are_on_new_line ) {
                        costs[p][d] = std::max(c-cost_correction[p],0.0);
                    } else {
                        // poses are on a line that has already
                        // been travelling assigned yet
                        if ( all_uav_poses[i][p].locked ) 
                            costs[p][d] = std::max(c,0.0);
                        else 
                            costs[p][d] = std::max(1000 + c-cost_correction[p],0.0);
                    }
                }
                d++;
                it2++;
            }
            it++;
        }
        
        double ret = computeHungarianAssignment2(costs, assignment);
        M_INFO1_D(DEBUG_HUNGARIAN,3,"Result: %1.2lf\n", ret);
        double max_sqrt_dist = 0;
        double max_time = 0;
        double max_time_pure = 0; // max_time without unlocked agents
        for (int p=0; p<numAgents; p++) {
            bool uav_moved = false;
            bool uav_locked = false;
            for (int d=0; d<numTargets; d++) {
                if ( assignment[p][d] ) {
                    // put uav p to location d
                    if ( DEBUG_HUNGARIAN > 2 ) {
                        std::cout << " uav " << p
                            << " from " << all_uav_poses[i][p].x
                            << "," << all_uav_poses[i][p].y
                            << " to " << new_poses[d].x 
                            << "," << new_poses[d].y 
                            << " at cost " << costs[p][d]
                            << " target " << d
                            << std::endl;
                    }
                    all_uav_poses[i+1][p] = new_poses[d];
                    if ( Params::g_motion_cost_computation == 2 ) {
                        all_uav_poses[i][p].getCostToPoint(
                            all_uav_poses[i+1][p] );
                    }
                    double xline 
                        = all_uav_poses[i+1][p].x - all_uav_poses[i][p].x;
                    double yline 
                        = all_uav_poses[i+1][p].y - all_uav_poses[i][p].y;
                    double sqrt_dist 
                        = all_uav_poses[i+1][p].getSqrDistToPoint( 
                            all_uav_poses[i][p]);
                    double the_time = costs[p][d];
                    if ( sqrt_dist >= 4 ) {
                        if ( Params::g_motion_cost_computation == 1 ) {
                            all_uav_poses[i+1][p].th = 
                                norm_angle(atan2(yline,xline));
                        }
                        //M_INFO1_D(DEBUG_HUNGARIAN,2,"up. th %1.2lf\n",all_uav_poses[i+1][p].th);
                        uav_moved = true;
                        if ( sqrt_dist > max_sqrt_dist ) {
                            max_sqrt_dist = sqrt_dist;
                        }
                        if ( the_time > max_time) {
                            max_time = the_time;
                        }
                        if ( i > 0
                          && all_uav_poses[i][p].locked 
                          && the_time > max_time_pure) {
                              max_time_pure = the_time;
                              std::cout << " MAX TIME  " << max_time_pure << std::endl;
                        } else if ( the_time > max_time_pure ) {
                            std::cout << " MAX TIME IGNORED " << std::endl;
                        }
                    }
                    uav_locked = true;
                }
           }
           if ( !uav_moved ) {
               all_uav_poses[i+1][p] = all_uav_poses[i][p];
           }
           all_uav_poses[i+1][p].locked = uav_locked;
        }
        //adjust cost by max_time
        time_since_new_split += max_time_pure;
        for (int p=0; p<numAgents; p++) {
            if ( !all_uav_poses[i+1][p].locked ) {
                cost_correction[p] += (max_time / 2);
            } else {
                cost_correction[p] = 0;
            }
        }
        last_line_at = line_at;
    }
    
    // save in file
    string fname = _lastMapPureFileName + ".pos";
    std::ofstream os(fname.c_str());
    if (!os.good()) {
       M_ERR("Cannot open file for writing %s\n",fname.c_str());
    }
    //vector< vector<NavPoint> > all_uav_poses;
    os << numAgents << " " << all_uav_poses.size() << std::endl;
    int start_at_i=0;
    for ( int i = 0; i < int(all_uav_poses.size()); i++ ) {
        for (int p=0; p<numAgents; p++) {
            if (all_uav_poses[i][p].locked) {
                start_at_i = i;
                i = int(all_uav_poses.size());
                break;
            }
        }
    }
    for ( int i = start_at_i; i < int(all_uav_poses.size()); i++ ) {
        for (int p=0; p<numAgents; p++) {
            if ( all_uav_poses[i][p].y > 0 && all_uav_poses[i][p].x > 0 )
            {
                all_uav_poses[i][p].z =
                    _map->getCellsMM()[int(all_uav_poses[i][p].x)][int(all_uav_poses[i][p].y)].getHeight()/1000.0
                  + _vis->getPursuerHeight() + p/3;
            }
            os << all_uav_poses[i][p].locked << " ";
            os << all_uav_poses[i][p].x << " ";
            os << all_uav_poses[i][p].y << " ";
            os << all_uav_poses[i][p].z << " ";
            os << all_uav_poses[i][p].th;
            if ( numAgents != p-1 ) 
                os << " ";
        }
        os << std::endl;;
    }
    os.close();
    
    string fname2 = _lastMapPureFileName + ".launch";
    std::ofstream os2(fname2.c_str());
    if (!os2.good()) {
       M_ERR("Cannot open file for writing %s\n",fname2.c_str());
    }
    
    for (int p=0; p<numAgents; p++) {
        double x,y,z;
        for ( int i = start_at_i; i < int(all_uav_poses.size()); i++ ) {
            if ( all_uav_poses[i][p].locked ) {
                x = all_uav_poses[i][p].x * _map->resolution();
                y = all_uav_poses[i][p].y * _map->resolution();
                z = all_uav_poses[i][p].z;
                break;
            }
        }
        char buffer [500];
        sprintf (buffer, "<group ns=\"drone%d\"><param name=\"tf_prefix\" value=\"drone%d\" /><include file=\"$(find drone_hri_launch)/launch/spawn_iros_quadrotor.launch\" ><arg name=\"prefix\"  value=\"drone%d\" /><arg name=\"start_position\" value=\"-x %f -y %f -z %f\" /></include></group>", p, p, p, x,y,z);
        os2 << buffer << std::endl;
    }
    os2.close();

    drawAllTrajectoriesFlag = false;
    return;
}

void Viewer::drawPolygonEnvironment() {
    if ( _pol == NULL ) {
        M_INFO3("No Polygon Environment there to draw \n");
        return;
    }
    
    //glLineWidth(2.0);
    //glColor3f(0.5, 0.0, 0.0);
    //for ( unsigned int i = 0; i < _pol->size(); i++ ) {
    //    // polygons are of type polygonization polygon
    //    M_INFO3("Drawing poly %d \n",i+1);
    //    polygonization::Polygon* poly = &((*_pol)[i]);
    //    this->drawPoly( poly );
    //}
    glEnable(GL_LIGHTING);
    glColor3f(1.0, 1.0, 0.0);
    if ( _pol->master_polygon != NULL ) 
        this->drawPoly( _pol->master_polygon );
    
    //glEnable(GL_LIGHTING);
}

void Viewer::drawPolygonRawEnvironment() {
    if ( _pol == NULL ) {
        M_INFO3("No Polygon Environment there to draw \n");
        return;
    }
    
    //glEnable(GL_LIGHTING);
    glLineWidth(1.0);
    glColor3f(0.3, 1.0, 0.0);
    for ( unsigned int i = 0; i < _pol->size(); i++ ) {
        //M_INFO3("Drawing poly %d \n",i+1);
        polygonization::Polygon* poly = &((*_pol)[i]);
        this->drawPoly( poly );
    }
}

void Viewer::drawPoly( polygonization::Polygon *poly ) {
    glLineWidth(3.0);
    double height = this->get_max_height() / 1000.0;
    for ( unsigned int j = 0; j < poly->size(); j++ ) {
        polygonization::Segment s = poly->edge(j);
        
        std::list<int>::iterator it;
        it = std::find(cleared_obstacles.begin(),cleared_obstacles.end(),j+1);
        if ( it == cleared_obstacles.end() ) {
            glColor3f(1.0, 0.0, 0.0);
        } else {
            glColor3f(0.0, 1.0, 0.0);
        }
        
        double wx,wy,w2x,w2y;
        int gx = ceil( CGAL::to_double(s.source().x())); 
        int gy = ceil( CGAL::to_double(s.source().y()));
        int g2x = ceil(CGAL::to_double(s.target().x()));
        int g2y = ceil(CGAL::to_double(s.target().y()));
        _map->grid2world(wx,wy,gx,gy);
        _map->grid2world(w2x,w2y,g2x,g2y);
        drawSphere(0.3,wx,wy,height);
        glBegin(GL_LINES);
         glVertex3f(wx,wy, height);
         glVertex3f(w2x,w2y, height);
        glEnd();
    }
}

void Viewer::drawVisibilityGraph()
{
    //
    //(*(_env->seg_vis_graph->g))::
    boost::graph_traits<Segment_Visibility_Graph::mygraph_t>::vertex_iterator 
        vi, vi_end;
    tie(vi, vi_end) = boost::vertices(*(_pol->seg_vis_graph->g));
    for (; vi != vi_end; ++vi)
    {
        boost::adjacent_vertices(*vi,*(_pol->seg_vis_graph->g));
    }
    boost::graph_traits<Segment_Visibility_Graph::mygraph_t>::edge_iterator 
        ei, ei_end;
    tie(ei, ei_end) = boost::edges(*(_pol->seg_vis_graph->g));
    Segment_Visibility_Graph::mygraph_t* g = _pol->seg_vis_graph->g;

    glColor3f(0.0, 1.0, 0.0);
    glLineWidth(1.0);
    
    for (; ei != ei_end; ++ei)
    {
        Segment_Visibility_Graph::mygraph_t::vertex_descriptor v_source
             = boost::source(*ei,*g);
        Segment_Visibility_Graph::mygraph_t::vertex_descriptor v_target
             = boost::target(*ei,*g);
        if ( (*g)[ v_source ].type == 2 || (*g)[ v_target ].type == 2 )
        {
            
        } else { continue; }
        this->draw_line_from_to(
            (*g)[ v_source ].p_x,
            (*g)[ v_source ].p_y,
            (*g)[ v_target ].p_x,
            (*g)[ v_target ].p_y,
            13.0);
        //drawText
        
    }
}

void Viewer::drawVisibilityGraphVertex(Segment_Visibility_Graph::mygraph_t::vertex_iterator v_it)
{
    Segment_Visibility_Graph::mygraph_t* g = _pol->seg_vis_graph->g;
    
    
    std::list<Segment_Visibility_Graph::vertex>::iterator spi = shortest_path.begin();
    double last_x = (*g)[ *v_it ].p_x;
    double last_y = (*g)[ *v_it ].p_y;

    for(++spi; spi != shortest_path.end(); ++spi) 
    {
        glLineWidth(3.0);
        glColor3f(1.0, 1.0, 0.0);
        this->draw_line_from_to(
            last_x,
            last_y,
            (*g)[ *spi ].p_x,
            (*g)[ *spi ].p_y,
            13.0);
        //renderText(100,150,"222");
        //drawText(last_x,last_y,"test");
        last_x = (*g)[ *spi ].p_x;
        last_y = (*g)[ *spi ].p_y;
    }
    
    glColor3f(0.0, 1.0, 0.0);
    glLineWidth(1.0);
    
    Segment_Visibility_Graph::mygraph_t::out_edge_iterator 
        ei, ei_end;
    tie(ei, ei_end) = boost::out_edges(*v_it,*g);
    M_INFO2("Edges of Vertex %d type %d\n", 
        (*g)[*v_it].segment_index, (*g)[*v_it].type);
    for (; ei != ei_end; ++ei)
    {
        Segment_Visibility_Graph::mygraph_t::vertex_descriptor v_source
             = boost::source(*ei,*g);
        Segment_Visibility_Graph::mygraph_t::vertex_descriptor v_target
             = boost::target(*ei,*g);
        if ( (*g)[ v_source ].type == 2 || (*g)[ v_target ].type == 2 )
        {
            glColor3f(0.0, 1.0, 0.0);
            glLineWidth(2.0);
        } else { 
            glColor3f(1.0, 0.5, 1.0);
            glLineWidth(2.0);
        }
        
        M_INFO2("Edges to vertex %d of type %d at %f distance \n", (*g)[v_target].segment_index, (*g)[v_target].type, (*g)[*ei].distance);
        this->draw_line_from_to(
            (*g)[ v_source ].p_x,
            (*g)[ v_source ].p_y,
            (*g)[ v_target ].p_x,
            (*g)[ v_target ].p_y,
            13.0);
    }
}

void Viewer::drawVisibilityPolygon()
{
    VisiLibity::Point poin = _pol->get_visi_vertex(visi_poly_index);
    int x = ceil(poin.x()); int y = ceil(poin.y());
    if ( !_map->pointInMap(x,y) )
        return;
    double wx,wy;
    _map->grid2world(wx,wy,x,y);
    double height = _map->getCellsMM()[x][y].getHeight()/ 1000.0;
    double h = _vis->getPursuerHeight() + 1.0;
    glColor3f(0.0, 1.0, 1.0);
    glLineWidth(3.0);
    drawSphere(0.3,wx,wy,height + h);
    
    for (int j = 1; j < _pol->visi_polies[visi_poly_index].n(); j++ ) {
        this->drawLine(
            _pol->visi_polies[visi_poly_index][j-1].x(),_pol->visi_polies[visi_poly_index][j-1].y(),
            _pol->visi_polies[visi_poly_index][j].x(),_pol->visi_polies[visi_poly_index][j].y()
        );
    }
    int j = _pol->visi_polies[visi_poly_index].n();
    this->drawLine(
        _pol->visi_polies[visi_poly_index][j-1].x(),_pol->visi_polies[visi_poly_index][j-1].y(),
        _pol->visi_polies[visi_poly_index][0].x(),_pol->visi_polies[visi_poly_index][0].y()
    );
}

void Viewer::drawStrategyStep() 
{
    // draw the proper lines for the step in the strategy
    //l1,l2,l3,l4
    glEnable(GL_LIGHTING);
    glLineWidth(12.0);
    glColor3f(1.0, 1.0, 0.0);
    //std::cout << " Drawing strategy step " << std::endl;
    if ( l1.target() != l1.source() )
        this->drawSegment(l1);
    if ( l2.target() != l2.source() )
        this->drawSegment(l2);
    
    //std::cout << " Drawing blocking lines " << std::endl;
    glLineWidth(8.0);
    glColor3f(0.0, 0.0, 1.0);
    std::map< std::pair<int,int>,lineclear::Segment>::iterator i;
    i = blocking_lines_map.begin();
    while ( i != blocking_lines_map.end() ) {
        drawSegment( i->second );
        i++;
    }
}

void Viewer::drawSegment(lineclear::Segment s) {
    double ground = this->get_max_height() / 1000.0;
    double wx,wy,w2x,w2y;
    int gx = ceil( CGAL::to_double(s.source().x())); 
    int gy = ceil( CGAL::to_double(s.source().y()));
    int g2x = ceil(CGAL::to_double(s.target().x()));
    int g2y = ceil(CGAL::to_double(s.target().y()));
    ground = max(
        _map->getCellsMM()[gx][gy].getHeight() / 1000.0,
        _map->getCellsMM()[g2x][g2y].getHeight() / 1000.0);
    _map->grid2world(wx,wy,gx,gy);
    _map->grid2world(w2x,w2y,g2x,g2y);
    drawSphere(0.1,wx,wy,ground);
    glBegin(GL_LINES);
     glVertex3f(wx,wy, ground);
     glVertex3f(w2x,w2y, ground);
    glEnd();
}

void Viewer::drawLine(double p1x,double p1y, double p2x, double p2y, double ground)
{
    ground = this->get_max_height() / 1000.0;
    double wx,wy,w2x,w2y;
    int gx = ceil( p1x), gy = ceil( p1y);
    int g2x = ceil(p2x), g2y = ceil(p2y);
    //ground = max(
    //    _map->getCellsMM()[gx][gy].getHeight() / 1000.0,
    //    _map->getCellsMM()[g2x][g2y].getHeight() / 1000.0);
    _map->grid2world(wx,wy,gx,gy);
    _map->grid2world(w2x,w2y,g2x,g2y);
    drawSphere(0.1,wx,wy,ground);
    glBegin(GL_LINES);
     glVertex3f(wx,wy, ground);
     glVertex3f(w2x,w2y, ground);
    glEnd();
}

void Viewer::drawGraphStructure()
{
    double scale = std::min(_map->sizeX(), _map->sizeZ()) * _map->resolution() / 1000.0;
    double rad = 2 * scale;
    double height = 1000.0 * scale * scale;

    glEnable(GL_LIGHTING);
    // Veritces from Perimeter
    for (unsigned int i=0; i<_per->vertices.size(); i++) 
    {
		double value = _per->vertices[i].percentage;
	    glColor3f(0.0, 1.0, 1.0);
        int gx = _per->vertices[i].pos.x;
        int gy = _per->vertices[i].pos.y;
        double ground = _map->getCellsMM()[gx][gy].getHeight()/ 1000.0;
        double wx,wy;
        _map->grid2world(wx,wy,gx,gy);
        drawSphere(rad,wx,wy,ground + height);
        glBegin(GL_LINES);
         glVertex3f(wx, wy, ground);
         glVertex3f(wx, wy, ground + height);
        glEnd();

	    glColor3f(0.0,value,0);
	    glPushMatrix();
	    glTranslatef(wx, wy, ground + 2*height);
	    drawBox(1, 1,  0.5);
	    glPopMatrix();
    }

    // Veritces
    for (unsigned int i=0; i<_vis->vertices.size(); i++) {
        int gx = _vis->vertices[i].pos.x;
        int gy = _vis->vertices[i].pos.y;
        double ground = _map->getCellsMM()[gx][gy].getHeight()/ 1000.0;
        double wx,wy;
        _map->grid2world(wx,wy,gx,gy);
        if ((int)i==_currentPoly)
            glColor3f(0.0, 0.0, 1.0);
        else
            glColor3f(0.0, 0.0, 0.0);
        drawSphere(rad,wx,wy,ground + height);
        if ((int)i==_currentPoly)
            glColor3f(0.0, 0.0, 1.0);
        else
            glColor3f(0.0, 0.0, 0.0);
        glBegin(GL_LINES);
        glVertex3f(wx, wy, ground);
        glVertex3f(wx, wy, ground + height);
        glEnd();
    }
    // Edges
    glLineWidth(2.0);
    glDisable(GL_LIGHTING);
    for (set<Visibility::Edge>::iterator it=_vis->edges.begin(); it != _vis->edges.end(); it++) {

        Visibility::Pos p1(0,0);
        for (unsigned int i=0; i<_vis->vertices.size(); i++) {
            if (_vis->vertices[i].id == it->id1){
                p1 = _vis->vertices[i].pos;
                break;
            }
        }
        Visibility::Pos p2(0,0);
        for (unsigned int i=0; i<_vis->vertices.size(); i++) {
            if (_vis->vertices[i].id == it->id2){
                p2 = _vis->vertices[i].pos;
                break;
            }
        }

        //glColor3f(0.5, 0.5, 0.5);

        if (it->shady) {
            //continue;
            glLineWidth(1.0);
            glColor3f(8.0, 0.0, 0.0);
        }
        else {
            glLineWidth(2.0);
            //glColor3f(0.0, 0.8, 0.0);
            glColor3f(1.0, 1.0, 1.0);
        }

        glBegin(GL_LINES);
        double wx,wy;
        _map->grid2world(wx,wy,p1.x,p1.y);
        double h1 = height + _map->getCellsMM()[p1.x][p1.y].getHeight()/ 1000.0;
        glVertex3f(wx, wy, h1);
        _map->grid2world(wx,wy,p2.x,p2.y);
        double h2 = height + _map->getCellsMM()[p2.x][p2.y].getHeight()/ 1000.0;
        glVertex3f(wx, wy, h2);
        glEnd();
    }
    glEnable(GL_LIGHTING);
}

void Viewer::setDrawWireFrame(bool on) {
    if (on) {
        glDisable(GL_LIGHTING);
        glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);
        glEnable(GL_POLYGON_OFFSET_LINE);
        displayMessage("Showing wire frame ON");
    }
    else {
        glEnable(GL_LIGHTING);
        glDisable(GL_POLYGON_OFFSET_LINE);
        glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
        displayMessage("Showing wire frame OFF");
    }
    terrainDraw(true, _map->worldOffsetX(),-_map->worldOffsetZ());
}

void Viewer::triggerSetRobotPose()
{
    setRobotPose = true;
    displayMessage("Select cell for robot cell by pressing the left mouse button");
}

std::string formatString(const char* fmt, ...)
{
    char* auxPtr = NULL;
    va_list arg_list;
    va_start(arg_list, fmt);
    int numChar = vasprintf(&auxPtr, fmt, arg_list);
    va_end(arg_list);
    string retString;
    if (numChar != -1)
        retString = auxPtr;
    else
        printf("Error while allocating memory\n");
    free(auxPtr);
    return retString;
}

// Screen shot dumping for video
void Viewer::dumpScreenShot()
{
    static int dumpCount = 0;
    static unsigned char* imgData = NULL;
    static unsigned int imgDataSizeX = 0;
    static unsigned int imgDataSizeY = 0;


    GLint size[4];
    glGetIntegerv(GL_VIEWPORT, size);
    delete [] imgData;
    imgDataSizeX = size[2];
    imgDataSizeY = size[3];
    imgData = new unsigned char[size[2]*size[3]*3];
    glReadPixels( 0, 0, size[2], size[3], GL_RGB, GL_UNSIGNED_BYTE, imgData );

    unsigned char* pixels = imgData;
    int width = size[2];
    int height = size[3];
    bool mirror = true;

    string filename = formatString("dump-%.6ld.jpg", dumpCount++);

    // yeehaw, opengl has first line at bottom, images have first line at top

    double quality = 100.0;
    if (mirror) {
        unsigned char* pixelsMirror = new unsigned char[width * height * 3];

        int count = 0;
        for(int i = height - 1; i >= 0; i--) {
            for(int j = 0; j < width; j++) {
                for(int k = 0; k < 3; k++) {
                    pixelsMirror[count++] = pixels[ i * width * 3 + j * 3 + k];
                }
            }
        }
        JpegImage ji = JpegImage::fromRGB(width, height, pixelsMirror, quality);
        ji.save(filename);
        //printf("writing %s\n",filename.c_str());
        delete [] pixelsMirror;
    }
    else {
        JpegImage ji = JpegImage::fromRGB(width, height, pixels, quality);
        ji.save(filename);
    }
    M_INFO3("Saved screen shot under %s\n",filename.c_str());
}

void Viewer::drawAgentSizes()
{
    if (_map == NULL)
        return;

    // Draw pursuer
    if (!_map->pointInMap(pursuerX,pursuerY)) {
        DO_EVERY(1,M_WARN("Pursuer not on map!\n"));
        return;
    }
    double wx,wy;
    _map->grid2world(wx, wy, pursuerX, pursuerY);
    double h = _vis->getPursuerHeight();
    double l = _map->resolution();
    double wz = _map->getCellsMM()[pursuerX][pursuerY].getHeight()/ 1000.0;
    if (!showHeightMap) {
        wz = 0.1;
        h = 100.0;
    }
    //DO_EVERY(5.0,printf("DRAWING PURSUER AT %lf   %lf  %lf with h= %lf   and l=%lf\n",wx, wy, wz, h, l));
    glColor3f(1.0,1.0,0);
    glPushMatrix();
    glTranslatef(wx, wy, wz + h);
    drawBox(l, l,  h);
    glPopMatrix();
    //drawSphere(5, wx, wy, wz + 10);

    // Draw target
    if (!_map->pointInMap(0,0))
        return;

    _map->grid2world(wx, wy, 0,0);
    wz = _map->getCellsMM()[0][0].getHeight()/ 1000.0;
    h = _vis->getTargetHeight();
    l = _map->resolution();
    glColor3f(0,.1,0);
    glPushMatrix();
    glTranslatef(wx, wy, wz + h);
    drawBox(l, l,  h);
    glPopMatrix();

}

void Viewer::setPursuerPosFromUTM(double easting, double northing)
{
    double wx = easting;
    double wy = northing;
    if (_geo)
        _geo->world2image(&wx, &wy);
    else if (_geoImg)
        _geoImg->world2image(&wx, &wy);
    else
        return;
    int x,y;
    //x = (int)wx;
    //y = (int)wy;
    _map->world2grid(wx, -wy, x, y);

    DO_EVERY(1,printf("Setting pursuer to %d %d  %lf %lf\n",x,y, easting, northing));
    pursuerX = x;
    pursuerY = y;
    updateGL();
}

void Viewer::drawVoronoiDiagram() {
    polygonization::Voronoi_Diagram* vd = _pol->VD;
    polygonization::Voronoi_Diagram::Edge_iterator e_i;
    glEnable(GL_LIGHTING);
    glLineWidth(1.0);
    glColor3f(1.0, 1.0, 0.0);
    M_INFO3("Drawing Voronoi Diagram");
    e_i = vd->edges_begin();
    for ( ; e_i != vd->edges_end(); ++e_i ) 
    {
        if ( e_i->has_source() && e_i->has_target() ) 
        {
            lineclear::Segment s(e_i->source()->point(),
                e_i->target()->point());
            //std::cout << s << std::endl;
            drawSegment(s);
        }
    }
}
