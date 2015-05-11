#include "lineclear/CGALTypes.h"
#include "lineclear/Environment.h"
#include "lineclear/ChoiceTree.h"

#include <iostream>
#include <QtGui>
#include <CGAL/Qt/GraphicsViewNavigation.h>
#include <QLineF>
#include <QRectF>
#include <qapplication.h>
//#include <CGAL/IO/Qt_widget_Polygon.h>
//#include <CGAL/IO/Qt_widget.h>

using std::cout; 
using std::endl;
using std::list;
using namespace lineclear;

int main(int argc, char **argv)
{
        
    // ====================================
    // = Generate polygon and Environment =
    // ====================================
    int n = 10;
    Polygon polygon;
    CGAL::random_polygon_2(n, std::back_inserter(polygon),
                       Point_generator(400));
    if( polygon.is_clockwise_oriented() ) {
        polygon.reverse_orientation();
    }
    Environment* e = new Environment( &polygon );

    // ===========================
    // = Create QT app and scene =
    // ===========================
    QApplication app(argc, argv);
    QGraphicsScene scene;
    scene.setSceneRect(-450,-450, 900, 900);
    
    QGraphicsView* view = new QGraphicsView(&scene);
    CGAL::Qt::GraphicsViewNavigation navigation;
    view->installEventFilter(&navigation);
    view->viewport()->installEventFilter(&navigation);
    view->setRenderHint(QPainter::Antialiasing);
    view->show();
    
    // ================================
    // = Draw the environment polygon =
    // ================================
    CGAL::Qt::PolygonGraphicsItem<Polygon> * pgi;    
    pgi = new CGAL::Qt::PolygonGraphicsItem<Polygon>(&polygon);
    pgi->setVerticesPen(QPen(Qt::yellow, 6, Qt::SolidLine, 
        Qt::RoundCap, Qt::RoundJoin));
    pgi->setEdgesPen(QPen(Qt::green, 2, Qt::SolidLine, Qt::RoundCap, 
        Qt::RoundJoin));
    scene.addItem(pgi);
    
    // =====================================
    // = Compute intersection & display it =
    // =====================================
    list< Polygon_wh > intR;
    list< Polygon_wh >::iterator i;
    e->get_shortest_line_intersected(2, 6, std::back_inserter(intR));
    int c = e->get_shortest_extension_cost(2, 6, 4);
    std::cout << " get_shortest_extension_cost(2, 6, 4); " << c << std::endl;
    std::cout << " poly line intersection " << intR.size() << std::endl;    
    for ( i = intR.begin(); i != intR.end(); i++ ) {            
        CGAL::Qt::PolygonWithHolesGraphicsItem<Polygon_wh> * minkgi;
        minkgi = new CGAL::Qt::PolygonWithHolesGraphicsItem<Polygon_wh>(&(*i));
        minkgi->setVerticesPen(QPen(Qt::red, 8, Qt::SolidLine, 
            Qt::RoundCap, Qt::RoundJoin));
        scene.addItem(minkgi);
        minkgi->setEdgesPen(QPen(Qt::blue, 4, Qt::SolidLine, Qt::RoundCap, 
            Qt::RoundJoin));
    
        Polygon outer_boundary = i->outer_boundary();
        for( int i = 0; i < outer_boundary.size() ; i++ ) {
            cout << outer_boundary[i] << endl;
        }
    }
    
    
    ChoiceTree* t = new ChoiceTree( e );
    
    //for( int i = 1; i <= n; i++ ) {
    //    for( int j = 1; j <= n; j++ ) {
    //        std::cout << i << ":" << j 
    //            << " = " << e->get_shortest_line_inside_cost(i,j);
    //        std::cout << std::endl;
    //    }
    //}
    
    
    return app.exec();
}
