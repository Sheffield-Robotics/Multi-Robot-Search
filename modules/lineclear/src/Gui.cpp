#include <QtGui>
#include <CGAL/Qt/GraphicsViewNavigation.h>
#include "lineclear/Gui.h"

using namespace lineclear;

Gui::Gui()
{
    polygon = NULL;
    e = NULL;
    pgi = NULL;
    clearing_line1 = NULL;
    clearing_line2 = NULL;
    current_i = 0;
    current_k = 1;
    current_cost = 0;
    current_blocking_cost = 0;
    scene = new QGraphicsScene(this);
    //scene->setSceneRect(-450,-450, 900, 900);
    //scene->setSceneRect(QRectF(0, 0, 5000, 5000));
    scene->setSceneRect(QRectF(-450,-450, 900, 900));

    QHBoxLayout *layout = new QHBoxLayout;    
    view = new QGraphicsView(scene);
    CGAL::Qt::GraphicsViewNavigation navigation;
    view->installEventFilter(&navigation);
    view->viewport()->installEventFilter(&navigation);
    view->setRenderHint(QPainter::Antialiasing);
    view->show();
    layout->addWidget(view);
    
    QWidget *widget = new QWidget;
    widget->setLayout(layout);
    setCentralWidget(widget);
    
    QWidget *topFiller = new QWidget;
    topFiller->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    infoLabel = new QLabel(tr("<i>Choose a menu option, or right-click to "
                              "invoke a context menu</i>"));
    infoLabel->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    infoLabel->setAlignment(Qt::AlignCenter);

    createActions();
    createMenus();
    QString message = tr("A context menu is available by right-clicking");
    statusBar()->showMessage(message);

    setWindowTitle(tr("Menus"));
    setMinimumSize(160, 160);
    resize(1200, 1200);
       
}

void Gui::contextMenuEvent(QContextMenuEvent *event)
{
    QMenu menu(this);
    menu.addAction(cutAct);
    menu.addAction(copyAct);
    menu.addAction(pasteAct);
    menu.exec(event->globalPos());
}

void Gui::newFile()
{
    infoLabel->setText(tr("Invoked <b>File|New</b>"));
}

void Gui::open()
{
    infoLabel->setText(tr("Invoked <b>File|Open</b>"));
}

void Gui::save()
{
    infoLabel->setText(tr("Invoked <b>File|Save</b>"));
}

void Gui::print()
{
    infoLabel->setText(tr("Invoked <b>File|Print</b>"));
}

void Gui::undo()
{
    infoLabel->setText(tr("Invoked <b>Edit|Undo</b>"));
}

void Gui::redo()
{
    std::cout << " Rebuilding the environment " << std::endl;
    current_i = 0;
    current_k = 1;
    if ( pgi != NULL ) { scene->removeItem(pgi); delete pgi; }
    if ( e != NULL ) { delete e;}
    if ( polygon != NULL ) { delete polygon;}
    // ====================================
    // = Generate polygon and Environment =
    // ====================================
    int n = 20;
    polygon = new lineclear::Polygon;
    CGAL::random_polygon_2(n, std::back_inserter(*polygon), 
        Point_generator(400));
    if( polygon->is_clockwise_oriented() ) { polygon->reverse_orientation(); }
    std::cout << *polygon << std::endl;
    e = new lineclear::Environment( polygon );
    // ================================
    // = Draw the environment polygon =
    // ================================    
    pgi = new CGAL::Qt::PolygonGraphicsItem<Polygon>(polygon);
    pgi->setVerticesPen(QPen(Qt::yellow, 4, Qt::SolidLine, 
        Qt::RoundCap, Qt::RoundJoin));
    pgi->setEdgesPen(QPen(Qt::black, 1, Qt::SolidLine, Qt::RoundCap, 
        Qt::RoundJoin));
    scene->addItem(pgi);
}

void Gui::cut()
{
    // =====================================
    // = Compute intersection & display it =
    // =====================================
    std::list< Polygon_wh > intR;
    std::list< Polygon_wh >::iterator i;
    int o1 = rand()%e->get_obstacle_number()+1;
    int o2 = rand()%e->get_obstacle_number()+1;
    
    std::cout << " get_shortest_line_intersected(" << o1 << "," << o2 << std::endl;
    e->get_shortest_line_intersected(o1, o2, std::back_inserter(intR));
    std::cout << " poly line intersection size " << intR.size() << std::endl;    
    
    for ( i = intR.begin(); i != intR.end(); i++ ) {
        Polygon_wh* poly_with_holes = new Polygon_wh(*i);
        CGAL::Qt::PolygonWithHolesGraphicsItem<Polygon_wh>* minkgi;
        minkgi = new CGAL::Qt::PolygonWithHolesGraphicsItem<Polygon_wh>(poly_with_holes);
        minkgi->setVerticesPen(QPen(Qt::red, 8, Qt::SolidLine, Qt::RoundCap,
            Qt::RoundJoin));
        minkgi->setEdgesPen(QPen(Qt::blue, 4, Qt::SolidLine, Qt::RoundCap, 
            Qt::RoundJoin));
        std::cout << "AddItem" << std::endl;
        scene->addItem(minkgi);
    }
}

void Gui::copy() {
    int c = -1;
    int o1,o2,o3,max;
    Segment l1,l2;
    
    
    o1 = rand()%e->get_obstacle_number()+1;
    o2 = rand()%e->get_obstacle_number()+1;
    o3 = rand()%e->get_obstacle_number()+1;
    max = 1000;
    while ( o2 == o1 && max > 0 ) {
        max--;
        o2 = rand()%e->get_obstacle_number()+1;
    }
    max = 1000;
    while ( (o3 == o1 || o3 == o2) && max > 0 ) {
        max--;
        o3 = rand()%e->get_obstacle_number()+1;
    }
    
    c = e->get_shortest_extension(o1,o2,o3,l1,l2);
    std::cout << o1 << "," << o2 << ","  << o3 << " c=" << c << std::endl;
        
    QPen p = QPen(Qt::red, 4, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
    QPen p2 = QPen(Qt::blue, 3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
    QPen p3 = QPen(Qt::green, 3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
    QPen p4 = QPen(Qt::red, 3, Qt::DashLine, Qt::RoundCap, Qt::RoundJoin);
    CGAL::Qt::Converter<K> convert;
    
    QLineF line1 = convert(l1);
    QLineF line2 = convert(l2);
    QLineF line3 = convert( e->get_edge(o1) );
    QLineF line4 = convert( e->get_edge(o2) );
    QLineF line5 = convert( e->get_edge(o3) );
    QLineF line6 = convert( e->get_shortest_line_inside(o1,o2) );
    
    line_items.push_back( scene->addLine(line1,p) );
    line_items.push_back( scene->addLine(line2,p) );
    line_items.push_back( scene->addLine(line3,p2) );
    line_items.push_back( scene->addLine(line4,p2) );
    line_items.push_back( scene->addLine(line5,p3) );
    line_items.push_back( scene->addLine(line6,p4) );
}

void Gui::paste()
{
    for (int i = 0; i < line_items.size(); i++ ) {
        scene->removeItem(line_items[i]);
    }
    line_items.clear();
}

void Gui::bold()
{
    std::cout << " Generating Choice Tree " << std::endl;
    t = new ChoiceTree(e);
    
    int first_obstacle;
    obstacle_sequence = t->get_optimal_obstacle_sequence(first_obstacle);    
    obstacle_sequence.push_front(first_obstacle);
    obstacle_sequence_it = obstacle_sequence.begin();
    
    std::list<int>::iterator i = obstacle_sequence.begin();
    while ( i != obstacle_sequence.end() ) {
        std::cout << *i << ",";
        i++;
    }
    std::cout << std::endl;
}

void Gui::italic()
{
    QPen p = QPen(Qt::red, 4, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
    QPen p2 = QPen(Qt::blue, 6, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
    if ( obstacle_sequence_it == obstacle_sequence.end()) {
        // TODO: restart the animation
        std::cout << " WE ARE DONE " << std::endl;
        return;
    }
    // display split between current obstacles and the new one 
    int new_obstacle = *obstacle_sequence_it;

    if ( clearing_line1 != NULL ) {
        // do we have to clear the memory for this one?
        scene->removeItem(clearing_line1);
        clearing_line1 = NULL;
    }
    if ( clearing_line2 != NULL ) {
        scene->removeItem(clearing_line2);
        clearing_line2 = NULL;
    }

    if ( cleared_obstacles.size() > 1 ) {
        // add a split between the two indices between new_obstacle
        std::list<int>::iterator i = cleared_obstacles.begin();
        int next_smaller_obstacle = 0, next_larger_obstacle = 0;
        while ( new_obstacle > *i && i != cleared_obstacles.end() ) { 
            next_smaller_obstacle = *i;
            i++;
        }
        if ( next_smaller_obstacle == 0 ) {
            next_smaller_obstacle = cleared_obstacles.back();
        } 
        if ( i == cleared_obstacles.end() ) {
            next_larger_obstacle = cleared_obstacles.front();
        } else {
            next_larger_obstacle = *i;
        }

        // we have smaller and larger obstacle
        Segment l1,l2,l3,l4; 
        std::cout << " extending  " << next_smaller_obstacle
            << " and " << next_larger_obstacle << " onto "
            << new_obstacle << std::endl;
        int ext_cost = e->get_shortest_extension(next_smaller_obstacle,
            next_larger_obstacle, new_obstacle,l1,l2);

        // remove old block
        std::pair<int,int> b_p(next_smaller_obstacle, next_larger_obstacle);
        scene->removeItem(blocking_lines_map[b_p]);
        blocking_lines_map.erase(b_p);
        current_blocking_cost -= e->get_shortest_line_inside_cost(
            next_smaller_obstacle, next_larger_obstacle);

        current_cost = ext_cost + current_blocking_cost;
        std::cout << " COST= " << current_cost << std::endl;
        std::cout << " ext_cost= " << ext_cost << std::endl;
        std::cout << " current_blocking_cost= " << current_blocking_cost << std::endl;
        
        
        
        CGAL::Qt::Converter<K> convert;
        if ( l1.target() != l1.source() ) {
            QLineF line1 = convert( l1 );
            clearing_line1 = scene->addLine(line1,p);
        }
        if ( l2.target() != l2.source() ) {
            QLineF line2 = convert( l2 );
            clearing_line2 = scene->addLine(line2,p);
        }
        
        l3 = e->get_shortest_line_inside(new_obstacle,next_larger_obstacle);
        if ( l3.target() != l3.source() ) {
            QLineF line3 = convert( l3 );
            std::pair<int,int> block_between(new_obstacle,next_larger_obstacle);
            blocking_lines_map[block_between] = scene->addLine(line3,p2);
            //blocking_lines.push_back( scene->addLine(line3,p2) );
            current_blocking_cost += sqrt(CGAL::to_double(l3.squared_length()));
        }
        l4 = e->get_shortest_line_inside(next_smaller_obstacle,new_obstacle);
        if ( l4.target() != l4.source() ) {
            QLineF line4 = convert( l4 );
            std::pair<int,int> block_between(next_smaller_obstacle, 
                new_obstacle);
            blocking_lines_map[block_between] = scene->addLine(line4,p2);
            current_blocking_cost += sqrt(CGAL::to_double(l4.squared_length()));
            //blocking_lines.push_back( scene->addLine(line4,p2) );
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

void Gui::leftAlign()
{
    //scene->clear();
    scene->addItem(pgi);
    QPen p1 = QPen(Qt::red, 5, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
    QPen p2 = QPen(Qt::green, 6, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
    QPen p3 = QPen(Qt::blue, 10, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
    QPen p4 = QPen(Qt::red, 2, Qt::DashLine, Qt::RoundCap, Qt::RoundJoin);
    CGAL::Qt::Converter<K> convert;
    
    int o1 = 0,o2 = 0;
    //o1 = rand()%e->get_obstacle_number()+1;
    //while ( o1 == o2 || o2 == 0 ) {
    //    o2 = rand()%e->get_obstacle_number()+1;
    //}
    o1 = current_i-1;
    o2 = current_i+current_k;
    e->fix_index(o1);
    e->fix_index(o2);
    
    std::cout << " calling get_shortest_line(" << o1 << "," << o2 << ")"
        << std::endl;
    Segment l = e->get_shortest_line(o1,o2);
    QLineF oLine1 = convert( e->get_edge(o1) );
    QLineF oLine2 = convert( e->get_edge(o2) );    
    scene->addLine(oLine1,p3);
    scene->addLine(oLine2,p3);

    QLineF oLine3 = convert( l );    
    scene->addLine(oLine3,p4);
    
    std::list<CGAL::Object> z;
    e->get_zone(l, z);

    std::list<CGAL::Object>::iterator i = z.begin();
    std::cout << " _________ line " << l << std::endl;
    std::cout << " _________ zone size " << z.size() << std::endl;
    bool inside = true;
    while ( i != z.end() ) {
        CGAL::Object obj = *i;        
        Arrangement::Vertex_handle vh;
        if ( CGAL::assign (vh, obj)) {
            std::cout << " ____ Vertex_handle ";
            std::cout << vh->point() << std::endl;
            QPointF qpoint = convert( vh->point() );
            scene->addRect(qpoint.x(),qpoint.y(),5,5,p1);
        }
        
        Arrangement::Halfedge_handle eh;
        if ( CGAL::assign (eh, obj)) {
            std::cout << " ____ Halfedge_handle ";
            Segment seg(eh->source()->point(),eh->target()->point());
            QLineF qline = convert( seg );
            scene->addLine(qline,p2);
            std::cout << eh->source()->point() << " to ";
            std::cout << eh->target()->point() << std::endl;
        }
        
        Arrangement::Face_handle fh;
        if ( CGAL::assign (fh, obj)) {
            std::cout << " ____ Face_handle ";
            std::cout << " is_unbounded?" << fh->is_unbounded();
            std::cout << " has_outer_ccb?" << fh->has_outer_ccb();
            std::cout << std::endl;
        }
        i++;
    }
}

void Gui::rightAlign()
{
    // go through all choice sets and draw their blocking line
    CGAL::Qt::Converter<K> convert;
    QPen pen = QPen(Qt::blue, 2, Qt::DashLine, Qt::RoundCap, Qt::RoundJoin);
    for (int k = 1; k < e->get_obstacle_number(); ++k ) {
        std::cout << " LEVEL 1 k=" << k << std::endl;
        for (int i = 1; i <= e->get_obstacle_number(); ++i ) {
            Segment l = e->get_shortest_line_inside(i-1,i+k);
            std::cout << i << ":" << k << " " << l << std::endl;
            if ( l.source() != l.target() ) {
                QLineF qline = convert(l);
                line_items.push_back( scene->addLine(qline,pen) );
            }   
        }
    }
}

void Gui::justify() {
    CGAL::Qt::Converter<K> convert;
    QPen p1 = QPen(Qt::blue, 2, Qt::DashLine, Qt::RoundCap, Qt::RoundJoin);
    QPen p2 = QPen(Qt::green, 3, Qt::DashLine, Qt::RoundCap, Qt::RoundJoin);
    QPen p3 = QPen(Qt::red, 1, Qt::DashLine, Qt::RoundCap, Qt::RoundJoin);
    bool success = false;
    int i = current_i;
    int k = current_k;
    while ( success == false ) {
        i++;
        if ( i > e->get_obstacle_number() ) {
            i = 1; k++;
            if ( k >= e->get_obstacle_number() ) {
                k = 1;
            }
        }
        int a = i-1;
        int b = i+k;
        e->fix_index(a);
        e->fix_index(b);
        QLineF oLine1 = convert( e->get_edge(a) );
        QLineF oLine2 = convert( e->get_edge(b) );
        line_items.push_back( scene->addLine(oLine1,p2) );
        line_items.push_back( scene->addLine(oLine2,p2) );
        
        Segment l_short = e->get_shortest_line(a,b);
        QLineF qline_short = convert(l_short);
        line_items.push_back( scene->addLine(qline_short,p3) );
        
        Segment l = e->get_shortest_line_inside(i-1,i+k);
        std::cout << i << ":" << k << " " << l << std::endl;
        if ( l.source() != l.target() ) {
            QLineF qline = convert(l);
            line_items.push_back( scene->addLine(qline,p1) );
        }
        success = true;
        current_i = i;
        current_k = k;
    }
}

void Gui::center()
{
    infoLabel->setText(tr("Invoked <b>Edit|Format|Center</b>"));
}

void Gui::setLineSpacing()
{
    infoLabel->setText(tr("Invoked <b>Edit|Format|Set Line Spacing</b>"));
}

void Gui::setParagraphSpacing()
{
    infoLabel->setText(tr("Invoked <b>Edit|Format|Set Paragraph Spacing</b>"));
}

void Gui::about()
{
    infoLabel->setText(tr("Invoked <b>Help|About</b>"));
    QMessageBox::about(this, tr("About Menu"),
            tr("The <b>Menu</b> example shows how to create "
               "menu-bar menus and context menus."));
}

void Gui::aboutQt()
{
    infoLabel->setText(tr("Invoked <b>Help|About Qt</b>"));
}

void Gui::createActions()
{
    newAct = new QAction(tr("&New"), this);
    newAct->setShortcuts(QKeySequence::New);
    newAct->setStatusTip(tr("Create a new file"));
    connect(newAct, SIGNAL(triggered()), this, SLOT(newFile()));

    openAct = new QAction(tr("&Open..."), this);
    openAct->setShortcuts(QKeySequence::Open);
    openAct->setStatusTip(tr("Open an existing file"));
    connect(openAct, SIGNAL(triggered()), this, SLOT(open()));

    saveAct = new QAction(tr("&Save"), this);
    saveAct->setShortcuts(QKeySequence::Save);
    saveAct->setStatusTip(tr("Save the document to disk"));
    connect(saveAct, SIGNAL(triggered()), this, SLOT(save()));

    printAct = new QAction(tr("&Print..."), this);
    printAct->setShortcuts(QKeySequence::Print);
    printAct->setStatusTip(tr("Print the document"));
    connect(printAct, SIGNAL(triggered()), this, SLOT(print()));

    exitAct = new QAction(tr("E&xit"), this);
    exitAct->setShortcuts(QKeySequence::Quit);
    exitAct->setStatusTip(tr("Exit the application"));
    connect(exitAct, SIGNAL(triggered()), this, SLOT(close()));

    undoAct = new QAction(tr("&Undo"), this);
    undoAct->setShortcuts(QKeySequence::Undo);
    undoAct->setStatusTip(tr("Undo the last operation"));
    connect(undoAct, SIGNAL(triggered()), this, SLOT(undo()));

    redoAct = new QAction(tr("&Redo"), this);
    redoAct->setShortcuts(QKeySequence::Redo);
    redoAct->setStatusTip(tr("Redo the last operation"));
    connect(redoAct, SIGNAL(triggered()), this, SLOT(redo()));

    cutAct = new QAction(tr("Cu&t"), this);
    cutAct->setShortcuts(QKeySequence::Cut);
    cutAct->setStatusTip(tr("Cut the current selection's contents to the "
                            "clipboard"));
    connect(cutAct, SIGNAL(triggered()), this, SLOT(cut()));

    copyAct = new QAction(tr("&Copy"), this);
    copyAct->setShortcuts(QKeySequence::Copy);
    copyAct->setStatusTip(tr("Copy the current selection's contents to the "
                             "clipboard"));
    connect(copyAct, SIGNAL(triggered()), this, SLOT(copy()));

    pasteAct = new QAction(tr("&Paste"), this);
    pasteAct->setShortcuts(QKeySequence::Paste);
    pasteAct->setStatusTip(tr("Paste the clipboard's contents into the current "
                              "selection"));
    connect(pasteAct, SIGNAL(triggered()), this, SLOT(paste()));

    boldAct = new QAction(tr("&Bold"), this);
    boldAct->setCheckable(true);
    boldAct->setShortcut(QKeySequence::Bold);
    boldAct->setStatusTip(tr("Make the text bold"));
    connect(boldAct, SIGNAL(triggered()), this, SLOT(bold()));

    QFont boldFont = boldAct->font();
    boldFont.setBold(true);
    boldAct->setFont(boldFont);

    italicAct = new QAction(tr("&Italic"), this);
    italicAct->setCheckable(true);
    italicAct->setShortcut(QKeySequence::Italic);
    italicAct->setStatusTip(tr("Make the text italic"));
    connect(italicAct, SIGNAL(triggered()), this, SLOT(italic()));

    QFont italicFont = italicAct->font();
    italicFont.setItalic(true);
    italicAct->setFont(italicFont);

    setLineSpacingAct = new QAction(tr("Set &Line Spacing..."), this);
    setLineSpacingAct->setStatusTip(tr("Change the gap between the lines of a "
                                       "paragraph"));
    connect(setLineSpacingAct, SIGNAL(triggered()), this, SLOT(setLineSpacing()));

    setParagraphSpacingAct = new QAction(tr("Set &Paragraph Spacing..."), this);
    setLineSpacingAct->setStatusTip(tr("Change the gap between paragraphs"));
    connect(setParagraphSpacingAct, SIGNAL(triggered()),
            this, SLOT(setParagraphSpacing()));

    aboutAct = new QAction(tr("&About"), this);
    aboutAct->setStatusTip(tr("Show the application's About box"));
    connect(aboutAct, SIGNAL(triggered()), this, SLOT(about()));

    aboutQtAct = new QAction(tr("About &Qt"), this);
    aboutQtAct->setStatusTip(tr("Show the Qt library's About box"));
    connect(aboutQtAct, SIGNAL(triggered()), qApp, SLOT(aboutQt()));
    connect(aboutQtAct, SIGNAL(triggered()), this, SLOT(aboutQt()));

    leftAlignAct = new QAction(tr("&Left Align"), this);
    leftAlignAct->setCheckable(true);
    leftAlignAct->setShortcut(tr("Ctrl+L"));
    leftAlignAct->setStatusTip(tr("Left align the selected text"));
    connect(leftAlignAct, SIGNAL(triggered()), this, SLOT(leftAlign()));

    rightAlignAct = new QAction(tr("&Right Align"), this);
    rightAlignAct->setCheckable(true);
    rightAlignAct->setShortcut(tr("Ctrl+R"));
    rightAlignAct->setStatusTip(tr("Right align the selected text"));
    connect(rightAlignAct, SIGNAL(triggered()), this, SLOT(rightAlign()));

    justifyAct = new QAction(tr("&Justify"), this);
    justifyAct->setCheckable(true);
    justifyAct->setShortcut(tr("Ctrl+J"));
    justifyAct->setStatusTip(tr("Justify the selected text"));
    connect(justifyAct, SIGNAL(triggered()), this, SLOT(justify()));

    centerAct = new QAction(tr("&Center"), this);
    centerAct->setCheckable(true);
    centerAct->setShortcut(tr("Ctrl+E"));
    centerAct->setStatusTip(tr("Center the selected text"));
    connect(centerAct, SIGNAL(triggered()), this, SLOT(center()));

    alignmentGroup = new QActionGroup(this);
    alignmentGroup->addAction(leftAlignAct);
    alignmentGroup->addAction(rightAlignAct);
    alignmentGroup->addAction(justifyAct);
    alignmentGroup->addAction(centerAct);
    leftAlignAct->setChecked(true);
}

void Gui::createMenus()
{
    fileMenu = menuBar()->addMenu(tr("&File"));
    fileMenu->addAction(newAct);
    fileMenu->addAction(openAct);
    fileMenu->addAction(saveAct);
    fileMenu->addAction(printAct);
    fileMenu->addSeparator();
    fileMenu->addAction(exitAct);

    editMenu = menuBar()->addMenu(tr("&Edit"));
    editMenu->addAction(undoAct);
    editMenu->addAction(redoAct);
    editMenu->addSeparator();
    editMenu->addAction(cutAct);
    editMenu->addAction(copyAct);
    editMenu->addAction(pasteAct);
    editMenu->addSeparator();

    helpMenu = menuBar()->addMenu(tr("&Help"));
    helpMenu->addAction(aboutAct);
    helpMenu->addAction(aboutQtAct);

    formatMenu = editMenu->addMenu(tr("&Format"));
    formatMenu->addAction(boldAct);
    formatMenu->addAction(italicAct);
    formatMenu->addSeparator()->setText(tr("Alignment"));
    formatMenu->addAction(leftAlignAct);
    formatMenu->addAction(rightAlignAct);
    formatMenu->addAction(justifyAct);
    formatMenu->addAction(centerAct);
    formatMenu->addSeparator();
    formatMenu->addAction(setLineSpacingAct);
    formatMenu->addAction(setParagraphSpacingAct);
}