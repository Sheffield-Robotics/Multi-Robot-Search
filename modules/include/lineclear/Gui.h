#ifndef Gui_H
#define Gui_H

#include <QMainWindow>
#include <vector>
#include <list>
#include "lineclear/CGALTypes.h"
#include "lineclear/Environment.h"
#include "lineclear/ChoiceTree.h"

#include <map>

class QGraphicsScene;
class QGraphicsView;
class QAction;
class QActionGroup;
class QLabel;
class QMenu;

using namespace lineclear;

class Gui : public QMainWindow
{
    Q_OBJECT

public:
    Gui();
    lineclear::Environment* e;
    lineclear::Polygon* polygon;
    CGAL::Qt::PolygonGraphicsItem<lineclear::Polygon> * pgi;
    std::vector<QGraphicsLineItem*> line_items;
    lineclear::ChoiceTree* t;

    int current_i;
    int current_k;
    
    int current_cost;
    int current_blocking_cost;

    std::list<int> obstacle_sequence;
    std::list<int>::iterator obstacle_sequence_it;
    std::list<int> cleared_obstacles;
    std::vector<QGraphicsLineItem*> blocking_lines;

    std::map< std::pair<int,int>,QGraphicsLineItem*> blocking_lines_map;
    
    QGraphicsLineItem* clearing_line1;
    QGraphicsLineItem* clearing_line2;

protected:
    void contextMenuEvent(QContextMenuEvent *event);

private slots:
    void newFile();
    void open();
    void save();
    void print();
    void undo();
    void redo();
    void cut();
    void copy();
    void paste();
    void bold();
    void italic();
    void leftAlign();
    void rightAlign();
    void justify();
    void center();
    void setLineSpacing();
    void setParagraphSpacing();
    void about();
    void aboutQt();

private:
    void createActions();
    void createMenus();

    QGraphicsScene* scene;
    QGraphicsView *view;
    
    QMenu *fileMenu;
    QMenu *editMenu;
    QMenu *formatMenu;
    QMenu *helpMenu;
    QActionGroup *alignmentGroup;
    QAction *newAct;
    QAction *openAct;
    QAction *saveAct;
    QAction *printAct;
    QAction *exitAct;
    QAction *undoAct;
    QAction *redoAct;
    QAction *cutAct;
    QAction *copyAct;
    QAction *pasteAct;
    QAction *boldAct;
    QAction *italicAct;
    QAction *leftAlignAct;
    QAction *rightAlignAct;
    QAction *justifyAct;
    QAction *centerAct;
    QAction *setLineSpacingAct;
    QAction *setParagraphSpacingAct;
    QAction *aboutAct;
    QAction *aboutQtAct;
    QLabel *infoLabel;
};

#endif