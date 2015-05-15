#include <QApplication>
#include "lineclear/Gui.h"
#include "gui/viewer.h"

Viewer *viewer = NULL;
int main(int argc, char *argv[]) {
  QApplication app(argc, argv);
  Gui gui;
  gui.show();
  return app.exec();
}