TEMPLATE = app
CONFIG -= silent
TARGET = pts2vrml
DEPENDPATH += . $(PURSUIT_EVASION)
INCLUDEPATH += . $(PURSUIT_EVASION)

DESTDIR = $(PURSUIT_EVASION)/bin/

OBJECTS_DIR = .obj/

#LIBS += 

macx|darwin-g++ {
   message("Compiling on Mac OS")
   INCLUDEPATH += . /opt/local/include
   INCLUDEPATH += . /sw/include/qt
   CONFIG *= lib_bundle
   CONFIG -= app_bundle
   DEFINES *=ifdef __APPLE__

    # GLUT for Macintosh architecture
    !isEmpty( USE_GLUT ) {
       QMAKE_LIBS_OPENGL -= -lglut
       QMAKE_LIBS_OPENGL *= -framework GLUT -lobjc
    }

   QMAKE_CXXFLAGS += -Wno-deprecated
   QMAKE_CXXFLAGS_RELEASE += -fast -fstrength-reduce -floop-optimize -finline-functions
   QMAKE_CFLAGS_RELEASE += -fast -fstrength-reduce -floop-optimize -finline-functions
   QMAKE_CXXFLAGS_DEBUG = $$QMAKE_CXXFLAGS_RELEASE
   QMAKE_CFLAGS_DEBUG = $$QMAKE_CFLAGS_RELEASE


    # This is a hack, please remove if you experience problems
    # or install the colorgcc package
   QMAKE_CXX=colorgcc
}



# Input
HEADERS += \

SOURCES += \
           main.cpp \
