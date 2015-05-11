cmake_minimum_required (VERSION 2.6)

project(gui)

file(GLOB gui_SRC "src/*.cpp")

#set(Boost_USE_STATIC_LIBS OFF) 
#set(Boost_USE_MULTITHREADED ON)  
#set(Boost_USE_STATIC_RUNTIME OFF) 
#find_package(GLU)
find_package(GLUT)
find_package(OpenGL)
find_package(QGLViewer REQUIRED)
include_directories(${QGLVIEWER_INCLUDE_DIR})
find_package(Qt4 COMPONENTS QtCore QtGui QtOpenGL REQUIRED)
include(${QT_USE_FILE})
SET(gui_FORMS viewer_interface.ui option_widget.ui)
QT4_WRAP_CPP(gui_HEADERS_MOC ${gui_HEADERS})
QT4_WRAP_UI(gui_FORMS_HEADERS ${gui_FORMS})
message(gui_FORMS_HEADERS ${gui_FORMS_HEADERS})
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})

message("CMAKE_BINARY_DIR")
message(${CMAKE_BINARY_DIR})
#include_directories("${CMAKE_BINARY_DIR}")
find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS}) 
find_package(GeoTiff REQUIRED) 
find_package(TIFF REQUIRED)
find_package(CGAL)
set(CGAL_DONT_OVERRIDE_CMAKE_FLAGS TRUE)
include( ${CGAL_USE_FILE} )
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

find_package( JPEG )
find_package( YAMLCPP )

add_library(gui ${gui_SRC} ${gui_HEADERS} ${gui_HEADERS_MOC} ${gui_FORMS_HEADERS})

add_executable(gui_exe ${gui_SRC} ${gui_HEADERS} ${gui_HEADERS_MOC} ${gui_FORMS_HEADERS})
target_link_libraries(gui_exe 
    utilities hungarian heightmap planner polygonization 
    agents lineclear uavmodel
    ${QT_LIBRARIES}
    ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES}
    ${Boost_LIBRARIES}
    ${GEOTIFF_LIBRARY}
    ${TIFF_LIBRARIES}
    ${QGLVIEWER_LIBRARY}
    ${OpenCV_LIBS}
    ${OPENGL_LIBRARIES}
    ${JPEG_LIBRARIES}
    ${YAMLCPP_LIBRARY})

message(STATUS "We're done with gui.")

#macro(remove_cxx_flag flag)
#    string(REPLACE "${flag}" "" CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}")
#endmacro()
#message(${CMAKE_CXX_FLAGS_RELEASE}) # print "-O3 -DNDEBUG"
#remove_cxx_flag("-arch")
#message(${CMAKE_CXX_FLAGS_RELEASE}) # print "-DNDEBUG"
