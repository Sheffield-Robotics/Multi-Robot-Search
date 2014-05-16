cmake_minimum_required (VERSION 2.6)

project(heightmap)

file(GLOB heightmap_SRC
    "src/*.cpp"
    "src/*.c"
)

#include_directories(${utilities_SOURCE_DIR}/include)
#include_directories(include)
find_package(GLU)
find_package(GLUT)
find_package(OpenGL)
find_package(QGLViewer)
include_directories(${QGLVIEWER_INCLUDE_DIR})
find_package(Qt4 COMPONENTS QtCore QtGui QtOpenGL REQUIRED)
include(${QT_USE_FILE})
QT4_WRAP_CPP(heightmap_HEADERS_MOC ${heightmap_HEADERS})
include_directories("${CMAKE_BINARY_DIR}")

add_library(heightmap 
    ${heightmap_SRC} 
    ${heightmap_HEADERS}
    ${heightmap_HEADERS_MOC})


#target_link_libraries(heightmap ${QGLVIEWER_INCLUDE_DIR})

message(STATUS "We're done with heightmap.")



