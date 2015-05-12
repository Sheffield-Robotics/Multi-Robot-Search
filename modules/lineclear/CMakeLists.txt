cmake_minimum_required (VERSION 2.6)

project(lineclear)

file(GLOB lineclear_SRC
    "src/*.cpp"
    "src/*.c"
)

add_library(lineclear ${lineclear_SRC})




message(${lineclear_SRC})

EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )
#set(CMAKE_OSX_ARCHITECTURES ${ARCHS_STANDARD_64_BIT})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/../../cmake/")

if(APPLE)
    set(CMAKE_CXX_FLAGS "-pipe -Os -I/opt/local/include -arch x86_64") 
    set(CMAKE_EXE_LINKER_FLAGS "-L/opt/local/lib -Wl,-headerpad_max_install_names -arch x86_64 -Wl,-dylib_file,/System/Library/Frameworks/OpenGL.framework/Versions/A/Libraries/libGL.dylib:/System/Library/Frameworks/OpenGL.framework/Versions/A/Libraries/libGL.dylib")
endif()

include_directories(../include)

find_package(CGAL COMPONENTS Qt4)
include( ${CGAL_USE_FILE} )
find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})
find_package(GLUT)
find_package(OpenGL)
find_package(QGLViewer REQUIRED)
include_directories(${QGLVIEWER_INCLUDE_DIR})
find_package(Qt4 COMPONENTS QtCore QtGui QtOpenGL REQUIRED)
include(${QT_USE_FILE})
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})
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

QT4_WRAP_CPP(lineclear_QT_header ../include/lineclear/Gui.h)

set(lineclear_exe_SRC ${lineclear_SRC} src/main.cxx)

add_executable(lineclear_exe ${lineclear_exe_SRC} ${lineclear_QT_header})
target_link_libraries(lineclear_exe 
    gui utilities hungarian heightmap planner polygonization 
    agents lineclear uavmodel
    ${QT_LIBRARIES}
    ${GEOTIFF_LIBRARY}
    ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES}
    ${TIFF_LIBRARIES}
    ${QGLVIEWER_LIBRARY}
    ${Boost_LIBRARIES}
    ${OpenCV_LIBS}
    ${OPENGL_LIBRARIES}
    ${JPEG_LIBRARIES}
    ${YAMLCPP_LIBRARY})

set(lineclear_gui_exe_SRC ${lineclear_SRC} src/Gui_main.cxx ${lineclear_QT_header})

add_executable(lineclear_gui_exe ${lineclear_gui_exe_SRC})
target_link_libraries(lineclear_gui_exe 
    gui utilities hungarian heightmap planner polygonization 
    agents lineclear uavmodel
    ${QT_LIBRARIES}
    ${GEOTIFF_LIBRARY}
    ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES}
    ${TIFF_LIBRARIES}
    ${QGLVIEWER_LIBRARY}
    ${Boost_LIBRARIES}
    ${OpenCV_LIBS}
    ${OPENGL_LIBRARIES}
    ${JPEG_LIBRARIES}
    ${YAMLCPP_LIBRARY})
