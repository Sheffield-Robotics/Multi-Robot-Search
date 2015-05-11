cmake_minimum_required (VERSION 2.6)

project(lineclear)

file(GLOB lineclear_SRC
    "src/*.cpp"
    "src/*.c"
)

add_library(lineclear ${lineclear_SRC})
find_package(CGAL COMPONENTS Qt4)
include( ${CGAL_USE_FILE} )

find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})
#find_package(Qt4 COMPONENTS QtCore QtGui QtOpenGL REQUIRED)
#include(${QT_USE_FILE})
#SET(gui_FORMS viewer_interface.ui option_widget.ui)
#QT4_WRAP_CPP(gui_HEADERS_MOC ${gui_HEADERS})
#QT4_WRAP_UI(gui_FORMS_HEADERS ${gui_FORMS})
#message(gui_FORMS_HEADERS ${gui_FORMS_HEADERS})
#INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})


#add_executable(lineclear_gui_exe ${lineclear_SRC})
#target_link_libraries(lineclear_gui_exe 
#    utilities polygonization lineclear
#    ${QT_LIBRARIES}
#    ${GEOTIFF_LIBRARY}
#    ${TIFF_LIBRARIES}
#    ${QGLVIEWER_LIBRARY}
#    ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES}
#    ${Boost_LIBRARIES}
#    ${YAMLCPP_LIBRARY})
#