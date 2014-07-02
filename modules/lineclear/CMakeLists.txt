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
