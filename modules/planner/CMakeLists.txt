cmake_minimum_required (VERSION 2.6)

project(planner)

include_directories(${utilities_SOURCE_DIR}/include)
include_directories(include)

file(GLOB planner_SRC
    "src/*.cpp"
    "src/*.c"
)

file(GLOB planner_HEADERS
    "include/*.h"
    "include/*.hpp"
)

add_library(planner 
    ${planner_SRC} 
    ${planner_HEADERS})

message(STATUS "We're done with planner.")



