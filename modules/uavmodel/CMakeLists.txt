cmake_minimum_required (VERSION 2.6)

project(uavmodel)

file(GLOB uavmodel_SRC
    "src/*.cpp"
    "src/*.c"
)

add_library(uavmodel ${uavmodel_SRC})

message(STATUS "We're done with uavmodel.")



