cmake_minimum_required (VERSION 2.6)

project(hungarian)

file(GLOB hungarian_SRC
    "src/*.cpp"
    "src/*.c"
)

file(GLOB hungarian_HEADERS
    "include/*.h"
    "include/*.hpp"
)

include_directories(include)

add_library(hungarian ${hungarian_SRC})

message(STATUS "We're done with hungarian.")
