cmake_minimum_required (VERSION 2.6)

project(agents)

file(GLOB agents_SRC
    "src/*.cpp"
    "src/*.c"
)

file(GLOB agents_HEADERS
    "include/*.h"
    "include/*.hpp"
)

add_library(agents 
    ${agents_SRC} 
    ${agents_HEADERS})

message(STATUS "We're done with agents.")
