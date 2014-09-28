cmake_minimum_required (VERSION 2.6)

project(agents)

file(GLOB agents_SRC
    "src/*.cpp"
    "src/*.c"
)

add_library(agents 
    ${agents_SRC})

message(STATUS "We're done with agents.")
