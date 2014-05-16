cmake_minimum_required (VERSION 2.6)

project(utilities)

file(GLOB utilities_SRC
    "src/*.cpp"
    "src/*.c"
)

add_library(utilities ${utilities_SRC})

message(STATUS "We're done with utilities.")



