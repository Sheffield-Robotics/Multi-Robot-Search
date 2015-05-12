cmake_minimum_required (VERSION 2.6)

project(hungarian)

file(GLOB hungarian_SRC
    "src/*.cpp"
    "src/*.c"
)

include_directories(include)

add_library(hungarian ${hungarian_SRC})
