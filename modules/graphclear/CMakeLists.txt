cmake_minimum_required (VERSION 2.6)

project(graphclear)

file(GLOB graphclear_SRC
    "src/*.cpp"
    "src/*.c"
)

add_library(graphclear ${graphclear_SRC})

add_executable(graphclear_exe ${graphclear_SRC})

find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS}) 

target_link_libraries(graphclear_exe
    graphclear
    ${Boost_LIBRARIES})

