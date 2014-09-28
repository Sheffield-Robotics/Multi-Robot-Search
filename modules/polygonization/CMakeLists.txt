cmake_minimum_required (VERSION 2.6)
project(polygonization)

file(GLOB polygonization_SRC
    "src/*.cpp"
)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/../../cmake/")

# DEFINE TARGETS
add_library(polygonization ${polygonization_SRC})

# FIND PACKAGES AND DEFINE INCLUDE DIRECTORIES
find_package(CGAL)
set(CGAL_DONT_OVERRIDE_CMAKE_FLAGS TRUE)
include( ${CGAL_USE_FILE} )

include_directories(../include)

# LINK TARGETS
target_link_libraries( polygonization 
    ${Boost_LIBRARIES}
    ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES}
    utilities heightmap)

message(STATUS "We're done with polygonization.")



