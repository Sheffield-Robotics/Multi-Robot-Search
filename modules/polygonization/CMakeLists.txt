cmake_minimum_required (VERSION 2.6)

project(polygonization)

file(GLOB polygonization_SRC
    "src/*.cpp"
)

#find_package(CGAL COMPONENTS Qt4)
#include( ${CGAL_USE_FILE} )

add_library(polygonization ${polygonization_SRC})
#target_link_libraries( 
#    polygonization 
#    ${CGAL_LIBRARIES}  
#    ${CGAL_3RD_PARTY_LIBRARIES})
#target_link_libraries(polygonization utilities)

message(STATUS "We're done with polygonization.")



