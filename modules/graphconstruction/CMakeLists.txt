cmake_minimum_required (VERSION 2.6)
project(graphconstruction)

file(GLOB graphconstruction_SRC
    "src/*.cpp"
    "src/*.c"
)

# FIND PACKAGES AND DEFINE INCLUDE DIRECTORIES 
include_directories(../include)

find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS}) 
find_package( OpenCV REQUIRED COMPONENTS)
include_directories( ${OpenCV_INCLUDE_DIRS} )
find_package(CGAL)
set(CGAL_DONT_OVERRIDE_CMAKE_FLAGS TRUE)
include( ${CGAL_USE_FILE} )

# DEFINE TARGETS (IMPORTANT TO HAVE THIS AFTER USE FILES)
add_executable(graphconstruction_exe ${graphconstruction_SRC})

# LINK TARGETS
target_link_libraries(graphconstruction_exe
    polygonization
    ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES}
    ${Boost_LIBRARIES}
    ${OpenCV_LIBS})
