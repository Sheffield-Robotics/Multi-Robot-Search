cmake_minimum_required (VERSION 2.6)

project(graphclear_legacy)

file(GLOB graphclear_legacy_SRC
    "src/*.cpp"
    "src/*.c"
)

EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )
message( STATUS "Architecture: ${ARCHITECTURE}" )
#set(CMAKE_OSX_ARCHITECTURES ${ARCHS_STANDARD_64_BIT})
#message( STATUS "Architecture: ${ARCHITECTURE}" )

message("CMAKE_EXE_LINKER_FLAGS" ${CMAKE_EXE_LINKER_FLAGS})

message("Cmake module path" ${CMAKE_MODULE_PATH}) 
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/../../cmake/")
message("Cmake module path" ${CMAKE_MODULE_PATH}) 

if(APPLE)
    set(CMAKE_CXX_FLAGS "-pipe -Os -I/opt/local/include -arch x86_64") 
    set(CMAKE_EXE_LINKER_FLAGS "-L/opt/local/lib -Wl,-headerpad_max_install_names -arch x86_64 -Wl,-dylib_file,/System/Library/Frameworks/OpenGL.framework/Versions/A/Libraries/libGL.dylib:/System/Library/Frameworks/OpenGL.framework/Versions/A/Libraries/libGL.dylib")
endif()

#add_library(graphclear_legacy ${graphclear_legacy_SRC})
add_executable(graphclear_legacy_exe ${graphclear_legacy_SRC})

find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS}) 
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
find_package(CGAL)
set(CGAL_DONT_OVERRIDE_CMAKE_FLAGS TRUE)
include( ${CGAL_USE_FILE} )
find_package( LibConfig REQUIRED )
include_directories( ${LIBCONFIGPP_INCLUDE_DIR} )
find_package( GSL REQUIRED )
include_directories( ${GSL_INCLUDE_DIR} )


include_directories(../include)

#include_directories(../3rdparty/gslwrap)
file(GLOB GSLWRAP_SRCS ../3rdparty/gslwrap/*.cc)
add_library(gslwrap ${GSLWRAP_SRCS})
include_directories(../3rdparty)

include_directories(${PROJECT_SOURCE_DIR})

target_link_libraries(graphclear_legacy_exe
    gslwrap
    ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES}
    ${OpenCV_LIBS}
    ${GSL_LIBRARIES}
    ${LIBCONFIGPP_LIBRARY}
    ${Boost_LIBRARIES})

