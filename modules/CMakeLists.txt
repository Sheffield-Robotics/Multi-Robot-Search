cmake_minimum_required (VERSION 2.6)

if (NOT CMAKE_BUILD_TYPE)
    message(STATUS "Default is Release")
    set(CMAKE_BUILD_TYPE "Release")
endif()

EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )
message( STATUS "Architecture: ${ARCHITECTURE}" )
#set(CMAKE_OSX_ARCHITECTURES ${ARCHS_STANDARD_64_BIT})
#message( STATUS "Architecture: ${ARCHITECTURE}" )

message("CMAKE_EXE_LINKER_FLAGS" ${CMAKE_EXE_LINKER_FLAGS})


if(APPLE)
    set(CMAKE_CXX_FLAGS "-pipe -Os -I/opt/local/include -arch x86_64") 
    set(CMAKE_EXE_LINKER_FLAGS "-L/opt/local/lib -Wl,-headerpad_max_install_names -arch x86_64 -Wl,-dylib_file,/System/Library/Frameworks/OpenGL.framework/Versions/A/Libraries/libGL.dylib:/System/Library/Frameworks/OpenGL.framework/Versions/A/Libraries/libGL.dylib")
endif()

find_package(QGLViewer)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ../../bin)

include_directories(include)

add_subdirectory(utilities)
add_subdirectory(uavmodel)
add_subdirectory(heightmap)
add_subdirectory(planner)
add_subdirectory(polygonization)
add_subdirectory(hungarian)
add_subdirectory(agents)
add_subdirectory(lineclear)

add_subdirectory(gui)
