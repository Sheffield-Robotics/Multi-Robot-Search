cmake_minimum_required (VERSION 2.6)
set(CMAKE_BUILD_TYPE "Release")
EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )
message( STATUS "Architecture: ${ARCHITECTURE}" )

set(CMAKE_CXX_FLAGS "-pipe -Os -stdlib=libc++")
#set(CMAKE_OSX_ARCHITECTURES ${ARCHS_STANDARD_64_BIT})

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/../cmake/")
message("Cmake module path" ${CMAKE_MODULE_PATH}) 
message("CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS}")
if(APPLE)
    include(CheckCXXCompilerFlag)
    CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
    CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
    if(COMPILER_SUPPORTS_CXX11)
    	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    elseif(COMPILER_SUPPORTS_CXX0X)
    	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
    else()
      message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
    endif()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -I/opt/local/include")
    #set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pipe -Os")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -arch x86_64") #i386
    set(CMAKE_EXE_LINKER_FLAGS "-L/opt/local/lib -Wl,-headerpad_max_install_names -arch x86_64 -Wl,-dylib_file,/System/Library/Frameworks/OpenGL.framework/Versions/A/Libraries/libGL.dylib:/System/Library/Frameworks/OpenGL.framework/Versions/A/Libraries/libGL.dylib")
endif()
message("CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS}")
message("CMAKE_EXE_LINKER_FLAGS" ${CMAKE_EXE_LINKER_FLAGS})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ../../bin)

include_directories(include)

add_subdirectory(agents)
add_subdirectory(utilities)
add_subdirectory(uavmodel)
add_subdirectory(heightmap)
add_subdirectory(planner)
add_subdirectory(polygonization)
add_subdirectory(hungarian)
add_subdirectory(graphclear)
#add_subdirectory(graphclear_legacy)
add_subdirectory(lineclear)

add_subdirectory(gui)

add_subdirectory(graphconstruction)


