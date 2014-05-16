cmake_minimum_required (VERSION 2.6)

project(lineclear)

file(GLOB lineclear_SRC
    "src/*.cpp"
    "src/*.c"
)

add_library(lineclear ${lineclear_SRC})
message("CMAKE_CXX_FLAGS " ${CMAKE_CXX_FLAGS}) 
find_package(CGAL COMPONENTS Qt4)
message("Cgal use file" ${CGAL_USE_FILE})
#set(CGAL_DONT_OVERRIDE_CMAKE_FLAGS TRUE)
include( ${CGAL_USE_FILE} )
message("CMAKE_CXX_FLAGS " ${CMAKE_CXX_FLAGS}) 
#set( QT_USE_QTXML    TRUE )
#set( QT_USE_QTMAIN   TRUE )
#set( QT_USE_QTSCRIPT  TRUE )
#set( QT_USE_QTOPENGL  TRUE )

find_package(Qt4 REQUIRED)
message("QT use file" ${QT_USE_FILE})
include(${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})
#QT4_WRAP_CPP(lineclear_HEADERS_MOC ${lineclear_HEADERS})
#
#if ( CGAL_FOUND AND CGAL_Qt4_FOUND AND QT4_FOUND )
#    #add_executable(LineClear ${lineclear_SRC} )
#    #target_link_libraries( LineClear ${QT_LIBRARIES})
#    #target_link_libraries( LineClear ${CGAL_LIBRARIES}  
#    #    ${CGAL_3RD_PARTY_LIBRARIES})
#
#    #add_executable(LineClearGui ${lineclear_SRC} ${lineclear_HEADERS_MOC})
#    #target_link_libraries( LineClearGui ${QT_LIBRARIES})
#    #target_link_libraries( LineClearGui ${CGAL_LIBRARIES} 
#    #    ${CGAL_3RD_PARTY_LIBRARIES})
#
#    
#
#else()
#  message(STATUS "Shit hit the fan.")
#endif()

