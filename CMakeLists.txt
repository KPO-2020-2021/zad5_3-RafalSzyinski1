cmake_minimum_required(VERSION 3.16)
project(Rotation3D)

set(EXECUTABLE ${CMAKE_PROJECT_NAME}_run)

set(CMAKE_CXX_STANDARD 17)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/lib/gplotpp)

add_subdirectory(lib/googletest)
add_subdirectory(test)

file(GLOB_RECURSE SOURCES LIST_DIRECTORIES true include/*.h source/*.cpp)

set(CMAKE_CXX_FLAGS "-pthread")
add_executable(${EXECUTABLE} ${SOURCES})
add_library(${CMAKE_PROJECT_NAME}_lib STATIC ${SOURCES})

# Doxygen

option(BUILD_DOC "Build documentation" ON)
find_package(Doxygen)
if (DOXYGEN_FOUND)
    set(DOXYGEN_IN ${CMAKE_SOURCE_DIR}/docs/Doxyfile.in)
    set(DOXYGEN_OUT ${CMAKE_BINARY_DIR}/Doxyfile)

    configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
    message("Doxygen build started")
    add_custom_target(doc_doxygen ALL COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT} WORKING_DIRECTORY ${CMAKE_BINARY_DIR} COMMENT "Generate doxygen" VERBATIM)
    set(GRAPH_DIR ${CMAKE_BINARY_DIR}/doc_doxygen/html/docs)
    file(MAKE_DIRECTORY ${GRAPH_DIR})
    add_custom_target(doc_doxygenGraph ALL COMMAND cp ${CMAKE_SOURCE_DIR}/docs/Graph-Drone.png ${GRAPH_DIR} COMMENT "Graph copy" VERBATIM)
else(DOXYGEN_FOUND)
    message("Doxygen not found, need to be installed to generate the doxygen")
endif()