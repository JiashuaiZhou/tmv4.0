cmake_minimum_required(VERSION 3.17)

get_filename_component(MYNAME ${CMAKE_CURRENT_LIST_DIR} NAME)
string(REPLACE " " "_" MYNAME ${MYNAME})
project(${MYNAME} C CXX)

file(GLOB SRC *.h *.cpp )

include_directories( ${CMAKE_SOURCE_DIR}/dependencies 
                     ${CMAKE_SOURCE_DIR}/source/lib/vdmcLibCommon )

set( LIBS PccLibCommon )
include_directories( . ${CMAKE_SOURCE_DIR}/source/lib/vdmcLibCommon )

message("CMAKE_CURRENT_BINARY_DIR = ${CMAKE_CURRENT_BINARY_DIR}/../../../draco")

# DRACO
message("CMAKE_BUILD_DIRECTORY  = ${CMAKE_BUILD_DIRECTORY}")
if( USE_DRACO_GEOMETRY_CODEC )
  add_compile_definitions(USE_DRACO_GEOMETRY_CODEC)
  include_directories( ${CMAKE_SOURCE_DIR}/dependencies/draco/src/ 
                       ${CMAKE_CURRENT_BINARY_DIR}/../../../ )
  set( LIBS ${LIBS} draco )
endif()
                     
add_library( ${MYNAME} ${SRC} )

target_link_libraries( ${MYNAME} ${LIBS} )

set_target_properties( ${MYNAME} PROPERTIES LINKER_LANGUAGE CXX)