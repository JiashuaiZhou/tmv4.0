cmake_minimum_required(VERSION 3.17)

get_filename_component(MYNAME ${CMAKE_CURRENT_LIST_DIR} NAME)
string(REPLACE " " "_" MYNAME ${MYNAME})
project(${MYNAME} C CXX)

file(GLOB SRC *.h *.cpp )

include_directories( ${CMAKE_SOURCE_DIR}/dependencies 
                     ${CMAKE_SOURCE_DIR}/source/lib/vmeshCommon 
                     ${CMAKE_SOURCE_DIR}/dependencies/tinyply/source
                     . )

set( LIBS vmeshCommon )

# DRACO
if( USE_DRACO_GEOMETRY_CODEC )
  link_directories( ${CMAKE_BINARY_DIR}/lib ) 
  include_directories( ${CMAKE_SOURCE_DIR}/dependencies/draco/src/ 
                       ${CMAKE_CURRENT_BINARY_DIR}/../../../ )
  set( LIBS ${LIBS} draco::draco )
endif()
                     
add_library( ${MYNAME} ${SRC} )

target_link_libraries( ${MYNAME} ${LIBS} )

set_target_properties( ${MYNAME} PROPERTIES LINKER_LANGUAGE CXX)