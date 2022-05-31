cmake_minimum_required(VERSION 3.17)

get_filename_component(MYNAME ${CMAKE_CURRENT_LIST_DIR} NAME)
string(REPLACE " " "_" MYNAME ${MYNAME})
set( MYNAME ${MYNAME}${CMAKE_DEBUG_POSTFIX} )
project(${MYNAME} C CXX)

file(GLOB SRC *.h *.cpp )

include_directories( ${CMAKE_SOURCE_DIR}/dependencies
                     ${CMAKE_SOURCE_DIR}/source/lib/vdmcLibCommon 
                     ${CMAKE_SOURCE_DIR}/source/lib/vdmcLibEncoder  )

add_executable( ${MYNAME} ${SRC} )

target_link_libraries( ${MYNAME} po vdmcLibCommon vdmcLibEncoder )

install( TARGETS ${MYNAME} DESTINATION bin )