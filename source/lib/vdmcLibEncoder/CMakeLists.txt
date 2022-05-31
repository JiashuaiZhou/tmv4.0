get_filename_component(MYNAME ${CMAKE_CURRENT_LIST_DIR} NAME)
string(REPLACE " " "_" MYNAME ${MYNAME})
project( ${MYNAME} C CXX )

enable_language( C )

file(GLOB SRC *.h *.cpp )

include_directories( ${CMAKE_SOURCE_DIR}/dependencies 
                     ${CMAKE_SOURCE_DIR}/source/lib/vdmcLibCommon )
 
add_library( ${MYNAME} ${SRC}  ${CMAKE_SOURCE_DIR}/dependencies/schroedinger/schroarith.c )

set_target_properties( ${MYNAME} PROPERTIES LINKER_LANGUAGE CXX)

