get_filename_component(MYNAME ${CMAKE_CURRENT_LIST_DIR} NAME)
string(REPLACE " " "_" MYNAME ${MYNAME})
project( ${MYNAME} C CXX )

enable_language( C )

file(GLOB SRC *.cpp )

include_directories( ${CMAKE_SOURCE_DIR}/dependencies/libmd5
                     ${CMAKE_SOURCE_DIR}/source/lib/v3cBitstream/syntaxElements
                     ${CMAKE_SOURCE_DIR}/source/lib/vmeshCommon
                     . )
 
add_library( ${MYNAME} ${SRC} )

target_link_libraries( ${MYNAME} )

set_target_properties( ${MYNAME} PROPERTIES LINKER_LANGUAGE CXX)


