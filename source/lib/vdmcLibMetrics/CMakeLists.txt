get_filename_component(MYNAME ${CMAKE_CURRENT_LIST_DIR} NAME)
string(REPLACE " " "_" MYNAME ${MYNAME})
project( ${MYNAME} C CXX )

enable_language( C )

file(GLOB SRC *.h *.cpp )

include_directories( ${CMAKE_SOURCE_DIR}/source/lib/vdmcLibCommon 
                     ${CMAKE_SOURCE_DIR}/dependencies/mmetric/source/lib/include
                     ${CMAKE_SOURCE_DIR}/dependencies/mmetric/dependencies
                     ${CMAKE_SOURCE_DIR}/dependencies/mmetric/dependencies/glad/include
                     ${CMAKE_SOURCE_DIR}/dependencies/mmetric/dependencies/glfw/include)
 
add_library( ${MYNAME} ${SRC} )

set_target_properties( ${MYNAME} PROPERTIES LINKER_LANGUAGE CXX)

