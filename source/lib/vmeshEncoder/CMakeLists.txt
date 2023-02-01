get_filename_component(MYNAME ${CMAKE_CURRENT_LIST_DIR} NAME)
string(REPLACE " " "_" MYNAME ${MYNAME})
project( ${MYNAME} C CXX )

enable_language( C )

file(GLOB SRC *.h *.cpp )

include_directories( ${CMAKE_SOURCE_DIR}/dependencies 
                     ${CMAKE_SOURCE_DIR}/dependencies/uvatlas/UVAtlas/inc
                     ${CMAKE_SOURCE_DIR}/dependencies/directx-math/Inc
                     ${CMAKE_SOURCE_DIR}/dependencies/directx-mesh/DirectXMesh
                     ${CMAKE_SOURCE_DIR}/dependencies/directx-headers/include
                     ${CMAKE_SOURCE_DIR}/source/lib/vmeshCommon
                     ${CMAKE_SOURCE_DIR}/source/wrapper/videoEncoder
                     ${CMAKE_SOURCE_DIR}/source/wrapper/geometryEncoder
                     ${CMAKE_SOURCE_DIR}/source/wrapper/colourConverter
                     ${CMAKE_SOURCE_DIR}/dependencies/mmetric/source/lib/include
                     ${CMAKE_SOURCE_DIR}/dependencies/mmetric/dependencies/
                     ${CMAKE_SOURCE_DIR}/dependencies/mmetric/dependencies/glad/include
                     ${CMAKE_SOURCE_DIR}/dependencies/mmetric/dependencies/glfw/include
                     ${CMAKE_SOURCE_DIR}/dependencies/tinyply/source )
 
add_library( ${MYNAME} ${SRC} ${CMAKE_SOURCE_DIR}/dependencies/schroedinger/schroarith.c )

target_link_libraries( ${MYNAME} vmeshCommon geometryEncoder videoEncoder colourConverter )

set_target_properties( ${MYNAME} PROPERTIES LINKER_LANGUAGE CXX)

