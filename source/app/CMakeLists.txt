
# program_options_lite 
add_library(po STATIC "${PROJECT_SOURCE_DIR}/dependencies/program-options-lite/program_options_lite.cpp" )
target_include_directories(po INTERFACE "${PROJECT_SOURCE_DIR}/dependencies/" )


include_directories( ${CMAKE_SOURCE_DIR}/dependencies
                     ${CMAKE_SOURCE_DIR}/source/lib/vdmcLibCommon
                     ${CMAKE_SOURCE_DIR}/source/lib/vdmcLibEncoder
                     ${CMAKE_SOURCE_DIR}/source/lib/vdmcLibDecoder )

set(LIBS po vdmcLibCommon vdmcLibEncoder vdmcLibDecoder UVAtlas DirectX-Headers DirectXMath DirectXMesh )

file(GLOB files *.cpp )
foreach(file ${files})
  get_filename_component( app ${file} NAME_WE)
  add_executable( ${app} ${file} )
  target_link_libraries( ${app} ${LIBS} )
  install( TARGETS ${app} DESTINATION bin )
endforeach()


