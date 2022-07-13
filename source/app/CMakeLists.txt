
# program_options_lite 
add_library(po STATIC "${PROJECT_SOURCE_DIR}/dependencies/program-options-lite/program_options_lite.cpp" )
target_include_directories(po INTERFACE "${PROJECT_SOURCE_DIR}/dependencies/" )

########################### Main applications ###########################

# Encode
create_executable( NAME      encode 
                   SOURCES   encode.cpp 
                   LIBRARIES po vdmcCommon vdmcEncoder UVAtlas DirectX-Headers DirectXMath DirectXMesh
                   INCLUDES  ${CMAKE_SOURCE_DIR}/source/lib/vdmcCommon
                             ${CMAKE_SOURCE_DIR}/source/lib/vdmcEncoder )  

# Decode
create_executable( NAME      decode 
                   SOURCES   decode.cpp 
                   LIBRARIES po vdmcCommon vdmcDecoder 
                   INCLUDES  ${CMAKE_SOURCE_DIR}/source/lib/vdmcCommon
                             ${CMAKE_SOURCE_DIR}/source/lib/vdmcDecoder )

########################### Unit tests ###########################
if ( BUILD_WRAPPER_APPS )
  # Draco Encoder/decoder
  create_executable( NAME      encodeDraco
                    SOURCES   wrapper/dracoEncoder.cpp 
                    LIBRARIES po geometryEncoder 
                    INCLUDES  ${CMAKE_SOURCE_DIR}/source/lib/vdmcCommon
                              ${CMAKE_SOURCE_DIR}/source/wrapper/geometryEncoder )

  create_executable( NAME      decodeDraco 
                    SOURCES   wrapper/dracoDecoder.cpp 
                    LIBRARIES po geometryDecoder
                    INCLUDES  ${CMAKE_SOURCE_DIR}/source/lib/vdmcCommon
                              ${CMAKE_SOURCE_DIR}/source/wrapper/geometryDecoder )

  # Video Encoder/decoder
  create_executable( NAME      encodeVideo 
                    SOURCES   wrapper/videoEncoder.cpp 
                    LIBRARIES po videoEncoder
                    INCLUDES  ${CMAKE_SOURCE_DIR}/source/lib/vdmcCommon
                              ${CMAKE_SOURCE_DIR}/source/wrapper/videoEncoder )

  create_executable( NAME      decodeVideo 
                    SOURCES   wrapper/videoDecoder.cpp 
                    LIBRARIES po videoDecoder
                    INCLUDES  ${CMAKE_SOURCE_DIR}/source/lib/vdmcCommon
                              ${CMAKE_SOURCE_DIR}/source/wrapper/videoDecoder )

  # Colour Converter
  create_executable( NAME      colourConverte
                    SOURCES   wrapper/colourConverter.cpp 
                    LIBRARIES po colourConverter
                    INCLUDES  ${CMAKE_SOURCE_DIR}/source/lib/vdmcCommon
                              ${CMAKE_SOURCE_DIR}/source/wrapper/colourConverter )

  # Metrics
  create_executable( NAME      metrics 
                    SOURCES   wrapper/metrics.cpp 
                    LIBRARIES po vdmcCommon
                    INCLUDES  ${CMAKE_SOURCE_DIR}/source/lib/vdmcCommon )
endif()

########################### Original applications ###########################

if( BUILD_ORIGINAL_APPS )
  # Gengof
  create_executable( NAME      gengof 
                    SOURCES   original/gengof.cpp 
                    LIBRARIES po vdmcCommon vdmcEncoder UVAtlas DirectX-Headers DirectXMath DirectXMesh
                    INCLUDES  ${CMAKE_SOURCE_DIR}/source/lib/vdmcCommon
                              ${CMAKE_SOURCE_DIR}/source/lib/vdmcEncoder )

  # Simplify
  create_executable( NAME      simplify 
                    SOURCES   original/simplify.cpp 
                    LIBRARIES po vdmcCommon vdmcEncoder UVAtlas DirectX-Headers DirectXMath DirectXMesh 
                    INCLUDES  ${CMAKE_SOURCE_DIR}/source/lib/vdmcCommon
                              ${CMAKE_SOURCE_DIR}/source/lib/vdmcEncoder )

  # Uvatlas
  create_executable( NAME      uvatlasapp
                    SOURCES   original/uvatlas.cpp
                    LIBRARIES po vdmcCommon vdmcEncoder UVAtlas DirectX-Headers DirectXMath DirectXMesh
                    INCLUDES  ${CMAKE_SOURCE_DIR}/source/lib/vdmcCommon
                              ${CMAKE_SOURCE_DIR}/source/lib/vdmcEncoder ) 

  # Fitsubdiv
  create_executable( NAME      fitsubdiv 
                    SOURCES   original/fitsubdiv.cpp 
                    LIBRARIES po vdmcCommon vdmcEncoder UVAtlas DirectX-Headers DirectXMath DirectXMesh 
                    INCLUDES  ${CMAKE_SOURCE_DIR}/source/lib/vdmcCommon
                              ${CMAKE_SOURCE_DIR}/source/lib/vdmcEncoder )
                              
  # Vmcenc
  create_executable( NAME      vmcenc 
                    SOURCES   original/vmcenc.cpp 
                    LIBRARIES po vdmcCommon vdmcEncoder UVAtlas DirectX-Headers DirectXMath DirectXMesh 
                    INCLUDES  ${CMAKE_SOURCE_DIR}/source/lib/vdmcCommon
                              ${CMAKE_SOURCE_DIR}/source/lib/vdmcEncoder )  

  # Vmcdec
  create_executable( NAME      vmcdec 
                    SOURCES   original/vmcdec.cpp 
                    LIBRARIES po vdmcCommon vdmcDecoder 
                    INCLUDES  ${CMAKE_SOURCE_DIR}/source/lib/vdmcCommon
                              ${CMAKE_SOURCE_DIR}/source/lib/vdmcDecoder )
endif()
                                                        
########################### Test applications ###########################
if( BUILD_TEST_APPS )
  CPMAddPackage( "gh:google/googletest#release-1.11.0" )
  enable_testing()

  create_executable( NAME     tests 
                    SOURCES   tests/main.cpp
                              tests/draco.cpp 
                              tests/metrics.cpp
                              tests/hm.cpp
                              tests/colourConvert.cpp
                              tests/gengof.cpp
                              tests/geometryDecimate.cpp
                              tests/textureParametrization.cpp
                              tests/geometryParametrization.cpp
                              tests/decode.cpp
                              tests/encode.cpp
                              tests/allchain.cpp
                    LIBRARIES gtest_main 
                              po
                              vdmcCommon 
                              vdmcEncoder 
                              vdmcDecoder 
                              geometryEncoder 
                              geometryDecoder 
                              videoEncoder 
                              videoDecoder 
                              colourConverter
                              UVAtlas 
                              DirectX-Headers 
                              DirectXMath 
                              DirectXMesh
                    INCLUDES  ${CMAKE_SOURCE_DIR}/dependencies
                              ${CMAKE_SOURCE_DIR}/source/lib/vdmcCommon
                              ${CMAKE_SOURCE_DIR}/source/lib/vdmcEncoder 
                              ${CMAKE_SOURCE_DIR}/source/lib/vdmcDecoder 
                              ${CMAKE_SOURCE_DIR}/source/wrapper/geometryEncoder
                              ${CMAKE_SOURCE_DIR}/source/wrapper/geometryDecoder  )
  include(GoogleTest)
  gtest_discover_tests( tests )
endif()


