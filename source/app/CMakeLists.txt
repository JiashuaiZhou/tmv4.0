
# program_options_lite 
add_library(po STATIC "${PROJECT_SOURCE_DIR}/dependencies/program-options-lite/program_options_lite.cpp" )
target_include_directories(po INTERFACE "${PROJECT_SOURCE_DIR}/dependencies/" )

########################### Main applications ###########################

# Encode
create_executable( NAME      encode 
                   SOURCES   encode.cpp 
                   LIBRARIES po vmeshCommon vmeshEncoder UVAtlas DirectX-Headers DirectXMath DirectXMesh
                   INCLUDES  ${CMAKE_SOURCE_DIR}/source/lib/vmeshCommon
                             ${CMAKE_SOURCE_DIR}/source/lib/vmeshCommon/util
                             ${CMAKE_SOURCE_DIR}/source/lib/vmeshEncoder
                             ${CMAKE_SOURCE_DIR}/dependencies/tinyply/source )  

# Decode
create_executable( NAME      decode 
                   SOURCES   decode.cpp 
                   LIBRARIES po vmeshCommon vmeshDecoder 
                   INCLUDES  ${CMAKE_SOURCE_DIR}/source/lib/vmeshCommon
                             ${CMAKE_SOURCE_DIR}/source/lib/vmeshCommon/util
                             ${CMAKE_SOURCE_DIR}/source/lib/vmeshDecoder
                             ${CMAKE_SOURCE_DIR}/dependencies/tinyply/source )

  # Metrics
  create_executable( NAME     metrics 
                    SOURCES   metrics.cpp 
                    LIBRARIES po vmeshCommon
                    INCLUDES  ${CMAKE_SOURCE_DIR}/source/lib/vmeshCommon
                              ${CMAKE_SOURCE_DIR}/dependencies/tinyply/source )

########################### Unit tests ###########################

if ( BUILD_WRAPPER_APPS )
  # ply
  create_executable( NAME     objToPlyConvert
                    SOURCES   wrapper/objToPlyConvert.cpp 
                    LIBRARIES po vmeshCommon 
                    INCLUDES  ${CMAKE_SOURCE_DIR}/source/lib/vmeshCommon
                              ${CMAKE_SOURCE_DIR}/source/lib/vmeshCommon/util
                              ${CMAKE_SOURCE_DIR}/dependencies/tinyply/source )

  # Draco Encoder/decoder
  create_executable( NAME     decodeDraco 
                    SOURCES   wrapper/dracoDecoder.cpp 
                    LIBRARIES po geometryDecoder
                    INCLUDES  ${CMAKE_SOURCE_DIR}/source/lib/vmeshCommon
                              ${CMAKE_SOURCE_DIR}/source/wrapper/geometryDecoder
                              ${CMAKE_SOURCE_DIR}/dependencies/tinyply/source )

  # Video Encoder/decoder
  create_executable( NAME     encodeVideo 
                    SOURCES   wrapper/videoEncoder.cpp 
                    LIBRARIES po videoEncoder
                    INCLUDES  ${CMAKE_SOURCE_DIR}/source/lib/vmeshCommon
                              ${CMAKE_SOURCE_DIR}/source/wrapper/videoEncoder )

  create_executable( NAME     decodeVideo 
                    SOURCES   wrapper/videoDecoder.cpp 
                    LIBRARIES po videoDecoder
                    INCLUDES  ${CMAKE_SOURCE_DIR}/source/lib/vmeshCommon
                              ${CMAKE_SOURCE_DIR}/source/wrapper/videoDecoder )

  # Colour Converter
  create_executable( NAME     colourConverte
                    SOURCES   wrapper/colourConverter.cpp 
                    LIBRARIES po colourConverter
                    INCLUDES  ${CMAKE_SOURCE_DIR}/source/lib/vmeshCommon
                              ${CMAKE_SOURCE_DIR}/source/wrapper/colourConverter )
endif()
                                                        
########################### Test applications ###########################
if( BUILD_UNIT_TEST_APPS )
  CPMAddPackage( "gh:google/googletest#release-1.11.0" )
  enable_testing()

  create_executable( NAME     unitTests 
                    SOURCES   unitTests/main.cpp
                              unitTests/draco.cpp 
                              unitTests/metrics.cpp
                              unitTests/hm.cpp
                              unitTests/colourConvert.cpp
                              unitTests/vmesh.cpp
                    LIBRARIES gtest_main 
                              po
                              vmeshCommon 
                              vmeshEncoder 
                              vmeshDecoder 
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
                              ${CMAKE_SOURCE_DIR}/source/lib/vmeshCommon
                              ${CMAKE_SOURCE_DIR}/source/lib/vmeshEncoder 
                              ${CMAKE_SOURCE_DIR}/source/lib/vmeshDecoder 
                              ${CMAKE_SOURCE_DIR}/source/wrapper/geometryEncoder
                              ${CMAKE_SOURCE_DIR}/source/wrapper/geometryDecoder
                              ${CMAKE_SOURCE_DIR}/dependencies/tinyply/source )
  include(GoogleTest)
  gtest_discover_tests( unitTests )
endif()


