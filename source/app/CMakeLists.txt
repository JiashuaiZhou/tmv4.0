
# program_options_lite 
add_library(po STATIC "${PROJECT_SOURCE_DIR}/dependencies/program-options-lite/program_options_lite.cpp" )
target_include_directories(po INTERFACE "${PROJECT_SOURCE_DIR}/dependencies/" )

########################### Unit tests ###########################

# Draco Encoder/decoder
create_executable( NAME      encodeDraco
                   SOURCES   unitTests/dracoEncoder.cpp 
                   LIBRARIES po geometryEncoder 
                   INCLUDES  ${CMAKE_SOURCE_DIR}/source/lib/vdmcUtil
                             ${CMAKE_SOURCE_DIR}/source/wrapper/geometryEncoder )
create_executable( NAME      decodeDraco 
                   SOURCES   unitTests/dracoDecoder.cpp 
                   LIBRARIES po geometryDecoder
                   INCLUDES  ${CMAKE_SOURCE_DIR}/source/lib/vdmcUtil
                             ${CMAKE_SOURCE_DIR}/source/wrapper/geometryDecoder )

# Video Encoder/decoder
create_executable( NAME      encodeVideo 
                   SOURCES   unitTests/videoEncoder.cpp 
                   LIBRARIES po videoEncoder
                   INCLUDES  ${CMAKE_SOURCE_DIR}/source/lib/vdmcUtil
                             ${CMAKE_SOURCE_DIR}/source/wrapper/videoEncoder )
create_executable( NAME      decodeVideo 
                   SOURCES   unitTests/videoDecoder.cpp 
                   LIBRARIES po videoDecoder
                   INCLUDES  ${CMAKE_SOURCE_DIR}/source/lib/vdmcUtil
                             ${CMAKE_SOURCE_DIR}/source/wrapper/videoDecoder )

# Colour Converter
create_executable( NAME      colourConverte
                   SOURCES   unitTests/colourConverter.cpp 
                   LIBRARIES po colourConverter
                   INCLUDES  ${CMAKE_SOURCE_DIR}/source/lib/vdmcUtil
                             ${CMAKE_SOURCE_DIR}/source/wrapper/colourConverter )

# Metrics
create_executable( NAME      metrics 
                   SOURCES   unitTests/metrics.cpp 
                   LIBRARIES po vdmcUtil
                   INCLUDES  ${CMAKE_SOURCE_DIR}/source/lib/vdmcUtil )

########################### Original applications ###########################

# Gengof
create_executable( NAME      gengof 
                   SOURCES   original/gengof.cpp 
                   LIBRARIES po vdmcUtil
                   INCLUDES  ${CMAKE_SOURCE_DIR}/source/lib/vdmcUtil )

# Simplify
create_executable( NAME      simplify 
                   SOURCES   original/simplify.cpp 
                   LIBRARIES po vdmcUtil vdmcEncoder 
                   INCLUDES  ${CMAKE_SOURCE_DIR}/source/lib/vdmcUtil
                             ${CMAKE_SOURCE_DIR}/source/lib/vdmcEncoder )

# Uvatlas
create_executable( NAME      uvatlas 
                   SOURCES   original/uvatlas.cpp 
                   LIBRARIES po vdmcUtil vdmcEncoder UVAtlas DirectX-Headers DirectXMath DirectXMesh
                   INCLUDES  ${CMAKE_SOURCE_DIR}/source/lib/vdmcUtil
                             ${CMAKE_SOURCE_DIR}/source/lib/vdmcEncoder ) 

# Fitsubdiv
create_executable( NAME      fitsubdiv 
                   SOURCES   original/fitsubdiv.cpp 
                   LIBRARIES po vdmcUtil vdmcEncoder 
                   INCLUDES  ${CMAKE_SOURCE_DIR}/source/lib/vdmcUtil
                             ${CMAKE_SOURCE_DIR}/source/lib/vdmcEncoder )
                             
# Vmcenc
create_executable( NAME      vmcenc 
                   SOURCES   original/vmcenc.cpp 
                   LIBRARIES po vdmcUtil vdmcEncoder 
                   INCLUDES  ${CMAKE_SOURCE_DIR}/source/lib/vdmcUtil
                             ${CMAKE_SOURCE_DIR}/source/lib/vdmcEncoder )  

# Vmcdec
create_executable( NAME      vmcdec 
                   SOURCES   original/vmcdec.cpp 
                   LIBRARIES po vdmcUtil vdmcDecoder 
                   INCLUDES  ${CMAKE_SOURCE_DIR}/source/lib/vdmcUtil
                             ${CMAKE_SOURCE_DIR}/source/lib/vdmcDecoder )

                             
########################### Main applications ###########################

# Encode
create_executable( NAME      encode 
                   SOURCES   encode.cpp 
                   LIBRARIES po vdmcUtil vdmcEncoder UVAtlas DirectX-Headers DirectXMath DirectXMesh
                   INCLUDES  ${CMAKE_SOURCE_DIR}/source/lib/vdmcUtil
                             ${CMAKE_SOURCE_DIR}/source/lib/vdmcEncoder )  

# Decode
create_executable( NAME      decode 
                   SOURCES   decode.cpp 
                   LIBRARIES po vdmcUtil vdmcDecoder 
                   INCLUDES  ${CMAKE_SOURCE_DIR}/source/lib/vdmcUtil
                             ${CMAKE_SOURCE_DIR}/source/lib/vdmcDecoder )


                             
########################### GTest applications ###########################

include( ${CMAKE_SOURCE_DIR}/dependencies/cmake/gtest.cmake )
enable_testing()
add_executable( hello_test tests/main.cpp )
target_link_libraries( hello_test gtest_main vdmcUtil)
include(GoogleTest)
gtest_discover_tests(hello_test)

