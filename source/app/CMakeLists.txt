
# program_options_lite 
add_library(po STATIC "${PROJECT_SOURCE_DIR}/dependencies/program-options-lite/program_options_lite.cpp" )
target_include_directories(po INTERFACE "${PROJECT_SOURCE_DIR}/dependencies/" )

# Draco Encoder/decoder
create_executable( NAME      encodeDraco
                   SOURCES   unitTesting/dracoEncoder.cpp 
                   LIBRARIES po geometryEncoder 
                   INCLUDES  ${CMAKE_SOURCE_DIR}/source/lib/vdmcUtil
                             ${CMAKE_SOURCE_DIR}/source/wrapper/geometryEncoder )
create_executable( NAME      decodeDraco 
                   SOURCES   unitTesting/dracoDecoder.cpp 
                   LIBRARIES po geometryDecoder
                   INCLUDES  ${CMAKE_SOURCE_DIR}/source/lib/vdmcUtil
                             ${CMAKE_SOURCE_DIR}/source/wrapper/geometryDecoder )

# Video Encoder/decoder
create_executable( NAME      encodeVideo 
                   SOURCES   unitTesting/videoEncoder.cpp 
                   LIBRARIES po videoEncoder
                   INCLUDES  ${CMAKE_SOURCE_DIR}/source/lib/vdmcUtil
                             ${CMAKE_SOURCE_DIR}/source/wrapper/videoEncoder )
create_executable( NAME      decodeVideo 
                   SOURCES   unitTesting/videoDecoder.cpp 
                   LIBRARIES po videoDecoder
                   INCLUDES  ${CMAKE_SOURCE_DIR}/source/lib/vdmcUtil
                             ${CMAKE_SOURCE_DIR}/source/wrapper/videoDecoder )

# Colour Converter
create_executable( NAME      colourConverte
                   SOURCES   unitTesting/colourConverter.cpp 
                   LIBRARIES po colourConverter
                   INCLUDES  ${CMAKE_SOURCE_DIR}/source/lib/vdmcUtil
                             ${CMAKE_SOURCE_DIR}/source/wrapper/colourConverter )

# Metrics
create_executable( NAME      metrics 
                   SOURCES   unitTesting/metrics.cpp 
                   LIBRARIES po vdmcUtil
                   INCLUDES  ${CMAKE_SOURCE_DIR}/source/lib/vdmcUtil )



