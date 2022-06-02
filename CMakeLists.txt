cmake_minimum_required(VERSION 3.17)

project(vmesh)

set(CMAKE_CXX_STANDARD 14)

# Option 
# Video encoder and decoder library and application:
OPTION( USE_JMAPP_VIDEO_CODEC     "Use JM application to encode and decode video"           FALSE )  # JMAPP  = 0
OPTION( USE_HMAPP_VIDEO_CODEC     "Use HM application to encode and decode video"           TRUE  )  # HMAPP  = 1
OPTION( USE_SHMAPP_VIDEO_CODEC    "Use SHM application to encode and decode video"          FALSE )  # SHMAPP = 2    
OPTION( USE_JMLIB_VIDEO_CODEC     "Clone, build and use JM video encoder and decoder"       FALSE )  # JMLIB  = 3
OPTION( USE_HMLIB_VIDEO_CODEC     "Clone, build and use HM video encoder and decoder"       TRUE  )  # HMLIB  = 4
OPTION( USE_VTMLIB_VIDEO_CODEC    "Clone, build and use VTM video encoder and decoder"      TRUE )   # VTMLIB = 5
OPTION( USE_FFMPEG_VIDEO_CODEC    "Clone, build and use FFMPEG video encoder and decoder"   FALSE )  # FFMPEG = 6                                                                                                 #UNKNOWN_CODEC = 7

# Color conversion library:
OPTION( USE_HDRTOOLS              "Clone, build and use HDRTools for color conversions"     FALSE )

SET( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/lib )
SET( CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/lib )
SET( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin )

include( dependencies/cmake/CPM.cmake )
include( dependencies/cmake/tools.cmake )
include( dependencies/cmake/geometry/draco.cmake )
include( dependencies/cmake/geometry/directx.cmake )
include( dependencies/cmake/geometry/uvatlas.cmake )
include( dependencies/cmake/geometry/mmetric.cmake )
if ( USE_HDRTOOLS )
    include( dependencies/cmake/video/hdrtools.cmake )
endif()


add_subdirectory(source)

