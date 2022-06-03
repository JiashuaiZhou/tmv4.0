cmake_minimum_required(VERSION 3.17)

project(vmesh)

set(CMAKE_CXX_STANDARD 14)

# Options:
# Video encoder and decoder library and application:
OPTION( USE_HM_VIDEO_CODEC     "Use HM video encoder and decoder"       TRUE )  # 0
OPTION( USE_VTM_VIDEO_CODEC    "Use VTM video encoder and decoder"      TRUE )  # 1
OPTION( USE_FFMPEG_VIDEO_CODEC "Use FFMPEG video encoder and decoder"   TRUE )  # 2

# Color conversion library:
OPTION( USE_HDRTOOLS           "Use HDRTools for color conversions"     TRUE )

# Use HM optimized RDO
OPTION( USE_HM_PCC_RDO         "Use HM optimized RDO "                  TRUE )

# Dependencies
include( dependencies/cmake/CPM.cmake )
include( dependencies/cmake/tools.cmake )
include( dependencies/cmake/geometry/draco.cmake )
include( dependencies/cmake/geometry/directx.cmake )
include( dependencies/cmake/geometry/uvatlas.cmake )
include( dependencies/cmake/geometry/mmetric.cmake )
if ( USE_HDRTOOLS )
    include( dependencies/cmake/video/hdrtools.cmake )
endif()
if ( USE_HM_VIDEO_CODEC )
    include( dependencies/cmake/video/hm.cmake )
endif()
if ( USE_VTM_VIDEO_CODEC )
    include( dependencies/cmake/video/vtm.cmake )
endif()
if ( USE_FFMPEG_VIDEO_CODEC )
    include( dependencies/cmake/video/ffmpeg.cmake )
endif()

# Build
add_subdirectory(source)

