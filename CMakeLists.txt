cmake_minimum_required(VERSION 3.17)
project(vmesh CXX C )

# Options:
set(VDMC_VERSION_MAJOR 1 )
set(VDMC_VERSION_MINOR 0 )

# Video encoder and decoder:
OPTION( USE_HM_VIDEO_CODEC       "Use HM video encoder and decoder"       TRUE )  # 0
OPTION( USE_VTM_VIDEO_CODEC      "Use VTM video encoder and decoder"      TRUE )  # 1
OPTION( USE_FFMPEG_VIDEO_CODEC   "Use FFMPEG video encoder and decoder"   TRUE )  # 2

# Geometry encoder and decoder:
OPTION( USE_DRACO_GEOMETRY_CODEC "Use draco geometry encoder and decoder" TRUE )  # 0

# Color conversion library:
OPTION( USE_HDRTOOLS             "Use HDRTools for color conversions"     TRUE )

# Use HM optimized RDO
OPTION( USE_HM_PCC_RDO           "Use HM optimized RDO "                  TRUE )

# Test applications
OPTION( BUILD_ORIGINAL_APPS      "Build original applications"            TRUE )
OPTION( BUILD_WRAPPER_APPS       "Build wrapper applications"             TRUE )
OPTION( BUILD_TEST_APPS          "Build test applications"                TRUE )

# Building options
set(CMAKE_CXX_STANDARD          17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS        OFF)
enable_language(C)
if (MSVC)
  add_compile_options(/W4)
else ()
  add_compile_options( -Wshadow -Wall -Wextra -pedantic -Wno-conversion -Wno-unused-parameter
                        -Wno-sign-conversion -Wno-long-long -Wno-unused-function )
endif()
if (UNIX)
  add_definitions(-D_POSIX_C_SOURCE=200809L)
elseif (WIN32)
  add_definitions(-D_USE_MATH_DEFINES)
endif()

# Dependencies
include( dependencies/cmake/CPM.cmake )
include( dependencies/cmake/tools.cmake )
include( dependencies/cmake/geometry/directx.cmake )
include( dependencies/cmake/geometry/uvatlas.cmake )
include( dependencies/cmake/geometry/mmetric.cmake )
if ( USE_HDRTOOLS )
  add_compile_definitions(USE_HDRTOOLS)
  include( dependencies/cmake/video/hdrtools.cmake )
endif()
if ( USE_HM_VIDEO_CODEC )
  add_compile_definitions(USE_HM_VIDEO_CODEC)
  include( dependencies/cmake/video/hm.cmake )
endif()
if ( USE_VTM_VIDEO_CODEC )
  add_compile_definitions(USE_VTM_VIDEO_CODEC)
  include( dependencies/cmake/video/vtm.cmake )
endif()
if ( USE_FFMPEG_VIDEO_CODEC )
  add_compile_definitions(USE_FFMPEG_VIDEO_CODEC)
  include( dependencies/cmake/video/ffmpeg.cmake )
endif()
if ( USE_DRACO_GEOMETRY_CODEC ) 
  add_compile_definitions(USE_DRACO_GEOMETRY_CODEC)
  add_compile_definitions(USE_DRACO_GEOMETRY_CODEC) 
  include( dependencies/cmake/geometry/draco.cmake )
endif()

# Build
add_all_subdirectory(source/wrapper)
add_all_subdirectory(source/lib)
add_subdirectory(source/app)

