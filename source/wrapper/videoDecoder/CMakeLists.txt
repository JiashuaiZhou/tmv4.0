cmake_minimum_required(VERSION 3.17)

get_filename_component(MYNAME ${CMAKE_CURRENT_LIST_DIR} NAME)
string(REPLACE " " "_" MYNAME ${MYNAME})
project(${MYNAME} C CXX)

file(GLOB SRC  *.h 
               *.cpp  
               ${HM_LIB_SOURCE_DIR}/TAppCommon/* )

include_directories( . ${CMAKE_SOURCE_DIR}/source/lib/vdmcUtil  )

set( LIBS vdmcUtil )

# HM 
if( USE_HM_VIDEO_CODEC )
  include_directories( ${HM_LIB_SOURCE_DIR}/  )
  set( LIBS ${LIBS} TLibCommon TLibDecoder )
endif()

# VTM 
if( USE_VTM_VIDEO_CODEC )
set( LIBS ${LIBS} CommonLib DecoderLib_vtm )
endif()

# FFMPEG
if( USE_FFMPEG_VIDEO_CODEC )  
  include_directories( ${FFMPEG_INCLUDE} )
  if( WIN32 OR MSVC OR MSYS OR MINGW )
    link_directories( ${FFMPEG_DEV_DIR}/lib ) 
  endif()  
  set( LIBS ${LIBS} ${AVCODEC_LIBRARY} ${AVUTIL_LIBRARY} ${AVFORMAT_LIBRARY} )
endif()

add_library( ${MYNAME}  ${SRC} )

If( MSVC )
  target_link_libraries( ${MYNAME} ${LIBS} ws2_32)
ELSE()
  target_link_libraries( ${MYNAME} ${LIBS} )
endif()

set_target_properties( ${MYNAME} PROPERTIES LINKER_LANGUAGE CXX)
