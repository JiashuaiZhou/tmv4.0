CMAKE_MINIMUM_REQUIRED(VERSION 3.17)

GET_FILENAME_COMPONENT(MYNAME ${CMAKE_CURRENT_LIST_DIR} NAME)
STRING(REPLACE " " "_" MYNAME ${MYNAME})
PROJECT(${MYNAME} C CXX)

FILE(GLOB SRC *.h *.cpp  
              ${CMAKE_SOURCE_DIR}/dependencies/hdrtools/projects/HDRConvert/inc/ProjectParameters.h
              ${CMAKE_SOURCE_DIR}/dependencies/hdrtools/projects/HDRConvert/src/ProjectParameters.cpp )

INCLUDE_DIRECTORIES( ${CMAKE_SOURCE_DIR}/dependencies 
                     ${CMAKE_SOURCE_DIR}/source/lib/vdmcLibCommon
                     ${CMAKE_SOURCE_DIR}/dependencies/hdrtools/common/inc
                     ${CMAKE_SOURCE_DIR}/dependencies/hdrtools/projects/HDRConvert/inc )

SET( LIBS vdmcLibCommon )
IF( USE_HDRTOOLS )
  add_compile_definitions(USE_HDRTOOLS=1)
  SET( LIBS ${LIBS} HDRLib )
ENDIF()

ADD_LIBRARY( ${MYNAME} ${SRC} )
TARGET_LINK_LIBRARIES( ${MYNAME} ${LIBS} )

SET_TARGET_PROPERTIES( ${MYNAME} PROPERTIES LINKER_LANGUAGE CXX)