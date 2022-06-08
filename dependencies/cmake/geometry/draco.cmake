set( DIR ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/draco )
if( NOT EXISTS ${DIR}/CMakeLists.txt )
  CPMAddPackage( NAME             draco
                GIT_REPOSITORY    https://github.com/google/draco.git
                GIT_TAG           1.4.3
                SOURCE_DIR        ${DIR}  
                DOWNLOAD_ONLY     YES)
endif()

if( EXISTS ${DIR}/CMakeLists.txt )
  if( NOT EXISTS ${DIR}/PATCHED )  
    file(GLOB files "${CMAKE_CURRENT_SOURCE_DIR}/dependencies/patches/draco/*")
    message("CREATE PATCH ")
    message("CMAKE_CURRENT_SOURCE_DIR = ${CMAKE_CURRENT_SOURCE_DIR}")
    foreach(file ${files})
      message("git am ${file}")
      execute_process( COMMAND git am ${file} WORKING_DIRECTORY ${DIR} RESULT_VARIABLE ret )
      if( NOT ${ret} EQUAL "0")
        message( FATAL_ERROR "Error during the draco patch process. ")
      endif()
    endforeach()
    file( WRITE ${DIR}/PATCHED "patched" )   
  endif()
endif()

add_subdirectory(dependencies/draco)

IF (MSVC)
  SET( DRACO_LIB draco )
ELSE()
  SET( DRACO_LIB ${CMAKE_BINARY_DIR}/lib/libdraco.a )
ENDIF()
