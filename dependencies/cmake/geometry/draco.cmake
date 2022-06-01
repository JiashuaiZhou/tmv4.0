set( DIR ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/draco )
if( NOT EXISTS ${DIR} )
  CPMAddPackage( NAME             draco
                GIT_REPOSITORY    https://github.com/google/draco.git
                GIT_TAG           1.4.3
                SOURCE_DIR        ${DIR}  
                DOWNLOAD_ONLY     YES)
endif()

if( NOT EXISTS ${DIR}/PATCHED )  
  file(GLOB files "${CMAKE_CURRENT_SOURCE_DIR}/patches/draco*")
  foreach(file ${files})
    execute_process( COMMAND git am ${file} WORKING_DIRECTORY ${DIR} RESULT_VARIABLE ret )
    if( NOT ${ret} EQUAL "0")
      message( FATAL_ERROR "Error during the draco patch process. ")
    endif()
  endforeach()
  file( WRITE ${DIR}/PATCHED "patched" )   
else()
  message("draco already patched")
endif()

add_subdirectory(dependencies/draco)