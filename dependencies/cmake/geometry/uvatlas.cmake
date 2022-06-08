set( DIR ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/uvatlas )
if( NOT EXISTS ${DIR} )
  CPMAddPackage( NAME             uvatlas
                GIT_REPOSITORY    https://github.com/microsoft/UVAtlas.git
                GIT_TAG           mar2022
                SOURCE_DIR        ${DIR}
                DOWNLOAD_ONLY     YES)    
endif()

if( NOT EXISTS ${DIR}/PATCHED )  
  file(GLOB files "${CMAKE_CURRENT_SOURCE_DIR}/dependencies/patches/uvatlas/*")
  foreach(file ${files})
    execute_process( COMMAND git am ${file} WORKING_DIRECTORY ${DIR} RESULT_VARIABLE ret )
    if( NOT ${ret} EQUAL "0")
      message( FATAL_ERROR "Error during the uvatlas patch process. ")
    endif()
  endforeach()
  file( WRITE ${DIR}/PATCHED "patched" )   
else()
  message("uvatlas already patched")
endif()

add_subdirectory(dependencies/uvatlas)