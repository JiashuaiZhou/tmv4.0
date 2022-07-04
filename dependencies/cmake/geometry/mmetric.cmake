set( DIR ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/mmetric )
if( NOT EXISTS ${DIR} )
  CPMAddPackage( NAME             mmetric
                GIT_REPOSITORY    http://mpegx.int-evry.fr/software/MPEG/PCC/mpeg-pcc-mmetric.git
                GIT_TAG           1_0_1_lib_beta 
                SOURCE_DIR        ${DIR}
                OPTIONS           "MM_BUILD_CMD off"
                OPTIONS           "USE_OPENMP   off"
                DOWNLOAD_ONLY     YES )
endif()
set( MM_BUILD_CMD on   )
set( USE_OPENMP   off  )
add_subdirectory(${DIR})
