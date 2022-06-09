set( DIR ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/mmetric )
if( NOT EXISTS ${DIR} )
  CPMAddPackage( NAME             mmetric
                GIT_REPOSITORY    ssh://git@mpegx.int-evry.fr:2222/MPEG/PCC/mpeg-pcc-mmetric.git
                GIT_TAG           new_library_architecture
                SOURCE_DIR        ${DIR}
                OPTIONS           "MM_BUILD_CMD off"
                OPTIONS           "USE_OPENMP   off"
                DOWNLOAD_ONLY     YES )
endif()
set( MM_BUILD_CMD on )
set( USE_OPENMP   off )
add_subdirectory(${DIR})
