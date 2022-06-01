# directx-headers
set( DIR ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/directx-headers )
if( NOT EXISTS ${DIR} )
  CPMAddPackage( NAME              directx-headers
                GIT_REPOSITORY    https://github.com/microsoft/DirectX-Headers.git
                GIT_TAG           v1.602.0
                SOURCE_DIR        ${DIR} 
                DOWNLOAD_ONLY     YES)               
endif()
add_subdirectory(dependencies/directx-headers)

# directx-math
set( DIR ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/directx-math )
if( NOT EXISTS ${DIR} )
  CPMAddPackage( NAME             directx-math
                GIT_REPOSITORY    https://github.com/microsoft/DirectXMath.git
                GIT_TAG           jan2022
                SOURCE_DIR        ${DIR} 
                DOWNLOAD_ONLY     YES)
endif()
add_subdirectory(dependencies/directx-math)
            
# directx-mesh    
set( DIR ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/directx-mesh )
if( NOT EXISTS ${DIR} )
  CPMAddPackage( NAME             directx-mesh
                GIT_REPOSITORY    https://github.com/microsoft/DirectXMesh.git
                GIT_TAG           mar2022
                SOURCE_DIR        ${DIR} 
                DOWNLOAD_ONLY     YES)
endif()
if( NOT EXISTS ${DIR}/PATCHED )  
  file(GLOB files "${CMAKE_CURRENT_SOURCE_DIR}/patches/dxmesh*")
  foreach(file ${files})
    execute_process( COMMAND git am ${file} WORKING_DIRECTORY ${DIR} RESULT_VARIABLE ret )
    if( NOT ${ret} EQUAL "0")
      message( FATAL_ERROR "Error during the dxmesh patch process. ")
    endif()
  endforeach()
  file( WRITE ${DIR}/PATCHED "patched" )   
else()
  message("directx-mesh already patched")
endif()
add_subdirectory(dependencies/directx-mesh)