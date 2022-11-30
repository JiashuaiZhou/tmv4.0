set( DIR ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/mmetric )
if( NOT EXISTS ${DIR} )
  CPMAddPackage( NAME             mmetric
                GIT_REPOSITORY    http://mpegx.int-evry.fr/software/MPEG/PCC/mpeg-pcc-mmetric.git
                GIT_TAG           1.1.0
                SOURCE_DIR        ${DIR}
                OPTIONS           "MM_BUILD_CMD off"
                OPTIONS           "USE_OPENMP   off"
                DOWNLOAD_ONLY     YES )
endif()

set(MM_BUILD_CMD on)
set(USE_OPENMP off)

# disable submodule warnings
if(MSVC)
  add_definitions("/wd4267")
else()
  if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wno-sign-compare
      -Wno-unused-variable
      -Wno-unused-but-set-variable
      -Wno-unused-local-typedef
      -Wno-unused-private-field
      -Wno-missing-field-initializers
      -Wno-ignored-qualifiers)
  else()
    add_compile_options(-Wno-sign-compare
      -Wno-unused-variable
      -Wno-maybe-uninitialized
      -Wno-unused-but-set-variable
      -Wno-missing-field-initializers
      -Wno-catch-value
      -Wno-ignored-qualifiers)
  endif()
endif()

add_subdirectory(${DIR})
