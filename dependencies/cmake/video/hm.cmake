if ( USE_HM_VIDEO_CODEC )
  add_compile_definitions(USE_HM_VIDEO_CODEC)
  set( HM_VERSION         HM-16.21+SCM-8.8 )
  set( HM_DIR             ${CMAKE_SOURCE_DIR}/dependencies/hm/ )
  set( HM_LIB_SOURCE_DIR  ${HM_DIR}/source/Lib )

  # Building options
  set(CMAKE_CXX_STANDARD             14)
  set(CMAKE_CXX_STANDARD_REQUIRED    ON)
  set(CMAKE_CXX_EXTENSIONS           OFF)
  set (CMAKE_EXPORT_COMPILE_COMMANDS ON)

  if( NOT EXISTS "${HM_DIR}/README" )
    execute_process( COMMAND git clone --depth 1 --branch ${HM_VERSION} https://vcgit.hhi.fraunhofer.de/jvet/HM.git ${HM_DIR} RESULT_VARIABLE ret)
    if( NOT ${ret} EQUAL "0")
      message( FATAL_ERROR "Error during the HM git clone process. Check that git is well installed on your system.")
    endif()  
  endif()

  if( NOT EXISTS "${HM_DIR}/PATCHED" )
    if( USE_HM_PCC_RDO )
      set( HM_PATCH ${CMAKE_SOURCE_DIR}/dependencies/patches/hm/HM-16.20+SCM-8.8_with_RDO.patch )
    else()
      set( HM_PATCH ${CMAKE_SOURCE_DIR}/dependencies/patches/hm/HM-16.21+SCM-8.8_nhbd.patch )
    endif()
    message("Apply patch: ${HM_PATCH}")
    execute_process( COMMAND git apply ${HM_PATCH} --whitespace=nowarn WORKING_DIRECTORY ${HM_DIR} RESULT_VARIABLE ret )
    if( NOT ${ret} EQUAL "0")
      message( FATAL_ERROR "Error during the HM patch process.")
    endif()
    file( WRITE ${HM_DIR}/PATCHED "HM patched with: " ${HM_PATCH} )   
  endif()

  function(add_hm_library module )
    file(GLOB cppSourceFiles "${HM_LIB_SOURCE_DIR}/${module}/*.cpp")
    file(GLOB cSourceFiles "${HM_LIB_SOURCE_DIR}/${module}/*.c")
    file(GLOB headerFiles "${HM_LIB_SOURCE_DIR}/${module}/*.h")
    add_library(${module} ${cppSourceFiles} ${cSourceFiles} ${headerFiles} )
    set_property(TARGET ${module} PROPERTY CXX_CLANG_TIDY) # no clang-tidy
    add_library(VPCC::${module} ALIAS ${module})
  endfunction()

  # disable submodule warnings
  if(MSVC)
  else()
    if (CMAKE_CXX_COMPILER_ID MATCHES "Clang")
      add_compile_options(-Wno-deprecated-register)
    endif()
  endif()

  add_hm_library(libmd5)
  target_compile_features(libmd5 PUBLIC cxx_std_11)
  target_include_directories(libmd5 PUBLIC "$<BUILD_INTERFACE:${HM_LIB_SOURCE_DIR}>")
    
  add_hm_library(TLibCommon)
  target_link_libraries(TLibCommon PRIVATE libmd5)
  target_compile_features(TLibCommon PUBLIC cxx_std_11)
  target_include_directories(TLibCommon PUBLIC "$<BUILD_INTERFACE:${HM_LIB_SOURCE_DIR}>")
  target_compile_definitions(TLibCommon PUBLIC "$<$<CXX_COMPILER_ID:MSVC>:_CRT_SECURE_NO_WARNINGS>")
  target_compile_options(TLibCommon PUBLIC "$<$<CXX_COMPILER_ID:Clang>:-w>")
  target_compile_options(TLibCommon PUBLIC "$<$<CXX_COMPILER_ID:GNU>:-w>")
    
  add_hm_library(TLibDecoder)
  target_link_libraries(TLibDecoder PUBLIC TLibCommon)
  
  file(GLOB sourceFiles "${HM_LIB_SOURCE_DIR}/Utilities/TVideoIOYuv.*" 
                        "${HM_LIB_SOURCE_DIR}/TLibEncoder/*.cpp"
                        "${HM_LIB_SOURCE_DIR}/TLibEncoder/*.c"
                        "${HM_LIB_SOURCE_DIR}/TLibEncoder/*.h" )
  add_library(TLibEncoder ${sourceFiles} )
  set_property(TARGET TLibEncoder PROPERTY CXX_CLANG_TIDY) # no clang-tidy
  add_library(VPCC::TLibEncoder ALIAS TLibEncoder)
  target_link_libraries(TLibEncoder PUBLIC TLibCommon )  
endif()