
# LIST OF SUB-DIRECTORIES
macro(subDirList result curdir)
  file(GLOB children ${curdir}/*) 
  set(dirlist "")
  foreach(child ${children})
    if(IS_DIRECTORY ${child})
      list(APPEND dirlist ${child})
    endif()
  endforeach()
  set(${result} ${dirlist})
endmacro()

# ADD ALL SUB-DIRECTORIES
function(add_all_subdirectory curdir)
  subDirList( directories ${curdir})
  foreach(dir ${directories})
    message( "add_subdirectory: ${dir}")
    add_subdirectory(${dir})
  endforeach()
endfunction()

# ADD ALL SUB-DIRECTORIES
function(create_executable file )
  cmake_parse_arguments(PARSE_ARGV 0 "ARG" "" "NAME" "LIBRARIES;SOURCES;INCLUDES")
  if ( "${ARG_NAME}" STREQUAL "" )
    message( FATAL_ERROR "NAME parameter not set")
  endif()  
  if ( "${ARG_SOURCES}" STREQUAL "" )
    message( FATAL_ERROR "SOURCES parameter not set")
  endif()
  message("Create executable: ${ARG_NAME} from ${ARG_SOURCES}")
  include_directories( ${ARG_INCLUDES} )
  # link_directories( ${CMAKE_BINARY_DIR}/lib )
  add_executable( ${ARG_NAME} ${ARG_SOURCES} )
  target_link_libraries( ${ARG_NAME} ${ARG_LIBRARIES} )
  install( TARGETS ${ARG_NAME} DESTINATION bin ) 
endfunction()