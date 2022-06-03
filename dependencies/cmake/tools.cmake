
# LIST OF SUB-DIRECTORIES
macro(subDirList result curdir)
    file(GLOB children ${curdir}/*) 
    set(dirlist "")
    foreach(child ${children})
        if(IS_DIRECTORY ${child})
            list(APPEND dirlist ${child})
        endif()
    endforeach()
    SET(${result} ${dirlist})
endmacro()

# ADD ALL SUB-DIRECTORIES
function(add_all_subdirectory curdir)
    subDirList( directories ${curdir})
    foreach(dir ${directories})
      message( "add_subdirectory: ${dir}")
      add_subdirectory(${dir})
  endforeach()
endfunction()