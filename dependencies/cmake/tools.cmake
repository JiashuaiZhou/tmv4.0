
# LIST OF SUB-DIRECTORIES
MACRO(subDirList result curdir)
    FILE(GLOB children ${curdir}/*) 
    SET(dirlist "")
    FOREACH(child ${children})
        IF(IS_DIRECTORY ${child})
            LIST(APPEND dirlist ${child})
        ENDIF()
    ENDFOREACH()
    SET(${result} ${dirlist})
ENDMACRO()

# ADD ALL SUB-DIRECTORIES
function(add_all_subdirectory curdir)
    subDirList( directories ${curdir})
    foreach(dir ${directories})
      message( "add_subdirectory: ${dir}")
      add_subdirectory(${dir})
  endforeach()
endfunction()