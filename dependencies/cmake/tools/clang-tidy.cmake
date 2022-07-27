# get all project files

# find_program(CLANG_TIDY_PATH NAMES clang-tidy )

# message( "CLANG_TIDY_PATH = ${CLANG_TIDY_PATH}" )

# if( EXISTS  "${CLANG_TIDY_PATH}" )
#   file(GLOB_RECURSE ALL_SOURCE_FILES ${CMAKE_CURRENT_SOURCE_DIR}/source/*.cpp  
#                                      ${CMAKE_CURRENT_SOURCE_DIR}/source/*.hpp)

#   add_custom_target(  clangformat COMMAND ${CLANG_TIDY_PATH} -style=file -i ${ALL_SOURCE_FILES} )
# else()
#     message("clang-format executable not found")
# endif()

if ( CMAKE_VERSION GREATER "3.5" )
  set(ENABLE_CLANG_TIDY ON CACHE BOOL "Add clang-tidy automatically to builds")
  if (ENABLE_CLANG_TIDY)
    find_program (CLANG_TIDY_EXE NAMES "clang-tidy" )
    if (CLANG_TIDY_EXE)
      message(STATUS "clang-tidy found: ${CLANG_TIDY_EXE}")
      set(CLANG_TIDY_CHECKS "-*,modernize-*")
      set(CMAKE_CXX_CLANG_TIDY "${CLANG_TIDY_EXE};-checks=${CLANG_TIDY_CHECKS};-header-filter='${CMAKE_SOURCE_DIR}/*'"
        CACHE STRING "" FORCE)
    else()
      message(AUTHOR_WARNING "clang-tidy not found!")
      set(CMAKE_CXX_CLANG_TIDY "" CACHE STRING "" FORCE) # delete it
    endif()
  endif()
endif()