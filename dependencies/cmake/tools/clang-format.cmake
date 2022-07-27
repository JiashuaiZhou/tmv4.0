# get all project files

find_program(CLANG_FORMAT_PATH NAMES clang-format )

message( "CLANG_FORMAT_PATH = ${CLANG_FORMAT_PATH}" )

if( EXISTS  "${CLANG_FORMAT_PATH}" )
  file(GLOB_RECURSE ALL_SOURCE_FILES ${CMAKE_CURRENT_SOURCE_DIR}/source/*.cpp  
                                     ${CMAKE_CURRENT_SOURCE_DIR}/source/*.hpp)

  add_custom_target(  clangformat COMMAND ${CLANG_FORMAT_PATH} -style=file -i ${ALL_SOURCE_FILES} )
else()
    message("clang-format executable not found")
endif()

