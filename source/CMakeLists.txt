enable_language(C)

if (MSVC)
  add_compile_options(/W4)
else ()
  add_compile_options( -Wshadow -Wall -Wextra -pedantic -Wno-conversion 
                        -Wno-sign-conversion -Wno-long-long -Wno-unused-function )
endif()

if (UNIX)
  add_definitions(-D_POSIX_C_SOURCE=200809L)
elseif (WIN32)
  add_definitions(-D_USE_MATH_DEFINES)
endif()

add_all_subdirectory( ${CMAKE_CURRENT_SOURCE_DIR} )