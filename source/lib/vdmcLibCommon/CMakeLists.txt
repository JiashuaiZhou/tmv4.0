get_filename_component(MYNAME ${CMAKE_CURRENT_LIST_DIR} NAME)
string(REPLACE " " "_" MYNAME ${MYNAME})
project( ${MYNAME} C CXX )

enable_language( C )

##
# Determine the software version from VCS
# Fallback to descriptive version if VCS unavailable
#  NB: the "unconfirmed-" part refers to this being a fallback option
#      and the reported version may or may not be accurate.
set(VERSION_FALLBACK "unconfirmed-release-0.0")
set(VERSION_FILE       ${CMAKE_CURRENT_BINARY_DIR}/version.cpp)
set(VERSION_FILE_IN    ${CMAKE_CURRENT_SOURCE_DIR}/version.cpp.in)
set(VERSION_CACHE_FILE ${CMAKE_CURRENT_BINARY_DIR}/version.cache)
find_package(Git)
add_custom_target(
  genversion
  COMMAND ${CMAKE_COMMAND} -D OUTPUT=${VERSION_FILE}
                           -D TEMPLATE=${VERSION_FILE_IN}
                           -D VERSION_FALLBACK=${VERSION_FALLBACK}
                           -D VERSION_EXTRA=${VERSION_EXTRA}
                           -D VERSION_CACHE_FILE=${VERSION_CACHE_FILE}
                           -D GIT_EXECUTABLE=${GIT_EXECUTABLE}
                           -P ${CMAKE_SOURCE_DIR}/dependencies/cmake/genversion.cmake
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)

add_custom_command( OUTPUT ${VERSION_FILE} DEPENDS genversion )

file(GLOB SRC *.h *.cpp ${CMAKE_SOURCE_DIR}/dependencies/schroedinger/schroarith.c )

include_directories( ${CMAKE_SOURCE_DIR}/dependencies
                     ${CMAKE_SOURCE_DIR}/dependencies/mmetric/source/lib/include
                     ${CMAKE_SOURCE_DIR}/dependencies/mmetric/dependencies/
                     ${CMAKE_SOURCE_DIR}/dependencies/mmetric/dependencies/glad/include
                     ${CMAKE_SOURCE_DIR}/dependencies/mmetric/dependencies/glfw/include
                     ./ )
 
add_library( ${MYNAME} ${SRC} ${VERSION_FILE} )

TARGET_LINK_LIBRARIES( ${MYNAME} mmlib )

set_target_properties( ${MYNAME} PROPERTIES LINKER_LANGUAGE CXX)

