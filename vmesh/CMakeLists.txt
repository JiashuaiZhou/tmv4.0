enable_language(C)

if (MSVC)
    add_compile_options(/W4)
else ()
    add_compile_options(
        -Wshadow -Wall -Wextra -pedantic
        -Wno-conversion -Wno-sign-conversion -Wno-long-long
        -Wno-unused-function
    )
endif()

if (UNIX)
  add_definitions(-D_POSIX_C_SOURCE=200809L)
elseif (WIN32)
  add_definitions(-D_USE_MATH_DEFINES)
endif()

##
# Determine the software version from VCS
# Fallback to descriptive version if VCS unavailable
#  NB: the "unconfirmed-" part refers to this being a fallback option
#      and the reported version may or may not be accurate.
set(VERSION_FALLBACK "unconfirmed-release-0.0")
set(VERSION_FILE ${CMAKE_CURRENT_BINARY_DIR}/version.cpp)
set(VERSION_FILE_IN ${CMAKE_CURRENT_SOURCE_DIR}/version.cpp.in)
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
                           -P ${CMAKE_SOURCE_DIR}/scripts/genversion.cmake
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)

add_custom_command(
  OUTPUT ${VERSION_FILE}
  DEPENDS genversion
)

#
# NB: avoid the use of wild cards for source files in this directory,
# since cmake is unable to detect additions or removals in such cases
# resulting in obscure broken builds when switching branches or
# otherwise adding/removing files.
#
# Dependencies are permitted to use the wild card since they are
# not expected to change without modifying this CMakeLists.txt.
#

#source_group (inc FILES ${PROJECT_INC_FILES})
#source_group (cpp FILES ${PROJECT_CPP_FILES})

include_directories(
    "${PROJECT_SOURCE_DIR}/dependencies"
)

add_library(vmesh STATIC
    "simplifymesh.cpp"
    "simplifymesh.hpp"
    "util/box.hpp"
    "util/image.cpp"
    "util/image.hpp"
    "util/kdtree.hpp"
    "util/matrix.hpp"
    "util/mesh.hpp"
    "util/misc.cpp"
    "util/misc.hpp"
    "util/mutablepriorityheap.hpp"
    "util/sparsematrix.hpp"
    "util/triangle.hpp"
    "util/vector.hpp"
    "util/verbose.cpp"
    "util/verbose.hpp"
    "error.hpp"
    "version.cpp"
)
target_include_directories(vmesh
    PRIVATE "${PROJECT_BINARY_DIR}"
    PUBLIC  "${PROJECT_SOURCE_DIR}"
)

add_library(po STATIC
    "${PROJECT_SOURCE_DIR}/dependencies/program-options-lite/program_options_lite.cpp"
)
target_include_directories(po INTERFACE
    "${PROJECT_SOURCE_DIR}/dependencies/"
)

add_executable(fitsubdiv)
target_link_libraries(fitsubdiv vmesh po)
target_sources(fitsubdiv PRIVATE
    "fitsubdiv.cpp"
)


add_executable(gengof)
target_link_libraries(gengof vmesh po)
target_sources(gengof PRIVATE
    "gengof.cpp"
)


add_executable(simplify)
target_link_libraries(simplify vmesh po)
target_sources(simplify PRIVATE
    "simplify.cpp"
)


add_executable(uvatlaslite)
set_target_properties(uvatlaslite PROPERTIES OUTPUT_NAME uvatlas)
target_link_libraries(uvatlaslite po vmesh UVAtlas DirectX-Headers DirectXMath DirectXMesh)
target_sources(uvatlaslite PRIVATE
    "uvatlas.cpp"
)


add_executable(vmc)
target_link_libraries(vmc vmesh po)
target_sources(vmc PRIVATE
    "vmc.cpp"
    "vmc.hpp"
    "bitstream.hpp"
    "contexts.hpp"
    "decoder.cpp"
    "decoder.hpp"
    "encoder.cpp"
    "encoder.hpp"
    "entropy.hpp"
    "entropydirac.cpp"
    "entropydirac.hpp"
    "entropyutils.hpp"
    "hls.cpp"
    "vmcstats.hpp"
    "${PROJECT_SOURCE_DIR}/dependencies/schroedinger/schroarith.c"
)
