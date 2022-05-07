cmake_minimum_required(VERSION 3.15)
project(vmesh)

set(CMAKE_CXX_STANDARD 11)

# fix policies in dependent projects by injecting code
# notably, set CMP0077 so that setting variables is honoured by subprojects
SET(CMAKE_PROJECT_INCLUDE_BEFORE ${CMAKE_SOURCE_DIR}/scripts/fix-policy.cmake)

# Some dependencies include find_library commands, which wont work in the
# in-tree build (because they've not been built yet).  Whereas actually
# there is no need to find anything.  The following fake package definitions
# will satisfy find_library without modifying the submodule
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ${CMAKE_SOURCE_DIR}/scripts/fakepkg)

# Code in directx dependencies may use SAL annotations.
# On non-win32 platforms, these are not defined.  Instead provide a SAL2 stub
if (NOT WIN32)
    include_directories(dependencies/sal)
endif()

# directx-mesh unconditionally includes install targets that need dxheaders'
set(DXHEADERS_INSTALL TRUE)
add_subdirectory(dependencies/draco)
add_subdirectory(dependencies/directx-headers)
add_subdirectory(dependencies/directx-math)
add_subdirectory(dependencies/directx-mesh)
add_subdirectory(dependencies/uvatlas)

add_subdirectory(vmesh)
