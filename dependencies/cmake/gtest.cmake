
set( DIR ${CMAKE_SOURCE_DIR}/dependencies/googletest )
message(" DIR = ${DIR}")
# CPMAddPackage( NAME              googletest
#                GIT_REPOSITORY    https://github.com/google/googletest.git
#                GIT_TAG           release-1.11.0
#                SOURCE_DIR        ${DIR} 
#                DOWNLOAD_ONLY     YES)     

CPMAddPackage( "gh:google/googletest#release-1.11.0" )