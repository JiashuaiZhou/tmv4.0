#!/bin/bash

CURDIR=$( cd "$( dirname "$0" )" && pwd ); 
echo -e "\033[0;32mBuild: ${CURDIR} \033[0m";

CMAKE=""; 
if [ "$( cmake  --version 2>&1 | grep version | awk '{print $3 }' | awk -F '.' '{print $1}' )" == 3 ] ; then CMAKE=cmake; fi
if [ "$( cmake3 --version 2>&1 | grep version | awk '{print $3 }' | awk -F '.' '{print $1}' )" == 3 ] ; then CMAKE=cmake3; fi
if [ "$CMAKE" == "" ] ; then echo "Can't find cmake > 3.0"; exit; fi

case "$(uname -s)" in
  Linux*)     MACHINE=Linux;;
  Darwin*)    MACHINE=Mac;;
  CYGWIN*)    MACHINE=Cygwin;;
  MINGW*)     MACHINE=MinGw;;
  MSYS*)      MACHINE=MSys;;
  *)          MACHINE="UNKNOWN:$(uname -s)"
esac

MODE=Release
TARGETS=()
CMAKE_FLAGS=()
if [ "$MACHINE" == "Linux" ] ; then NUMBER_OF_PROCESSORS=$( grep -c ^processor /proc/cpuinfo ); fi

for i in "$@"
do  
  case "$i" in
    doc       ) make -C "${CURDIR}/doc/"; exit 0;;
    debug     ) MODE=Debug; CMAKE_FLAGS+=("-DCMAKE_C_FLAGS=\"-g3\"" "-DCMAKE_CXX_FLAGS=\"-g3\"" );;
    release   ) MODE=Release;;
	  test      ) CMAKE_FLAGS+=( "-DBUILD_UNIT_TEST_APPS=TRUE" ) ;;
    format    ) TARGETS+=( "clang-format" );;
    tidy      ) TARGETS+=( "clang-tidy" );;
    cppcheck  ) TARGETS+=( "cppcheck" );;
    tidy      ) TIDY=1;;
    *         ) echo "ERROR: arguments \"$i\" not supported: option = [debug|release]"; exit 1;;
  esac
done

CMAKE_FLAGS+=( "-DCMAKE_BUILD_TYPE=$MODE" ) 
CMAKE_FLAGS+=( "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON" )
CMAKE_FLAGS+=( "-DCMAKE_RUNTIME_OUTPUT_DIRECTORY=${CURDIR}/build/${MODE}/bin" )
CMAKE_FLAGS+=( "-DCMAKE_ARCHIVE_OUTPUT_DIRECTORY=${CURDIR}/build/${MODE}/lib" )
CMAKE_FLAGS+=( "-DCMAKE_LIBRARY_OUTPUT_DIRECTORY=${CURDIR}/build/${MODE}/lib" )
# CMAKE_FLAGS+=( "-DCMAKE_CXX_FLAGS=-stdlib=libc++" )

echo -e "\033[0;32mCmake: ${CURDIR}: CMAKE_FLAGS = ${CMAKE_FLAGS[@]}\033[0m";
${CMAKE} -H${CURDIR} -B"${CURDIR}/build/${MODE}" "${CMAKE_FLAGS[@]}"
echo -e "\033[0;32mdone \033[0m";

for TARGET in ${TARGETS}
do 
  echo -e "\033[0;32m${TARGET}: ${CURDIR} \033[0m";
  ${CMAKE} --build "${CURDIR}/build/${MODE}" --target ${TARGET}
  echo -e "\033[0;32mdone \033[0m";
done

echo -e "\033[0;32mBuild: ${CURDIR} \033[0m";
if ! ${CMAKE} --build "${CURDIR}/build/${MODE}" --config ${MODE} --parallel "${NUMBER_OF_PROCESSORS}" ;
then
  echo -e "\033[1;31mfailed \033[0m"
  exit 1;
fi 
echo -e "\033[0;32mdone \033[0m";

