#!/bin/bash

CURDIR=$( cd "$( dirname "$0" )" && pwd ); 
MAINDIR=$( dirname ${CURDIR} )
echo -e "\033[0;32mBuild external tools: HM, HDRTools and  ${MAINDIR} \033[0m";
  
DEPDIR=${CURDIR}/../externaltools

case "$(uname -s)" in
  Linux*)     MACHINE=Linux;;
  Darwin*)    MACHINE=Mac;;
  CYGWIN*)    MACHINE=Cygwin;;
  MINGW*)     MACHINE=MinGw;;
  MSYS*)      MACHINE=MSys;;
  *)          MACHINE="UNKNOWN:$(uname -s)"
esac

CMAKE=cmake; 
if [ "$( cmake  --version 2>&1 | grep version | awk '{print $3 }' | awk -F '.' '{print $1}' )" == 3 ] ; then CMAKE=cmake; fi
if [ "$( cmake3 --version 2>&1 | grep version | awk '{print $3 }' | awk -F '.' '{print $1}' )" == 3 ] ; then CMAKE=cmake3; fi
if [ "$CMAKE" == "" ] ; then echo "Can't find cmake > 3.0"; exit; fi
if [ "$MACHINE" == "Linux" ] ; then NUMBER_OF_PROCESSORS=$( grep -c ^processor /proc/cpuinfo ); fi

# MM
MMETRIC=${DEPDIR}/mpeg-pcc-mmetric
if [ ! -d ${MMETRIC} ] 
then 
  echo -e "\033[0;32mClone: ${MMETRIC} \033[0m";
  git clone http://mpegx.int-evry.fr/software/MPEG/PCC/mpeg-pcc-mmetric.git -b 1_0_1 ${MMETRIC} 
fi    
echo -e "\033[0;32mBuild: ${MMETRIC} \033[0m";
cd ${MMETRIC}
pwd
if [ ! -d ./deps/glfw/ ] ; then ./build-deps.sh ; fi
if [ ! -d build ] ; then mkdir build; fi
cd build 
cmake3 ..
cmake3 --build . --config Release --parallel 20

cd ${CURDIR}

