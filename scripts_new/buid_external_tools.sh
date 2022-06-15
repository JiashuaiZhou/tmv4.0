#!/bin/bash

CURDIR=$( cd "$( dirname "$0" )" && pwd ); 
echo -e "\033[0;32mBuild: ${CURDIR} \033[0m";
  
DEPDIR=${CURDIR}/../externaltools

case "$(uname -s)" in
  Linux*)     MACHINE=Linux;;
  Darwin*)    MACHINE=Mac;;
  CYGWIN*)    MACHINE=Cygwin;;
  MINGW*)     MACHINE=MinGw;;
  MSYS*)      MACHINE=MSys;;
  *)          MACHINE="UNKNOWN:$(uname -s)"
esac

CMAKE=""; 
if [ "$( cmake  --version 2>&1 | grep version | awk '{print $3 }' | awk -F '.' '{print $1}' )" == 3 ] ; then CMAKE=cmake; fi
if [ "$( cmake3 --version 2>&1 | grep version | awk '{print $3 }' | awk -F '.' '{print $1}' )" == 3 ] ; then CMAKE=cmake3; fi
if [ "$CMAKE" == "" ] ; then echo "Can't find cmake > 3.0"; exit; fi
if [ "$MACHINE" == "Linux" ] ; then NUMBER_OF_PROCESSORS=$( grep -c ^processor /proc/cpuinfo ); fi

# HM-16.21+SCM-8.8
HMDIR=${DEPDIR}/hm-16.21+scm-8.8
if [ ! -d ${HMDIR} ] 
then 
  echo -e "\033[0;32mClone: ${HMDIR} \033[0m";
  git clone https://vcgit.hhi.fraunhofer.de/jvet/HM.git -b HM-16.20+SCM-8.8 ${HMDIR} 
fi    
echo -e "\033[0;32mBuild: ${HMDIR} \033[0m";
${CMAKE} -B ${HMDIR}/build_dir ${HMDIR}/
${CMAKE} --build ${HMDIR}/build_dir --config Release --parallel ${NUMBER_OF_PROCESSORS}  

# HDRTOOLS
HDRTOOLSDIR=${DEPDIR}/hdrtools
if [ ! -d ${HDRTOOLSDIR} ] 
then 
  echo -e "\033[0;32mClone: ${HDRTOOLSDIR} \033[0m";
  git clone https://gitlab.com/standards/HDRTools.git -b master ${HDRTOOLSDIR}; 
fi    

if [ "${MACHINE}" != "Linux" ] 
then 
  # clone and build zlib
  ZLIBDIR=${DEPDIR}/zlib
  echo -e "\033[0;32mDonwload and build: ${ZLIBDIR} \033[0m";
  if [ ! -f ${ZLIBDIR}/lib/zlib.lib ]
  then 
    rm -rf ${ZLIBDIR}/   
    mkdir ${ZLIBDIR}
    curl https://zlib.net/zlib1212.zip -L -o ${ZLIBDIR}/zlib1212.zip    
    unzip -q -o ${ZLIBDIR}/zlib1212.zip -d ${ZLIBDIR}
    cmake -B ${ZLIBDIR}/build ${ZLIBDIR}/zlib-1.2.12/ -DCMAKE_INSTALL_PREFIX=${ZLIBDIR}/   
    cmake --build ${ZLIBDIR}/build/ --config Release --target install --parallel ${NUMBER_OF_PROCESSORS} 
    rm -rf ${ZLIBDIR}/zlib1212.zip ${ZLIBDIR}/build/ ${ZLIBDIR}/zlib-1.2.12/
  fi

  echo -e "\033[0;32mBuild: ${HDRTOOLSDIR} \033[0m";
  # Build HdrTools
  ${CMAKE} -B ${HDRTOOLSDIR}/build ${HDRTOOLSDIR}  \
    -DCMAKE_BUILD_TYPE=Release  \
    -DLIBPNG=ON  \
    -DZLIB_LIBRARY=${ZLIBDIR}/lib/zlib.lib  \
    -DZLIB_INCLUDE_DIR=${ZLIBDIR}/include/ 

  ${CMAKE} --build ${HDRTOOLSDIR}/build/ --config Release -j ${NUMBER_OF_PROCESSORS}     
    
  cp -f ${ZLIBDIR}/bin/zlib.dll ${HDRTOOLSDIR}/build/bin/Release/  
else 
  echo -e "\033[0;32mBuild: ${HDRTOOLSDIR} \033[0m";
  ${CMAKE} -B ${HDRTOOLSDIR}/build/ ${HDRTOOLSDIR} \
    -DCMAKE_BUILD_TYPE=Release \
    -DLIBPNG=ON 
  ${CMAKE} --build ${HDRTOOLSDIR}/build/ --config Release -j ${NUMBER_OF_PROCESSORS}  
fi 

# VDMC SRC
VDMC=${DEPDIR}/mpeg-vmesh-tm
if [ ! -d ${VDMC} ] 
then 
  echo -e "\033[0;32mClone: ${VDMC} \033[0m";
  git clone ssh://git@mpegx.int-evry.fr:2222/MPEG/dmc/mpeg-vmesh-tm.git -b EE42_integration ${VDMC} 
  cd ${VDMC}
  git checkout 970066c11881ef19f76c2147a06df75de6dad92c
  cd ${CURDIR}
fi    
echo -e "\033[0;32mBuild: ${HMDIR} \033[0m";
cd ${VDMC}
./build.sh

exit()