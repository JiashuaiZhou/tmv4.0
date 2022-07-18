#!/bin/bash

CURDIR=$( cd "$( dirname "$0" )" && pwd ); 
MAINDIR=$( dirname ${CURDIR} )
echo -e "\033[0;32mRun vdmc encoder/decoder/metrics: ${CURDIR} \033[0m";

function formatCmd(){ local f=${1}; for s in ${2} ; do f=${f// ${s}/ \\\\\\n ${s}}; done ; echo -e $f; }

BINDIR=$( dirname ${CURDIR} )/build/Release/bin
if [ "$( uname -s )" == "Linux" ] ; then SUFFIX=; else SUFFIX=.exe; fi
FRAMECOUNT=1
OUTDIR=""
CFGDIR=""
CONDID=1
SEQID=1
RATEID=1
print_usage()
{
  echo "$0 execute encoding/decoding/metrics "
  echo "";
  echo "    Usage:"   
  echo "       -f|--frames : frame count           (default: $FRAMECOUNT )";  
  echo "       --c|-cfgdir : configured directory  (default: $CFGDIR )";           
  echo "       --o|-outdir : output directory      (default: $OUTDIR )";         
  echo "       --condId=   : condition: 1, 2       (default: $CONDID )";
  echo "       --seqId=    : seq: 1,2,3,4,5,6,7,8  (default: $SEQID )";
  echo "       --rateId=   : rate: 1,2,3,4,5       (default: $RATEID )";           
  echo "";
  echo "    Examples:";
  echo "      - $0  "; 
  echo "      - $0 --condId=1 --seqId=3 --rateId=3 --cfgdir=generatedConfigFiles --outdir=results/new/s3c1r3/ ";
  echo "    ";
  if [ "$#" != 2 ] ; then echo -e "ERROR: $1 \n"; fi
  exit 0;
}
while [[ $# -gt 0 ]] ; do  
  C=$1; if [[ "$C" =~ [=] ]] ; then V=${C#*=}; elif [[ $2 == -* ]] ; then  V=""; else V=$2; shift; fi;
  case "$C" in    
    -f|--frame*     ) FRAMECOUNT=$V;; 
    -o|--outdir*    ) OUTDIR=$V;; 
    -c|--cfgdir=*   ) CFGDIR=$V;; 
    --condId=*      ) CONDID=$V;;
    --seqId=*       ) SEQID=$V;;
    --rateId=*      ) RATEID=$V;;
    -h|--help       ) print_usage;;
    *               ) print_usage "unsupported arguments: $C ";;
  esac
  shift;
done

ENCODER=${MAINDIR}/build/Release/bin/encode   
DECODER=${MAINDIR}/build/Release/bin/decode
METRICS=${MAINDIR}/externaltools/mpeg-pcc-mmetric/build/mm

# check parameters
for NAME in $ENCODER $DECODER $METRICS ;  
do  
  if [ ! -f $NAME ] 
  then 
    print_usage "$NAME not exist, please run ./build.sh and ./scripts_new/get_external_tools.sh"; 
  fi 
done

case $CONDID in
  1) COND="ai";;
  2) COND="ld";;
  *) print_usage "condId not valid ($CONDID)";;
esac
case $SEQID in
  1) SEQ="longdress";;
  2) SEQ="soldier";;
  3) SEQ="basketball_player";;
  4) SEQ="dancer";;
  5) SEQ="mitch";;
  6) SEQ="thomas";;
  7) SEQ="football";;
  8) SEQ="levi";;
  *) print_usage "seqId not valid ($SEQID)";;
esac
case $RATEID in
  1) RATE="r1";;
  2) RATE="r2";;
  3) RATE="r3";;
  4) RATE="r4";;
  5) RATE="r5";;
  *) print_usage "rateId not valid ($RATEID)";;
esac

CFGSUBDIR=$( cd "$CFGDIR" && pwd )/s${SEQID}c${CONDID}r${RATEID}_${SEQ:0:4}/

if [ ! -d $CFGDIR ] || [ ! -f ${CFGSUBDIR}/encoder.cfg ] || [ ! -f ${CFGSUBDIR}/decoder.cfg ] || [ ! -f ${CFGSUBDIR}/mmetric.cfg ]
then 
  print_usage "cfgdir = \"$CFGDIR\" is not valid configuration directory"; 
fi
if [ "$OUTDIR" == "" ] ; then OUTDIR=$CFGDIR; elif [ ! -d $OUTDIR ] ; then mkdir -p $OUTDIR; fi

NAME=$( basename $CFGDIR )
NAME=$( printf "${OUTDIR}/${NAME}_F%03d" $FRAMECOUNT )
VDMC=${NAME}.vdmc

LOGENC=${NAME}_enc.log
LOGDEC=${NAME}_dec.log
LOGPCC=${NAME}_pcc.log
LOGIBSM=${NAME}_ibsm.log

# Encoder
if [ ! -f ${LOGENC} ]
then 
  echo -e "\033[0;32mEncode: ${NAME} \033[0m";
  startTime=$( date +%s.%N )
  CMD="$ENCODER \
        --config=${CFGSUBDIR}/encoder.cfg \
        --fcount=${FRAMECOUNT} \
        --compressed=${VDMC} \
        --recmesh=${NAME}_%04d_rec.obj \
        --rectex=${NAME}_%04d_rec.png \
        --recmat=${NAME}_%04d_rec.mtl \
        2>&1 > ${LOGENC} ";
  formatCmd "$CMD" "-- -c >"
  eval $CMD    
  endTime=$( date +%s.%N )
  encTime=$( echo "$endTime - $startTime" | bc )
  echo "$encTime" > ${NAME}_enctime.log
else 
  echo "${LOGENC} already exist"
  encTime=$( cat ${NAME}_enctime.log )
fi

# Decoder
if [ ! -f ${LOGDEC} ]
then 
  echo -e "\033[0;32mDecode: ${NAME} \033[0m";
  startTime=$( date +%s.%N )
  CMD="$DECODER \
    --config=${CFGSUBDIR}/decoder.cfg \
    --compressed=${VDMC} \
    --decmesh=${NAME}_%04d_dec.obj \
    --dectex=${NAME}_%04d_dec.png \
    --decmat=${NAME}_%04d_dec.mtl \
    > ${LOGDEC} ";
  formatCmd "$CMD" "-- -c >"
  eval $CMD
  endTime=$( date +%s.%N )
  decTime=$( echo "$endTime - $startTime" | bc )
  echo "$decTime" > ${NAME}_dectime.log
else 
  echo "${LOGDEC} already exist"
  decTime=$( cat ${NAME}_dectime.log )
fi

# Metrics
if [ ! -f ${LOGIBSM} ]
then 
  echo -e "\033[0;32mMetrics IBSM: ${NAME} \033[0m";
  start=$(       cat ${CFGSUBDIR}/mmetric.cfg | grep "fstart:" | awk '{print $2}' )
  fcount=$(      cat ${CFGSUBDIR}/mmetric.cfg | grep "fcount:" | awk '{print $2}' )
  srcMesh=$(     cat ${CFGSUBDIR}/mmetric.cfg | grep "srcMesh:" | awk '{print $2}' )
  srcTex=$(      cat ${CFGSUBDIR}/mmetric.cfg | grep "srcTex:" | awk '{print $2}' )
  qp=$(          cat ${CFGSUBDIR}/mmetric.cfg | grep "qp:" | awk '{print $2}' )
  qt=$(          cat ${CFGSUBDIR}/mmetric.cfg | grep "qt:" | awk '{print $2}' )
  minPosition=$( cat ${CFGSUBDIR}/mmetric.cfg | grep "minPosition:" | awk '{print $2" "$3" "$4}' )
  maxPosition=$( cat ${CFGSUBDIR}/mmetric.cfg | grep "maxPosition:" | awk '{print $2" "$3" "$4}' )
  gridSize=$(    cat ${CFGSUBDIR}/mmetric.cfg | grep "gridSize:" | awk '{print $2}' )
  last=$(( start + FRAMECOUNT - 1 ))
  CMD="$METRICS \
    sequence \
      --firstFrame    ${start} \
      --lastFrame     ${last} \
      END \
    dequantize \
      --inputModel    ${srcMesh} \
      --outputModel   ID:deqRef \
      --useFixedPoint \
      --qp            ${qp} \
      --minPos        \"${minPosition}\" \
      --maxPos        \"${maxPosition}\" \
      --qt            ${qt} \
      --minUv         \"0 0\" \
      --maxUv         \"1.0 1.0\" \
      END \
    dequantize \
      --inputModel    ${NAME}_%04d_dec.obj \
      --outputModel   ID:deqDis \
      --useFixedPoint \
      --qp            ${qp} \
      --minPos        \"${minPosition}\" \
      --maxPos        \"${maxPosition}\" \
      --qt            ${qt} \
      --minUv         \"0 0\" \
      --maxUv         \"1.0 1.0\" \
      END \
    compare \
      --mode          ibsm \
      --inputModelA   ID:deqRef \
      --inputModelB   ID:deqDis \
      --inputMapA     ${srcTex} \
      --inputMapB     ${NAME}_%04d_dec.png \        
      --outputCsv     ${LOGIBSM%.???}.csv \
    > ${LOGIBSM}"
  formatCmd "$CMD" "-- -c > compare dequantize sequence END"
  eval $CMD
else 
  echo "${LOGIBSM} already exist"
fi

if [ ! -f ${LOGPCC} ]
then 
  echo -e "\033[0;32mMetrics PCC: ${NAME} \033[0m";  
  start=$(       cat ${CFGSUBDIR}/mmetric.cfg | grep "fstart:" | awk '{print $2}' )
  fcount=$(      cat ${CFGSUBDIR}/mmetric.cfg | grep "fcount:" | awk '{print $2}' )
  srcMesh=$(     cat ${CFGSUBDIR}/mmetric.cfg | grep "srcMesh:" | awk '{print $2}' )
  srcTex=$(      cat ${CFGSUBDIR}/mmetric.cfg | grep "srcTex:" | awk '{print $2}' )
  qp=$(          cat ${CFGSUBDIR}/mmetric.cfg | grep "qp:" | awk '{print $2}' )
  qt=$(          cat ${CFGSUBDIR}/mmetric.cfg | grep "qt:" | awk '{print $2}' )
  minPosition=$( cat ${CFGSUBDIR}/mmetric.cfg | grep "minPosition:" | awk '{print $2" "$3" "$4}' )
  maxPosition=$( cat ${CFGSUBDIR}/mmetric.cfg | grep "maxPosition:" | awk '{print $2" "$3" "$4}' )
  gridSize=$(    cat ${CFGSUBDIR}/mmetric.cfg | grep "gridSize:" | awk '{print $2}' )
  last=$(( start + FRAMECOUNT - 1 ))
  IFS=" " read -r -a posMin <<< "$minPosition"
  IFS=" " read -r -a posMax <<< "$maxPosition"
  dimX=$( echo "${posMax[0]} - ${posMin[0]}" | bc )
  dimY=$( echo "${posMax[1]} - ${posMin[1]}" | bc )
  dimZ=$( echo "${posMax[2]} - ${posMin[2]}" | bc )
  function getMax() { awk -v n1="$1" -v n2="$2" 'BEGIN {print (n1>n2?n1:n2)}'; }
  RESOLUTION=$( getMax $dimX $dimY )
  RESOLUTION=$( getMax $dimZ $RESOLUTION )
  echo "posMin     = ${posMin[0]} ${posMin[1]} ${posMin[2]}"
  echo "posMax     = ${posMax[0]} ${posMax[1]} ${posMax[2]}"
  echo "DimXYZ     = ${dimX} ${dimY} ${dimZ}"
  echo "RESOLUTION = ${RESOLUTION} "

  CMD="$METRICS \
    sequence  \
      --firstFrame    ${start} \
      --lastFrame     ${last} \
      END \
    dequantize \
      --inputModel    ${srcMesh} \
      --outputModel   ID:deqRef \
      --useFixedPoint \
      --qp            ${qp} \
      --qt            ${qt} \
      --minPos        \"${minPosition}\" \
      --maxPos        \"${maxPosition}\" \
      --minUv         \"0.0 0.0\" \
      --maxUv         \"1.0 1.0\" \
      END \
    dequantize \
      --inputModel    ${NAME}_%04d_dec.obj \
      --outputModel   ID:deqDis \
      --useFixedPoint \
      --qp            ${qp} \
      --qt            ${qt} \
      --minPos        \"${minPosition}\" \
      --maxPos        \"${maxPosition}\" \
      --minUv         \"0.0 0.0\" \
      --maxUv         \"1.0 1.0\" \
      END \
    reindex \
      --inputModel    ID:deqRef \
      --sort          oriented \
      --outputModel   ID:ref_reordered \
      END \
    reindex \
      --inputModel    ID:deqDis \
      --sort          oriented \
      --outputModel   ID:dis_reordered \
      END \
    sample  \
      --inputModel    ID:ref_reordered \
      --inputMap      ${srcTex} \
      --mode          grid \
      --useNormal     \
      --useFixedPoint \
      --minPos        \"${minPosition}\" \
      --maxPos        \"${maxPosition}\" \
      --bilinear      \
      --gridSize      ${gridSize} \
      --hideProgress  1 \
      --outputModel   ID:ref_pc \
      END \
    sample  \
      --inputModel    ID:dis_reordered \
      --inputMap      ${NAME}_%04d_dec.png \
      --mode          grid \
      --useNormal     \
      --useFixedPoint \
      --minPos        \"${minPosition}\" \
      --maxPos        \"${maxPosition}\" \
      --bilinear      \
      --gridSize      ${gridSize} \
      --hideProgress  1 \
      --outputModel   ID:dis_pc \
      END \
    compare \
      --mode          pcc  \
      --inputModelA   ID:ref_pc \
      --inputModelB   ID:dis_pc \
      --resolution    ${RESOLUTION} \
      --outputCsv     $${LOGPCC%.???}.csv  \
    > ${LOGPCC}"   
  formatCmd "$CMD" "-- -c > compare dequantize sequence END"
  eval $CMD
else 
  echo "${LOGPCC} already exist"
fi

# Get number of output faces
NBOUTPUTFACES=$(cat ${LOGENC} | grep "Sequence face count" | awk '{print $4}' )

# Get bitstream total size in bits as reported by the encoding process
TOTALSIZEBITS=$(stat -c%s ${VDMC} | awk '{printf "%f\n", $1 * 8 }' )

# Get grid metric results
GRIDD1=$(cat ${LOGPCC} | grep -m${FRAMECOUNT} "mseF,PSNR (p2point):" | awk 'NR == 1 { sum=0 } { if ( $3=="inf" ) sum+=999.99; else sum+=$3; } END {printf "%f\n", sum/NR}')
GRIDD2=$(cat ${LOGPCC} | grep -m${FRAMECOUNT} "mseF,PSNR (p2plane):" | awk 'NR == 1 { sum=0 } { if ( $3=="inf" ) sum+=999.99; else sum+=$3; } END {printf "%f\n", sum/NR}')
GRIDY=$( cat ${LOGPCC} | grep -m${FRAMECOUNT} "c\[0\],PSNRF"         | awk 'NR == 1 { sum=0 } { if ( $3=="inf" ) sum+=999.99; else sum+=$3; } END {printf "%f\n", sum/NR}')
GRIDU=$( cat ${LOGPCC} | grep -m${FRAMECOUNT} "c\[1\],PSNRF"         | awk 'NR == 1 { sum=0 } { if ( $3=="inf" ) sum+=999.99; else sum+=$3; } END {printf "%f\n", sum/NR}')
GRIDV=$( cat ${LOGPCC} | grep -m${FRAMECOUNT} "c\[2\],PSNRF"         | awk 'NR == 1 { sum=0 } { if ( $3=="inf" ) sum+=999.99; else sum+=$3; } END {printf "%f\n", sum/NR}')  

# # Get ibsm metric results
IBSMG=$( cat ${LOGIBSM} | grep -m${FRAMECOUNT} "GEO PSNR ="           | awk 'NR == 1 { sum=0 } { if ( $4=="inf" ) sum+=999.99; else sum+=$4; } END {printf "%f\n", sum/NR}')
IBSMY=$( cat ${LOGIBSM} | grep -m${FRAMECOUNT} "Y   PSNR ="           | awk 'NR == 1 { sum=0 } { if ( $4=="inf" ) sum+=999.99; else sum+=$4; } END {printf "%f\n", sum/NR}')

echo "EncTime            : ${encTime}"
echo "DecTime            : ${decTime}"
echo "NbOutputFaces      : ${NBOUTPUTFACES}"
echo "TotalBitstreamBits : ${TOTALSIZEBITS}"
echo "GridD1             : ${GRIDD1}"
echo "GridD2             : ${GRIDD2}"
echo "GridLuma           : ${GRIDY}"
echo "GridChromaCb       : ${GRIDU}"
echo "GridChromaCr       : ${GRIDV}"
echo "IbsmGeom           : ${IBSMG}"
echo "IbsmLuma           : ${IBSMY}"