#!/bin/bash

CURDIR=$( cd "$( dirname "$0" )" && pwd ); 
MAINDIR=$( dirname ${CURDIR} )
echo -e "\033[0;32mRun vmesh encoder/decoder/metrics: ${CURDIR} \033[0m";

function formatCmd(){ 
  local f=${1}; 
  for s in ${2} ; 
  do 
    if [ $s == "--" ]
    then 
      f=${f//${s}/ \\\\\\n    ${s}}; 
    else
      f=${f// ${s}/ \\\\\\n   ${s}}; 
    fi
  done   
  echo -e "$f" | sed 's/ *\\$/ \\/' 
}
function clip() { awk -v v="$1" -v a="$2" -v b="$3" 'BEGIN {print (v<a?a:v>b?b:v)}'; }

function isNotFinish(){   
  if [ -f "$1" ] && [[ "$( cat $1 | grep 'All frames have been' )" =~ "All frames have been" ]] ;
  then
    false 
    return
  else
    true 
    return
  fi
}

BINDIR=$( dirname ${CURDIR} )/build/Release/bin
if [ "$( uname -s )" == "Linux" ] ; then SUFFIX=; else SUFFIX=.exe; fi
FRAMECOUNT=1
OUTDIR="results"
CFGDIR=""
CONDID=1
SEQID=1
RATEID=1
VMESHMETRICSW=0
print_usage()
{
  echo "$0 execute encoding/decoding/metrics "
  echo "";
  echo "    Usage:"   
  echo "       -f|--frames    : frame count               (default: $FRAMECOUNT )";  
  echo "       --c|-cfgdir    : configured directory      (default: $CFGDIR )";           
  echo "       --o|-outdir    : output directory          (default: $OUTDIR )";         
  echo "       --condId=      : condition: 1, 2           (default: $CONDID )";
  echo "       --seqId=       : seq: 1,2,3,4,5,6,7,8      (default: $SEQID )";
  echo "       --rateId=      : rate: 1,2,3,4,5           (default: $RATEID )";         
  echo "       --rateId=      : rate: 1,2,3,4,5           (default: $RATEID )";        
  echo "       --vmeshMetricSW: Use vmesh metric software (default: $VMESHMETRICSW )";           
  echo "";
  echo "    Examples:";
  echo "      - $0  "; 
  echo "      - $0 --condId=1 --seqId=3 --rateId=3 --cfgdir=generatedConfigFiles ";
  echo "      - $0 --condId=1 --seqId=3 --rateId=3 --cfgdir=generatedConfigFiles --vmeshMetricSW ";
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
    --vmeshMetricSW ) VMESHMETRICSW=1;;
    -h|--help       ) print_usage;;
    *               ) print_usage "unsupported arguments: $C ";;
  esac
  shift;
done

# set encoder/decoder/metric sw path
ENCODER=${MAINDIR}/build/Release/bin/encode   
DECODER=${MAINDIR}/build/Release/bin/decode
METRICS=${MAINDIR}/externaltools/mpeg-pcc-mmetric/build/mm
if [ ! -f "${ENCODER}" ] ; then ENCODER=${MAINDIR}/build/Release/bin/Release/encode.exe; fi
if [ ! -f "${DECODER}" ] ; then DECODER=${MAINDIR}/build/Release/bin/Release/decode.exe; fi
if [ ${VMESHMETRICSW} == 1 ]
then 
  METRICS=${MAINDIR}/build/Release/bin/metrics
  if [ ! -f "${METRICS}" ] ; then METRICS=${MAINDIR}/build/Release/bin/Release/metrics.exe; fi
else
  METRICS=${MAINDIR}/externaltools/mpeg-pcc-mmetric/build/mm
  if [ ! -f "${METRICS}" ] ; then METRICS=${MAINDIR}/externaltools/mpeg-pcc-mmetric/bin/mm.exe; fi
  if [ ! -f "${METRICS}" ] ; then 
    echo "Please run ${CURDIR}/get_external_tools.sh script to install mmetric software"
    exit -1;
  fi
fi
   
# check parameters
for NAME in $ENCODER $DECODER $METRICS ;  
do  
  if [ ! -f $NAME ] 
  then 
    print_usage "$NAME not exist, please run ./build.sh and ./scripts/get_external_tools.sh"; 
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

# Check configuration files
CFGSUBDIR=$( cd "$CFGDIR" && pwd )/s${SEQID}c${CONDID}r${RATEID}_${SEQ:0:4}/
if [ ! -d $CFGDIR ] || [ ! -f ${CFGSUBDIR}/encoder.cfg ] || [ ! -f ${CFGSUBDIR}/decoder.cfg ] || [ ! -f ${CFGSUBDIR}/mmetric.cfg ]
then 
  print_usage "Config directory is not valid (${CFGDIR}) \n please run ./scripts/create_configuration_files.sh"; 
fi

# set output directory
NAME=$( basename "$CFGSUBDIR" )
OUTDIR=${OUTDIR}/$( printf "F%03d" "$FRAMECOUNT" )/${NAME}; 
if [ ! -d $OUTDIR ] ; then mkdir -p $OUTDIR; fi
NAME=${OUTDIR}/${NAME}
VDMC=${NAME}.vmesh
fcountmax=$( cat ${CFGSUBDIR}/mmetric.cfg | grep "frameCount:" | awk '{print $2}' )
FRAMECOUNT=$( clip $FRAMECOUNT 1 $fcountmax )

# Encoder
LOGENC=${OUTDIR}/encoder.log
if isNotFinish ${LOGENC}
then 
  echo -e "\033[0;32mEncode: ${NAME} \033[0m";
  startTime=$( date +%s.%N )
  CMD="$ENCODER \
        --config=${CFGSUBDIR}/encoder.cfg \
        --frameCount=${FRAMECOUNT} \
        --compressed=${VDMC} \
       > ${LOGENC} 2>&1";
  formatCmd "$CMD" "-- >"
    if ! eval $CMD ; then echo "ERROR: encode sw return !0"; exit; fi
else 
  echo "${LOGENC} already exist"
fi

# Decoder
LOGDEC=${OUTDIR}/decoder.log
if isNotFinish ${LOGDEC}
then
  echo -e "\033[0;32mDecode: ${NAME} \033[0m";
  startTime=$( date +%s.%N )
  CMD="$DECODER \
    --config=${CFGSUBDIR}/decoder.cfg \
    --compressed=${VDMC} \
    --decMesh=${NAME}_%04d_dec.obj \
    --decTex=${NAME}_%04d_dec.png \
    --decMat=${NAME}_%04d_dec.mtl \
    > ${LOGDEC} 2>&1 ";
  formatCmd "$CMD" "-- >"
  if ! eval $CMD ; then echo "ERROR: encode sw return !0"; exit; fi
else
  echo "${LOGDEC} already exist"
fi

# Metrics
if [ ${VMESHMETRICSW} == 1 ]
then 
  LOGMET=${OUTDIR}/metric_met.log  
  if isNotFinish ${LOGMET}
  then
    echo -e "\033[0;32mMetrics: ${NAME} \033[0m";
    CMD="${METRICS} \
      --config=${CFGSUBDIR}/mmetric.cfg \
      --decMesh=${NAME}_%04d_dec.obj \
      --decTex=${NAME}_%04d_dec.png \
      --frameCount=${FRAMECOUNT} \
      > ${LOGMET}";
    formatCmd "$CMD" "-- > "
    if ! eval $CMD ; then echo "ERROR: metrics sw return !0"; exit; fi
  else 
    echo "${LOGMET} already exist"
  fi
  RESULTS=( $( cat ${LOGMET} | grep "Metric_results" | awk '{ printf("%s %s %s %s %s %s %s ", $3, $4, $5, $6, $7, $8, $9 )}') )
else
  LOGPCC=${OUTDIR}/metric_pcc.log
  LOGIBSM=${OUTDIR}/metric_ibsm.log
  if [ ! -f ${LOGIBSM} ]
  then 
    echo -e "\033[0;32mMetrics IBSM: ${NAME} \033[0m";
    start=$(       cat ${CFGSUBDIR}/mmetric.cfg | grep "startFrameIndex:"  | awk '{print $2}' )
    srcMesh=$(     cat ${CFGSUBDIR}/mmetric.cfg | grep "srcMesh:"          | awk '{print $2}' )
    srcTex=$(      cat ${CFGSUBDIR}/mmetric.cfg | grep "srcTex:"           | awk '{print $2}' )
    qp=$(          cat ${CFGSUBDIR}/mmetric.cfg | grep "positionBitDepth:" | awk '{print $2}' )
    qt=$(          cat ${CFGSUBDIR}/mmetric.cfg | grep "texCoordBitDepth:" | awk '{print $2}' )
    minPosition=$( cat ${CFGSUBDIR}/mmetric.cfg | grep "minPosition:"      | awk '{print $2" "$3" "$4}' )
    maxPosition=$( cat ${CFGSUBDIR}/mmetric.cfg | grep "maxPosition:"      | awk '{print $2" "$3" "$4}' )
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
    formatCmd "$CMD" "-- > compare dequantize sequence END"
    if ! eval $CMD ; then echo "ERROR: mmetric sw return !0"; exit; fi
  else 
    echo "${LOGIBSM} already exist"
  fi

  if [ ! -f ${LOGPCC} ]
  then 
    echo -e "\033[0;32mMetrics PCC: ${NAME} \033[0m";  
    start=$(       cat ${CFGSUBDIR}/mmetric.cfg | grep "startFrameIndex:"  | awk '{print $2}' )
    srcMesh=$(     cat ${CFGSUBDIR}/mmetric.cfg | grep "srcMesh:"          | awk '{print $2}' )
    srcTex=$(      cat ${CFGSUBDIR}/mmetric.cfg | grep "srcTex:"           | awk '{print $2}' )
    qp=$(          cat ${CFGSUBDIR}/mmetric.cfg | grep "positionBitDepth:" | awk '{print $2}' )
    qt=$(          cat ${CFGSUBDIR}/mmetric.cfg | grep "texCoordBitDepth:" | awk '{print $2}' )
    minPosition=$( cat ${CFGSUBDIR}/mmetric.cfg | grep "minPosition:"      | awk '{print $2" "$3" "$4}' )
    maxPosition=$( cat ${CFGSUBDIR}/mmetric.cfg | grep "maxPosition:"      | awk '{print $2" "$3" "$4}' )
    gridSize=$(    cat ${CFGSUBDIR}/mmetric.cfg | grep "gridSize:"         | awk '{print $2}' )
    resolution=$(  cat ${CFGSUBDIR}/mmetric.cfg | grep "resolution:"       | awk '{print $2}' )
    last=$(( start + FRAMECOUNT - 1 ))
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
        --resolution    ${resolution} \
        --outputCsv     ${LOGPCC%.???}.csv  \
      > ${LOGPCC}"   
    formatCmd "$CMD" "-- -c > compare dequantize sequence reindex sample END"
    if ! eval $CMD ; then echo "ERROR: mmetric sw return !0"; exit; fi
  else 
    echo "${LOGPCC} already exist"
  fi

  # Get grid metric results
  RESULTS[0]=$(cat ${LOGPCC} | grep -m${FRAMECOUNT} "mseF,PSNR (p2point):" | awk 'NR == 1 { sum=0 } { if ( $3=="inf" ) sum+=999.99; else sum+=$3; } END {printf "%f\n", sum/NR}')
  RESULTS[1]=$(cat ${LOGPCC} | grep -m${FRAMECOUNT} "mseF,PSNR (p2plane):" | awk 'NR == 1 { sum=0 } { if ( $3=="inf" ) sum+=999.99; else sum+=$3; } END {printf "%f\n", sum/NR}')
  RESULTS[2]=$( cat ${LOGPCC} | grep -m${FRAMECOUNT} "c\[0\],PSNRF"         | awk 'NR == 1 { sum=0 } { if ( $3=="inf" ) sum+=999.99; else sum+=$3; } END {printf "%f\n", sum/NR}')
  RESULTS[3]=$( cat ${LOGPCC} | grep -m${FRAMECOUNT} "c\[1\],PSNRF"         | awk 'NR == 1 { sum=0 } { if ( $3=="inf" ) sum+=999.99; else sum+=$3; } END {printf "%f\n", sum/NR}')
  RESULTS[4]=$( cat ${LOGPCC} | grep -m${FRAMECOUNT} "c\[2\],PSNRF"         | awk 'NR == 1 { sum=0 } { if ( $3=="inf" ) sum+=999.99; else sum+=$3; } END {printf "%f\n", sum/NR}')  

  # Get ibsm metric results
  RESULTS[5]=$( cat ${LOGIBSM} | grep -m${FRAMECOUNT} "GEO PSNR ="           | awk 'NR == 1 { sum=0 } { if ( $4=="inf" ) sum+=999.99; else sum+=$4; } END {printf "%f\n", sum/NR}')
  RESULTS[6]=$( cat ${LOGIBSM} | grep -m${FRAMECOUNT} "Y   PSNR ="           | awk 'NR == 1 { sum=0 } { if ( $4=="inf" ) sum+=999.99; else sum+=$4; } END {printf "%f\n", sum/NR}')
fi

# Get number of output faces
NBOUTPUTFACES=$(cat ${LOGENC} | grep "Sequence face count" | awk '{print $4}' )

# Get bitstream total size in bits as reported by the encoding process
TOTALSIZEBITS=$(stat -c%s ${VDMC} | awk '{printf "%d\n", $1 * 8 }' )

# Get user encoder and decoder runtimes
ENCTIME=$(cat ${LOGENC} | grep "Sequence processing time" | awk '{print $4}' )
DECTIME=$(cat ${LOGDEC} | grep "Sequence processing time" | awk '{print $4}' )

echo "NbOutputFaces      : ${NBOUTPUTFACES}"
echo "TotalBitstreamBits : ${TOTALSIZEBITS}"
echo "GridD1             : ${RESULTS[0]}"
echo "GridD2             : ${RESULTS[1]}"
echo "GridLuma           : ${RESULTS[2]}"
echo "GridChromaCb       : ${RESULTS[3]}"
echo "GridChromaCr       : ${RESULTS[4]}"
echo "IbsmGeom           : ${RESULTS[5]}"
echo "IbsmLuma           : ${RESULTS[6]}"
echo "EncTime            : ${ENCTIME}"
echo "DecTime            : ${DECTIME}"