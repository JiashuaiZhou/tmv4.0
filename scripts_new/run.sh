#!/bin/bash

CURDIR=$( cd "$( dirname "$0" )" && pwd ); 
MAINDIR=$( dirname ${CURDIR} )
echo -e "\033[0;32mRun vdmc encoder/decoder/metrics: ${CURDIR} \033[0m";

function formatCmd(){ local f=${1}; for s in ${2} ; do f=${f// ${s}/ \\\\\\n ${s}}; done ; echo -e $f; }

BINDIR=$( dirname ${CURDIR} )/build/Release/bin
if [ "$( uname -s )" == "Linux" ] ; then SUFFIX=; else SUFFIX=.exe; fi
ENCODER=${BINDIR}/encode${SUFFIX}
DECODER=${BINDIR}/decode${SUFFIX}
METRICS=${BINDIR}/metrics${SUFFIX}
FRAMECOUNT=1
CFGDIR=""
OUTDIR=""
print_usage()
{
  echo "$0 execute encoding/decoding/metrics/rendering "
  echo "";
  echo "    Usage:"   
  echo "       --c|-cfgdir : configured directory  (default: $CFGDIR )";           
  echo "       --o|-outdir : output directory      (default: $OUTDIR )";      
  echo "       --encoder   : encoder path          (default: $ENCODER )";       
  echo "       --decoder   : decoder path          (default: $DECODER )";        
  echo "       --metrics   : metrics path          (default: $METRICS )";            
  echo "       -f|--frames : frame count           (default: $FRAMECOUNT )";             
  echo "";
  echo "    Examples:";
  echo "      - $0  "; 
  echo "      - $0 -f 2 ";  
  echo "    ";
  if [ "$#" != 2 ] ; then echo -e "ERROR: $1 \n"; fi
  exit 0;
}
while [[ $# -gt 0 ]] ; do  
  C=$1; if [[ "$C" =~ [=] ]] ; then V=${C#*=}; elif [[ $2 == -* ]] ; then  V=""; else V=$2; shift; fi;
  case "$C" in    
    -f|--frame=*    ) FRAMECOUNT=$V;;
    -c|--cfgdir=*   ) CFGDIR=$( cd "$V" && pwd );;
    -o|--outdir=*   ) OUTDIR=$( cd "$V" && pwd );;
    --encoder=*     ) ENCODER=$V;;
    --decoder=*     ) DECODED=$V;;
    --metrics=*     ) METRICS=$V;;
    -h|--help       ) print_usage ;;
    *               ) print_usage "unsupported arguments: $C ";;
  esac
  shift;
done

# check parameters
for NAME in $ENCODER $DECODER $METRICS ;  do  if [ ! -f $NAME ] ; then print_usage "$NAME not exist"; fi done
if [ ! -d $CFGDIR ] || [ ! -f ${CFGDIR}/encoder.cfg ] || [ ! -f ${CFGDIR}/decoder.cfg ] || [ ! -f ${CFGDIR}/mmetric.cfg ]
then 
  print_usage "CFGDIR = \"$CFGDIR\" not valid configuration directory"; 
fi
if [ "$OUTDIR" == "" ] ; then OUTDIR=$CFGDIR; elif [ ! -d $OUTDIR ] ; then mkdir -p $OUTDIR; fi

NAME=$( basename $CFGDIR )
NAME=$( printf "${OUTDIR}/${NAME}_F%03d" $FRAMECOUNT )
VDMC=${NAME}.vdmc

# Encoder
echo -e "\033[0;32mEncode: ${NAME} \033[0m";
if [ ! -f ${VDMC} ]
then 
  CMD="$ENCODER \
        --config=${CFGDIR}/encoder.cfg \
        --fcount=${FRAMECOUNT} \
        --compressed=${VDMC} \
        --recmesh=${NAME}_%04d_rec.obj \
        --rectex=${NAME}_%04d_rec.png \
        --recmat=${NAME}_%04d_rec.mtl ";
  formatCmd "$CMD" "-- -c >"
  eval $CMD
else 
  echo "${VDMC} already exist"
fi

# Decoder
echo -e "\033[0;32mDecode: ${NAME} \033[0m";
CMD="$DECODER \
  --config=${CFGDIR}/decoder.cfg \
  --compressed=${VDMC} \
  --decmesh=${NAME}_%04d_dec.obj \
  --dectex=${NAME}_%04d_dec.png \
  --decmat=${NAME}_%04d_dec.mtl ";
formatCmd "$CMD" "-- -c >"
eval $CMD

# Metrics
echo -e "\033[0;32mMetrics: ${NAME} \033[0m";
CMD="$METRICS \
  --config=${CFGDIR}/mmetric.cfg \
  --fcount=${FRAMECOUNT} \
  --decMesh=${NAME}_%04d_dec.obj \
  --decTex=${NAME}_%04d_dec.png \
  --ibsm=0";
formatCmd "$CMD" "-- -c >"
eval $CMD
