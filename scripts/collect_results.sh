#!/bin/bash

CURDIR=$( cd "$( dirname "$0" )" && pwd ); 
MAINDIR=$( dirname ${CURDIR} )

function print_usage() {
  echo "$0 Collect results from log files"
  echo "";
  echo "  Usage:"   
  echo "    -h|--help : print help ";
  echo "    -q|--quiet: disable logs         (default: $VERBOSE )";
  echo "    --condId= : condition: 1, 2      (default: $CONDID )";
  echo "    --seqId=  : seq: 1,2,3,4,5,6,7,8 (default: $SEQID )";
  echo "    --rateId= : Rate: 1,2,3,4,5      (default: $RATEID )";
  echo "    --vdmc    : vdmc bitstream file  (default: \"$VDMC\" )";
  echo "    --logenc  : encoder log file     (default: \"$LOGENC\" )";
  echo "    --logdec  : decoder log file     (default: \"$LOGDEC\" )";
  echo "    --logmet  : metrics log file     (default: \"$LOGMET\" )";  
  echo "    --csv     : generate .csv file   (default: \"$CSV\" )";    
  echo "";
  echo "  Examples:";
  echo "    $0 -h"; 
  echo "    $0 \\"
  echo "      --condId=1 \\"
  echo "      --seqId=3 \\"
  echo "      --rateId=3 \\"
  echo "      --vdmc=test.bin \\"
  echo "      --logenc=encoder.log \\"
  echo "      --logdec=decoder.log \\"
  echo "      --logmet=metric.log ";
  echo "    ";
  if [ "$#" != 0 ] ; then echo -e "ERROR: $1 \n"; fi
  exit 0;
}

# Set default parameters
VERBOSE=1
CONDID=1
SEQID=1
RATEID=1
VDMC=
LOGENC=
LOGDEC=
LOGMET=
CSV=

# Parse input parameters
while [[ $# -gt 0 ]] ; do  
  C=$1; if [[ "$C" =~ [=] ]] ; then V=${C#*=}; elif [[ $2 == -* ]] ; then  V=""; else V=$2; shift; fi;
  case "$C" in    
    -h|--help  ) print_usage;;    
    -q|--quiet ) VERBOSE=0;;
    --condId   ) CONDID=$V;;
    --seqId    ) SEQID=$V;;
    --rateId   ) RATEID=$V;;    
    --vdmc     ) VDMC=$V;; 
    --logenc   ) LOGENC=$V;; 
    --logdec   ) LOGDEC=$V;; 
    --logmet   ) LOGMET=$V;;
    --csv      ) CSV=$V;;
    *          ) print_usage "unsupported arguments: $C ";;
  esac
  shift;
done

# Check parameters
if [ ! -f "$VDMC"   ] ; then print_usage "vdmc = \"$VDMC\" not exists"; exit -1; fi
if [ ! -f "$LOGENC" ] ; then print_usage "logenc = \"$LOGENC\" not exists"; exit -1; fi
if [ ! -f "$LOGDEC" ] ; then print_usage "logdec = \"$LOGDEC\" not exists"; exit -1; fi
if [ ! -f "$LOGMET" ] ; then print_usage "logmet = \"$LOGMET\" not exists"; exit -1; fi

# Get metrics results
if [ "$( cat ${LOGMET} | grep "mseF, PSNR(p2point) Mean=" )" == "" ] ; then 
  RESULTS=( $( cat ${LOGMET} | grep "Metric_results" | awk -f ',' '{ printf("%s %s %s %s %s %s %s ", $3, $4, $5, $6, $7, $8, $9 )}') )
else
  RESULTS[0]=$( cat ${LOGMET} | grep "mseF, PSNR(p2point) Mean="   | awk -F= '{ printf $2; }' )
  RESULTS[1]=$( cat ${LOGMET} | grep "mseF, PSNR(p2plane) Mean="   | awk -F= '{ printf $2; }' )
  RESULTS[2]=$( cat ${LOGMET} | grep "c\[0\],PSNRF          Mean=" | awk -F= '{ printf $2; }' )
  RESULTS[3]=$( cat ${LOGMET} | grep "c\[1\],PSNRF          Mean=" | awk -F= '{ printf $2; }' )
  RESULTS[4]=$( cat ${LOGMET} | grep "c\[2\],PSNRF          Mean=" | awk -F= '{ printf $2; }' )
  RESULTS[5]=$( cat ${LOGMET} | grep "GEO PSNR Mean="              | awk -F= '{ printf $2; }' )
  RESULTS[6]=$( cat ${LOGMET} | grep "Y   PSNR Mean="              | awk -F= '{ printf $2; }' )
fi

# Get number of output faces
NBOUTPUTFACES=$( cat ${LOGENC} | grep "Sequence face count" | awk '{print $4}' )

# Get bitstream total size in bits as reported by the encoding process
TOTALSIZEBITS=$( stat -c%s ${VDMC} | awk '{printf "%d\n", $1 * 8 }' )

# Get user encoder and decoder runtimes
ENCTIME=$( cat ${LOGENC} | grep "Sequence processing time" | awk '{print $4}' )
DECTIME=$( cat ${LOGDEC} | grep "Sequence processing time" | awk '{print $4}' )
ENCMEMO=$( cat ${LOGENC} | grep "Sequence peak memory"     | awk '{print $4 / 1024. }' )
DECMEMO=$( cat ${LOGDEC} | grep "Sequence peak memory"     | awk '{print $4 / 1024. }' )

# Output results 
if [ "${CSV}" != "" ]; then
  SEQLINE=( 0 1 2 3 6 7 4 5 )
  if [ ! -f ${CSV} ] ; then 
    HEADER="SeqId,CondId,RateId,NbOutputFaces,TotalBitstreamBits,GridD1,GridD2,GridLuma,GridChromaCb,GridChromaCr,"
    HEADER+="IbsmGeom,IbsmLuma,UserEncoderRuntime,UserDecoderRuntime,PeakEncoderMemory,PeakDecoderMemory"
    echo $HEADER > ${CSV}
  fi
  STR="${SEQID},${CONDID},${RATEID},${NBOUTPUTFACES},${TOTALSIZEBITS},"
  STR+="${RESULTS[0]},${RESULTS[1]},${RESULTS[2]},${RESULTS[3]},${RESULTS[4]},${RESULTS[5]},${RESULTS[6]},"
  STR+="${ENCTIME},${DECTIME},${ENCMEMO},${DECMEMO}"
  echo $STR >> ${CSV}
  if (( $VERBOSE )) ; then 
    printf "%2d %2d %2d | %9d %9d | %9.5f %9.5f %9.5f %9.5f %9.5f | %9.5f %9.5f | %9.2f %9.2f | %9.2f %9.2f | %s \n" \
        ${SEQID} ${CONDID} ${RATEID} ${NBOUTPUTFACES} ${TOTALSIZEBITS} \
        ${RESULTS[0]} ${RESULTS[1]} ${RESULTS[2]} ${RESULTS[3]} ${RESULTS[4]} ${RESULTS[5]} ${RESULTS[6]} \
        ${ENCTIME} ${DECTIME} ${ENCMEMO} ${DECMEMO} ${OUTDIR}
  fi
else
  STR="${SEQID} ${CONDID} ${RATEID} ${NBOUTPUTFACES} ${TOTALSIZEBITS} "
  STR+="${RESULTS[0]} ${RESULTS[1]} ${RESULTS[2]} ${RESULTS[3]} ${RESULTS[4]} ${RESULTS[5]} ${RESULTS[6]} "
  STR+="${ENCTIME} ${DECTIME} ${ENCMEMO} ${DECMEMO}"
  echo $STR
fi 
