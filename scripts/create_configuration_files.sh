#!/bin/bash

CURDIR=$( cd "$( dirname "$0" )" && pwd ); 
MAINDIR=$( dirname ${CURDIR} )
echo -e "\033[0;32mCreate configuration files: ${CURDIR} \033[0m";

OUTDIR="generatedConfigFiles";
SEQDIR=""
UPDATE=0
print_usage()
{
  echo "$0 Create configuration files: "
  echo "";
  echo "    Usage:" 
  echo "       -o|--outdir=: configured directory                   (default: $OUTDIR )"  
  echo "       -s|--seqdir=: source sequence directory              (default: $SEQDIR )"
  echo "       --update:     update cfg files stored in ./cfg/vmesh/ (default: $UPDATE )"
  echo "";
  echo "    Examples:";
  echo "      - $0  "; 
  echo "      - $0 --outdir=generatedConfigFiles --seqdir=/home/library24/PCC/contents/mpeg_vmesh_cfp_final/contents/voxelized/"; 
  echo "      - $0 --update";  
  echo "    ";
  if [ "$#" != 2 ] ; then echo -e "ERROR: $1 \n"; fi
  exit 0;
}
while [[ $# -gt 0 ]] ; do  
  C=$1; if [[ "$C" =~ [=] ]] ; then V=${C#*=}; elif [[ $2 == -* ]] ; then  V=""; else V=$2; shift; fi;
  case "$C" in    
    -o|--outdir=*   ) OUTDIR=$V;;
    -s|--seqdir=*   ) SEQDIR=$V;;
    --update        ) UPDATE=1;;
    -h|--help       ) print_usage ;;
    *               ) print_usage "unsupported arguments: $C ";;
  esac
  shift;
done

if [ "${OUTDIR}" != "" ] ; then mkdir -p ${OUTDIR}; fi
if [ $UPDATE == 1 ]
then 
  cd ${MAINDIR}
  CFGDIR=./cfg
  OUTDIR=${CFGDIR}/vmesh/
  MAINDIR=/path/to/mpeg-vmesh-tm
  SEQDIR=/path/to/contents
  echo "SEQDIR  = $SEQDIR"
  echo "MAINDIR = $MAINDIR"
else 
  if [ "$SEQDIR" == "" ] || [ ! -d ${SEQDIR} ] ; then print_usage "SEQDIR = \"${SEQDIR}\" not exists"; fi
  OUTDIR=$( cd "$OUTDIR" && pwd )
  SEQDIR=$( cd "$SEQDIR" && pwd )
  # Update windows path
  MAINDIR=${MAINDIR/\/c\//C:\/}
  SEQDIR=${SEQDIR/\/c\//C:\/}
  CFGDIR=${MAINDIR}/cfg
fi
# Update cfg-site.yaml file
echo -e "\033[0;32mUpdate cfg-site.yaml file: ${CURDIR} \033[0m";
CFGSITE=${CFGDIR}/cfg-site.yaml
rm ${CFGSITE} -f
cat << EOF >> ${CFGSITE}
# Site-specific configuration 
--- 
 
vars: 
  # this is the directory containing all the source sequences 
  seq-prefix: ${SEQDIR}/
 
  # this is the directory containing the hm/hdrtools config 
  cfg-prefix: ${MAINDIR}/cfg/

EOF

cat $CFGSITE

# Generate configurations
if [ "$( command -v singularity )" == "" ]
then 
  ${CURDIR}/gen-cfg.sh --cfgdir=${CFGDIR} --outdir=${OUTDIR}
else
  singularity exec --bind /home/scratch03,/home/wp21 /home/isl_tools/SHUB/tools/perl5.30 \
    ${CURDIR}/gen-cfg.sh --cfgdir=${CFGDIR} --outdir=${OUTDIR}
fi
    
exit 
