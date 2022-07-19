#!/bin/bash

CURDIR=$( cd "$( dirname "$0" )" && pwd ); 
MAINDIR=$( dirname ${CURDIR} )
echo -e "\033[0;32mCreate configuration files: ${CURDIR} \033[0m";

OUTDIR="generatedConfigFiles";
SEQDIR=""
print_usage()
{
  echo "$0 Create configuration files: "
  echo "";
  echo "    Usage:" 
  echo "       -o|--outdir=: configured directory       (default: $OUTDIR )";      
  echo "       -s|--seqdir=: source sequence directory  (default: $SEQDIR )";
  echo "";
  echo "    Examples:";
  echo "      - $0  "; 
  echo "      - $0 --outdir=generatedConfigFiles --seqdir=/home/library24/PCC/contents/mpeg_vmesh_cfp_final/contents/voxelized/";  
  echo "    ";
  if [ "$#" != 2 ] ; then echo -e "ERROR: $1 \n"; fi
  exit 0;
}
while [[ $# -gt 0 ]] ; do  
  C=$1; if [[ "$C" =~ [=] ]] ; then V=${C#*=}; elif [[ $2 == -* ]] ; then  V=""; else V=$2; shift; fi;
  case "$C" in    
    -o|--outdir=*   ) OUTDIR=$V;;
    -s|--seqdir=*   ) SEQDIR=$V;;
    -h|--help       ) print_usage ;;
    *               ) print_usage "unsupported arguments: $C ";;
  esac
  shift;
done
if [ "${OUTDIR}" != "" ] ; then mkdir -p ${OUTDIR}; fi
if [ "$SEQDIR" == "" ] || [ ! -d ${SEQDIR} ] ; then print_usage "SEQDIR = \"${SEQDIR}\" not exists"; fi
OUTDIR=$( cd "$OUTDIR" && pwd )
SEQDIR=$( cd "$SEQDIR" && pwd )

# Update windows path
MAINDIR=${MAINDIR/\/c\//C:\/}
SEQDIR=${SEQDIR/\/c\//C:\/}

CFGDIR=${MAINDIR}/cfg_new
CFGSITE=${CFGDIR}/cfg-site.yaml
rm ${CFGSITE} -f
cat << EOF >> ${CFGSITE}
# Default site-specific configuration 
# - Copy this file as cfg-site.yaml 
# - Update the definitions with the paths to the binaries if 
#   they are not on \$PATH 
--- 
 
vars: 
  # this is the directory containing all the source sequences 
  # NB: it must end with a directory separator 
  seq-prefix: ${SEQDIR}/
 
  # this is the directory containing the hm/hdrtools config 
  # NB: it must end with a directory separator 
  cfg-prefix: ${MAINDIR}/cfg/
EOF

if [ "$( command -v singularity )" == "" ]
then 
  ${CURDIR}/gen-cfg.sh --cfgdir=${CFGDIR} --outdir=${OUTDIR}
else
  singularity exec --bind /home/scratch03,/home/wp21 /home/isl_tools/SHUB/tools/perl5.30 \
    ${CURDIR}/gen-cfg.sh --cfgdir=${CFGDIR} --outdir=${OUTDIR}
fi
    
exit 
