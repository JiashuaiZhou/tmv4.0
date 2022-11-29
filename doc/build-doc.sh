#!/bin/bash

CURDIR=$( cd "$( dirname "$0" )" && pwd );
MAINDIR=$( dirname "$CURDIR" )
echo -e "\033[0;32mBuild doc: $(readlink -f $MAINDIR) \033[0m";

# this file must be executed to regenerate the README.md
# upon binary internal documentation updates
# it requires the binary to be up to date and compiled

echo -e \
	"\n\n<!--- \n" \
	"  ###################################################### \n" \
	"  # File automatically generated by ./doc/build-doc.sh # \n" \
	"  # do not edit manually.                              # \n" \
	"  ###################################################### \n" \
	"--> \n\n" > ${MAINDIR}/README.md \

for FILE in about.md clone.md build.md architecture.md usage.md enc_params.md dec_params.md met_params.md contact.md ; do 
	if [ ! -f ${CURDIR}/readme/$FILE ] ; then echo "${CURDIR}/readme/$FILE not exists "; exit -1; fi
	sed 's/(images\//(doc\/images\//g' ${CURDIR}/readme/${FILE} >> ${MAINDIR}/README.md
done



# homogeneize line endings
dos2unix ${MAINDIR}/README.md
#EOF