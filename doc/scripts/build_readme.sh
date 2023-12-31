#!/bin/bash

CURDIR=$( cd "$( dirname "$0" )" && pwd );
CURDIR=$( dirname "$CURDIR" )
MAINDIR=$( dirname "$CURDIR" )
echo -e "\033[0;32mBuild doc: $(readlink -f $MAINDIR) \033[0m";

# Create tmp files
if [ ! -f ${CURDIR}/readme/dependencies.md ]; then 
	${CURDIR}/scripts/build_dependencies.sh > ${CURDIR}/readme/dependencies.md
	echo "create readme/dependencies.md"
fi
if [ ! -f ${CURDIR}/readme/enc_params.md ]; then 
	${CURDIR}/scripts/build_input_parameters.sh ${MAINDIR}/build/Release/bin/Release/encode.exe > ${CURDIR}/readme/enc_params.md
	echo "create readme/enc_params.md"
fi
if [ ! -f ${CURDIR}/readme/dec_params.md ]; then 
	${CURDIR}/scripts/build_input_parameters.sh ${MAINDIR}/build/Release/bin/Release/decode.exe > ${CURDIR}/readme/dec_params.md
	echo "create readme/dec_params.md"
fi
if [ ! -f ${CURDIR}/readme/met_params.md ]; then 
	${CURDIR}/scripts/build_input_parameters.sh ${MAINDIR}/build/Release/bin/Release/metrics.exe > ${CURDIR}/readme/met_params.md
	echo "create readme/met_params.md"
fi

# this file must be executed to regenerate the README.md
# upon binary internal documentation updates
# it requires the binary to be up to date and compiled

echo -e \
	"\n\n<!--- \n" \
	"  ################################################################# \n" \
	"  # File automatically generated by ./doc/scripts/build_readme.sh # \n" \
	"  # script.                                                       # \n" \
	"  ################################################################# \n" \
	"  # Do not edit manually this file                                # \n" \
	"  ################################################################# \n" \
	"  # Please update the following files if needed:                  # \n" \
	"  #  - doc/readme/about.md                                        # \n" \
  "  #  - doc/readme/clone.md                                        # \n" \
  "  #  - doc/readme/build.md                                        # \n" \
  "  #  - doc/readme/dependencies.md                                 # \n" \
  "  #  - doc/readme/architecture.md                                 # \n" \
  "  #  - doc/readme/usage.md                                        # \n" \
	"  # and execute script:                                           # \n" \
  "  #  - ./doc/scripts/build_readme.sh                              # \n" \
	"  ################################################################# \n" \
	"--> \n\n" > ${MAINDIR}/README.md \

for FILE in about.md \
					  clone.md \
					  build.md \
					  dependencies.md \
					  architecture.md \
					  usage.md \
					  enc_params.md \
					  dec_params.md \
					  met_params.md \
					  contact.md ; do 
	if [ ! -f ${CURDIR}/readme/$FILE ] ; then echo "${CURDIR}/readme/$FILE not exists "; exit -1; fi
	sed 's/(images\//(doc\/images\//g' ${CURDIR}/readme/${FILE} >> ${MAINDIR}/README.md
done

rm ${CURDIR}/readme/dependencies.md \
   ${CURDIR}/readme/enc_params.md   \
   ${CURDIR}/readme/dec_params.md   \
   ${CURDIR}/readme/met_params.md

# homogeneize line endings
dos2unix ${MAINDIR}/README.md

#EOF
exit 

