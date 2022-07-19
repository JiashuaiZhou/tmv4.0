#!/usr/bin/env bash
#
# Run named steps for a single experiment test point:
#   "pp": gengof, simplify, uvatlas, fitsubdiv_intra, fitsubdiv_inter
#   encoder
#   decoder
#   psnr
#
# Options:
#   -C <dir>  Execute test point in configured directory <dir>
#
# Environment variables:
#   SKIP      Space separated list of steps to skip
#   ONLY      Space separated list of steps to perform
#   V=""      Pretty print progress, hide command output
#   V=1       Verbose printing of commands
#   V=s       Show command stdout/stderr
#   V=1s      Show command and their stdout/stderr
#
#   GENGOF    command to run gengof
#   SIMPLIFY  command to run simplify
#   UVATLAS   command to run uvatlas
#   FITSUBDIV command to run fitsubdiv
#   VMC       command to run vmc
#   MMETRIC   command to run mm

set -e
set -o pipefail

# option processing
while (( $# )); do
	case $1 in
    -C) cd "$2" ; shift ;;
    *) ONLY="$ONLY${ONLY:+ }$1" ;;
	--help|-h)
		echo -e "usage:\n vmesh-step.sh [-C <dir>] [steps ...]"
		exit 1
	esac
	shift;
done

# Convert verbose options to V and Vs
[[ $V = *s* ]] && Vs=1
[[ $V = *1* ]] && V=1 || V=

##
# the top level steps
#  1. Generate initial gop structure if required
#  2. Simplify input meshes
#  3. Isocharting
# 4a. Subdivision meshes (intra)
# 4b. Subsivision meshes (inter)
function perform_each_step {
    # load sequence configuration
    source_cfg seqcfg.cfg
    source_cfg mmetric.cfg

    # all pre-processing steps
    do? pp gengof           && gengof
    do? pp simplify         && foreach_frame simplify
    do? pp uvatlas          && foreach_frame uvatlas
    do? pp fitsubdiv_intra  && foreach_frame fitsubdiv_intra
    do? pp fitsubdiv_inter  && fitsubdiv_inter_and_choose

    # actual coding
    do? encode encoder      && encode
    do? decode decoder      && decode

    # psnr metric
    do? psnr                && decode_psnr
}

# default commands
function _gengof ()    { _command "${GENGOF:-gengof}" "$@" ; }
function _simplify ()  { _command "${SIMPLIFY:-simplify}" "$@" ; }
function _uvatlas ()   { _command "${UVATLAS:-uvatlas}" "$@" ; }
function _fitsubdiv () { _command "${FITSUBDIV:-fitsubdiv}" "$@" ; }
function _vmc ()       { _command "${VMC:-vmc}" "$@" ; }
function _mmetric ()   { _command "${MMETRIC:-mm}" "$@" ; }

###############################################################################
# utility functions

# only do a step if it is a) configured and b) not skipped
function do? {
    # if ONLY specified, skip if none of the steps are in ONLY
    [[ -n $ONLY ]] && none_in ONLY $@ && return 1

    # not allowed if any of the steps are in skipped
    ! none_in SKIP $@ && return 1

    # skip if no config (second argument only if present)
    [[ $# == 1 || -f $2.cfg ]] || return 1

    return 0
}

# true if any of $2... are in $!1
function none_in {
    arr=$1
    shift
    for x in "$@" ; do is_in $x $arr && return 1 ; done
    return 0
}

# true if $1 is in !$2
function is_in {
    for x in ${!2} ; do [[ $x == $1 ]] && return 0 ; done
    return 1
}

# run a command over each frame (frame number in FNUM variable)
function foreach_frame {
    prepare_"$1"
    for FNUM in $( seq $seqcfg_firstframe $seqcfg_lastframe ) ; do
        "$@"
    done
}

###############################################################################
# Perparation for each tool, ensures that output directories exist

prepare_gengof () {
    source_cfg gengof.cfg
}

prepare_simplify () {
    source_cfg simplify.cfg
    mkdir -p "$(dirname "${simplify_decimated}")"
    mkdir -p "$(dirname "${simplify_reference}")"
    mkdir -p "$(dirname "${simplify_mapped}")"
}

prepare_uvatlas () {
    source_cfg simplify.cfg
    mkdir -p "$(dirname "${uvatlas_output}")"
}

prepare_fitsubdiv_intra () {
    source_cfg fitsubdiv_intra.cfg
    mkdir -p "$(dirname "${fitsubdiv_intra_base}")"
    mkdir -p "$(dirname "${fitsubdiv_intra_subdiv}")"
    mkdir -p "$(dirname "${fitsubdiv_intra_nsubdiv}")"
}

prepare_fitsubdiv_inter () {
    source_cfg fitsubdiv_inter.cfg
    mkdir -p "$(dirname "${fitsubdiv_inter_base}")"
    mkdir -p "$(dirname "${fitsubdiv_inter_subdiv}")"
    mkdir -p "$(dirname "${fitsubdiv_inter_nsubdiv}")"
}


###############################################################################
# tool invocation

# The following commands assume that their config file specifies appropriate
# input and output file patterns.
gengof () { prepare_gengof && run gengof -c gengof.cfg $GENGOFFLAGS ; }
simplify () { run simplify -c simplify.cfg $SIMPLIFYFLAGS --fnum=$FNUM ; }
uvatlas () { run uvatlas -c uvatlas.cfg $UVATLASFLAGS --fnum=$FNUM ; }

# fitsubdiv for intra frames
fitsubdiv_intra () {
    type=I run \
    fitsubdiv -c fitsubdiv_intra.cfg $FITSUBDIVIFLAGS --fnum=$FNUM
}

# fitsubdiv for inter frames
fitsubdiv_inter () {
    type=P run \
    fitsubdiv -c fitsubdiv_inter.cfg $FITSUBDIVPFLAGS \
        --fnum=$FNUM --rnum=$RNUM
}

# The encoder inputs are specified by the config file, the output not
encode () {
    source_cfg encoder.cfg
    out=${encoder_compressed} run \
    vmc -c encoder.cfg \
        --recmesh=${encoder_compressed}.%04d_encoded.obj \
        --rectex=${encoder_compressed}.%04d_encoded.png \
        --recmat=${encoder_compressed}.%04d_encoded.mtl \
        $ENCFLAGS
}

# The decoder inputs are specified by the config file, the output not
decode () {
    source_cfg decoder.cfg
    out=${decoder_compressed}.decoded run \
    vmc -c decoder.cfg \
        --decmesh=${decoder_compressed}.%04d_decoded.obj \
        --dectex=${decoder_compressed}.%04d_decoded.png \
        --decmat=${decoder_compressed}.%04d_decoded.mtl \
        $DECFLAGS
}

# The decoder inputs are specified by the config file, the output not
decode_psnr () {
    source_cfg decoder.cfg
    decoded_mmpsnr_pcc ${decoder_compressed}
    decoded_mmpsnr_ibsm ${decoder_compressed}
}

##############################################################################
## The intra-inter mode decision process
#
# Using the initial GOF from gengof
#  - Generate inter subdivision
#  - Decide if inter is better than intra for that frame
#  - Write the result to a new GOF file
#
# This process is more involved, since each invocation must pick the correct
# reference frame number.
fitsubdiv_inter_and_choose () {
    prepare_fitsubdiv_inter

    # read the dependent configs
    source_cfg gengof.cfg
    source_cfg fitsubdiv_choose.cfg

    # start afresh
    rm -f "${fitsubdiv_choose_ld_gof}"

    # read the initial gof
    while read FNUM RNUM GNUM DONTCARE ; do
        # for fitsubdiv_with_mapping = 0, ignore gengof RNUMs; instead:
        #  - first frame in each gof is intra coded
        #  - each subsequent frame in the gof is predicted from the previous
        if is_fitsubdiv_without_mapping ; then
            [[ $GNUM != $GNUM_prev ]] && RNUM=$FNUM || RNUM=$((FNUM - 1))
            GNUM_prev=$GNUM
        fi

        printf -v iframe ${fitsubdiv_choose_ai_subdiv} $FNUM
        printf -v pframe ${fitsubdiv_inter_subdiv} $FNUM

        # If the frame should be an intra frame, don't bother with the inter
        # process at all
        if is_intra_frame ; then
            decision=intra
        else
            # otherwise, perform subdivision
            out=${pframe%_*}_fitsubdiv fitsubdiv_inter

            # measure psnr of inter and intra versions
            fitsubdiv_mmpsnr_pcc "$iframe"
            fitsubdiv_mmpsnr_pcc "$pframe"

            # choose
            decision=$( i-p_decide "$iframe.mmetric.csv" "$pframe.mmetric.csv" )
        fi

        case "$decision" in
            intra) REF=$FNUM ;;
            #inter) REF=$RNUM ;;
            #*) die "decision failure at frame $FNUM: '$decision'"
        esac
        echo "  [decide]  $testname frame $FNUM $decision ref:$REF"

        # copy intra files to replace inter if intra frame won
        if [[ "$decision" == "intra" ]] ; then
            cp_frame "${fitsubdiv_choose_ai_base}" "${fitsubdiv_inter_base}"
            cp_frame "${fitsubdiv_choose_ai_subdiv}" "${fitsubdiv_inter_subdiv}"
            cp_frame "${fitsubdiv_choose_ai_nsubdiv}" "${fitsubdiv_inter_nsubdiv}"
        fi

        # append the decision to the output gof file
        echo $FNUM $REF $GNUM >> "${fitsubdiv_choose_ld_gof}"
    done < "$gengof_output"
}

##
# A frame is an intra frame if:
function is_intra_frame {
    # If the input gof says so:
    [[ $FNUM -eq $RNUM ]]
}

##
# mtarget is only set if there is a mapping
function is_fitsubdiv_without_mapping {
    [[ -z ${fitsubdiv_inter_mtarget} ]]
}

##
# copy one frame to another location, expanding fame numbers
function cp_frame {
    cp $( printf $1 $FNUM ) $( printf $2 $FNUM )
}

##
# Choose between the two modes by comparing psnr.
# usage: i-p_decide <intra.csv> <inter.csv>
function i-p_decide {
    # NB: the column name in the csv file is wrong
    mmetric_psnr_col="haus_c2c_psnr"

    # extract the correct column from the csv file, compare floating point
    awk 'BEGIN { FS=";" ; FN=0 } FNR==1 { FN++ }
        FNR==1 { for (col = 1; col < NF; col++) cols[$col] = col }
        FNR==2 { psnr[FN] = $cols["'${mmetric_psnr_col}'"] }
        END {
          if (psnr[1] - psnr[2] > '${fitsubdiv_choose_maxAllowedD2PSNRLoss}')
              print "intra" ; else print "inter"
        }' \
        "$@"
}

###############################################################################
# PSNR calculation

function fitsubdiv_mmpsnr_pcc {
    first=$FNUM last=$FNUM \
    recon_obj="$1" \
    recon_tex="${mmetric_srctex}" \
    out="$1.mmetric.csv" mmpsnr_pcc
}

function decoded_mmpsnr_pcc {
    first=${seqcfg_firstframe} last=${seqcfg_lastframe} \
    recon_obj="$1.%04d_decoded.obj" \
    recon_tex="$1.%04d_decoded.png" \
    out="$1.decoded.pcc_mmetric.csv" mmpsnr_pcc
}

function decoded_mmpsnr_ibsm {
    first=${seqcfg_firstframe} last=${seqcfg_lastframe} \
    recon_obj="$1.%04d_decoded.obj" \
    recon_tex="$1.%04d_decoded.png" \
    out="$1.decoded.ibsm_mmetric.csv" mmpsnr_ibsm
}

mmpsnr_seq () {
    run mmetric \
        sequence --firstFrame ${first} --lastFrame ${last} END \
        dequantize --inputModel "${mmetric_srcmesh}" \
          --outputModel ID:deqRef --useFixedPoint --qp ${mmetric_srcgeombits} \
          --minPos "${mmetric_srcminpos}" \
          --maxPos "${mmetric_srcmaxpos}" \
          --qt ${mmetric_srctexcoordbits} --minUv "0 0" --maxUv "1.0 1.0" END \
        dequantize --inputModel "${recon_obj}" \
          --outputModel ID:deqDis --useFixedPoint --qp ${mmetric_srcgeombits} \
          --minPos "${mmetric_srcminpos}" \
          --maxPos "${mmetric_srcmaxpos}" \
          --qt ${mmetric_srctexcoordbits} --minUv "0 0" --maxUv "1.0 1.0" END \
        "$@"
}

mmpsnr_ibsm () {
    rm -f "${out}"
    mmpsnr_seq \
        compare --mode ibsm \
        --inputModelA ID:deqRef --inputMapA "${mmetric_srctex}" \
        --inputModelB ID:deqDis --inputMapB "${recon_tex}" \
        --outputCsv "${out}"
}

mmpsnr_pcc () {
    rm -f "${out}"
    mmpsnr_seq \
        reindex --sort oriented -i ID:deqRef -o ID:ref_reordered END \
        sample --mode grid --gridSize ${mmetric_gridsize} --hideProgress \
          --useNormal --useFixedPoint \
          --minPos "${mmetric_srcminpos}" \
          --maxPos "${mmetric_srcmaxpos}" \
          --bilinear -i ID:ref_reordered \
          -m "${mmetric_srctex}" -o ID:pcRef END \
        reindex --sort oriented -i ID:deqDis -o ID:ref_reordered END \
        sample --mode grid --gridSize ${mmetric_gridsize} --hideProgress \
          --useNormal --useFixedPoint \
          --minPos "${mmetric_srcminpos}" \
          --maxPos "${mmetric_srcmaxpos}" \
          --bilinear -i ID:ref_reordered \
          -m "${recon_tex}" -o ID:pcDis END \
        compare --mode pcc --resolution ${mmetric_srcmaxbblen} \
          --inputModelA ID:pcRef --inputModelB ID:pcDis \
          --outputCsv "${out}"
}

###############################################################################
# Configuration file loading

# import a config file, discarding comments
# the option X in foo.cfg is imported as the variable foo_X
function source_cfg {
    is_loaded_cfg "$1" && return

    while IFS="#:$IFS" read ARG VAL ; do
        [[ -z "$ARG" ]] && continue
        eval "${1%.cfg}_${ARG//-/_}=\$VAL"
    done < "$1"

    eval "loaded_${1%.cfg}=1"
}

# has the config file been loaded?
function is_loaded_cfg {
    var="loaded_${1%.cfg}"
    [[ -n ${!var} ]]
}

###############################################################################
# pretty-printing

# generate the test name based on the last three directories in the path
function testname {
    local IFS=/
    set -- $1
    shift $(( $# - 3 ))
    echo "$*"
}
testname="${testname:-$(testname ${PWD})}"

# log a command (non-verbose)
function log {
    printf "  [%-6.6s]" ${1#_}
    echo "${type:- }" "$testname" ${FNUM:+frame $FNUM} "$out"
}

# log a command (verbose)
function log1 {
    declare -a argv
    for x in "$@" ; do
        printf -v y "%q" "$x"
        argv+=("$y")
    done
    echo "${argv[@]}"
}

# calcalate output names for pretty printing and logging if not already known
function fix_outname {
    case $1$type in
        gengof) out=${gengof_output} ;;
        simplify) printf -v out ${simplify_decimated%_decimated.obj} $FNUM ;;
        uvatlas) printf -v out ${uvatlas_output%_decimated_tex.obj} $FNUM ;;
        fitsubdivI) printf -v out ${fitsubdiv_intra_base%_base.obj} $FNUM ;;
        fitsubdivP) printf -v out ${fitsubdiv_inter_base%_base.obj} $FNUM ;;
    esac
}

###############################################################################
# command launching and logging

# workaround a bug in bash-3.2 whereby failure in 'command' builtin
# can cause immediate exit
function _command () {
    local cmd=$1
    shift
    "$(which $cmd)" "$@"
}

# redirect stdout, stderr and launch command
# cat stderr if failed
function redir () {
    "$@" > "$out.log" 2> "$out.err" || {
        err=$?
        echo -n "*** Failed: "; log1 "$@"
        cat "$out.err"
        exit $err
    }
}

# verbose, launch command, tee output
function redir1 () {
    "$@" 2> >( tee "$out.err" ) | tee "$out.log"
}

# log a command, redirect its output
function run {
    fix_outname "$@"
    log$V "$@"
    redir$Vs "_$@"

    # delete error log if empty
    [[ -s "$out.err" ]] || rm "$out.err"
}

###############################################################################
# Perform the actual steps
perform_each_step
