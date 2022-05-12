#!/bin/bash
#
# Generate a configuration tree in $PWD from YAML files in the same
# directory.

set -e
shopt -s nullglob

script_dir="$(dirname $0)"
src_cfg_dir="$script_dir/../cfg"
outdir="."

while (( $# )); do
	case $1 in
	--outdir=*) outdir="${1#--outdir=}" ;;
	--cfgdir=*) src_cfg_dir="${1#--cfgdir=}" ;;
	--) shift; break ;;
	--help|*)
		echo -e "usage:\n $0\n" \
			"    [--outdir=<dir>] [--cfgdir=<dir>]"
		exit 1
	esac
	shift;
done

extra_args=("$@")

# add trailing slash to src_cfg_dir if not empty
src_cfg_dir="${src_cfg_dir}${src_cfg_dir:+/}"

# load site configuration if present
sitecfg="${src_cfg_dir:-}cfg-site.yaml"
[[ -f $sitecfg ]] || sitecfg=

##
# NB: it is important that the configs in each config set are
# capable of being merged together by gen-cfg.pl.  Ie, no two
# configs may have different definitions of one category.
cfg_all=(
	cfg-cond-ai.yaml
	cfg-cond-ld.yaml
)


do_one_cfgset() {
	local what=$1

	mkdir -p "${outdir}"

	cfgset="cfg_${what}[@]"

	for f in ${!cfgset}
	do
		echo "${src_cfg_dir}$f -> $outdir" ...
	done

	# NB: specifying extra_args at the end does not affect option
	# processing since gen-cfg.pl is flexible in argument positions
	$script_dir/gen-cfg.pl \
		--prefix="$outdir" --no-skip-sequences-without-src \
		"${src_cfg_dir}cfg-site-default.yaml" \
		"${src_cfg_dir}cfg-tools.yaml" \
		"${!cfgset/#/${src_cfg_dir}}" \
		"${src_cfg_dir}sequences.yaml" \
		${sitecfg:+"${sitecfg}"} \
		"${extra_args[@]}"

	rm -f "$outdir/config-merged.yaml"
}

do_one_cfgset "all"
