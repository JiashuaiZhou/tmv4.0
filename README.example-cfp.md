Purpose
=======

This is a transcript of an experiment to reproduce and verify the CfP results.
It serves as both a description of the verification process, and an aide to
constructing experiments using the provided tools and scripts.

- First, the proponents bitstreams are decoded.
- Then, encoding and decoding is performed starting from the source data.

After each step, the results are verified against the published CfP data.

The CfP submission was, unfortunately, produced using a version of `uvatlas`
that uses a random number to seed its PRNG.  Other issues notwithstanding,
it is impossible to reproduce this number, and therefore the `uvatlas` results.

The following processes that are independent of `uvatlas` can be verified
directly:

- `gengof`: Initial group-of-frames generation.
- `simplify`: Decimation (sub-sampling) of input meshes.

For the following `uvatlas`-dependent steps, the `uvatlas`-processed data
provided by the proponents is used instead of invoking `uvatlas` itself:

- `fitsubdiv_intra`: Subdivision surfaces and fitting for intra frames (ai).
- `fitsubdiv_inter`: Subdivision surfaces and fitting for inter frames (ai+ld).
- inter-gof: Per-frame decisions for intra/inter coding.
- `encode`: Encoding the sequence.
- `decode`: Decoding the sequence and PSNR computation.


Remarks
=======

All examples presented assume use of the bash shell.

The makefiles require GNU make.  If using parallel jobs with GNU make 4,
job output can be deinterleaved using the `-O` option.  For example, `-Oline`
for interactive jobs, or `-Otarget` or `-Orecurse` when logging to a file.


Prerequisites
-------------

This verification was performed using an Ubuntu 18.04 GNU/Linux system.
It required the following packages:

- wget, git
- cmake, make, ninja-build,
- clang-13, libc++-13-dev, libc++abi-13-dev
- libomp5
- cpanminus, liblist-moreutils-perl, libyaml-perl

They may be installed using `apt-get install --no-install-recommends $pkg`.

> Note: up-to-date versions of Clang and CMake can be obtained by adding the
> following apt repositories and performing an `apt-get update` before
> installation of the packages.
>
> LLVM/Clang:
> ```bash
> wget -O - https://apt.llvm.org/llvm-snapshot.gpg.key | apt-key add -
> add-apt-repository deb http://apt.llvm.org/bionic/ llvm-toolchain-bionic-13 main
> ```
>
> CMake:
> ```bash
> wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc | apt-key add -
> add-apt-repository deb https://apt.kitware.com/ubuntu/ bionic main
> ```

`libyaml-perl` (providing YAML) should be at least version 1.25.  Use `cpanm`
to install locally if the system provided version is insufficient:

  ```bash
  cpanm -n YAML
  ```


Data
====

Original sequences
------------------

Test sequences, as obtained from MPEG, are assumed to be stored as
`/data/$seqname`.

  ```console
  $ ls /data
  basketball_player_voxelized  football_voxelized  longdress_voxelized
  thomas_voxelized             dancer_voxelized    levi_voxelized
  mitch_voxelized              soldier_voxelized
  ```


Proponent's pre-processed data (`simplify`, `uvatlas`, `fitsubdiv`)
-------------------------------------------------------------------

The decimated meshes, isocharted meshes, and subdivision meshes are downloaded
and extracted to `/data/cfp/`:

  ```bash
  for d in decimated uvatlas preprocessed ; do
      mkdir -p /data/cfp/$d
      cd !$

      wget -r -nd -l1 --accept=.zip --http-user=mpeg --http-password=XXX \
          https://content.mpeg.expert/MPEG/CfP/Mesh/P11/$d/

      for f in *.zip ; do unzip -q $f && rm $f ; done
  done
  ```

### Munging filenames to match the default configuration

The filenames used in the CfP are different to those produced by the
provided configuration and expected by `Makefile.vmesh-step`.  They
are converted as follows:

  ```bash
  for d in /data/cfp/{decimated,uvatlas,preprocessed} ; do
      cd $d

      # the configuration maintains the full sequence name
      for pfx in basketball_player dancer football longdress mitch soldier thomas levi
      do
          rename -e "s/^${pfx}_r/${pfx}_voxelized_r/" ${pfx}_r*/
      done

      # _ai and _ld are more obvious than _c01 and _c02
      rename -e "s/_c01$/_ai/" *_c01
      rename -e "s/_c02$/_ld/" *_c02

      # the configuration and makefile currently append "_foo.obj" to the
      # sequence file names, resulting in "xxx.obj_foo.obj"
      for s in */ ; do
          rename -e 's/_([^_.]+(_tex)?).obj$/.obj$&/' $s/*.obj
      done
  done
  ```


Proponent's pre-processed data (`gengof`)
-----------------------------------------

The initial per-sequence GoF files are downloaded to `/data/cfp/gengof`:

  ```bash
  mkdir -p /data/cfp/gengof
  cd !$

  wget -r -nd -l1 --accept=.gof --http-user=mpeg --http-password=XXX \
      https://content.mpeg.expert/MPEG/CfP/Mesh/P11/gengof/
  ```

The file names are converted to match those used by the provided configuration:

  ```bash
  rename -e 's/\.gof$/_voxelized$&/' /data/cfp/gengof/*.gof
  ```


Proponent's coded bitstreams (`encode`)
---------------------------------------

The bitstreams are downloaded, flattening the directory structure:

  ```bash
  mkdir -p /data/cfp/bs
  cd !$

  wget -r -nd -np -l3 --accept=.bin --http-user=mpeg --http-password=XXX \
      https://content.mpeg.expert/MPEG/CfP/Mesh/P11/bitstreams/
  ```

The sequence filenames are converted to something more literate:

  ```bash
  cd /data/cfp/bs

  # mapping of file name components to expected names
  seqs=( 0 longdress soldier basketball_player dancer mitch thomas football levi )
  conds=( 0 ai ld )
  rates=( 0 r1 r2 r3 r4 r5 )

  for f in *.bin ; do
      IFS=SCR. read P S C R X <<<"$f"
      mv $f ${conds[$C]}_${seqs[${S#0}]}_${rates[$R]}.vmesh
  done
  ```


Proponent's decoded files' checksums (`decode`)
-----------------------------------------------

The checksums of decoded files are downloaded:

  ```bash
  mkdir -p /data/cfp/decoded
  cd !$

  wget -r -nd -l1 --accept=.txt,.objsums --http-user=mpeg --http-password=XXX \
      https://content.mpeg.expert/MPEG/CfP/Mesh/P11/decoded/
  ```

The sequence filenames are converted to match the simulation's.  NB, this
is applied to the individual lines in the files too.

  ```bash
  cd /data/cfp/decoded

  # mapping of filename components to expected names
  seqs=( 0 longdress soldier basketball_player dancer mitch thomas football levi )
  conds=( 0 ai ld )
  rates=( 0 r1 r2 r3 r4 r5 )
  PNG=png MTL=mtl OBJ=obj

  # md5sums
  for f in *_MD5.txt ; do
      IFS=SCR._ read P S C R T X <<<"$f"

      perl -p -e "s/\(P11.+_fr(\d\d\d\d)\.(...)\)/(${seqs[${S#0}]}_voxelized.vmesh.\$1_decoded.\$2)/" \
          < $f > ${conds[$C]}_${seqs[${S#0}]}_${rates[$R]}.decoded.${!T}.md5sums
      rm $f
  done

  # objsums
  for f in *.objsums
  do
      IFS=SCR._ read P S C R X <<<"$f"

      perl -p -e "s/  P11.+_fr(\d\d\d\d).*$/  ${seqs[${S#0}]}_voxelized.vmesh.\$1_decoded.obj/" \
          < $f > ${conds[$C]}_${seqs[${S#0}]}_${rates[$R]}.objsums
      rm $f
  done
  ```


Software
========

The software and its dependencies are downloaded to, and built from
`/data/sw`.

  ```bash
  mkdir -p /data/sw/
  ```

## mpeg-vmesh-tm

The CfP encoder was compiled with clang-13 for MacOS.  To reproduce the
CfP results on Linux, it is necessary to use `libc++` as the standard library
implementation rather than the platform default `libstdc++.`

  ```bash
  cd /data/sw
  git clone http://mpegx.int-evry.fr/software/MPEG/dmc/mpeg-vmesh-tm.git
  cd mpeg-vmesh-tm

  cmake -S$PWD -B$PWD/build -G Ninja \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_C_COMPILER=clang-13 -DCMAKE_CXX_COMPILER=clang++-13 \
      -DCMAKE_CXX_FLAGS=-stdlib=libc++

  ninja -C build
  ```

## mpeg-pcc-mmetric

As for mpeg-vmesh-tm, it is necessary to use `libc++` as the standard library
implementation to reproduce the CfP results.

  ```bash
  cd /data/sw
  git clone http://mpegx.int-evry.fr/software/MPEG/PCC/mpeg-pcc-mmetric.git
  cd mpeg-pcc-mmetric

  cmake -S$PWD -B$PWD/build -G Ninja \
      -DGLFW_USE_OSMESA=TRUE \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_C_COMPILER=clang-13 -DCMAKE_CXX_COMPILER=clang++-13 \
      -DCMAKE_CXX_FLAGS=-stdlib=libc++

  ninja -C build
  ```

## jtcvc-hm

NB: The CfP results were obtained using HM-16.21+SCM-8.8.

  ```bash
  cd /data/sw
  git clone https://vcgit.hhi.fraunhofer.de/jvet/HM.git jctvc-hm
  cd jctvc-hm
  git checkout HM-16.21+SCM-8.8

  cmake -S$PWD -B$PWD/build -G Ninja \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_C_COMPILER=clang-13 -DCMAKE_CXX_COMPILER=clang++-13

  ninja -C build
  ```

## mpeg-hdrtools

  ```bash
  cd /data/sw
  git clone https://gitlab.com/standards/HDRTools hdrtools
  cd hdrtools

  cmake -S$PWD -B$PWD/build -G Ninja \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_C_COMPILER=clang-13 -DCMAKE_CXX_COMPILER=clang++-13

  ninja -C build
  ```


Common experiment configuration
===============================

It is necessary to set the paths for the source sequences, the location of the
external tool configuration files, and to the built versions of the tools used
by the codec.

This configuration is site-specific.  In the context of this verification task,
save the following configuration to `/data/sw/mpeg-vmesh-tm/cfg/cfg-site.yaml`.

```yaml
--- # cfg-site.yaml
encoder:
  - gmenc: /data/sw/mpeg-vmesh-tm/build/dependencies/draco/draco_encoder-1.5.2
  - gmdec: /data/sw/mpeg-vmesh-tm/build/dependencies/draco/draco_decoder-1.5.2
  - gvenc: /data/sw/jctvc-hm/bin/TAppEncoderStatic
  - tvenc: /data/sw/jctvc-hm/bin/TAppEncoderStatic
  - csc: /data/sw/hdrtools/build/bin/HDRConvert

decoder:
  - gmdec: /data/sw/mpeg-vmesh-tm/build/dependencies/draco/draco_decoder-1.5.2
  - gvdec: /data/sw/jctvc-hm/bin/TAppDecoderStatic
  - tvdec: /data/sw/jctvc-hm/bin/TAppDecoderStatic
  - csc: /data/sw/hdrtools/build/bin/HDRConvert

vars:
  seq-prefix: /data/
  cfg-prefix: /data/sw/mpeg-vmesh-tm/cfg/
```

PATH
----

Instead of setting the `GENGOF`, `SIMPLIFY`, `UVATLAS`, `FITSUBDIV`, `VMC`,
`MMETRIC`, and `OBJSUM` environment or make variables to the path of the
respective tool, it is assumed that the tools are findable via `$PATH`.

  ```bash
  PATH=/data/sw/mpeg-vmesh-tm/build/vmesh:$PATH
  PATH=/data/sw/mpeg-pcc-mmetric/bin:$PATH
  ```


CfP decoding verification experiment
====================================

To verify the decoded streams:

1.  Configure an experiment using the default parameters

      ```console
      $ /data/sw/mpeg-vmesh-tm/scripts/gen-cfg.sh --outdir=/data/verify/decode
      ```


2.  Symlink the bitstreams into the directory structure.

      ```bash
      cd /data/verify/decode
      for d in */*/* ; do
          path=(${d//\// })
          ln -s /data/cfp/bs/${d//\//_}.vmesh $d/${path[1]}_voxelized.vmesh
      done
      ```


3.  Run the decoder.

      ```console
      $ cd /data/verify/decode
      $ make -f /data/sw/mpeg-vmesh-tm/scripts/Makefile.vmesh-top decode
      ...
        [decode]  levi_voxelized.vmesh.decoded.stamp
        [metric]  levi_voxelized.vmesh.decoded.ibsm_mmetric.csv
        [metric]  levi_voxelized.vmesh.decoded.pcc_mmetric.csv
        [md5sum]  levi_voxelized.vmesh.decoded.md5sums
        [md5sum]  levi_voxelized.vmesh.decoded.objsums
      ...
      ```

    It is safe to ignore warnings such as
      `Makefile.cfg.seqcfg: No such file or directory`.
    These are generated by make when it tries to include a file that does
    not (yet) exist.  Make will then build the file and include it.
    Unfortunately disabling the warning also prevents make from stopping if
    there is a problem generating the file.

    > Note: Multiple tasks can be performed in parallel using `-j`.  For
    > example `-j40` for a machine with 40 CPU cores and sufficient RAM.


4.  Use the proponent's md5sums and objsums to check the decoded result.

      ```bash
      cd /data/verify/decode
      for d in */*/* ; do (
          cd $d

          objsum --quiet -c --cache="$(echo *.decoded.objsums)" \
              /data/cfp/decoded/${d//\//_}.decoded.objsums \
          && echo $d: obj OK \
          || echo $d: obj FAILED

          md5sum --quiet -c /data/cfp/decoded/${d//\//_}.decoded.png.md5sums \
          && echo $d: png OK \
          || echo $d: png FAILED
      ) ; done
      ```


CfP pre-processing verification (`gengof` & `simplify`)
=======================================================

To verify the initial pre-processing stages:

1.  Configure an experiment using the default parameters

      ```console
      $ /data/mpeg-vmesh-tm/scripts/gen-cfg.sh --outdir=/data/verify/cfpenc
      ```

    To save repeated typing of `-f .../Makefile.vmesh-top` when launching make,
    a makefile is added that includes this file:

      ```console
      $ cat > /data/verify/cfpenc/Makefile <EOF
      include /data/sw/mpeg-vmesh-tm/scripts/Makefile.vmesh-top
      EOF
      ```


2.  Run the two pre-processing stages

      ```console
      $ make -C /data/verify/cfpenc gengof simplify
      ...
        [gengof]  levi_voxelized.gof
        [decimx]  levi_voxelized_r0.03/levi_fr0000_qp12_qt13.obj
      ...
      ```

    > Note: Multiple tasks can be performed in parallel using `-j`.  For
    > example `-j40` for a machine with 40 CPU cores and sufficient RAM.


3.  Compare with the published data

    For the initial `gengof`-generated GoF structure:

      ```bash
      cd /data/verify/cfpenc
      for f in */*/*/*_voxelized.gof ; do
          ( cd /data/cfp/gengof && md5sum ${f##*/} ) \
          | ( cd ${f%/*} && md5sum -c )
      done
      ```

    For the `simplify`-generated decimated meshes:

      ```bash
      cd /data/verify/cfpenc
      for d in */*/*/*_voxelized_r???? ; do
          ( cd /data/cfp/decimated/${d##*/}
            ( f=(*) ; IFS=$'\n' ; echo "${f[*]}" ) \
            | xargs objsum
          ) \
          |  ( cd $d && objsum --quiet -c ) \
          && echo $d: OK \
          || echo $d: FAILED
      done
      ```


CfP pre-processing verification (`fitsubdiv`)
=============================================

This continues the experiment in `/data/verify/cfpenc` after the verification
of `gengof` and `simplify`, skipping `uvatlas`.


4.  The following configuration snippet will be used to select an alternative
    input to the `fitsubdiv` process.

      ```yaml
      --- # cfg-prebuilt-uvatlas.yaml
      vars:
        pp_pfx: /data/cfp/uvatlas/

      fitsubdiv_intra:
        - source: ${pp_pfx}${src-dir}_r${pp}/${src-mesh}_decimated_tex.obj

      fitsubdiv_inter:
        - - !conditional '${fitsubdiv_with_mapping}'
          - source: ${pp_pfx}${src-dir}_r${pp}/${src-mesh}_decimated_tex.obj
      ```


5.  Update the configuration using the above snippet

      ```console
      $ cd /data/verify/cfpenc
      $ cat > cfg-prebuilt-uvatlas.yaml <<'EOF'
      ... text from (4) above ...
      EOF
      $ /data/sw/mpeg-vmesh-tm/scripts/gen-cfg.sh -- cfg-prebuilt-uvatlas.yaml
      ```

    > Note: this could have been performed instead of step (1) above.


6.  Generate the subdivision meshes using `fitsubdiv`.

      ```console
      $ make -C /data/verify/cfpenc fitsubdiv_intra fitsubdiv_inter
      ...
      [fsbdiv]I levi_voxelized_r0.12_ai/levi_fr0000_qp12_qt13.obj
      [metric]  levi_voxelized_r0.12_ai/levi_fr0003_qp12_qt13.obj_subdiv.obj.mmetric.csv
      [fsbdiv]P levi_voxelized_r0.12_ld/levi_fr0003_qp12_qt13.obj
      [decide]P levi_voxelized_r0.12_ld/levi_fr0003_qp12_qt13.obj.decision inter 2
      ...
      ```

    > Note: to be certain that uvatlas is not used, add SKIP_UVATLAS=1 to
    > the make command line.

    > Note: multiple tasks can be performed in parallel using `-j`.  For
    > example `-j40` for a machine with 40 CPU cores and sufficient RAM.



7.  Compare the with the published preprocessed data.

    Verify for the all-intra condition.  The verification excludes checking
    the subdivision meshes for decimation rates 0.48, 0.60, and 0.72 since
    they are not used by the encoding process and the proponent used different
    options to generate them.

      ```bash
      cd /data/verify/cfpenc
      for d in ai/*/*/*_voxelized_r????_ai ; do
          ( cd /data/cfp/preprocessed/${d##*/}
            ( f=(*) ; IFS=$'\n' ; echo "${f[*]}" ) \
            | if [[ $d = *_r0.48_ai || $d = *_r0.60_ai || $d = *_r0.72_ai ]]
                then awk '/_subdiv.obj$/ { next } { print }'
                else cat
              fi \
            | xargs objsum
          ) \
          |  ( cd $d && objsum --quiet -c ) \
          && echo $d: OK \
          || echo $d: FAILED
      done
      ```

    Verify for inter condition.  In this case, the generated `_ai` data is not
    compared since it is not always generated with the same options as for the
    all-intra condition:

      ```bash
      cd /data/verify/cfpenc
      for d in ld/*/*/*_voxelized_r????_ld ; do
          ( cd /data/cfp/preprocessed/${d##*/}
            ( f=(*) ; IFS=$'\n' ; echo "${f[*]}" ) \
            | xargs objsum
          ) \
          |  ( cd $d && objsum --quiet -c ) \
          && echo $d: OK \
          || echo $d: FAILED
      done
      ```

    Verify the generated GoF structure that results from the `fitsubdiv_inter`
    mode decision process.

      ```bash
      cd /data/verify/cfpenc
      for d in ld/*/*/*_voxelized_r????_ld ; do
          # the sequence name
          s=${d##*/}
          s=${s%_r????_??}

          for f in /data/cfp/preprocessed/${d##*/}/*.gof ; do
              [[ -f $f ]] && cut -d' ' -f 1-3 < $f \
              | md5sum \
              | sed -e "s@-@$d/$s.gof@" \
              | md5sum -c
          done
      done
      ```


CfP encoding verification
=========================

This continues the experiment in `/data/verify/cfpenc` after the verification
of `fitsubdiv`.


8.  Run the encoder

      ```console
      $ make -C /data/verify/cfpenc encode decode
      ...
      [encode]  levi_voxelized.vmesh
      [decode]  levi_voxelized.vmesh.decoded.stamp
      ```


9.  Verify the generated bitstreams.

      ```bash
      cd /data/verify/cfpenc
      for d in */*/* ; do
          path=(${d//\// })
          cat /data/cfp/bs/${d//\//_}.vmesh \
          | md5sum \
          | sed -e "s@-@$d/${path[1]}_voxelized.vmesh@"
      done | md5sum -c
      ```


10. Verify the checksums of the decoded frames.

      ```bash
      for d in */*/* ; do (
          cd $d

          objsum --quiet -c --cache="$(echo *.decoded.objsums)" \
              /data/cfp/decoded/${d//\//_}.decoded.objsums \
          && echo $d: obj OK \
          || echo $d: obj FAILED

          md5sum --quiet -c /data/cfp/decoded/${d//\//_}.decoded.png.md5sums \
          && echo $d: png OK \
          || echo $d: png FAILED
      ) ; done
      ```


How to ...
==========

Skip `simplify`, `uvatlas`, and `fitsubdiv`
-------------------------------------------

   ```yaml
   ---
   vars:
     pp_pfx: /data/cfp/

   fitsubdiv_intra:
     - target: ${pp_pfx}${src-dir}_r${pp}/${src-mesh}_reference.obj
     - source: ${pp_pfx}${src-dir}_r${pp}/${src-mesh}_decimated_tex.obj
     - mapped: ${pp_pfx}${src-dir}_r${pp}/${src-mesh}_mapped.obj

   fitsubdiv_inter:
     - target: ${pp_pfx}${src-dir}_r${pp}/${src-mesh}_reference.obj
     -
       - !conditional '${fitsubdiv_with_mapping}'
       - source: ${pp_pfx}${src-dir}_r${pp}/${src-mesh}_decimated_tex.obj
       - mapped: ${pp_pfx}${src-dir}_r${pp}/${src-mesh}_mapped.obj
   ```


Running for select subtrees sequences/testpoints only
-----------------------------------------------------

Sometimes it is desirable to run the experiment for a particular directory
only, or a set of directories.

> Note: This assumes the makefile from (1) above was created.  If not,
> `-f /data/sw/mpeg-vmesh-tm/scripts/Makefile.vmesh-top` must be added.

For a single directory:
  ```bash
  make dirs=ai/levi/r1
  ```

For multiple directories:
  ```bash
  make dirs='ai/levi/r1 ai/levi/r2'
  ```

Using wildcards: The wildcard is expanded by make, not the shell.
  ```bash
  make dirs='ai/levi/*'
  ```


Running from within a subdirectory of an experiment tree
--------------------------------------------------------

Assuming the makefile from (1) above was created, then:

  ```bash
  cd ai/levi/r1
  make -C ../../.. dirs=$PWD
  ```
