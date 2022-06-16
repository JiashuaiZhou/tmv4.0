Video-based dynamic mesh coding software model
==============================================

Bootstrapping
-------------

1. Fetch and unpack submodules using `git submodule init`

2. Apply patches to the external dependencies

```console
$ cd dependencies/draco
$ git am < ../../patches/draco-0001-*.patch

$ cd dependencies/directx-mesh
$ git am < ../../patches/dxmesh-0001-*.patch
$ git am < ../../patches/dxmesh-0002-*.patch

$ cd dependencies/uvatlas
$ git am < ../../patches/uvatlas-0001-*.patch
```


Building
--------

### OSX
```console
$ mkdir build
$ cmake -S. -Bbuild -G Xcode
$ xcodebuild -project build/vmesh.xcodeproj -configuration Debug
$ build/vmesh/Debug/vmc --help
```

### Linux
```console
$ mkdir build
$ cmake -DCMAKE_BUILD_TYPE=Release -S. -Bbuild -G Ninja
$ ninja -C build
$ build/vmesh/vmc --help
```

### Windows
```console
> md build
> cmake -S. -Bbuild -G "Visual Studio 15 2017 Win64"
```

Running
-------

This software model is split into multiple independent tools.


### Runtime configuration and configuration files

A set of reference configuration files are provided in the `cfg/cfp`
directory.


### Example

To generate the configuration files,

1. copy and edit `cfg/cfg-site-default.yaml` as `cfg/cfg-site.yaml`,
   the paths for the binaries, sequence prefix, and the external
   tool configuration prefix;

2. run the gen-cfg.sh script:

```console
mpeg-vmesh-tm$ ../scripts/gen-cfg.sh --outdir=experiment
```

#### Using Makefile.vmesh-step

An example script (`scripts/Makefile.vmesh-step`) demonstrates how
to launch the toolchain for a single job in the configured experiment.
The VERBOSE=1 make variable causes the detailed command execution to be output.

```console
mpeg-vmesh-tm$ make -f $PWD/scripts/Makefile.vmesh-step  \
    -C experiment/ai/levi/r1/ \
    GENGOF=$PWD/build/vmesh/gengof \
    SIMPLIFY=$PWD/build/vmesh/simplify \
    UVATLAS=$PWD/build/vmesh/uvatlas \
    FITSUBDIV=$PWD/build/vmesh/fitsubdiv \
    VMC=$PWD/build/vmesh/vmc \
    MMETRIC=path/to/mm.1.0.1

  [gengof]  levi_voxelized.gof
  [decimx]  levi_voxelized_r0.03/levi_fr0000_qp12_qt13.obj_decimated.obj
  [decimx]  levi_voxelized_r0.03/levi_fr0001_qp12_qt13.obj_decimated.obj
...
  [uvatls]  levi_voxelized_r0.03/levi_fr0000_qp12_qt13.obj_decimated_tex.obj
  [uvatls]  levi_voxelized_r0.03/levi_fr0001_qp12_qt13.obj_decimated_tex.obj
...
  [fsbdiv]  levi_voxelized_r0.03_ai/levi_fr0000_qp12_qt13.obj_base.obj
  [fsbdiv]  levi_voxelized_r0.03_ai/levi_fr0001_qp12_qt13.obj_base.obj
...
  [encode]  levi_voxelised.vmesh
  [decode]  levi_voxelised.vmesh.decoded.stamp
  [metric]  levi_voxelised.vmesh.decoded.ibsm_mmetric.csv
  [metric]  levi_voxelised.vmesh.decoded.pcc_mmetric.csv
```

#### Alternative

The example script (`scripts/vmesh-step.sh`) forgoes dependency tracking and
parallel builds:

```console
mpeg-vmesh-tm$ env \
    GENGOF=$PWD/build/vmesh/gengof \
    SIMPLIFY=$PWD/build/vmesh/simplify \
    UVATLAS=$PWD/build/vmesh/uvatlas \
    FITSUBDIV=$PWD/build/vmesh/fitsubdiv \
    VMC=$PWD/build/vmesh/vmc \
    MMETRIC=path/to/mm.1.0.1 \
    $PWD/scripts/vmesh-step.sh -C experiment/ai/levi/r1/
...
```
