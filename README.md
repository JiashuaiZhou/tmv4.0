Video-based dynamic mesh coding software test model 
==============================================

<!--- Building  --->
# Building 

## Building script

A bash script is provided to facilitate the building operations. 

To build V-DMC test model softwares with this script please use the following command line:

```console
$ ./build.sh
```` 

Another script could be used to clean the current solutions with the following command lines:

```console
$ ./clear.sh      # Remove ./build/ sub-folder.
$ ./clear.sh all  # Remove ./build/ sub-folder and all cloned dependencies.
```

## Build manually

Standard CMake build commands can be used to build the software depending on the system you used.

### OSX
```console
$ mkdir build
$ cmake -S. -Bbuild -G Xcode
$ xcodebuild -project build/vmesh.xcodeproj -configuration Debug
```

### Linux
```console
$ mkdir build
$ cmake -DCMAKE_BUILD_TYPE=Release -S. -Bbuild/Release
$ cmake --build ./build/Release --config Release --parallel 12
```

### Windows
```console
$ md build
$ cmake -DCMAKE_BUILD_TYPE=Release -S. -Bbuild/Release
$ cmake --build ./build/Release --config Release --parallel 12
```

## Dependencies

The V-DMC test model software uses several dependencies that are cloned and patched by the CMake building process. These dependencies are: 
 - Directx-headers: https:/github.com/microsoft/DirectX-Headers.git (checkout: 1b79ddaeabc4b16c772ca63adc5bdf7d5f741460)
 - Directx-math: https:/github.com/microsoft/DirectXMath.git (checkout: b404898c9dcaff7b686bbaf6d2fba8ff0184a17e)
 - Directx-mesh: https:/github.com/microsoft/DirectXMesh.git (checkout: 2c0ed18e271afa99a70948f784dfe082127fa0de)
 - UVAatlas:  https:/github.com/microsoft/UVAtlas.git (checkout: 5af1b5d2a0fd9e0e5d17aa0971ab17c890e318e0)
 - Draco: https:/github.com/google/draco.git (checkout: 266f47ce58b0568ff9328e12174b25cb0fbd3b2e)
 - HDRTools: http:/gitlab.com/standards/HDRTools.git (tag: v0.23)
 - HM: https:/vcgit.hhi.fraunhofer.de/jvet/HM.git (tag: HM-16.21+SCM-8.8)
 - VTM: https:/vcgit.hhi.fraunhofer.de/jvet/VVCSoftware_VTM.git (tag: VTM-13.0)
 - mpeg-pcc-mmetric: http://mpegx.int-evry.fr/software/MPEG/PCC/mpeg-pcc-mmetric.git (tag: 1_0_1_lib_beta_v2)

<!--- Architecture  --->
# Architecture 

V-Mesh test model software is organized as follow: 

![Alt text](doc/architecture.png?raw=true "Architecture")

## Core libraries

The core codec processes are grouped in three lirbaries: 
- `vmeshCommon` containing the util objects and the processes shared by V-Mesh encoding and decoding processes. 
- `vmeshEncoder` containing the V-Mesh encoding processes. 
- `vmeshDecoder` containing the V-Mesh decoding processes. 

## Wrapper libraries

To unify interfaces with external libraries used to encode/decode meshes, encode/decode/convert videos and compute metrics, wrapper libraries
have been created:
- `videoEncoder`: wrapper to HM encoder, VTM encoder;
- `videoDecoder`: wrapper to HM decoder, VTM decoder;
- `geometryEncoder`: wrapper to Draco encoder; 
- `geometryDecoder`: wrapper to Draco decoder;
- `colourConverter: wrapper to HDRTools.

These libraries are based on a virtual object that can be derived to implement one specific interface with the external libraries. The source codes of the wrapper libraries are stored in the `source/wrapper/` sub-folder.

## Applications 

The source codes of V-Mesh applications are stored in the `source/app/` sub-folder. 

### main applications

The two main application of the V-Mesh test model are: 
- `encode`: that can be used to encode mesh sequence fo a V-MEsh bitstream. 
- `decode`: that decode V-Mesh bitstream.

The following section shows examples of the usage of these softwares.

### Wrapper applications

To evaluate the wrapper libraries specific applications have been created. These applications can be used to crosscheck the usage of the external applications. The source code of these applications are in `source/app/wrapper/` sub-folder: 
- `encodeDraco`;
- `decodeDraco`;
- `encodeVideo`;
- `decodeVideo`;
- `colourConverte`;
- `metrics`.

### Unit test applications

To evaluate the source code and to garantee a early regression detection, unit test application has been created. 

The unit test application is based on [Google Testing Framework](https://github.com/google/googletest). The source code of this software is stored in `source/app/unitTests/`. 
The list of the unit tests that are implemented can be logged with the following command line:

```console
$ ./build/Release/bin/unitTests --gtest_list_tests
draco.
  encode
  decode
metrics.
  compare
hm.
  disp
  disp2
  texture
colourConvert.
  hdrToolsUp
  hdrToolsDown
vmesh.
  all
```

The unit tests can be executed with:
```console
$ ./build/Release/bin/unitTests -v 0
... 
[       OK ] hm.texture (4770 ms)
[----------] 3 tests from hm (5595 ms total)

[----------] 2 tests from colourConvert
[ RUN      ] colourConvert.hdrToolsUp
[       OK ] colourConvert.hdrToolsUp (190 ms)
[ RUN      ] colourConvert.hdrToolsDown
[       OK ] colourConvert.hdrToolsDown (197 ms)
[----------] 2 tests from colourConvert (388 ms total)

[----------] 1 test from vmesh
[ RUN      ] vmesh.all
[       OK ] vmesh.all (49463 ms)
[----------] 1 test from vmesh (49463 ms total)

[----------] Global test environment tear-down
[==========] 9 tests from 5 test suites ran. (64516 ms total)
[  PASSED  ] 9 tests.
```

Note: it's greatly recommended to execute the unit test application before each submission in the repository.

<!--- Usage --->
# Usage

## Encode 

The encode command line is the following one: 

```console
$ ./build/Release/bin/encode \
  --config=./generatedConfigFiles/s8c1r1_levi/encoder.cfg \
  --fcount=2 \
  --compressed=s8c1r1_levi_F002.vmesh
```

## Decode

The decode can be executed with:

```console
./build/Release/bin/decode \
  --compressed=s8c1r1_levi_F002.vmesh \
  --cscdecconfig=cfg/hdrconvert/yuv420tobgr444.cfg \
  --decmesh=s8c1r1_levi_F002_%04d_dec.obj \
  --dectex=s8c1r1_levi_F002_%04d_dec.png \
  --decmat=s8c1r1_levi_F002_%04d_dec.mtl
```

## Runtime configuration and configuration files

A set of reference configuration files are provided in the `cfg/vmesh/`
directory for example.

To generate the configuration files according to your system paths, the following action must be made: 

1. copy and edit `cfg/cfg-site-default.yaml` as `cfg/cfg-site.yaml`,
   the paths for the binaries, sequence prefix, and the external
   tool configuration prefix;

2. run the ./scripts/gen-cfg.sh' script:

```console
$ ./scripts/gen-cfg.sh --cfgdir=./cfg/ --outdir=/path/to/generated/cfgfiles
```

This operation can be executed with the script './scripts/create_configuration_files.sh' and in this case the file 'cfg/cfg-site.yaml' is generated automatically according to the current folder.

```console
$ ./scripts/gen-cfg.sh --outdir=experiment 

./scripts/create_configuration_files.sh Create configuration files:

    Usage:
       -o|--outdir=: configured directory                   (default: generatedConfigFiles )
       -s|--seqdir=: source sequence directory              (default:  )
       --update:     update cfg files stored in ./cfg/vmesh/ (default: 0 )

    Examples:
      - ./scripts/create_configuration_files.sh
      - ./scripts/create_configuration_files.sh --outdir=generatedConfigFiles --seqdir=/path/to/contents/
      - ./scripts/create_configuration_files.sh --update
```

## Run experiment

An example script (`scripts/run.sh`) demonstrates how
to launch the entire toolchain for a single job in the configured experiment. This scripts starts:
- encoding process
- decoding process
- pcc metrics computation
- ibsm metrics computation

The usage of this script are presented below: 
```console
$ ./scripts/run.sh 

./scripts/run.sh execute encoding/decoding/metrics

    Usage:
       -f|--frames : frame count           (default: 1 )
       --c|-cfgdir : configured directory  (default:  )
       --o|-outdir : output directory      (default:  )
       --condId=   : condition: 1, 2       (default: 1 )
       --seqId=    : seq: 1,2,3,4,5,6,7,8  (default: 1 )
       --rateId=   : rate: 1,2,3,4,5       (default: 1 )

    Examples:
      - ./scripts/run.sh
      - ./scripts/run.sh --condId=1 --seqId=3 --rateId=3 --cfgdir=generatedConfigFiles --outdir=s3c1r3/

```

Note: The preceding script uses the `mpeg-pcc-mmetric` software and this dependency can be cloned and built with the following command line:

```console
$ ./scripts/get_external_tools.sh 
```

A example of execution of this script is:

```console
$ ./scripts/run.sh \
    --condId=1 \
    --seqId=3 \
    --rateId=3 \
    --cfgdir=generatedConfigFiles \
    --outdir=s3c1r3/

Run vmesh encoder/decoder/metrics: ./scripts
# Encode: s3c1r3/s3c1r3_bask_F001
./build/Release/bin/encode \
  --config=./generatedConfigFiles/s3c1r3_bask/encoder.cfg \
  --fcount=1 \
  --compressed=s3c1r3/s3c1r3_bask_F001.vmesh \
  --recmesh=s3c1r3/s3c1r3_bask_F001_%04d_rec.obj \
  --rectex=s3c1r3/s3c1r3_bask_F001_%04d_rec.png \
  --recmat=s3c1r3/s3c1r3_bask_F001_%04d_rec.mtl 2>&1 \
  > s3c1r3/s3c1r3_bask_F001_enc.log
...
# Decode: s3c1r3/s3c1r3_bask_F001
./build/Release/bin/decode \
  --config=./generatedConfigFiles/s3c1r3_bask/decoder.cfg \
  --compressed=s3c1r3/s3c1r3_bask_F001.vmesh \
  --decmesh=s3c1r3/s3c1r3_bask_F001_%04d_dec.obj \
  --dectex=s3c1r3/s3c1r3_bask_F001_%04d_dec.png \
  --decmat=s3c1r3/s3c1r3_bask_F001_%04d_dec.mtl \
  > s3c1r3/s3c1r3_bask_F001_dec.log
...
# Metrics IBSM: s3c1r3/s3c1r3_bask_F001
./externaltools/mpeg-pcc-mmetric/build/mm \
  sequence \
  --firstFrame 1 \
  --lastFrame 1 \
  END \
  dequantize \
  --inputModel /path/to/contents/basketball_player_voxelized/basketball_player_fr%04d_qp12_qt12.obj \
  --outputModel ID:deqRef \
  --useFixedPoint \
  --qp 12 \
  --minPos "-725.812988 -483.908997 -586.02002" \
  --maxPos "1252.02002 1411.98999 1025.34998" \
  --qt 12 \
  --minUv "0 0" \
  --maxUv "1.0 1.0" \
  END \
  dequantize \
  --inputModel s3c1r3/s3c1r3_bask_F001_%04d_dec.obj \
  --outputModel ID:deqDis \
  --useFixedPoint \
  --qp 12 \
  --minPos "-725.812988 -483.908997 -586.02002" \
  --maxPos "1252.02002 1411.98999 1025.34998" \
  --qt 12 \
  --minUv "0 0" \
  --maxUv "1.0 1.0" \
  END \
  compare \
  --mode ibsm \
  --inputModelA ID:deqRef \
  --inputModelB ID:deqDis \
  --inputMapA /path/to/contents/basketball_player_voxelized/basketball_player_fr%04d.png \
  --inputMapB s3c1r3/s3c1r3_bask_F001_%04d_dec.png \ \
  --outputCsv s3c1r3/s3c1r3_bask_F001_ibsm.csv \
  > s3c1r3/s3c1r3_bask_F001_ibsm.log
...
# Metrics PCC: s3c1r3/s3c1r3_bask_F001
./externaltools/mpeg-pcc-mmetric/build/mm \
  sequence \
  --firstFrame 1 \
  --lastFrame 1 \
  END \
  dequantize \
  --inputModel /path/to/contents/basketball_player_voxelized/basketball_player_fr%04d_qp12_qt12.obj \
  --outputModel ID:deqRef \
  --useFixedPoint \
  --qp 12 \
  --qt 12 \
  --minPos "-725.812988 -483.908997 -586.02002" \
  --maxPos "1252.02002 1411.98999 1025.34998" \
  --minUv "0.0 0.0" \
  --maxUv "1.0 1.0" \
  END \
  dequantize \
  --inputModel s3c1r3/s3c1r3_bask_F001_%04d_dec.obj \
  --outputModel ID:deqDis \
  --useFixedPoint \
  --qp 12 \
  --qt 12 \
  --minPos "-725.812988 -483.908997 -586.02002" \
  --maxPos "1252.02002 1411.98999 1025.34998" \
  --minUv "0.0 0.0" \
  --maxUv "1.0 1.0" \
  END \
  reindex \
  --inputModel ID:deqRef \
  --sort oriented \
  --outputModel ID:ref_reordered \
  END \
  reindex \
  --inputModel ID:deqDis \
  --sort oriented \
  --outputModel ID:dis_reordered \
  END \
  sample \
  --inputModel ID:ref_reordered \
  --inputMap /path/to/contents/basketball_player_voxelized/basketball_player_fr%04d.png \
  --mode grid \
  --useNormal \
  --useFixedPoint \
  --minPos "-725.812988 -483.908997 -586.02002" \
  --maxPos "1252.02002 1411.98999 1025.34998" \
  --bilinear \
  --gridSize 1024 \
  --hideProgress 1 \
  --outputModel ID:ref_pc \
  END \
  sample \
  --inputModel ID:dis_reordered \
  --inputMap s3c1r3/s3c1r3_bask_F001_%04d_dec.png \
  --mode grid \
  --useNormal \
  --useFixedPoint \
  --minPos "-725.812988 -483.908997 -586.02002" \
  --maxPos "1252.02002 1411.98999 1025.34998" \
  --bilinear \
  --gridSize 1024 \
  --hideProgress 1 \
  --outputModel ID:dis_pc \
  END \
  compare \
  --mode pcc \
  --inputModelA ID:ref_pc \
  --inputModelB ID:dis_pc \
  --resolution 1977.83 \
  --outputCsv s3c1r3/s3c1r3_bask_F001_pcc.csv \
  > s3c1r3/s3c1r3_bask_F001_pcc.log
...
EncTime            : 36.7691
DecTime            : 1.69012
NbOutputFaces      : 151360
TotalBitstreamBits : 299128.000000
GridD1             : 77.092239
GridD2             : 79.183113
GridLuma           : 39.001595
GridChromaCb       : 44.571358
GridChromaCr       : 47.598488
IbsmGeom           : 48.669617
IbsmLuma           : 36.119125
```
