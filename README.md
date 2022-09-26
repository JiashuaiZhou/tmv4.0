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
 - mpeg-pcc-mmetric: http://mpegx.int-evry.fr/software/MPEG/PCC/mpeg-pcc-mmetric.git (tag: 1_0_1_lib_beta_v5)

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
- `encode`: that can be used to encode mesh sequence fo a V-Mesh bitstream. 
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
  --frameCount=2 \
  --compressed=s8c1r1_levi_F002.vmesh
```

## Decode

The decode can be executed with:

```console
./build/Release/bin/decode \
  --config=./generatedConfigFiles/s8c1r1_levi/decoder.cfg \
  --compressed=s8c1r1_levi_F002.vmesh \
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

This operation can be executed with script 
'./scripts/create_configuration_files.sh' and in this case the file 'cfg/cfg-site.yaml' 
is generated automatically according to the current folder.

```console
$ ./scripts/gen-cfg.sh --outdir=experiment 

$ ./scripts/create_configuration_files.sh 

./scripts/create_configuration_files.sh Create configuration files:

    Usage:
       -o|--outdir=: configured directory                    (default: generatedConfigFiles )
       -s|--seqdir=: source sequence directory               (default:  )
       -c|--codec=:  video codec: hm, vtm                    (default: hm )
       --update:     update cfg files stored in ./cfg/vmesh/ (default: 0 )

    Examples:
      ./scripts/create_configuration_files.sh
      ./scripts/create_configuration_files.sh \
        --outdir=generatedConfigFilesHM  \
        --seqdir=/path/to/contents/voxelized/ \
        --codec=hm
      ./scripts/create_configuration_files.sh \
        --outdir=generatedConfigFilesVTM \
        --seqdir=/path/to/contents/voxelized \
        --codec=vtm
      ./scripts/create_configuration_files.sh \
        --update \
        --codec=hm
```

## Run experiment

An example script (`scripts/run.sh`) demonstrates how
to launch the entire toolchain for a single job in the configured experiment. 

This scripts starts:

- encoding process
- decoding process
- pcc metrics computation
- ibsm metrics computation

The usage of this script are presented below: 

```console
$ ./scripts/run.sh 

./scripts/run.sh execute encoding/decoding/metrics

    Usage:
       -f|--frames    : frame count               (default: 1 )
       --c|-cfgdir    : configured directory      (default:  )
       --o|-outdir    : output directory          (default: results )
       --condId=      : condition: 1, 2           (default: 1 )
       --seqId=       : seq: 1,2,3,4,5,6,7,8      (default: 1 )
       --rateId=      : rate: 1,2,3,4,5           (default: 1 )
       --rateId=      : rate: 1,2,3,4,5           (default: 1 )
       --vmeshMetricSW: Use vmesh metric software (default: 0 )

    Examples:
      - ./scripts/run.sh
      - ./scripts/run.sh \
        --condId=1 \
        --seqId=3 \
        --rateId=3 \
        --cfgdir=generatedConfigFiles \
        --output=results
      - ./scripts/run.sh \
        --condId=1 \
        --seqId=3 \
        --rateId=3 \
        --cfgdir=generatedConfigFiles \
        --output=results \
        --vmeshMetricSW 
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
    --rateId=2 \
    --cfgdir=generatedConfigFiles \
    --outdir=results
Run vmesh encoder/decoder/metrics: ./scripts
Encode: results/F001/s3c1r2_bask/s3c1r2_bask
./build/Release/bin/encode \
    --config=./generatedConfigFiles/s3c1r2_bask//encoder.cfg \
    --frameCount=1 \
    --compressed=results/F001/s3c1r2_bask/s3c1r2_bask.vmesh \
   > results/F001/s3c1r2_bask/encoder.log 2>&1
Decode: results/F001/s3c1r2_bask/s3c1r2_bask
./build/Release/bin/decode \
    --config=./generatedConfigFiles/s3c1r2_bask//decoder.cfg \
    --compressed=results/F001/s3c1r2_bask/s3c1r2_bask.vmesh \
    --decMesh=results/F001/s3c1r2_bask/s3c1r2_bask_%04d_dec.obj \
    --decTex=results/F001/s3c1r2_bask/s3c1r2_bask_%04d_dec.png \
    --decMat=results/F001/s3c1r2_bask/s3c1r2_bask_%04d_dec.mtl \
   > results/F001/s3c1r2_bask/decoder.log 2>&1
Metrics IBSM: results/F001/s3c1r2_bask/s3c1r2_bask
./externaltools/mpeg-pcc-mmetric/build/mm \
   sequence \
    --firstFrame    1 \
    --lastFrame     1 \
   END \
   dequantize \
    --inputModel    /path/to/contents/basketball_player_fr%04d_qp12_qt12.obj \
    --outputModel   ID:deqRef \
    --useFixedPoint \
    --qp            12 \
    --minPos        "-725.812988 -483.908997 -586.02002" \
    --maxPos        "1252.02002 1411.98999 1025.34998" \
    --qt            12 \
    --minUv         "0 0" \
    --maxUv         "1.0 1.0" \
   END \
   dequantize \
    --inputModel    results/F001/s3c1r2_bask/s3c1r2_bask_%04d_dec.obj \
    --outputModel   ID:deqDis \
    --useFixedPoint \
    --qp            12 \
    --minPos        "-725.812988 -483.908997 -586.02002" \
    --maxPos        "1252.02002 1411.98999 1025.34998" \
    --qt            12 \
    --minUv         "0 0" \
    --maxUv         "1.0 1.0" \
   END \
   compare \
    --mode          ibsm \
    --inputModelA   ID:deqRef \
    --inputModelB   ID:deqDis \
    --inputMapA     /path/to/contents/basketball_player_fr%04d.png \
    --inputMapB     results/F001/s3c1r2_bask/s3c1r2_bask_%04d_dec.png \
    --outputCsv     results/F001/s3c1r2_bask/metric_ibsm.csv \
   > results/F001/s3c1r2_bask/metric_ibsm.log
Metrics PCC: results/F001/s3c1r2_bask/s3c1r2_bask
./externaltools/mpeg-pcc-mmetric/build/mm \
   sequence \
    --firstFrame    1 \
    --lastFrame     1 \
   END \
   dequantize \
    --inputModel    /path/to/contents/basketball_player_fr%04d_qp12_qt12.obj \
    --outputModel   ID:deqRef \
    --useFixedPoint \
    --qp            12 \
    --qt            12 \
    --minPos        "-725.812988 -483.908997 -586.02002" \
    --maxPos        "1252.02002 1411.98999 1025.34998" \
    --minUv         "0.0 0.0" \
    --maxUv         "1.0 1.0" \
   END \
   dequantize \
    --inputModel    results/F001/s3c1r2_bask/s3c1r2_bask_%04d_dec.obj \
    --outputModel   ID:deqDis \
    --useFixedPoint \
    --qp            12 \
    --qt            12 \
    --minPos        "-725.812988 -483.908997 -586.02002" \
    --maxPos        "1252.02002 1411.98999 1025.34998" \
    --minUv         "0.0 0.0" \
    --maxUv         "1.0 1.0" \
   END \
   reindex \
    --inputModel    ID:deqRef \
    --sort          oriented \
    --outputModel   ID:ref_reordered \
   END \
   reindex \
    --inputModel    ID:deqDis \
    --sort          oriented \
    --outputModel   ID:dis_reordered \
   END \
   sample \
    --inputModel    ID:ref_reordered \
    --inputMap      /path/to/contents/basketball_player_fr%04d.png \
    --mode          grid \
    --useNormal \
    --useFixedPoint \
    --minPos        "-725.812988 -483.908997 -586.02002" \
    --maxPos        "1252.02002 1411.98999 1025.34998" \
    --bilinear \
    --gridSize      1024 \
    --hideProgress  1 \
    --outputModel   ID:ref_pc \
   END \
   sample \
    --inputModel    ID:dis_reordered \
    --inputMap      results/F001/s3c1r2_bask/s3c1r2_bask_%04d_dec.png \
    --mode          grid \
    --useNormal \
    --useFixedPoint \
    --minPos        "-725.812988 -483.908997 -586.02002" \
    --maxPos        "1252.02002 1411.98999 1025.34998" \
    --bilinear \
    --gridSize      1024 \
    --hideProgress  1 \
    --outputModel   ID:dis_pc \
   END \
   compare \
    --mode          pcc \
    --inputModelA   ID:ref_pc \
    --inputModelB   ID:dis_pc \
    --resolution    1977.833008 \
    --outputCsv     results/F001/s3c1r2_bask/metric_pcc.csv \
   > results/F001/s3c1r2_bask/metric_pcc.log
NbOutputFaces      : 75648
TotalBitstreamBits : 150448
GridD1             : 73.833939
GridD2             : 75.434639
GridLuma           : 36.139542
GridChromaCb       : 43.351307
GridChromaCr       : 45.376358
IbsmGeom           : 46.775490
IbsmLuma           : 33.573721
EncTime            : 27.0134349
DecTime            : 0.34059222

```

The --vmeshMetricSW parameter executes the vmesh metric software to compute metrics rather than the mm software. 
In this case, the logs are as follows: 

```console
$ ./scripts/run.sh \
  --condId=1 \
  --seqId=3 \
  --rateId=2 \
  --cfgdir=generatedConfigFiles \
  --outdir=results \
  --vmeshMetricSW
Encode: results/F001/s3c1r2_bask/s3c1r2_bask
./build/Release/bin/encode \
    --config=./generatedConfigFiles/s3c1r2_bask//encoder.cfg \
    --frameCount=1 \
    --compressed=results/F001/s3c1r2_bask/s3c1r2_bask.vmesh \
   > results/F001/s3c1r2_bask/encoder.log 2>&1
Decode: results/F001/s3c1r2_bask/s3c1r2_bask
./build/Release/bin/decode \
    --config=./generatedConfigFiles/s3c1r2_bask//decoder.cfg \
    --compressed=results/F001/s3c1r2_bask/s3c1r2_bask.vmesh \
    --decMesh=results/F001/s3c1r2_bask/s3c1r2_bask_%04d_dec.obj \
    --decTex=results/F001/s3c1r2_bask/s3c1r2_bask_%04d_dec.png \
    --decMat=results/F001/s3c1r2_bask/s3c1r2_bask_%04d_dec.mtl \
   > results/F001/s3c1r2_bask/decoder.log 2>&1
Metrics: results/F001/s3c1r2_bask/s3c1r2_bask
./build/Release/bin/metrics \
    --config=./generatedConfigFiles/s3c1r2_bask//mmetric.cfg \
    --decMesh=results/F001/s3c1r2_bask/s3c1r2_bask_%04d_dec.obj \
    --decTex=results/F001/s3c1r2_bask/s3c1r2_bask_%04d_dec.png \
    --frameCount=1 \
   > results/F001/s3c1r2_bask/metric_met.log
NbOutputFaces      : 75648
TotalBitstreamBits : 150448
GridD1             : 73.8339386
GridD2             : 75.434639
GridLuma           : 36.1395416
GridChromaCb       : 43.3513069
GridChromaCr       : 45.376358
IbsmGeom           : 46.7754899
IbsmLuma           : 33.5737214
EncTime            : 28.7190478
DecTime            : 0.355798779
```

# Contacts and reporting issues
For any issues or questions don't hesitate to open issues in V-Mesh git repository or to contact us:

- Julien Ricard (julien.ricard@interdigital.com)
- Wenjie Zou (wjzou@xidian.edu.cn)