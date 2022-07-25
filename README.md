Video-based dynamic mesh coding software test model 
==============================================

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

The V-DMC software uses several dependencies that are cloned and patched by the CMake building process. These dependencies are: 
 - Directx-headers: https:/github.com/microsoft/DirectX-Headers.git
 - Directx-math: https:/github.com/microsoft/DirectXMath.git
 - Directx-mesh: https:/github.com/microsoft/DirectXMesh.git
 - UVAatlas:  https:/github.com/microsoft/UVAtlas.git
 - Draco: https:/github.com/google/draco.git
 - HDRTools: http:/gitlab.com/standards/HDRTools.git
 - HM: https:/vcgit.hhi.fraunhofer.de/jvet/HM.git
 - VTM: https:/vcgit.hhi.fraunhofer.de/jvet/VVCSoftware_VTM.git

# Running

## Encoder 

The encoder command line is the following one: 

```console
$ ./build/Release/bin/encode \
  --config=./generatedConfigFiles/s8c1r1_levi/encoder.cfg \
  --fcount=2 \
  --compressed=s8c1r1_levi_F002.vdmc
```

## Decoder 

The decoder can be executed with:

```console
./tmc_vdmc/build/Release/bin/decode \
  --compressed=s8c1r1_levi_F002.vdmc \
  --cscdecconfig=tmd_vdmc/cfg/hdrconvert/yuv420tobgr444.cfg \
  --decmesh=s8c1r1_levi_F002_%04d_dec.obj \
  --dectex=s8c1r1_levi_F002_%04d_dec.png \
  --decmat=s8c1r1_levi_F002_%04d_dec.mtl
```

## Runtime configuration and configuration files

A set of reference configuration files are provided in the `cfg/vdmc/`
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
       --update:     update cfg files stored in ./cfg/vdmc/ (default: 0 )

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

Run vdmc encoder/decoder/metrics: ./scripts
# Encode: s3c1r3/s3c1r3_bask_F001
./build/Release/bin/encode \
  --config=./generatedConfigFiles/s3c1r3_bask/encoder.cfg \
  --fcount=1 \
  --compressed=s3c1r3/s3c1r3_bask_F001.vdmc \
  --recmesh=s3c1r3/s3c1r3_bask_F001_%04d_rec.obj \
  --rectex=s3c1r3/s3c1r3_bask_F001_%04d_rec.png \
  --recmat=s3c1r3/s3c1r3_bask_F001_%04d_rec.mtl 2>&1 \
  > s3c1r3/s3c1r3_bask_F001_enc.log
...
# Decode: s3c1r3/s3c1r3_bask_F001
./build/Release/bin/decode \
  --config=./generatedConfigFiles/s3c1r3_bask/decoder.cfg \
  --compressed=s3c1r3/s3c1r3_bask_F001.vdmc \
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

Contacts and reporting issues
-------

For any issues or questions don't hesitate to open issues in V-DLC git repository of to contact us: 
- Julien Ricard (julien.ricard@interdigital.com)
- Wenjie Zou (wjzou@xidian.edu.cn)