
<!--- Usage  --->
# Usage

## Encode 

The encode command line is the following one: 

```console
$ ./build/Release/bin/encode \
    --config=./generatedConfigFiles/s3c1r3_bask/encoder.cfg \
    --frameCount=1 \
    --compressed=s3c1r3_bask.vmesh 
```

## Decode

The decode can be executed with:

```console
./build/Release/bin/decode \
  --config=./generatedConfigFiles/s3c1r3_bask/decoder.cfg \
  --compressed=s3c1r3_bask.vmesh \
  --decMesh=s3c1r3_bask_%04d_dec.obj \
  --decTex=s3c1r3_bask_%04d_dec.png \
  --decMat=s3c1r3_bask_%04d_dec.mtl \
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
$ ./scripts/gen-cfg.sh \
    --cfgdir=./cfg/ \
    --outdir=/path/to/generated/cfgfiles
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
    --rateId=3 \
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

The --vmeshMetricSW paramerter executes the vmesh metric software to compute metrics rather than the mm software. 
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

