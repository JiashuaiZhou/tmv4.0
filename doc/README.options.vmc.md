Vmc
===

General options
---------------

### `--help`
Print a list of available command line (and configuration file) options
along with their default values and exit.

### `--config=FILE`, `-c`
Specifies a configuration file to be immediately loaded.

### `--mode=VALUE`
Selects the codec's mode of operation.  A value of 0 enables
encoding functionality.  A value of 1 switches to decoding mode.


I/O parameters
--------------

### `--compressed=FILE`
Path to the generated compressed bitstream.

### `--imesh=FILE` (Encoder)
Path to the input meshes in OBJ format.

### `--itex=FILE` (Encoder)
Path to the input texture images in PNG format.

### `--base=FILE` (Encoder)
Path to the base meshes in OBJ format.

### `--gofstruct=FILE` (Encoder)
Path to the group of frames structure file.

### `--subdiv=FILE` (Encoder)
Path to the displaced subdivision meshes in OBJ format.

### `--recmat=FILE` (Encoder), `--decmat=FILE` (Decoder)
Path to the reconstructed/decoded material file in MTL format.

### `--recmesh=FILE` (Encoder), `--decmesh=FILE` (Decoder)
Path to the reconstructed/decoded meshes in OBJ format.

### `--rectex=FILE` (Encoder), `--dectex=FILE` (Decoder)
Path to the reconstructed/decoded texture images in PNG format.

### `--keep=0|1`
Keep intermediate files.


Common encoder and decoder parameters
-------------------------------------

### `--fstart=INT-VALUE`
First frame number.

### `--fcount=INT-VALUE`
Number of frames.

### `--framerate=REAL-VALUE`
Frame rate.

### `--normuv=0|1`
Normalize uv texture coordinates.


Encoder parameters
------------------

### `--gofmax=INT-VALUE`
Maximum group of frames size.

### `--it=INT-VALUE`
Subdivision iteration count.

### `--gqp=INT-VALUE`
Quantization bits for base mesh positions.

### `--tqp=INT-VALUE`
Quantization bits for base mesh texture coordinates.

### `--dqp=INT-VALUE-LIST`
Quantization parameter for displacements.

### `--dqb=REAL-VALUE-LIST`
Quantization bias for displacements.

### `--tvqp=INT-VALUE`
Quantization parameter for texture video.

### `--gdepth=INT-VALUE`
Input position bit depth.

### `--tdepth=INT-VALUE`
Input texture coordinate bit depth.

### `--texwidth=INT-VALUE`
Output texture width.

### `--texheight=INT-VALUE`
Output texture height.

### `--invorient=0|1`
Invert triangles orientation.

### `--unifvertices=0|1`
Unify duplicated vertices.

### `--encdisp=0|1`
Encode displacements video.

### `--enctex=0|1`
Encode texture video.


External tools
--------------

### `--gmenc=CMD` (Encoder)
Mesh encoder cmd.

### `--gmdec=CMD` (Encoder & Decoder)
Mesh decoder cmd.

### `--gvenc=CMD` (Encoder)
Displacements video encoder cmd.

### `--gvencconfig=FILE` (Encoder)
Displacements video encoder configuration file.

### `--tvenc=CMD` (Encoder)
Texture video encoder cmd.

### `--tvdec=CMD` (Decoder)
Texture video decoder cmd.

### `--tvencconfig=FILE` (Encoder)
Texture video encoder configuration file.

### `--csc=CMD` (Encoder & Decoder)
HDRTools command.

### `--cscencconfig=FILE` (Encoder)
HDRTools configuration file to convert colour space for encoding.

### `--cscdecconfig=FILE` (Encoder & Decoder)
HDRTools configuration file to convert colour space for decoding.


Example
-------

The following example shows how to apply ```vmc``` to encode a dynamic mesh sequence.

```console
./vmc --mode=0 \
  --invorient=0 --unifvertices=0 \
  --imesh=levi_voxelized/levi_fr%04d_qp12_qt13.obj \
  --itex=levi_voxelized/levi_fr%04d.png \
  --base=levi_voxelized_r0.03_ai/levi_fr%04d_qp12_qt13.obj_base.obj \
  --subdiv=levi_voxelized_r0.03_ai/levi_fr%04d_qp12_qt13.obj_subdiv.obj \
  --compressed=P11S08C01R01.bin \
  --recmesh=P11S08C01R01_%04d_rec.obj \
  --rectex=P11S08C01R01_%04d_rec.png \
  --recmat=P11S08C01R01_%04d.mtl \
  --fstart=1 --fcount=300 --gofmax=32 \
  --texwidth=2048 --texheight=2048 \
  --csc=path/to/HDRConvert \
  --gmenc=path/to/draco_encoder \
  --gmdec=path/to/draco_decoder \
  --gvenc=path/to/TAppEncoder_hm1621scm88 \
  --tvenc=path/to/TAppEncoder_hm1621scm88 \
  --gvencconfig=$SRCROOT/cfg/hm/ctc-hm-displacements-map-ai-main10.cfg \
  --tvencconfig=$SRCROOT/cfg/hm/ctc-hm-texture-ai.cfg \
  --cscencconfig=$SRCROOT/cfg/hdrconvert/bgr444toyuv420.cfg \
  --cscdecconfig=$SRCROOT/cfg/hdrconvert/yuv420tobgr444.cfg \
  --it=3 --gqp=11 --tqp=10 --tvqp=38 \
  --gdepth=12 --tdepth=13 --dqp=39,51,51 --dqb=0.3333,0.3333,0.3333
```

The following example shows how to apply ```vmc``` to decode a dynamic mesh sequence.

```console
./vmc --mode=1 \
      --compressed=P11S08C01R01.bin \
      --decmesh=P11S08C01R01_%04d.obj \
      --dectex=P11S08C01R01_%04d.png \
      --decmat=P11S08C01R01_%04d.mtl \
      --fstart=1 \
      --gmdec=path/to/draco_decoder \
      --gvdec=path/to/TAppDecoder_hm1621scm88 \
      --tvdec=path/to/TAppDecoder_hm1621scm88 \
      --csc=path/to/HDRConvert \
      --cscdecconfig=$SRCROOT/cfg/hdrconvert/yuv420tobgr444.cfg \
      --normuv=0
```
