UVatlas
=======

General options
---------------

### `--help`
Print a list of available command line (and configuration file) options
along with their default values and exit.

### `--config=FILE`, `-c`
Specifies a configuration file to be immediately loaded.


I/O parameters
--------------

### `--input=FILE`, `-i`
Path to the input mesh in OBJ format.

### `--output=FILE`, `-o`
Path to the generated mesh with texture coordinates in OBJ format.


Iso chart
---------

### `--quality=DEFAULT|FAST|QUALITY`, `-q`
Controls the trade-off between processing time and parametrization
[quality](https://github.com/microsoft/UVAtlas/wiki/UVAtlasCreate)

  | Value   | Description                                                 |
  |:------- | ----------------------------------------------------------- |
  | DEFAULT | Select FAST or QUALITY according to face count              |
  | FAST    | Faster but at the cost of increased stretch or more charts  |
  | QUALITY | Higher quality charts, but slower and more memory intensive |

### `-n INT-VALUE`
The maximum number of charts required for the atlas. If this is 0, it will be
parameterized based solely on stretch. Note that this is not a hard limit, but
the isochart will stop when a valid charting is found that is greater than or
equal to this number.

### `--stretch=REAL-VALUE`
The maximum amount of stretch. If 0.0, no stretching is allowed. If 1.0, then
any amount of stretching is allowed.

### `--gutter=REAL-VALUE`
The minimum distance, in texels between two charts on the atlas. This gets
scaled by the width, so if gutter is 2.5, and it is used on a 512x512 texture,
then the minimum distance will be 2.5 / 512 in u-v space.

### `--width=INT-VALUE`
The width of the texture the atlas will be used on.

### `--height=INT-VALUE`
The height of the texture the atlas will be used on.

Example
-------
The following example shows how to apply ```uvatlas``` to a mesh.

```console
./uvatlas \
  --input=levi_voxelized_r0.03/levi_fr%04d_qp12_qt13.obj_decimated.obj \
  --output=levi_voxelized_r0.03/levi_fr%04d_qp12_qt13.obj_decimated_tex.obj \
  --gutter=32 --width=2048 --height=2048 --quality=QUALITY
```
