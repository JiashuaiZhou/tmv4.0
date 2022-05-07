Gengof
======

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
Path to input meshes in OBJ format.

### `--output=FILE`, `-o`
Path to the generated group-of-frames structure file.


GoF parameters
--------------

### `--startFrame=INT-VALUE`
The initial frame number of the input or output sequence.
The software replaces any instance of a `%d` printf format directive
with the current frame number when evaluating the `--input` or `--output`
options.

### `--frameCount=INT-VALUE`
Number of frames.

Example
-------
The following example shows how to generate a group-of-frames structure file
for an input mesh sequence.

```console
./gengof \
  --input=levi_voxelized/levi_fr%04d_qp12_qt13.obj \
  --ouput=levi_voxelized.gof --startFrame=0 --frameCount=150
```
