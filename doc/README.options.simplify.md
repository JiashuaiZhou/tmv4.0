Simplify
========

General options
---------------

### `--help`
Print a list of available command line (and configuration file) options
along with their default values and exit.

### `--config=FILE`, `-c`
Specifies a configuration file to be immediately loaded.


I/O parameters
--------------

### `--input=FILE`
Path to the input mesh in OBJ format.

### `--decimated=FILE`
Path to the generated decimated mesh in OBJ format.

### `--mapped=FILE`
Path to the generated mapped mesh in OBJ format. The mapped mesh has the same
connectivity as the reference mesh with positions obtained by projection on
the decimated mesh.

### `--reference=FILE`
Path to the generated reference mesh in OBJ format. The reference mesh is
obtained after vertex unification, duplicated triangles removal, and small
connected components filtering.

### `--fnum=INT-VALUE`
Frame number, used to expand `%d` format specifies in `input`, `decimated`,
`mapped`, and `reference` file names.


Simplification Parameters
-------------------------

### `--target=REAL-VALUE`
Target ratio of decimated mesh triangle count compared to the input mesh
triangle count.

### `--qt=INT-VALUE`
Bit depth for quantized texture coordinates.

### `--cctcount=INT-VALUE`
Minimum triangle count per connected component.


Example
-------
The following example shows how to apply the `simplify` tool to a mesh.

```console
./simplify \
  --input=levi_voxelized/levi_fr%04d_qp12_qt13.obj \
  --decimated=levi_voxelized_r0.03/levi_fr%04d_qp12_qt13.obj_decimated.obj \
  --reference=levi_voxelized_r0.03/levi_fr%04d_qp12_qt13.obj_reference.obj \
  --mapped=levi_voxelized_r0.03/levi_fr%04d_qp12_qt13.obj_mapped.obj \
  --target=0.03 --cctcount=8 --qt=12
```
