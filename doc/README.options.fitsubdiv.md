Fitsubdiv
=========

General options
---------------

### `--help`
Print a list of available command line (and configuration file) options
along with their default values and exit.

### `--config=FILE`, `-c`
Specifies a configuration file to be immediately loaded.


I/O parameters
--------------

### `--source=FILE`, `-i`
Path to the source mesh in OBJ format. The source mesh is considered as the
base mesh for the subdivision process.

### `--target=FILE`
Path to the target mesh in OBJ format. The subdivided version of the source
mesh is deformed in order to match the geometry of the target mesh.

### `--mtarget=FILE`
Path to the modifier mesh in OBJ format. The modifier mesh, usually obtained
by temporal tracking process, has the same connectivity as the target mesh.
Its geometry is different. The modifier mesh is used to compute motion that
would be applied after the source mesh is subdivided.

### `--mapped=FILE`
Path to the mapped mesh in OBJ format. The mapped mesh, usually generated
during the mesh simplification process, has the same connectivity as the
target mesh. Its geometry is different. The mapped mesh is used to is used to
compute displacements that would be applied after the source mesh is
subdivided.

### `--base=FILE`
Path to the generated base mesh in OBJ format.

### `--subdiv0=FILE`
Path to the initial subdivided mesh in OBJ format. When defined, the initial
subdivided mesh is used instead of computing the subdivision of the base mesh.

### `--subdiv=FILE`
Path to the generated displaced subdivision surface in OBJ format. The
generated mesh is obtained by applying 3D the displacements to the subdivided
base mesh.

### `--nsubdiv=FILE`
Path to the generated subdivided and deformed normal mesh in OBJ format. The
generated mesh is obtained by applying 1D displacements to the subdivided base
mesh along the normals to the surface.


Subdivision parameters
----------------------

### `--sdeform=0|1`
Apply deformation refinement stage.

### `--it=INT-VALUE`
subdivision iteration count.

### `--forceNormalDisp=0|1`
Force displacements to aligned with the surface normals.

### `--unifyVertices=0|1`
Unify duplicated vertices.

### `--deformNNCount=INT-VALUE`
Number of nearest neighbors used during the initial deformation stage.

### `--deformNormalThres=REAL-VALUE`
Maximum allowed normal deviation during the initial deformation stage.

### `--sampIt=INT-VALUE`
Number of subdivision iterations used for geometry sampling.

### `--fitIt=INT-VALUE`
Number of iterations used during the deformation refinement stage.

### `--smoothCoeff=REAL-VALUE`
Initial smoothing coefficient used when smoothing the deformed mesh during the
deformation refinement stage.

### `--smoothDecay=REAL-VALUE`
Decay factor applied to initial smoothing coefficient after every iteration of
the deformation refinement stage.

### `--smoothMissedCoeff=REAL-VALUE`
Smoothing coefficient applied to the missed vertices.

### `--smoothMissedIt=INT-VALUE`
Number of iterations when smoothing the positions of the missed vertices.

### `--smoothMethod=0|1`
Smoothing method to be applied when smoothing the deformed mesh during the
deformation refinement stage.

  | Value | Description       |
  |:-----:| ------------------|
  | 0     | None              |
  | 1     | Vertex constraint |


### `--deformUpdateNormals=0|1`
Recompute normals after each iteration of the deformation refinement stage.

### `--deformFlipThres=REAL-VALUE`
Threshold to detect triangle normals flip.

### `--useInitialGeom=0|1`
Use the initial geometry during the deformation refinement stage.

### `--fitSubdiv=0|1`
Update the positions of the base mesh to minimize displacements between the
subdivided mesh and the displaced subdivided mesh.

### `--smoothMotion=0|1`
Apply smoothing to motion instead of vertex positions.

Example
-------
The following example shows how to apply the `fitsubdiv` tool to generate
a displaced subdivision surface with 3 levels of subdivision.

```console
./fitsubdiv \
  --target=levi_voxelized_r0.03/levi_fr%04d_qp12_qt13.obj_reference.obj \
  --source=levi_voxelized_r0.03/levi_fr%04d_qp12_qt13.obj_decimated_tex.obj \
  --mapped=levi_voxelized_r0.03/levi_fr%04d_qp12_qt13.obj_mapped.obj \
  --base=levi_voxelized_r0.03_ai/levi_fr%04d_qp12_qt13.obj_base.obj \
  --subdiv=levi_voxelized_r0.03_ai/levi_fr%04d_qp12_qt13.obj_subdiv.obj \
  --nsubdiv=levi_voxelized_r0.03_ai/levi_fr%04d_qp12_qt13.obj_nsubdiv.obj \
  --sdeform=1 --it=3 --forceNormalDisp=0 --deformNormalThres=-2 \
  --deformNNCount=1 --smoothMethod=1 - --deformFlipThres=-2 \
  --fitSubdiv=1 --unifyVertices=1 --useInitialGeom=1 --smoothMotion=0
```
