
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
$ ./clear.sh all  # Remove all cloned dependencies.
```

## Build manually

Standard CMake build commands can be used to build the 
software depending on the system you used.

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

The V-DMC test model software uses several dependencies that are cloned and patched by the CMake building process. 

These dependencies are: 

| **URL** | **Commit/tag** | 
|---|---| 
| [Directx-headers](https:/github.com/microsoft/DirectX-Headers.git) | `1b79ddaeabc4b16c772ca63adc5bdf7d5f741460` |
| [Directx-math](https:/github.com/microsoft/DirectXMath.git) | `b404898c9dcaff7b686bbaf6d2fba8ff0184a17e` |
| [Directx-mesh](https:/github.com/microsoft/DirectXMesh.git) | `2c0ed18e271afa99a70948f784dfe082127fa0de` |
| [UVAatlas](https:/github.com/microsoft/UVAtlas.git) | `5af1b5d2a0fd9e0e5d17aa0971ab17c890e318e0` |
| [Draco](https:/github.com/google/draco.git) | `266f47ce58b0568ff9328e12174b25cb0fbd3b2e` |
| [HDRTools](http:/gitlab.com/standards/HDRTools.git) | `v0.23` |
| [HM](https:/vcgit.hhi.fraunhofer.de/jvet/HM.git) | `HM-16.21+SCM-8.8` |
| [VTM](https:/vcgit.hhi.fraunhofer.de/jvet/VVCSoftware_VTM.git) | `VTM-13.0` |
| [mpeg-pcc-mmetric](http://mpegx.int-evry.fr/software/MPEG/PCC/mpeg-pcc-mmetric.git) | `1_0_1_lib_beta_v5` |
 