
<!--- encode input paramters --->

<!--- 
  Note: 

        This file has been created automatically by 
        ./update_input_parameters.sh script. 
        Please re-run this script whereas edit this file

--->

# encode input paramters
 
| **--key=value** | **Usage** | 
|---|---|
| **Common** |  | 
| --help=0 | This help text | 
| -c, --config=... | --config=... Configuration file name | 
| -v, --verbose=1 | --verbose=1 Verbose output | 
| **Input** |  | 
| --imesh="" | Input mesh | 
| --itex="" | Input texture | 
| **Output** |  | 
| --compressed="" | Compressed bitstream | 
| --recmat="" | Reconstructed materials | 
| --recmesh="" | Reconstructed mesh | 
| --rectex="" | Reconstructed texture | 
| **General** |  | 
| --fstart=0 | First frame number | 
| --fcount=1 | Number of frames | 
| --framerate=30 | Frame rate | 
| --keep=0 | Keep intermediate files | 
| --intermediateFilesPathPrefix="" |  Intermediate files path prefix | 
| **Gof analysis** |  | 
| --gofmax=32 | Maximum group of frames size | 
| --analyzeGof=0 | Analyze group of frames | 
| **Geometry decimate** |  | 
| --target=0.125 | Target triangle count ratio | 
| --qt=0 | texture coordinate quantization bits | 
| --cctcount=0 | minimum triangle count per connected component | 
| --minPosition="0,0,0" | Min position | 
| --maxPosition="0,0,0" | Max position | 
| **Texture parametrization** |  | 
| --quality=DEFAULT | Quality level of DEFAULT, FAST or QUALITY | 
| --maxCharts=0 | Maximum number of charts to generate | 
| --stretch=0.16667 | Maximum amount of stretch 0 to 1 | 
| --gutter=2 | Gutter width betwen charts in texels | 
| --width=512 | texture width | 
| --height=512 | texture height | 
| **Geometry parametrization** |  | 
| --baseIsSrc=0 | Base models are src models | 
| --subdivIsBase=0 | Subdiv models are src models | 
| --subdivInter=0 | Subdiv inter | 
| --subdivInterWithMapping=0 | Subdiv inter with mapping | 
| --maxAllowedD2PSNRLoss=1 | Maximum allowed D2 PSNR Loss | 
| **Intra geometry parametrization** |  | 
| --ai_sdeform=1 | Apply deformation refinement stage | 
| --ai_subdivIt=3 | Subdivision iteration count | 
| --ai_forceNormalDisp=0 | Force displacements to aligned with the surface normals | 
| --ai_unifyVertices=1 | Unify duplicated vertices | 
| --ai_deformNNCount=1 | Number of nearest neighbours used during the initial deformation stage | 
| --ai_deformNormalThres=0.1 | Maximum allowed normal deviation during the initial deformation stage | 
| --ai_sampIt=3 | Number of subdivision iterations used for geometry sampling | 
| --ai_fitIt=16 | Number of iterations used during the deformation refinement stage | 
| --ai_smoothCoeff=0.25 | Initial smoothing coefficient used to smooth the deformed mesh during deformation refinement | 
| --ai_smoothDecay=0.75 | Decay factor applied to intial smoothing coefficient after every iteration of deformation refinement | 
| --ai_smoothMissedCoeff=0.1 | Smoothing coefficient applied to the missed vertices | 
| --ai_smoothMissedIt=10 | Number of iterations when smoothing the positions of the missed vertices | 
| --ai_smoothMethod=1 | Smoothing method to be applied when smoothing the deformed mesh duringthe deformation refinement stage | 
| --ai_deformUpdateNormals=1 | Recompute normals after each iteration of deformation refinement | 
| --ai_deformFlipThres=-0.5 | Threshold to detect triangle normals flip | 
| --ai_useInitialGeom=1 | Use the initial geometry during the the deformation refinement stage | 
| --ai_fitSubdiv=1 | Update the positions of the decimated mesh to minimize displacements between the subdivided mesh and the deformed mesh | 
| --ai_smoothMotion=1 | Apply smoothing to motion instead of vertex positions | 
| **Inter geometry parametrization** |  | 
| --ld_sdeform=1 | Apply deformation refinement stage | 
| --ld_subdivIt=3 | Subdivision iteration count | 
| --ld_forceNormalDisp=0 | Force displacements to aligned with the surface normals | 
| --ld_unifyVertices=1 | Unify duplicated vertices | 
| --ld_deformNNCount=1 | Number of nearest neighbours used during the initial deformation stage | 
| --ld_deformNormalThres=0.1 | Maximum allowed normal deviation during the initial deformation stage | 
| --ld_sampIt=3 | Number of subdivision iterations used for geometry sampling | 
| --ld_fitIt=16 | Number of iterations used during the deformation refinement stage | 
| --ld_smoothCoeff=0.25 | Initial smoothing coefficient used to smooth the deformed mesh during deformation refinement | 
| --ld_smoothDecay=0.75 | Decay factor applied to intial smoothing coefficient after every iteration of deformation refinement | 
| --ld_smoothMissedCoeff=0.1 | Smoothing coefficient applied to the missed vertices | 
| --ld_smoothMissedIt=10 | Number of iterations when smoothing the positions of the missed vertices | 
| --ld_smoothMethod=1 | Smoothing method to be applied when smoothing the deformed mesh duringthe deformation refinement stage | 
| --ld_deformUpdateNormals=1 | Recompute normals after each iteration of deformation refinement | 
| --ld_deformFlipThres=-0.5 | Threshold to detect triangle normals flip | 
| --ld_useInitialGeom=1 | Use the initial geometry during the the deformation refinement stage | 
| --ld_fitSubdiv=1 | Update the positions of the decimated mesh to minimize displacements between the subdivided mesh and the deformed mesh | 
| --ld_smoothMotion=1 | Apply smoothing to motion instead of vertex positions | 
| **Mesh** |  | 
| --gqp=10 | Quantization bits for base mesh positions | 
| --tqp=8 | Quantization bits for base mesh texture coordinates | 
| --gdepth=12 | Input positions bit depth | 
| --tdepth=12 | Input texture coordinates bit depth | 
| --invorient=0 | Invert triangles orientation | 
| --unifvertices=0 | Unify duplicated vertices | 
| --normuv=0 | Normalize uv texture coordinates | 
| **Geometry video** |  | 
| --gvencconfig="" | Geometry video cfg | 
| **Texture video** |  | 
| --tvencconfig="" | Texture video cfg | 
| --cscencconfig="" | HDRTools encode cfg | 
| --cscdecconfig="" | HDRTools decode cfg | 
| --tvqp=8 | Quantization parameter for texture video | 
| **Displacements** |  | 
| --encdisp=1 | Encode displacements video | 
| --enctex=1 | Encode texture video | 
| **Lifting** |  | 
| --liftingIt=2 | Lifting subdivision iteration count | 
| --dqp="16,28,28" | Quantization parameter for displacements | 
| --dqb="0.333333,0.333333,0.333333" |  Quantization bias for displacements | 
| **Texture transfer** |  | 
| --texwidth=2048 | Output texture width | 
| --texheight=2048 | Output texture height | 
| **Bug fix** |  | 
| --forceCoordTruncation=1 | Force truncation of the precision of the intermediate mesh files | 
