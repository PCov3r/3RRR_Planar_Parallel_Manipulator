# 3RRR Planar Parallel Manipulator 

This repository contains all the Matlab codes for the study of the 3-RRR planar parallel manipulator. 

<img src="https://github.com/PCov3r/3RRR_Planar_Parallel_Manipulator/blob/main/images/3RRR.png" width=50% height=50%>

<br>

## Repository Organization

The repository is organised as follows: 
* [Kinematics](./Kinematics) contains the files relative to the kinematics study:
    * [inverse](./Kinematics/ikm.m) kinematics
    * [direct](./Kinematics/dkm.m) kinematics
    * [workspace approximation](./Kinematics/approximated_workspace.m)
    * [workspace analysis](./Kinematics/get_workspace.m)

* [Singularities](./Singularities) contains the files relative to the singularity analysis:
    * [Jacobian determinant](./Singularities/det_jacobian.m) calculation
    * [Jacobian condition number](./Singularities/jacobian_cond.m) calculation
    * [singularity loci](./Singularities/singularity_loci.m) analysis
 
* [Compliant Robot](./Compliant_Robot) contains the files relative to the compliant version of the 3-RRR robot:
  * compliant workspace from [inverse kinematics-based method](./Compliant_Robot/get_compliant_workspace_ikm.m)
  * compliant workspace from [geometry-based method](./Compliant_Robot/get_compliant_workspace.m)
  * [workspace optimization](./Compliant_Robot/workspace_optimization.m) based on singularity avoidance
  * [kinetostatics](./Compliant_Robot/kinetostatics.m) analysis
