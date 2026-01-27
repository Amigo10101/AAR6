# IK-Geo revised
This is a fork of the IK-GEO repository by Alexander J. Elias, John T. Wen.
This repository aims to improve and partly enhance the provided C++ implementations and is still under construction.

Please see their paper for more information:
Implementation of subproblem solutions using a linear algebra approach from ["Canonical Subproblems for Robot Inverse Kinematics"](https://arxiv.org/abs/2211.05737). We also include inverse kinematics solutions to a number of 6-dof robot types, examples with specific robots, and timing tests.

For 7-DOF inverse kinematics using the Shoulder-Elbow-Wrist (SEW) angle, please see the [stereo-sew](https://github.com/rpiRobotics/stereo-sew) repo.

### Subproblem 1: Circle and point

$$\min_\theta \lVert R(k,\theta)p_1 - p_2\rVert$$

### Subproblem 2: Two circles

$$\min_{\theta_1,\theta_2} \lVert R(k_1,\theta_1)p_1 - R(k_2,\theta_2)p_2\rVert$$

### Subproblem 3: Circle and sphere

$$\min_\theta \lvert \lVert R(k,\theta)p_1-p_2\rVert-d\rvert$$

### Subproblem 4: Circle and plane

$$\min_\theta \lvert h^\top R(k,\theta)p -d \rvert$$

### Subproblem 5: Three circles

$$ p_0 + R(k_1,\theta_1)p_1=
 R(k_2,\theta_2)(p_2+ R(k_3,\theta_3)p_3)$$

### Subproblem 6: Four circles

$$\begin{cases}
    h_1^\top R(k_1, \theta_1)p_1 + h_2^\top R(k_2, \theta_2)p_2 = d_1\\
    h_3^\top R(k_3, \theta_1)p_3 + h_3^\top R(k_4, \theta_2)p_4 = d_2
\end{cases}$$

## Folder breakdown
Make sure to switch to the right branch to see the most recent progress.

`cpp`: C++ implementation (Implemented/Enhanced within this fork). 
Please see the "tests_and_demos" folder for information how to implement the C++ code into your project.
Currently working implementation contains:

* Solving Spherical-Wrist Robots with 6-DOF Revolute joints
* Implementation for 6-DOF Revolute joint robots with 3 parallel axes is under construction
* Construction of a variation of the 1D search algorithm is in planning

The following folders are unchanged from the original repository and (probably) won't be adapted in the future:

`ikfast`: Inverse kinematics comparison with IKFast

`matlab`: Reference MATLAB implementation

`python`: Python implementation (Work in progress)

`rust`: Rust implementation

## Contributing
If you have any improvements you'd like to make, or even ideas or requests for improvements, please start a GitHub issue.
