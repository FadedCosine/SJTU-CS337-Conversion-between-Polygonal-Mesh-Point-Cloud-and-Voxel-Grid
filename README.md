# SJTU-CS337-Conversion-between-Polygonal-Mesh-Point-Cloud-and-Voxel-Grid
For group project of Computer Graph on 3D reconstruct.

In this project, we dive into the problem of conversion between polygonal mesh, point cloud and voxel grid. Given a dataset
of 3D models formatted in .obj/.stl, we implement algorithms that transform these formats into voxel and point cloud representation, and
also algorithms transform the result backward to polygonal mesh. We implement the Greedy Projection Triangulation algrithm and
Marching Cube from scratch without any external library’s help, and final obtain a high-quality conversion between 3D models

For stl2obj, just put the stl model in stl2obj/, and change the coresponding filename in stl2obj/stl2obj.py , and run it.

For point cloud to mesh, we have tried two ways to convert in GreedyTrangulation/ and Marching Cube/

For mesh to voxel and voxel to mesh, all the code is in  Marching Cube/
