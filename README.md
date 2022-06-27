# mesh_shortest_path
This repository contains the source code for rough extrema estimation given a source human body point cloud/mesh. The following projects are coded in C#/C++ with the exception of some python wrappers. 

To use this repository a version of Visual Studio (2019 and earlier), happly.h header, and reference point clouds are required.

Note: Before using custom meshes, the number of vertices must be declared in the #define Vtx line in mesh_shortest_path.cpp. To find the number of vertices of your desired mesh you can open the .PLY file in VS Code and the number can be seen at the top.

Additionally, be sure to replace the PLY and PCL reader lines with your desired mesh/point cloud's file location.
