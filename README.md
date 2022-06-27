# mesh_shortest_path
This repository contains the source code for rough extrema estimation given a source human body point cloud/mesh. The following project is coded in C#/C++ and the build is facilitated using **CMake**. 

To use this repository a version of **Visual Studio (2019 and earlier), happly.h header, and reference point clouds** are required.

Requirements  | Download
------------- | -------------
Visual Studio 2019  | https://visualstudio.microsoft.com/vs/older-downloads/
happly.h  | https://github.com/nmwsharp/happly
CMake | https://cmake.org/download/
Point Clouds | N/A

Note: Before using custom meshes, the number of vertices must be declared in the **#define Vtx line** in mesh_shortest_path.cpp. To find the number of vertices of your desired mesh you can open the .PLY file in VS Code and the number can be seen at the top.

![image](https://user-images.githubusercontent.com/64251398/175909493-577d1291-5d5b-444e-94e8-aca9c4bd2b68.png)

Additionally, be sure to replace the PLY and PCL reader lines with your desired mesh/point cloud's file location.
Ex:
![image](https://user-images.githubusercontent.com/64251398/175911601-a777efb9-7501-4dee-871e-c205af577c97.png)

