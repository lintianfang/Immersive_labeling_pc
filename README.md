An Immersive Lableing Method for Large Point Clouds.

Setup: Our method is implemented in C++ with OpenGL and OpenVR. Our test system was equipped with an Intel Core i7-9700F with 8×3.0GHz, 16 GB RAM and an Nvidia GeForce RTX 2080 with 8GB VRAM. Current button design is for HTC Vive Pro.

Build Instructions:

Using the Integrated Build System (Windows only). Currently, the Immersive Labeling works best on Windows. A Visual Studio project for building can be generated via the dedicated build system of the CGV Framework via: https://github.com/sgumhold/cgv/tree/develop-rgbd. Please choose develop-rgbd branch.

How to use cgv framework:
1. create a new folder "develop"
2. create two subfolders in the folder "develop", one is "build", another is "projects"
3. in the subfolder "projects", git clone https://github.com/sgumhold/cgv.git
4. open the folder "develop", drag the "projects" to define_project_dir.bat
5. drag the "build" folder to define_system_variables.bat
6. open the batch file define_platform.bat to set the win32 or 64
7. check the configuration in the batch file show_system_variables.bat

8. open the folder "plugins" to find the "vr_test", open the file
9. in the folder "Immersive_labeling_pc", drag the pc_cleaning_tool.pj to ..\bin\geberate_makefiles.bat
10. waiting for generateing makefiles, and then press any key to continue, the project should be open in vs automatically.
11. please choose "Release DLL" before you build.
12. press f7 to build
13. press Ctrl+f5 to run

License: Our code is supported with MIT license.

