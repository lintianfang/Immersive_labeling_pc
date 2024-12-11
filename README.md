An Immersive Lableing Method for Large Point Clouds.（Currently we publish a test version, we will refine this repo in next weeks, thank you very much）

一种大型点云的沉浸式标注方法。

大規模点群に対する没入型注釈方法。

<img width="1424" alt="Graphical_abstract" src="https://github.com/user-attachments/assets/e60ef5e1-9d7f-4328-a0c6-1289a146684a">


Our paper is published in the journal of Computers & Graphics. The paper can be accessed via the [link](https://www.sciencedirect.com/science/article/pii/S009784932400236X).

Please cite using:

    @article{lin2024immersive,
      title={An immersive labeling method for large point clouds},
      author={Lin, Tianfang and Yu, Zhongyuan and McGinity, Matthew and Gumhold, Stefan},
      journal={Computers \& Graphics},
      pages={104101},
      year={2024},
      publisher={Elsevier}
    }

Setup: Our method is implemented in C++ with OpenGL and OpenVR. Current button design is for HTC Vive Pro.

**Build Instructions:**

Using the Integrated Build System (Windows only). Currently, the Immersive Labeling works best on Windows. A Visual Studio project for building can be generated via the dedicated build system of the CGV Framework via: https://github.com/sgumhold/cgv/tree/develop-rgbd. Please choose develop-rgbd branch.

Recommended IDE: Visual Studio 2019 or 2022.

**How to use cgv framework:**
1. create a new folder "develop"
2. create two subfolders in the folder "develop", one is "build", another is "projects"
3. in the subfolder "projects", git clone https://github.com/sgumhold/cgv.git, please choose develop-rgbd branch.
4. open the folder "develop", drag the "projects" to define_project_dir.bat
5. drag the "build" folder to define_system_variables.bat
6. open the batch file define_platform.bat to set the win32 or 64
7. check the configuration in the batch file show_system_variables.bat, select version of Visual Studio

8. git clone "Immersive_labeling_pc" from current repo: https://github.com/lintianfang/Immersive_labeling_pc
9. in the folder "Immersive_labeling_pc", drag the pc_labeling_tool.pj to ..\bin\geberate_makefiles.bat
10. waiting for generateing makefiles, and then press any key to continue, the project should be open in vs automatically.
11. please choose "Release DLL" or "Debug DLL" before you build.
12. press f7 to build
13. press Ctrl+f5 to run

**License: Our code is supported with MIT license.**

