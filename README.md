# Sensor Fusion Self-Driving Car Course

<img src="media/ObstacleDetectionFPS.gif" width="700" height="400" />

### Welcome to the Sensor Fusion course for self-driving cars.

In this course we will be talking about sensor fusion, whch is the process of taking data from multiple sensors and combining it to give us a better understanding of the world around us. we will mostly be focusing on two sensors, lidar, and radar. By the end we will be fusing the data from these two sensors to track multiple cars on the road, estimating their positions and speed.

**Lidar** sensing gives us high resolution data by sending out thousands of laser signals. These lasers bounce off objects, returning to the sensor where we can then determine how far away objects are by timing how long it takes for the signal to return. Also we can tell a little bit about the object that was hit by measuring the intesity of the returned signal. Each laser ray is in the infrared spectrum, and is sent out at many different angles, usually in a 360 degree range. While lidar sensors gives us very high accurate models for the world around us in 3D, they are currently very expensive, upwards of $60,000 for a standard unit.

**Radar** data is typically very sparse and in a limited range, however it can directly tell us how fast an object is moving in a certain direction. This ability makes radars a very pratical sensor for doing things like cruise control where its important to know how fast the car infront of you is traveling. Radar sensors are also very affordable and common now of days in newer cars.

**Sensor Fusion** by combing lidar's high resolution imaging with radar's ability to measure velocity of objects we can get a better understanding of the sorrounding environment than we could using one of the sensors alone.

## Classroom Workspace

The workspace provided in the SFND classroom comes preinstallated with everything that you need to finish the exercises and projects. Versions used by Udacity for this ND are as follows:

* Ubuntu 16.04
* PCL - v1.7.2
* C++ v11
* gcc v5.5

**Note** The [[CMakeLists.txt](https://github.com/udacity/SFND_Lidar_Obstacle_Detection/blob/master/CMakeLists.txt)] file provided in this repo can be used locally if you have the same package versions as mentioned above. If you want to run this project locally (outside the Udacity workspace), please follow the steps under the **Local Installation** section.


## Local Installation

### Ubuntu 

1. Clone this github repo:

   ```sh
   cd ~
   git clone https://github.com/udacity/SFND_Lidar_Obstacle_Detection.git
   ```

2.  Edit [CMakeLists.txt](https://github.com/udacity/SFND_Lidar_Obstacle_Detection/blob/master/CMakeLists.txt) as follows:

   ```cmake
   cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
   
   add_definitions(-std=c++14)
   
   set(CXX_FLAGS "-Wall")
   set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")
   
   project(playback)
   
   find_package(PCL 1.11 REQUIRED)
   
   include_directories(${PCL_INCLUDE_DIRS})
   link_directories(${PCL_LIBRARY_DIRS})
   add_definitions(${PCL_DEFINITIONS})
   list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
   
   
   add_executable (environment src/environment.cpp src/render/render.cpp src/processPointClouds.cpp)
   target_link_libraries (environment ${PCL_LIBRARIES})
   ```

3. Execute the following commands in a terminal

   ```shell
   sudo apt install libpcl-dev
   cd ~/SFND_Lidar_Obstacle_Detection
   mkdir build && cd build
   cmake ..
   make
   ./environment
   ```

   This should install the latest version of PCL. You should be able to do all the classroom exercises and project with this setup.
   
**Note** The library version of PCL being distributed by the apt repository for 18.04 and 20.04 are both older than v1.11. The following links have the information regarding the versions-

[Bionic 18.04](https://www.ubuntuupdates.org/package/core/bionic/universe/updates/libpcl-dev)
[Focal 20.04](https://www.ubuntuupdates.org/package/core/focal/universe/base/libpcl-dev)

You can either build PCL from source (for v1.11) or use the older version.

### MAC

#### Install via Homebrew
1. install [homebrew](https://brew.sh/)
2. update homebrew 
	```bash
	$> brew update
	```
3. add  homebrew science [tap](https://docs.brew.sh/Taps) 
	```bash
	$> brew tap brewsci/science
	```
4. view pcl install options
	```bash
	$> brew options pcl
	```
5. install PCL 
	```bash
	$> brew install pcl
	```

6. Clone this github repo

   ```shell
   cd ~
   git clone https://github.com/udacity/SFND_Lidar_Obstacle_Detection.git
   ```

7. Edit the CMakeLists.txt file as shown in Step 2 of Ubuntu installation instructions above.

8. Execute the following commands in a terminal

   ```shell
   cd ~/SFND_Lidar_Obstacle_Detection
   mkdir build && cd build
   cmake ..
   make
   ./environment
   ```
If you get build errors related to Qt5, make sure that the path for Qt5 is correctly set in .bash_profile or .zsh_profile (Refer [#45](https://github.com/udacity/SFND_Lidar_Obstacle_Detection/issues/45))

### WINDOWS

#### Install via cvpkg

1. Follow the steps [here](https://pointclouds.org/downloads/) to install PCL.

2. Clone this github repo

   ```shell
   cd ~
   git clone https://github.com/udacity/SFND_Lidar_Obstacle_Detection.git
   ```

3. Edit the CMakeLists.txt file as shown in Step 2 of Ubuntu installation instructions above.

4. Execute the following commands in Powershell or Terminal

   ```shell
   cd ~/SFND_Lidar_Obstacle_Detection
   mkdir build && cd build
   cmake ..
   make
   ./environment
   ```

#### Build from Source

[PCL Source Github](https://github.com/PointCloudLibrary/pcl)

[PCL Mac Compilation Docs](https://pcl.readthedocs.io/projects/tutorials/en/latest/compiling_pcl_macosx.html#compiling-pcl-macosx)



### Compile and Run on ubuntu 20.04

#### 1st time set up

1.  `sudo apt install libpcl-dev` does not work, because it install libpcl 1.10 on ubuntu 20.04.
2.  I installed pcl 1.13.0. (https://pcl.readthedocs.io/projects/tutorials/en/master/compiling_pcl_posix.html)
3.  during cmake pcl, has error: `No package 'libusb-1.0' found`.
4.  use `sudo apt install libusb-1.0-0-dev` and solved the error in 3. `sudo apt-get install libusb-1.0-0` cannot solve.
5.  During make pcl, has error: "fatal error: XnOS.h: No such file or directory 47 | #include <XnOS.h>"
6.  install `sudo apt-get install libopenni-dev`  (https://programmer.group/solving-openni-problem-xnos.h-no-such-file-or-directory.html) to solve error in 5.
7.  after install pcl. continue to compile code.
8.  has the error (https://github.com/mrpt-ros-pkg/mrpt_slam/issues/28)
9.  modified CMakeLists.txt according to README.me `add_definitions(-std=c++14)` `find_package(PCL 1.11 REQUIRED)`
10. has error `error: ‘filesystem’ is not a member of ‘boost’`
11. add #include <boost/filesystem.hpp> in processPointClouds.cpp
12. success compile and run.

#### 2nd time.

##### error:

CMake Warning at /usr/local/share/pcl-1.13/PCLConfig.cmake:267 (find_package):
  By not providing "FindVTK.cmake" in CMAKE_MODULE_PATH this project has
  asked CMake to find a package configuration file provided by "VTK", but
  CMake did not find one.

  Could not find a package configuration file provided by "VTK" with any of
  the following names:

    VTKConfig.cmake
    vtk-config.cmake

##### solution:
`sudo apt-get purge libvtk7-dev libvtk7-qt-dev`
`sudo apt-get install libvtk7-dev libvtk7-qt-dev`

#### 3rd time.

##### this warning is fine if use vtk7:

-- The imported target "vtkParseOGLExt" references the file
   "/usr/bin/vtkParseOGLExt-7.1"
but this file does not exist.  Possible reasons include:
* The file was deleted, renamed, or moved to another location.
* An install or uninstall procedure did not complete successfully.
* The installation package was faulty and contained
   "/usr/lib/cmake/vtk-7.1/VTKTargets.cmake"
but not all the files it references.

-- The imported target "vtkRenderingPythonTkWidgets" references the file
   "/usr/lib/x86_64-linux-gnu/libvtkRenderingPythonTkWidgets.so"
but this file does not exist.  Possible reasons include:
* The file was deleted, renamed, or moved to another location.
* An install or uninstall procedure did not complete successfully.
* The installation package was faulty and contained
   "/usr/lib/cmake/vtk-7.1/VTKTargets.cmake"
but not all the files it references.

-- The imported target "vtk" references the file
   "/usr/bin/vtk"
but this file does not exist.  Possible reasons include:
* The file was deleted, renamed, or moved to another location.
* An install or uninstall procedure did not complete successfully.
* The installation package was faulty and contained
   "/usr/lib/cmake/vtk-7.1/VTKTargets.cmake"
but not all the files it references.

-- The imported target "pvtk" references the file
   "/usr/bin/pvtk"
but this file does not exist.  Possible reasons include:
* The file was deleted, renamed, or moved to another location.
* An install or uninstall procedure did not complete successfully.
* The installation package was faulty and contained
   "/usr/lib/cmake/vtk-7.1/VTKTargets.cmake"
but not all the files it references.

##### error:

/usr/bin/ld: CMakeFiles/environment.dir/src/environment.cpp.o: in function `initCamera(CameraAngle, std::shared_ptr<pcl::visualization::PCLVisualizer>&)':
environment.cpp:(.text+0x7b8): undefined reference to `pcl::visualization::PCLVisualizer::setBackgroundColor(double const&, double const&, double const&, int)'
/usr/bin/ld: environment.cpp:(.text+0x7cc): undefined reference to `pcl::visualization::PCLVisualizer::initCameraParameters()'
/usr/bin/ld: environment.cpp:(.text+0x84c): undefined reference to `pcl::visualization::PCLVisualizer::setCameraPosition(double, double, double, double, double, double, int)'
/usr/bin/ld: environment.cpp:(.text+0x897): undefined reference to `pcl::visualization::PCLVisualizer::setCameraPosition(double, double, double, double, double, double, int)'
/usr/bin/ld: environment.cpp:(.text+0x8de): undefined reference to `pcl::visualization::PCLVisualizer::setCameraPosition(double, double, double, double, double, double, int)'
/usr/bin/ld: environment.cpp:(.text+0x924): undefined reference to `pcl::visualization::PCLVisualizer::setCameraPosition(double, double, double, double, double, double, int)'
/usr/bin/ld: environment.cpp:(.text+0x980): undefined reference to `pcl::visualization::PCLVisualizer::addCoordinateSystem(double, std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > const&, int)'

##### solution:
reinstall the pcl like the 1st time.