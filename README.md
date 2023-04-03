# SCL-SLAM

A Scan Context-enabled LiDAR SLAM Using Factor Graph-Based Optimization.

## Prerequisites
  - CMake (compilation configuration tool)
  ```
  sudo apt-get install cmake
  ```

  - [Boost](http://www.boost.org/) (portable C++ source libraries)
  ```
  sudo apt-get install libboost-all-dev
  ```

  - [GTSAM](https://github.com/borglab/gtsam/releases) (Georgia Tech Smoothing and Mapping library)

  Install during the compilation.

  - [Distributed-Mapper](https://github.com/lajoiepy/robust_distributed_mapper/tree/d609f59658956e1b7fe06c786ed7d07776ecb426) (Georgia Tech, MIT and Army Research Lab)

  Install during the compilation.

  - Python (required system packages)
  ```
  sudo apt-get install python-wstool python-catkin-tools
  ```

## Compilation
  Set up the workspace configuration:
  ```
  mkdir -p ~/scl_slam_ws/src
  cd ~/scl_slam_ws
  catkin init
  catkin config --merge-devel
  catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
  ```

  Then use wstool for fetching catkin dependencies:
  ```
  cd src
  git clone https://github.com/thisparticle/scl_slam.git
  git clone https://github.com/thisparticle/fast-lio.git
  wstool init
  wstool merge dlc-slam/dependencies.rosinstall
  wstool update
  ```

  Build 
  ```
  catkin build dlc_fast_lio
  ```
