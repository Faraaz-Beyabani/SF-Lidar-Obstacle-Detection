# Lidar-Obstacle-Detection

In this project, a sequence of lidar point clouds are fed through a processing system, which highlights the road plane in green, and highlights various obstacles in alternating colors of blue, red, and yellow. Furthermore, bounding boxes are placed around these obstacles to create a 'zone of avoidance' representing areas in which the ego car (center of the point cloud) must not enter.

Completed code can be found in the src/ folder.

---

## Important Dependencies
* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
 * PCL 1.2

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run: `./environment`
