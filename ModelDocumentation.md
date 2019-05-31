# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

## Used extra-resource

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
---

## Description in detail

The main workflow of my implementation is:

1. We set initial velocity as 0 m/s, to solve cold start issue.
2. We set waypoints based on start point(current position or planned trajectory last time) and target point(ahead of ego-car), which could be used to change lane, into anchor point list. 
3. We use this anchor point list and spline library to generate smoother trajectory. (Btw, in order to make calculation more convenient: convert coordinate system of "anchor" to make ego-position as original point.)
4. Checking the distance between ego-car and the car ahead, if it is too close, set velocity down and check feasibility of changing lane.
5. If the distance in the next few seconds is too close, we set velocity slow down and try to change lane when it's possible. Otherwise we accelerate until velocity reach max-limit.
6. Based on spline curve from current position to the target-lane, we generate waypoints and convert coordinate system into global coordinate system, then output to simulator.

The main part of my implementation come from video of class. My improvement mainly focuses on check the feasibility of changing lane: 

1. Firstly we set flags: no possiblity to change lane: default as true, i.e. can't change lane in default case
2. We read data form sensor fusion, check the distance between other cars and ego-car.
3. Once the car in front of ego-car is too close: check if it is possible to change lane:
   * Check left lane: if currently ego-car is too close( < 35 meter ) to any cars on left lane ahead, or there is a car behind too close( < 15 meter), we set find-car-flag as true
   * If we checked all the cars on the left lane far away: set no-change-lane-flag to false, that means we can change lane to left side.
   * And if left-change impossible, or ego-car is on the leftmost lane, check right lane: similar like above steps.   
4. If we find it is possible to change lane: set global variable "lane" to target lane, which will work on the generating-waypoints part

Notice: here the distance threshold of changing lane (>35) must be farther then too-close's threshold (>25), otherwise, it could cause changing lane back and forth.
