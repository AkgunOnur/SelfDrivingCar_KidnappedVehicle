# Particle Filter

[//]: # (Image References)

[image1]: ./examples/particle_filter.JPG "Results"

## Overview

In this scenario, the robot has been kidnapped and transported to a new location! Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.
In the project, 2 dimensional particle filter is implemented in C++. A map and some initial localization information (analogous to what a GPS would provide) are given. At each time step the filter gets observation and control data and according to the sensor measurements, it determines which particle shows more likely the current place of the car

## Prerequisites

The project has the following dependencies :

* cmake >= 3.5
* make >= 4.1
* gcc/g++ >= 5.4
* Udacity's simulator (can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases) )

In order to install necessary libraries on Windows, [install-ubuntu.sh](./install-ubuntu.sh) needs to be executed.

## Implementing the Particle Filter

The directory structure of this repository is as follows:

```
root
|   build.sh
|   clean.sh
|   CMakeLists.txt
|   README.md
|   run.sh
|
|___data
|   |   
|   |   map_data.txt
|   
|   
|___src
    |   helper_functions.h
    |   main.cpp
    |   map.h
    |   particle_filter.cpp
    |   particle_filter.h
```

## Compilation and Execution the Project

* Clone the repo and cd to it on a Terminal.
* Create a build file and cd to it `mkdir build && cd build`
* Compile it `cmake .. && make`
* Execute it `./particle_filter`

## Results

![alt text][image1]
