# Car Path Planning Project

My solution of CarND-Path-Planning-Project assignment from Udacity Self Driving Car nanodegree course, Term 3. See project assignment starter code in https://github.com/udacity/CarND-Path-Planning-Project

---

## Dependencies

The project compilation and work have been verified under MacOS High Sierra XCode 8.3.1 

I used cmake 3.7.2 to build project files.

## Code Style

To enforce [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html), I included Google's `cpplint.py` file available in `./src/Lint` folder. This tool expects installed python2.7. To check style of my code, run the following command line (from `./src/Lint`):

```
./cpplint.py ../*.h ../*.cpp
```

## Project Structure

The project consists of **path_planning** and **unit_test_path_planning** applications.

**path_planning** application loads `./data/highway_map.csv` file and connects with the car simulator, just like in the original [repo](https://github.com/udacity/CarND-Unscented-Kalman-Filter-Project). It implements path planning AI to control the car behavior in the highway with the simulated traffic. 

For **unit_test_path_planning**, I used [Catch](https://github.com/philsquared/Catch) framework, consisting of a single file `./src/Catch/catch.hpp`.

The code consists of the following classes:
* `Map.h`/`.cpp` loads the track map from `*.csv` file and performs Frenet<->Cartesian reference frame transformations. This is a polished, unit-tested, bug-fixed and optimized version of what contained in the original `main.cpp` file. In my implementation of transformations, Frenet s coordinate is correctly wrapped around the track length, so that the car behaves correctly at the end of the first lap and the beginning of the next.

* `Obstacles.h`/`.cpp` loads and holds a list of obstacles returned by Sensor Fusion module for prediction, collision and forward requests with Trajectory. In prediction code, each obstacle linear velocity is converted into Frenet reference frame, and obstacle position is extrapolated along the track, using only "velocity.s" component. So, all obstacles are assumed to follow the constant velocity along their lane.

* `Trajectory.h`/`.cpp` traces the smoothed trajectory along the track lanes, using Frenet coordinates to simplify calculations. To avoid big jerks and accelerations, the trajectory is build on the top of the previous one; and the current speed value is linearly interpolated between the last value and the desired one during trajectory point sampling.

* `Planner.h`/`.cpp` implements a car path planning algorithm on the top of other components. Planner estimates several trajectories by sampling target speed and lane values from a number of most likely options. It considers trajectories of following the car on its own trajectory, following the car on the neighbouring trajectories, moving with the fastest speed, moving the the slowest speed (e.g. due to emergency braking), increasing or decreasing the current speed by a factor of two. The estimation cost of the trajectory considers  minimum distance to the nearest predicted obstacle (the higher the better), maximum desired speed (the smaller the better) and changing previous target speed and lane (preferring not to change if not necessary).

* `main.cpp` is an entry point for **path_planning** application

* `*Tests.cpp` implement test cases for corresponding modules, run by **unit_test_path_planning** application

Most of the classes modules corresponding `I*.h` interfaces. This simplifies module isolation during unit tests.

## Reflection on Future Enhancements
