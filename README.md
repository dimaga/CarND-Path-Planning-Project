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

The code consists of the following modules:
* `ukf.h`/`.cpp` implements Unscented Kalman Filter mathematics and CTRV car model with Radar and Lidar sensors
* `tools.h`/`.cpp` implements EvaluateRmse() function to calculate RMSE against data set
* `ground_truth_package.h`, `measurement_package.h` declares structures for ground truth and measurement data
* `main.cpp` is an entry point for **UnscentedKF** application
* `*_test.cpp` implement test cases for corresponding modules, run by **UnscentedKFTests** application

## Reflection on Future Enhancements

