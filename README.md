# Extended Kalman Filter
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)


[//]: # (Image References)
[image1]: ./images/error.png
[image2]: ./images/correct.png

## Basic instructions to run the code
1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF `

## [Rubric](https://review.udacity.com/#!/rubrics/748/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.
---

### Compiling

#### 1. Code must compile without errors with cmake and make. Given that we've made CMakeLists.txt as general as possible, it's recommended that you do not change it unless you can guarantee that your changes will still compile on any platform.

My code is compiling ok, as you can see here:
`Scanning dependencies of target ExtendedKF`
`[ 20%] Building CXX object CMakeFiles/ExtendedKF.dir/src/FusionEKF.cpp.o`
`[ 40%] Building CXX object CMakeFiles/ExtendedKF.dir/src/tools.cpp.o`
`[ 60%] Building CXX object CMakeFiles/ExtendedKF.dir/src/kalman_filter.cpp.o`
`[ 80%] Linking CXX executable ExtendedKF`
`[100%] Built target ExtendedKF`


### Accuracy

#### 1. Your algorithm will be run against Dataset 1 in the simulator which is the same as "data/obj_pose-laser-radar-synthetic-input.txt" in the repository. We'll collect the positions that your algorithm outputs and compare them to ground truth data. Your px, py, vx, and vy RMSE should be less than or equal to the values [.11, .11, 0.52, 0.52].

My final accuracy for the **Dataset** 1 was **[0.0960, 0.0850, 0.3962, 0.4517]**
A good improvement was change the matrix P to `0.01, 0, 0, 0; 0, 0.01, 0, 0; 0, 0, 1, 0; 0, 0, 0, 1`

### Follows the Correct Algorithm

#### 1. While you may be creative with your implementation, there is a well-defined set of steps that must take place in order to successfully build a Kalman Filter. As such, your project should follow the algorithm as described in the preceding lesson.

I have followed the class for all implementation. At the code is possible to see where in the class I got the reference for the code, just like [this line](reference) where I show that was from lesson 6 section 13.

	
#### 2. Your algorithm should use the first measurements to initialize the state vectors and covariance matrices.

The first measurement is handled at [FusionEKF.cpp](reference) from line 55 to line 93.

#### 3. Upon receiving a measurement after the first, the algorithm should predict object position to the current timestep and then update the prediction using the new measurement.

The predict and update steps can be found in [FusionEKF.cpp](reference) from line 95 to line 132.


#### 4. Your algorithm sets up the appropriate matrices given the type of measurement and calls the correct measurement function for a given sensor type.

The code can handle both Radar and Lidar sensors, you can find in [FusionEKF.cpp](reference) from line 124 to line 132.

### Code Efficiency

#### 1. Your algorithm should avoid unnecessary calculations.

As an example of optimization of the code is the calculation of the matrix Q in  [FusionEKF.cpp](reference) from line 112 to line 115.

---

### Discussion
This is project was a good warm up for me, since I do not code in C++ in years! The Slack community helped me a lot, specially this [post](https://youtu.be/J7WK9gEUltM).
Moreover, I had some issues at the beginning because at some point my predictions got totally lost, as you can see in the image below.
![alt text][image1]
However, after some search in the Slack channel I found that I should normalize the angle after calculating phi for radar measurements. Therefore, I had to use this [code](reference), that fixed the issue.
`while(y(1) > M_PI || y(1) < -M_PI) {
    if (y(1) > M_PI) {
        y(1) -= M_PI;
    }
    else {
        y(1) += M_PI;
    }
}`

My final result is in the image below.
![alt text][image2]