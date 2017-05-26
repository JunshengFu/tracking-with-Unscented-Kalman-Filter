# **Object Tracking with Unscented Kalman Filter**

## Objective
Utilize sensor data from both LIDAR and RADAR measurements for object (e.g. pedestrian, vehicles, or other moving objects) 
tracking with the Unscented Kalman Filter.


## Kalman Filters variances

All Kalman filters have the same mains steps: 1. Initialization, 2. Prediction, 3. Update.
A **Standard Kalman Filter** (KF) can only handle linear equations. 
Both the **Extended Kalman Filter** (EKF) and the **Unscented Kalman Filter** 
(UKF will be disuccsed in the next project) allow you to use non-linear equations; the difference between 
EKF and UKF is how they handle non-linear equations: Extended Kalman Filter uses the Jacobian matrix to 
linearize non-linear functions; Unscented Kalman Filter, on the other hand, does not need to linearize non-linear 
functions, insteadly, the unscented Kalman filter takes representative points from a Gaussian distribution. 



## Unscented Kalman Filter VS Extended Kalman Filter

Table 1: Accuracy comparison in RMSE by using EKF and UKF with both lidar and radar measurements. The lidar and radar 
measurements are included in the txt file under the [data](data) folder. 

|            state        |  UKF          |    EKF     |
|:-----------------------:|:-------------:|:----------:|
|            px           | **0.0640299** |  0.0972256 |
|            py           | **0.0832734** |  0.0853761 |
|            vx           | **0.330315**  |  0.450855  |
|            vy           | **0.212456**  |  0.450855  |


Table 2: Accuracy comparison in RMSE by UKF with different sensor measurements. 

|           state         | lidar and radar | only lidar | only radar|
|:-----------------------:|:---------------:|:----------:|:---------:|
|            px           | **0.0640299**   |  0.168267  |  0.203744 |
|            py           | **0.0832734**   |  0.146901  |  0.250427 |
|            vx           | **0.330315**    |  0.613026  |  0.450143 |
|            vy           | **0.212456**    |  0.252216  |  0.249284 |



**Conclusions from aboves**:

* From Table 1, we can see the UKF outperform EKF in estimation of all states.
* From Table 2, we can see that ultizing both lidar and radar measurements improves the tracking results.

### Motion Model: CTRV model
![][image1] 

### Unscented Kalman Filter roadmap
![][image2] 

---


## Code & Files
### 1. Dependencies & environment

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
* [Eigen library](src/Eigen)


### 2. My project files

* [CMakeLists.txt](CMakeLists.txt) is the cmake file.

* [data](data) folder contains test lidar and radar measurements.

* [Docs](Docs) folder contains docments which describe the data.

* [src](src) folder contains the source code.


### 3. Code Style

* [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).


### 4. How to run the code

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it by the following commands: 
   * `./ExtendedKF  ../data/obj_pose-laser-radar-synthetic-input.txt ./output.txt`



### 5. Release History

* 0.1.1
    * First proper release
    * Update documentation
    * Date 25 May 2017

* 0.1.0
    * Initiate the repo
    * Date 22 May 2017



[//]: # (Image References)
[image1]: ./images/ctrv.jpg
[image2]: ./images/ukf_roadmap.jpg
