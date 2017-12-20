# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

---
## Intro
In this project, The goal is to implement Model Predictive Control to drive around the track as fast as possible and safely. Model Predictive Control (MPC), described in wikipedia, 
is an advanced method of process control and it rely on dynamic models of the process (vehicle in this project). For this project, MPC reframes the task of following a trajectory 
as an optimization problem, and it involves simulating different actuator inputs, then predicting the result of the trajectory and selecting certain trajectory with a minimum cost.


## The Model
The MPC controller use vehicle dynamic model and actuator inputs to build state equation for trajectory optimization.

The state of dynamic model includes: position (x, and y), orientation (psi), velocity (v), acceleration (a), steering angle of vehicle (delta), cross-track error (cte) and angle 
difference (error) between vehicle orientation and track direction (epsi). In this project, we could use actuators (throttle and steering) to control acceleration and steering angle, 
then update vehicle state and follow (optimize) trajectory. Below is the state equations:

```
x = x + v * cos(psi) * dt
y = y + v * sin(psi) * dt
psi = psi + (V/Lf) * delta * dt
v = v + a * dt
```

Where, 
Lf is the length from the front wheel to the center of gravity (CoG) of the vehicle in the simulation.
dt is the assumption time step for state prediction.

## Polynomial Fitting and MPC Preprocessing.
Because the sensor input for waypoints are in the world (map) coordination system (and 90 degree difference at orientation), 
we need to transform position & orientation from world to vehicle coordination system. Here is the codes for preprocessing:

``` cpp
for(size_t e=0; e < ptsx.size(); ++e){
  double x_car = ptsx[e] - px;
  double y_car = ptsy[e] - py;

  xvals[e] = x_car * cos(0-psi) - y_car * sin(0-psi);
  yvals[e] = x_car * sin(0-psi) + y_car * cos(0-psi);
}
```

After preprocessing, we could use transformed waypoints to do polynomial fitting. Here, in order to fit the track, we used 3rd order to fit
possible double curve waypoints.

## Model Predictive Control with Latency.
In this simulation (also real world situation) there is a latency between command sending and execution, and the output command may not optimize
the current situation. To deal with latency, we need to update the vehicle state model depend on the possible latency (100 ms for the simulation).
Here is the codes for updating state model.

``` cpp
// In vehicle coordination, positon and orientation are always zeros
// so px, py, and psi = 0;
double latency = 0.1;
double est_px = v * latency;
double est_py = 0;
double est_psi = (v / Lf) * delta * latency;
double est_v = v + acc * latency;
double est_cte = cte + v * sin(epsi) * latency;
double est_epsi = epsi + v * (delta / Lf) * latency;
```

## Timestep Length and Elapsed Duration (N & dt)
The product of timestep length (N) and elapsed duration (dt) is called Prediction horizon (T), which is the duration over the future predictions are 
made. N is the number of step in the prediction horizon and dt is how much time elapse between each actuation inputs. There is tradeoffs to choose N 
and dt. From the lesson, T should be a few seconds for predicting the process (tracking trajectory), because the trajectory for optimizing will change 
faster when vehicle driving in higher speed. In this project, I set T about 1.0 to 1.5 seconds. In order to control actuator to react fast changing
trajectory in high speed (over 70 mph), I set dt to 0.05 second. To predict 1 second duration with 0.05 second step, I set N to 20.

For tuning the parameters in cost function, I first testing with low speed (~ 40 mph) and tune one parameter each time. If I only tune the weighting 
of `cte`, the vehicle will try to follow the track but driving oscillating to follow the track. Then, I need to tune the weight of `epsi`. In order 
to keep vehicle to drive more stable, I will tune the weight of `delta (steer angle)` to turn the vehicle more smoothly.


Below is the original README form the Udacity repo.

---
## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
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
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
