**CarND-Controls-MPC**


Self-Driving Car Engineer Nanodegree Program

In this project the Model Predictive Control is implemented to drive the car around the track. This time however the cross track error is not given, it has be calculated to on the fly. Additionally, there's a 100 millisecond latency between actuations commands on top of the connection latency.



### Build

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This project requires the installation of:
* [uWebSocketIO](https://github.com/uWebSockets/uWebSockets), the repository includes some files to install it.
* Ipopt and CppAD: Please refer to [this document for](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) installation instructions.
* Eigen. This is already part of the repo.

Once the installation of the previously mentioned dependencies is complete, the main program can be built and run by doing the following from the project top directory.

```
mkdir build
cd build
cmake ..
make
./mpc
```


## [Rubric](https://review.udacity.com/#!/rubrics/896/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Compilation

#### Your code should compile.
##### Code must compile without errors with ```cmake``` and ```make```. Given that we've made CMakeLists.txt as general as possible, it's recommend that you do not change it unless you can guarantee that your changes will still compile on any platform.

As the file ```CMakeLists.txt``` was made as general as possible (including necessary steps for compiling in Ubuntu, which is the platform I used to work in this project) I left if unchanged. Thus, after setting up uWebSocketIO, Ipopt and CppAD, the compilation and building can be done using the standard procedure (from the project's top directory):

```
mkdir build
cd build
cmake ..
make
./mpc
```


### Implementation

#### The Model.
##### Student describes their model in detail. This includes the state, actuators and update equations.

The state is described by the following variables:
* ```x``` position, which is 0 since it's from the car's point of reference.
* ```y``` position, which is 0 since it's from the car's point of reference.
* ```psi```, which is the orientation's angle of the car
* ```v```, it's the velocity of the Car.
* ```cte```, the cross track error, which would be the difference of the car's position in regards with the one for the optimal track (only ```y``` position is considered since it's assumed than ```x``` position is ```0```).
* ```epsi```, ```psi``` error, the difference of orientation's angle in regards with the one for the optimal track.

The car's state is set in lines 131 and 132 of [main.cpp](./src/main.cpp), after fitting the trajectory's  polynomial.

Actuators are:
* Steering angle
* Throttle

Actuators are calculated by the MPC in the given state for each iteration, this can be seen in lines 133 to 135 of [main.cpp](./src/main.cpp).


Equations to update the state are the following ones:

```
x[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt
y[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt
psi[t] = psi[t-1] + v[t-1] / Lf * delta[t-1] * dt
v[t] = v[t-1] + a[t-1] * dt
cte[t] = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt
epsi[t] = psi[t] - psides[t-1] + v[t-1] * delta[t-1] / Lf * dt
```

* In addition to the state variables described before, ```delta``` is the steering angle, ```a``` is the throttle value.
* An instant of time is represented by ```t```, then ```t-1``` would be the previous time instant in the sequence.
* ```f``` is the function given by the trajectory's polynomial.
* ```psides``` is the ```atan``` (inverse of tangent) of the derivative of the trajectories polynomial (```f```) evaluated in ```x```.
* ```Lf``` is the value obtained by measuring the radius formed by running the vehicle in the simulator around in a circle with a constant steering angle and velocity on a flat terrain.

These equations are used to implement the cost function of the MPC, see lines 87 to 128 of [MPC.cpp](./src/MPC.cpp).


#### Timestep Length and Elapsed Duration (N & dt).
##### Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally the student details the previous values tried.

The final value I chose for ```N``` was 20. The size of ```N``` impacts the performance of the calculation of the solution given by the IPOPT solver. I tried with larger values like 50, but it took a lot of time than the car was able to perform adequately, I tested with smaller values like 10 and 6, however the look ahead range was short. I also try with 25 which was adjusted well to the polynomial, however as the polynomial having conflicts to reflect the real track on close curves. So at the end 20 offered a good balance in performance and look-ahead accuracy.

The final value I chose for ```dt``` was 0.05. ```dt``` is inversely proportional to ```N```, i.e. smaller values of ```dt```, would required larger values of ```N```, so the trade-off is that smaller values of ```dt``` could be useful for more precise actions in short term against long term prediction. I experimented with larger values like 1.0 and 2.0, and smaller like 0.01 and 0.005. At the end 0.05 (which is is half of the latency time) offered a good balance between precision and performance.


#### Polynomial Fitting and MPC Preprocessing.
##### A polynomial is fitted to waypoints. If the student preprocesses waypoints, the vehicle state, and/or actuators prior to the MPC procedure it is described.

The code in charge of calculating the polynomial based on the waypoints can be found on [main.cpp](./src/main.cpp) from lines 100 to 120.

To get the waypoints the car's orientation is used to do the transformation by the points given by the simulator's interface. Once the waypoints are calculated, they are used to fit the polynomial, then the polynomial is used to get ```cte``` and ```epsi``` of the current state, by evaluating it and its derivative when ```x``` and ```y``` positions are supposed to be 0 (i.e. from the car's point of view).

The state and polynomial is given to the MPC in order to calculate the value of the actuators (steering angle and throttle), which will keep the vehicle on track.

I experimented with different degrees of polynomials 5, 3 and 2. A value of 5 was very unstable, specially in close curves, because it has a lot of flexibility to adjust to the waypoints, it allows 4 curves. The degree 3 was more accurate in general terms however it also presented unstability at times. At the end I chose the one of degree 2 because it was more stable (it allows just 1 curve), and performed consistently during the track, with a little bit of unaccuracy in long distances ahead, but this was much preferable rater than the car zig-zagging in some sections.


#### Model Predictive Control with Latency.
##### The student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.

Though the IPOPT solver, MPC approximates a continuous reference trajectory by means of discrete paths between actuations, each actions is given by a ```dt``` interval. In order to get the actuation corresponding to the existing latency, the index of the actuation that matches with the latency time can be used. In my particular implementation ```dt``` is 0.05s and the latency is 0.1s (100 milliseconds), which means that the index 2 (corresponding to 0.1s) can be used.

As part of the arguments for the constructor of the MPC, I added the latency to be used, which is translated to the index to look for in the IPOPT solver's solution, this can be seen in the lines 136 to 140 of [MPC.cpp](./src/MPC.cpp). Then this index is used to get the actuators' values for the specified latency (lines 255 and 256 of [MPC.cpp](./src/MPC.cpp)).

Update: In the second version the state was calculated with the prediction for the latency time (lines 133 to 145 of [main.cpp](./src/main.cpp)), then this is passed to the IPOPT solver, with this change the latency management doesn't depend on ```dt```.

### Simulation

#### The vehicle must successfully drive a lap around the track.
##### No tire may leave the drivable portion of the track surface. The car may not pop up onto ledges or roll over any surfaces that would otherwise be considered unsafe (if humans were in the vehicle). The car can't go over the curb, but, driving on the lines before the curb is ok..

After the previously described implementation of the Model Predictive Control, the execution in my computer using the Term 2 Simulator was able to successfully drive the car within the track surface. This can be seen in the following video of two laps.

[![Two Laps Run](http://img.youtube.com/vi/d_5yOt9_-JY/0.jpg)](http://www.youtube.com/watch?v=d_5yOt9_-JY)

[![Two Laps Run with latency management update](http://img.youtube.com/vi/Sx0QxBEwaBw/0.jpg)](http://www.youtube.com/watch?v=Sx0QxBEwaBw)
