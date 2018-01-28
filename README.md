# Model Predictive Control

In this project I have implemented Model Predictive Control algorithm that drives the car around the track in [simulator](https://github.com/udacity/self-driving-car-sim/releases). There is an additional challenge: a 100 millisecond latency between actuations commands on top of the connection latency that I had to deal with.

### Compilation and building instructions

* Clone this repository
* Make a build directory: `mkdir build && cd build`
* Compile the project with `cmake .. && make`
* Run it: `./mpc`

### The model

In the project I used the following model:

1) kinematic model:

![equation](http://latex.codecogs.com/gif.latex?%5C%5C%20x_%7Bt&plus;1%7D%20%3D%20x_t%20&plus;%20v_t%20%5Ccdot%20%5Ccos%7B%5Cpsi_t%7D%20%5Ccdot%20dt%20%5C%5C%20y_%7Bt&plus;1%7D%20%3D%20y_t%20&plus;%20v_t%20%5Ccdot%20%5Csin%7B%5Cpsi_t%7D%20%5Ccdot%20dt%20%5C%5C%20%5Cpsi_%7Bt&plus;1%7D%20%3D%20%5Cpsi_t%20-%20%5Cfrac%7Bv_t%7D%7BL_f%7D%20%5Ccdot%20%5Cdelta_t%20%5Ccdot%20dt%20%5C%5C%20v_%7Bt&plus;1%7D%20%3D%20v_t%20&plus;%20a_t%20%5Ccdot%20dt)

2) cross track error update equation:

![equation](http://latex.codecogs.com/gif.latex?cte_%7Bt&plus;1%7D%20%3D%20cte_t%20&plus;%20v_t%20%5Ccdot%20%5Csin%7Be%5Cpsi_t%7D%20%5Ccdot%20dt%20%5C%5C), 

![equation](http://latex.codecogs.com/gif.latex?cte_t%20%3D%20y_t%20-%20f%28x_t%29)

3) orientation error update equation:

![equation](http://latex.codecogs.com/gif.latex?e%5Cpsi_%7Bt&plus;1%7D%20%3D%20e%5Cpsi_t%20-%20%5Cfrac%7Bv_t%7D%7BL_f%7D%20%5Ccdot%20%5Cdelta_t%20%5Ccdot%20dt%20%5C%5C)

![equation](http://latex.codecogs.com/gif.latex?e%5Cpsi_t%20%3D%20%5Cpsi_t%20-%20%5Carctan%20f%27%28x_t%29)
