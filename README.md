[image1]: ./pics/position.png

# Model Predictive Control

In this project I have implemented Model Predictive Control algorithm that drives the car around the track in Udacity [car simulator](https://github.com/udacity/self-driving-car-sim/releases). There is an additional challenge: a 100 millisecond latency between actuations commands on top of the connection latency that I had to deal with.

### Compilation and building instructions

* Clone this repository
* Make a build directory: `mkdir build && cd build`
* Compile the project with `cmake .. && make`
* Run it: `./mpc`

### The model

#### Update equations:

1) kinematic update equations:

![equation](http://latex.codecogs.com/gif.latex?%5C%5C%20x_%7Bt&plus;1%7D%20%3D%20x_t%20&plus;%20v_t%20%5Ccdot%20%5Ccos%7B%5Cpsi_t%7D%20%5Ccdot%20dt%20%5C%5C%20y_%7Bt&plus;1%7D%20%3D%20y_t%20&plus;%20v_t%20%5Ccdot%20%5Csin%7B%5Cpsi_t%7D%20%5Ccdot%20dt%20%5C%5C%20%5Cpsi_%7Bt&plus;1%7D%20%3D%20%5Cpsi_t%20-%20%5Cfrac%7Bv_t%7D%7BL_f%7D%20%5Ccdot%20%5Cdelta_t%20%5Ccdot%20dt%20%5C%5C%20v_%7Bt&plus;1%7D%20%3D%20v_t%20&plus;%20a_t%20%5Ccdot%20dt)

2) cross track error update equation:

![equation](http://latex.codecogs.com/gif.latex?cte_%7Bt&plus;1%7D%20%3D%20cte_t%20&plus;%20v_t%20%5Ccdot%20%5Csin%7Be%5Cpsi_t%7D%20%5Ccdot%20dt%20%5C%5C), 

![equation](http://latex.codecogs.com/gif.latex?cte_t%20%3D%20y_t%20-%20f%28x_t%29)

3) orientation error update equation:

![equation](http://latex.codecogs.com/gif.latex?e%5Cpsi_%7Bt&plus;1%7D%20%3D%20e%5Cpsi_t%20-%20%5Cfrac%7Bv_t%7D%7BL_f%7D%20%5Ccdot%20%5Cdelta_t%20%5Ccdot%20dt%20%5C%5C)

![equation](http://latex.codecogs.com/gif.latex?e%5Cpsi_t%20%3D%20%5Cpsi_t%20-%20%5Carctan%20f%27%28x_t%29)

#### State variables vector: 

![equation](http://latex.codecogs.com/gif.latex?%5Cmathbf%7Bx%7D_t%20%3D%20%5Bx_t%2C%20%7Ey_t%2C%20%7Epsi_t%2C%20%7Ev_t%2C%20%7Ecte_t%2C%20%7Ee%5Cpsi_t%5D%5ET)

#### Actuators vector: 

![equations](http://latex.codecogs.com/gif.latex?%5Cmathbf%7Bu%7D_t%20%3D%20%5B%5Cdelta_t%2C%20%7Ea_t%5D%5ET)

### MPC tuning

`N` - finite horizon size (number of predicted timesteps). This parameter heavily depends on computational capabilities, and in my case, I limited myself with `N = 5`. Higher `N` leads to quite interesting effects on the turns: vehicle slightly overshoots but makes the turn without reducing the speed, whereas in case of small `N` it has to push the brake since it was not expecting the turn.

`dt` - time step. Again, this parameter depends on computer hardware. In case `dt` is small the controller calculates "very detailed" trajectory consisting of `N` pieces. The price for it is the need for fast computations between consecutive trajectory points. In my case, I observed increasing vehicle wiggling when dt is comparably small to the MPC algorithm execution time. So I set it to `dt = 0.5`.

The following code snippet demonstrates how I tuned cost function:

```cpp
const AD<double> v_ref =  30; // reference velocity
const int w_cte =         1000; // cross-track error weight
const int w_epsi =        1000; // orientation error weight
const int w_delta =       1; // steering actuator action weight
const int w_a =           1; // acceleration actuator action weight
const int w_vel_diff =    1; // v - v_ref weight
const int w_delta_diff =  10; // change in steering actuator action weight
const int w_a_diff =      10; // change in acceleration actuator action weight
```
Number one preferences are cross-track error and orientation error, then derivatives of actuator actions so that the trajectory of car movement stayed smooth enough. Finally, following velocity reference and reducing the amount of actuators power are the least prefferable goals of the controller. 

### MPC preprocessing. Latency

#### Conversion to a local reference frame:

By default, the car gets global coordinates that need to be converted in local reference frame. The following code snippet shows how this can be done (note that `psi` is positive in clockwise direction):

```cpp
// Convert ptsx and ptsy to local reference frame
vector<double> x_refs = {};
vector<double> y_refs = {};
double cos_psi = std::cos(psi);
double sin_psi = std::sin(psi);
for (size_t i=0; i < ptsx.size(); ++i) {
  double x_local = cos_psi*(ptsx[i] - px) + sin_psi*(ptsy[i] - py);
  double y_local = cos_psi*(ptsy[i] - py) - sin_psi*(ptsx[i] - px);
  x_refs.push_back(x_local);
  y_refs.push_back(y_local);
} 
```

#### Latency. Initial state prediction

There is an additional challenge in the system - 100 ms latency that is modelled by the following command: `this_thread::sleep_for(chrono::milliseconds(delay));`. The problem is that MPC controller sees the system 100 ms before actuators are actually applied. The solution is to predict the system state for 100ms in future and use it as an initial condition for MPC controller:

```cpp
if (delay) 
{
  x += v * std::cos(psi_local) * dt;
  y += v * std::sin(psi_local) * dt;
  std::vector<double> f_df = polyeval(coeffs, x);
  psi_local -= v/Lf * steer_value * deg2rad(25.0) * dt;
  v += throttle_value * dt;
  cte += v * std::sin(epsi) * dt;
  epsi -= v/Lf * steer_value * deg2rad(25.0) * dt;
}
state << x, y, psi_local, v, cte, epsi;
```

#### Latency. Prediciton of reference points

Additionally, when the local reference coordinates are calculated for the car, we need to predict car's position after 100 ms as well and then convert them to a local coordinate frame:

```cpp
const int delay = 100; 
const double Lf = 2.67; 
const double dt = ((delay) ? delay/1000.0 : 0.0);
double dx = v * std::cos(psi) * dt;
double dy = v * std::sin(psi) * dt;
// predict coordinates for the reference trajectory
if (delay) {
    px += dx;
    py += dy;
    psi -= v/Lf * steer_value * deg2rad(25.0) * dt;    
}
// ...
// conversion to a local reference frame
```

### Results

The following graph compares the reference position of the car and its real position controlled my MPC algorithm. 

![alt_text][image1]

-----------------------------------------------------------------------------------------------------------------

Note, that the performance of the controller strongly depends on your hardware and it may differ from the results presented above. I used ThinkPad t450s laptop (i5-5200, RAM 12GB, Intel HD 5500). 
