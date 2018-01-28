[image1]: ./pics/position.png

# Model Predictive Control

In this project I have implemented Model Predictive Control algorithm that drives the car around the track in [simulator](https://github.com/udacity/self-driving-car-sim/releases). There is an additional challenge: a 100 millisecond latency between actuations commands on top of the connection latency that I had to deal with.

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

### MPC preprocessing. Latency

#### Conversion to a local reference frame:

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

#### Latency. Prediciton of reference points

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

#### Latency. Initial state prediction



### Results

![alt_text][image1]
