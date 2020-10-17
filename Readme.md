## Project Introduction
Your robot has been kidnapped and transported to a new location! Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.

In this project we implement a 2 dimensional particle filter in C++. The particle filter will be given a map and some initial localization information (analogous to what a GPS would provide). At each time step the filter will also get observation and control data.

## A high level view of Particle Filter used in the given  Scenario

The flowchart below should make the working of a Particle Filter a bit clearer.
<img src="images/particle_filter_flowchart.png" width="700" height="400" />

## Download the data folder and it should contain all the folders mention below use this link to download the data folder from the repository referred

https://github.com/vatsl/ParticleFilter


## Inputs to the Particle Filter
You can find the inputs to the particle filter in the `data` directory. 

### The Map

`map_data.txt` includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. Each row has three columns
1. x position
2. y position
3. landmark id

> * Map data provided by 3D Mapping Solutions GmbH.


### Control Data

`control_data.txt` contains rows of control data. Each row corresponds to the control data for the corresponding time step. The two columns represent
1. vehicle speed (in meters per second)
2. vehicle yaw rate (in radians per second)

### Observation Data

The `observation` directory includes around 2444 files. Each file is numbered according to the timestep in which that observation takes place. 

These files contain observation data for all "observable" landmarks. Here observable means the landmark is sufficiently close to the vehicle. Each row in these files corresponds to a single landmark. The two columns represent:
1. x distance to the landmark in meters (right is positive) RELATIVE TO THE VEHICLE/Robot. 
2. y distance to the landmark in meters (forward is positive) RELATIVE TO THE VEHICLE/Robot.

> **NOTE**
> The vehicle's/Robot coordinate system is NOT the map coordinate system. 
> The code will handle this transformation.

### Ground Truth Data
`gt_data.txt` contains rows of ground truth data. Each row corresponds to the ground truth data for the corresponding time step. The three columns represent
1. Global vehicle/Robot x position [m]
2. Global vehicle/Robot y position [m]
3. Global vehicle/Robot yaw [rad]


## The method used for resampling 
<img src="images/discrete_dist.png" width="700" height="400" />




# Compile code.

* mkdir  build
* cd build
* cmake -D CMAKE_BUILD_TYPE=Debug ..
* make -j `nproc` $*

# Run code

* ./particle_filter

# Expected output after running for all 2444 timesteps

* Runtime (in secs): ...some value in seconds shoild be less than 45 to pass...
* Success! Your particle filter passed!