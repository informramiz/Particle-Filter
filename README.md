# Overview
A 2D particle filter in C++ to localize a lost vehicle, given a map of its lost location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data. 

I ran this particle filter on an input data present in `data` folder of this repo and I got following accuracy and performance.

```
Cumulative mean weighted error: x 0.111211 y 0.10463 yaw 0.00367565
Runtime (sec): 1.23799
```

## Running the Code
Once you have this repository on your machine, `cd` into the repository's root directory and run the following commands from the command line:

```
> ./clean.sh
> ./build.sh
> ./run.sh
```

> **NOTE**

> If you get any `command not found` problems, you will have to install 
> the associated dependencies (for example, 
> [cmake](https://cmake.org/install/))

If everything worked you should see something like the following output:

```
Time step: 2444
Cumulative mean weighted error: x .1 y .1 yaw .02
Runtime (sec): 38.187226
Success! Your particle filter passed!
```


Otherwise you might get

```
Time step: 100
Cumulative mean weighted error: x 39.8926 y 9.60949 yaw 0.198841
Your x error, 39.8926 is larger than the maximum allowable error, 1
```

# Directory Structure
The directory structure of this repository is as follows:

```
root
|   build.sh
|   clean.sh
|   CMakeLists.txt
|   README.md
|   run.sh
|
|___data
|   |   control_data.txt
|   |   gt_data.txt
|   |   map_data.txt
|   |
|   |___observation
|       |   observations_000001.txt
|       |   ... 
|       |   observations_002444.txt
|   
|___src
    |   helper_functions.h
    |   main.cpp
    |   map.h
    |   particle_filter.cpp
    |   particle_filter.h
```

The main file is `particle_filter.cpp` in the `src` directory. The file contains the main code of a `ParticleFilter` and some associated methods. Read through the code, the comments, and the header file `particle_filter.h` to get a sense for what this code do.

If you are interested, take a look at `src/main.cpp` as well. This file contains the code that will actually be running particle filter and calling the associated methods.

## Using this Code in Your Own Project

There is a detailed coding example in main.cpp which you can follow to see how to use this particle in your own project and below is a very high level view of which functions are needed to be called and in which order. 

```

//Set up parameters here
double delta_t = 0.1; // Time elapsed between measurements [sec]
double sensor_range = 50; // Sensor range [m]

double sigma_pos [3] = {0.3, 0.3, 0.01}; // GPS measurement uncertainty [x [m], y [m], theta [rad]]
double sigma_landmark [2] = {0.3, 0.3}; // Landmark measurement uncertainty [x [m], y [m]]

ParticleFilter pf;

// Initialize particle filter if this is the first time step.
if (!pf.IsInitialized()) {
    pf.Init(gps_x, gps_y, gps_yaw, sigma_pos);
}
else {
    // Predict the vehicle's next state (noiseless).
    pf.Prediction(delta_t, sigma_pos, velocity, yawrate);
}

// Update the weights and resample
pf.UpdateWeights(sensor_range, sigma_landmark, noisy_observations, map);
pf.Resample();

```


## Inputs to the Particle Filter

You can find the inputs to the particle filter in the `data` directory. 

#### The Map*
`map_data.txt` includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. Each row has three columns
1. x position
2. y position
3. landmark id

> * Map data provided by 3D Mapping Solutions GmbH.


#### Control Data
`control_data.txt` contains rows of control data. Each row corresponds to the control data for the corresponding time step. The two columns represent
1. vehicle speed (in meters per second)
2. vehicle yaw rate (in radians per second)

#### Observation Data
The `observation` directory includes around 2000 files. Each file is numbered according to the timestep in which that observation takes place. 

These files contain observation data for all "observable" landmarks. Here observable means the landmark is sufficiently close to the vehicle. Each row in these files corresponds to a single landmark. The two columns represent:
1. x distance to the landmark in meters (right is positive) RELATIVE TO THE VEHICLE. 
2. y distance to the landmark in meters (forward is positive) RELATIVE TO THE VEHICLE.


The two things test for success of this filter are:

1. **Accuracy**: Particle filter should localize vehicle position and yaw to within the values specified in the parameters `max_translation_error` (maximum allowed error in x or y) and `max_yaw_error` in `src/main.cpp`.
2. **Performance**: Particle filter should complete execution within the time specified by `max_runtime` in `src/main.cpp`.



