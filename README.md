
# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

---
[//]: # (Image References)

[image1]: ./screenshots/FSM.png "Finite State Machine"
[image2]: ./screenshots/Overview.png "Overview"
[image3]: ./screenshots/Frenet.png "Frenet"
[image4]: ./screenshots/Prediction.png "Prediction"
[image5]: ./screenshots/Trajectory.png "Trajectory"
[image6]: ./screenshots/ModelBasedApproach1.png "Defining  process models"
[image7]: ./screenshots/ModelBasedApproach2.png "Using  process models"
[image8]: ./screenshots/ModelBasedApproach3.png "Probabilistically classifying  driver intent"
[image9]: ./screenshots/frenet-2.png "Trajectory in (x,y)"
[image10]: ./screenshots/GNB.png "Gaussian Naive Bayes"
[image11]: ./screenshots/PP1.png "Path Planner In Action"
[image12]: ./screenshots/PP2.png "Path Planner In Action"
[image13]: ./screenshots/PP3.png "Path Planner In Action"
[image14]: ./screenshots/PP4.png "Path Planner In Action"
[image15]: ./screenshots/PP5.png "Path Planner In Action"

## Introduction
In this project, our goal is to design a path planner that is able to create smooth, safe paths for the car to follow along a 3 lane highway with traffic. A successful path planner will be able to keep inside its lane, avoid hitting other cars, and pass slower moving traffic all by using localization, sensor fusion, and map data.

Using the path planner, we need to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. We will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Implementation

### Overview
The Autonomous Vehicle Driving pipeline consists of the following components:

 - **Perception** - Understanding the world.
	 - **Computer Vision** - Detection of roads, signs, lights, objects etc.
	 - **Sensor Fusion**- Detection of vehicles, obstacles, objects etc.
 - **Localization** - Mapping & finding where are we in the world.
 - **Path Planning** - Calculating trajectories from our position and our understanding of the world.
	 - **Prediction** - Predicting what other vehicles/pedestrians will do a few seconds in the future.
	 - **Behavior Planning** - Decision making (speed up/slow down/change lanes etc) based on the prediction data.
	 - **Trajectory Generation** - A trajectory that goes through a list of points that are positioned in space and time.
 - **Motion Control** - Following the calculated trajectory.

	![alt text][image2]

The goal of this project is to build a path planner that creates smooth, safe trajectories for the car to follow. The highway track has other vehicles, all going different speeds, but approximately obeying the 50 MPH speed limit.

The car transmits its location, along with its sensor fusion data, which estimates the location of all the vehicles on the same side of the road.

The path planner output a list of **x** and **y** global map coordinates. Each pair of **x** and **y** coordinates is a point, and all of the points together form a trajectory. We can use any number of points that we want, but the **x** list should be the same length as the **y** list.

Every 20 ms the car moves to the next point on the list. The car's new rotation becomes the line between the previous waypoint and the car's new location. The car moves from point to point perfectly, so we don't have to worry about building a controller for this project.

The **velocity** of the car depends on the spacing of the points. Because the car moves to a new waypoint every 20ms, the larger the spacing between points, the faster the car will travel. The speed goal is to have the car traveling at (but not above) the 50 MPH speed limit as often as possible. But there will be times when traffic gets in the way.

Here in this  **Path Planning**, we are now in charge of  **making decisions**. This subject is one of the most difficult of all and it’s about **implementing the brain of an autonomous vehicle**.

### Prediction
A prediction module uses a map (localization) and data from sensor fusion to generate predictions for what all other **dynamic** objects in view are likely to do.

There are broadly three categories of approaches to prediction:
 1. Data-Driven Approaches
 2. Model Based Approaches
 3. Hybrid Approaches
 
	![alt text][image4]

**Data-driven approaches** solve the prediction problem in two phases:
1.  **Offline training** -  In this phase the goal is to feed some machine learning algorithm a lot of data to train it. For the trajectory clustering example this involved:
	1.  **Define similarity**  - we first need a definition of similarity that agrees with human common-sense definition.
	2.  **Unsupervised clustering**  - at this step some machine learning algorithm clusters the trajectories we've observed.
	3.  **Define Prototype Trajectories**  - for each cluster identify some small number of typical "prototype" trajectories.
2.  **Online Prediction** - Once the algorithm is trained we bring it onto the road. When we encounter a situation for which the trained algorithm is appropriate (returning to an intersection for example) we can use that algorithm to actually predict the trajectory of the vehicle.
	1.  **Observe Partial Trajectory**  - As the target vehicle drives we can think of it leaving a "partial trajectory" behind it.
	2.  **Compare to Prototype Trajectories**  - We can compare this partial trajectory to the  _corresponding parts_  of the prototype trajectories. When these partial trajectories are more similar (using the same notion of similarity defined earlier) their likelihoods should increase relative to the other trajectories.
	3.  **Generate Predictions**  - For each cluster we identify the most likely prototype trajectory. We broadcast each of these trajectories along with the associated probability (see the image below).
		![](https://d17h27t6h515a5.cloudfront.net/topher/2017/June/595407d1_prediction-1/prediction-1.jpg)

**Model Based Approaches** can also be modelled to have an "offline" and online component.
3.  **_Defining_**  process models (offline).
		![alt text][image6]
4.  **_Using_**  process models to compare driver behavior to what would be expected for each model.
		![alt text][image7]
5.  **_Probabilistically classifying_**  driver intent by comparing the likelihoods of various behaviors with a multiple-model algorithm.
		 ![alt text][image8]
6.  **_Extrapolating_**  process models to generate trajectories.
		![alt text][image9]

**Hybrid Approaches** can also be used to generate predictions. It is similar to the Model Based Approach but the multiple-model algorithm is replaced by machine learning here. For example, a Gaussian Naive Bayes classifier can be used to predict the behavior of vehicles on a highway.
	![alt text][image10]

### Behavior Planning

![alt text][image1]

### Trajectory Generation
![alt text][image3]
![alt text][image5]

### Conclusion

![alt text][image15]

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Dependencies

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
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    
## Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).
