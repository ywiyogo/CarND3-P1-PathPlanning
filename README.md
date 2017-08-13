# Path Planning Project
**Self-Driving Car Engineer Nanodegree Program**

## Introduction

The goal of this project is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. A simulator is provided for the virtual environment. The car's localization and sensor fusion data are provided by the simulator, and there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

## Simulation Model

Here is the data provided from the Simulator to the C++ Program. The main car's localization Data (No Noise) is defined as following:

* `["x"]` The car's x position in map coordinates
* `["y"]` The car's y position in map coordinates
* `["s"]` The car's s position in frenet coordinates
* `["d"]` The car's d position in frenet coordinates
* `["yaw"]` The car's yaw angle in the map
* `["speed"]` The car's speed in MPH

the pevious path data given to the planner:

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

* `["previous_path_x"]` The previous list of x points previously given to the simulator
* `["previous_path_y"]` The previous list of y points previously given to the simulator

and the previous path's end s and d values:

* `["end_path_s"]` The previous list's last point's frenet s value
* `["end_path_d"]` The previous list's last point's frenet d value

For the sensor fusion data, a list of all other car's attributes on the same side of the road. (No Noise):

`["sensor_fusion"]` A 2D-vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

#### Details and Constraints

There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. 

The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

The map data of the highway can be found in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

The Simulator can be downloaded from this link: [releases tab](https://github.com/udacity/self-driving-car-sim/releases).


## Reflection

This section describes my reflections how I finish this project. Due to its complexity I divided this section to three subsections: the software architecture, the technical approach for the motion control and the behavior planning.

### Software Architecture

The codes are divided in several cpp and h files which repesents the object-oriented programming approach. Each h or cpp filename represents the class name. For this project, I created these classes:

* Vehicle: represents the base class for all vehicle in the simulator, inclusive the self-driving car (ego-vehicle)
* SDVehicle: represents the class of the self-driving car
* BehaviorFSM: represents the finite state machine of the behavior planning of the self-driving car
* Prediction: a class that provides prediction functionalities

Moreover, I shifted several helper functions in to a additional namespace, called Helper.

I apply the [state design pattern](https://en.wikipedia.org/wiki/State_pattern) for the behavior planner. The state design pattern can represent the finite state machine of the behavior planner. This design decreases the number of the line of codes and the use of `switch-case` or `if-else` statements. A simple illustration can be seen below:


![alt text][image8]

The below figure shows the class diagram of the BehaviorFSM class.
![alt text][image5]

### Motion Control

In my opinion, the main issue of this project is to generate paths using a collection of (x,y) waypoints that represents the speeds and the acceleration of the self-driving car.

To create a smooth trajectory, the instructor has introduced a library to create a spline: [http://kluge.in-chemnitz.de/opensource/spline/](http://kluge.in-chemnitz.de/opensource/spline/), the spline function is in a single hearder file is really easy to use.

I've done several experiments to apply the jerk minimizing trajectory (JMT). However, since I cannot control the trajectory between a start and goal points, I avoid to use JMT. The below figure shows different trajectories by varying the variable `T`
![alt text][image1]

As shown above, the acceleration and the velocity can oscilate very high if the algorithm apply a wrong `T` value.

Here are my strategies to deal with the motion issue and the acceleration constraints:

1. incorporate the previous path (`previous_path_x` and `previous_path_y`): if the previous path points is less than a particular limit, then the algorithm compute the behavior planning using the current measurement update.
2. smoothing the `getXY()` function: the return value of this function is not smooth enough because the provided map waypoints are not dense enough. This sparse map waypoints causes acceleration peaks.
3. Using spline instead of JMT based on the walkthrough video
4. different simulation delay from 0.02 until  s causes higher jerk and acc
![alt text][image2]

The self-driving car can drive more then one lap as shown in this figure:
![alt text][image3]

#### Behavior Planning

For the behavior planning, I implement 6 states in my FSM, as stated in the behavior planning lesson. The illustration of this FSM and its transitions is showed bellow:
![alt text][image4]

The transition between the states is trigger by a cost function. The cost function decides in which direction the self-driving car is going to drive next. The total cost function is defined as a sum of lane change cost, distance cost and time-to-collision cost with its weight:

    totalcost = time_to_collision_cost * COLL_W + distance_cost * DIST_W + lane_cost * LANE_W;

Based on the detected cars and the available lanes in the simulator, the algorithm separates the cars based on its lane position, and it calculates the cost function of each lanes. The lane cost is defined linearly based on the lane difference between the current lane of the self-driving car and the lane.

In order to calculate the distance cost, first the algorithm converts the Frenet to Cartesian coordinates. Then it calculates the Euclidean distance from the both (x,y) points. Finally the distance cost is defined as:

    distance_cost = 10*exp(-dist/20);

The below figure shows the graph of the distance cost.
![alt text][image6]

The time-to-collision cost uses this formula:

    time_to_collision_cost = 10 * exp(-coll_t/0.4);

 The time when the self-driving car and another car collides is calculated by dividing the distance by the velocity of the self-driving car. For this cost function the algorithm concerns only the behind cars.

![alt text][image7]

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.



## Tips

A really helpful resource for doing this project and 

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

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).


[//]: # (Image References)
[image1]: ./img/jmt_comparison.png
[image2]: ./img/sim_update_time.png
[image3]: ./img/track_record.png
[image4]: ./img/behavior_fsm.jpg
[image5]: ./img/classBehaviorFSM.png
[image6]: ./img/distance_cost_func.png
[image7]: ./img/timecollision_cost_func.png
[image8]: ./img/state_design_pattern.png



