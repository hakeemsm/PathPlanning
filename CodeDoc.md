# Path Planning design

The goal of the program is to make the car drive along a set of waypoints and drive at the given speed. The car can change lanes if a car in front is moving very slowly and if it's safe to do a lane change. The forward way points for the path planner are calculated using a polynomial fit on the waypoints derived from previous points.

## Path Set up

The waypoints are setup by calculating a trajectory from the previous points. The set of previous points will be empty initially so the ref points will be set to the car's starting points.

The model starts off by setting a path that's tangential to the car's path. Ref. Lines 258-271 in main.cpp
If the vehicle is already in motion, the points for the tangential path are calculated from the previous points using arctan. All these points are added to the set of previous points

After the previous points are set, the forward way points are calculated as outlined below

* Three waypoints are created, spaced 30m apart. They are set in a way to factor in the longitudinal (lane) distance d using the ego car's current lane
* These waypoints get added to the list of existing way points
* The points are iterated over and a shift & rotate is done for each point so that the angle is always zero degree from the car's reference
* A spline is set over the list of forward waypoints. Spline helps with smoothing out the curve
* Previous points are added to the list to be sent to the simulator
* A spline is used for a set of to points 30m apart and the number of points along the curve are calculated 
* A loop is set to iterate over the diff. of 50 and the previous way point size. Inside the body of the loop, points are computed along the spine, transformed from local (b/w the two pts for the spine) to global (along the entire set of waypoints) and added to the list of forward way points

All the code for setting the waypoints for the planner could be found between lines 248-370 in main.cpp


## Lane changes

The ego car will try to change lanes when a car in front is going very slow and there are no other cars either in the right or left lanes within a safe distance. The logic for changing lanes is as outlined below

* Get the list of all nearby cars from sensor fusion and iterate thru the list
* For each car, extract it's lane, speed and determine projected position based on its speed and point spacing
* If a car is in the same lane and less than 30m apart, check left and right lanes
* If either left or right lanes have no cars within a 30m distance then change to either lane
* If car in front is too slow and there are cars within an unsafe distance in the other two lanes, stay in lane and slow down to increase distance with car in front
* If car in front is at a safe distance, try to get to max allowed speed
* If ego car is not in center lane, try to change to center lane when it is safe to do so using the lane variable

Code for lane keeping is between lines 173-241 in main.cpp

