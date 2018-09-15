# Path Planning design

The goal of the program is to make the car stay in it's lane and drive at a constant speed. This is accomplished using a polynomial fit on the waypoints derived from previous points and the trajectory

## Path Set up

The waypoints are setup by calculating a trajectory from the previous points