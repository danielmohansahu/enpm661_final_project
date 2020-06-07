# enpm661_final_project
Repository of code for ENPM661 Final Project (Spring 2020)


## Usage:

The core functionality is wrapped in the MATLAB interface function `align.m`. This function accepts two arguments: 
 - the path search "method" ("astar" or "rrt") 
 - an optional initial angular offset (in degrees). If unuspplied this is generated randomly.

``` matlab
% use A* to solve the given initial conditions
align("astar", [5, 10, 20])
% use RRT to solve a random initial condition
align("rrt")
```

## Dependencies:

A modern version of MATLAB (this code was run in 2019A).
