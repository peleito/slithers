# Lie Theory

Run `main.m` to use the Lie theory based optimization solver. Results will be saved to a `.mat` file and plotted, but can be plotted and analyzed later. The results can also be used to run an open loop simulation in the `coppeliasim` directory.

The `helper` directory contains all of the core functions for computing transformations, optimizations, etc. 

The `robots` directory contains the robot models in screw coordinates and state limits.

The `tests` directory contains various testing used during development to ensure pieces of code were working as expected. A lot of it has to do with ROS communication.