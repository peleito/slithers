# manipulator_control

## Directories

### lie_theory

The `lie_theory` directory contains the main files and functions for solving inverse kinematics. The main functionality of the repository is located here.

### coppeliasim

The `coppeliasim` directory contains the files and scenes to run simulations with the tested robot models. The code for simulations is set up to run open loop by precomputing the states and playing them back.

### robot_models

The `coppeliasim` directory contains the robot model files (`urdf`, `dae`, `stl`, etc).

### kinematic

The `kinematics` directory contains the orginal code and inspiration for shifting to the Lie Theory based optimization.

### export_fig

The `export_fig` directory contains a copy of [export_fig](https://github.com/altmany/export_fig) for generating figures.