# Coppeliasim Files

Open the appropriate CoppeliaSim scene file to run the simulation. To test the Husky mobile manipulator open `ur5_husky_testing_joint_commands.ttt`. To test the holonomic platform open `ur5_husky_testing_joint_commands_holonomic.ttt`

Run `simulation_main.m` to use the results (open loop simulation) from the Lie theory based optimization solver. The data for running the open loop simulation is loaded on lines 34 (`results = load("husky_ur5e_results_full.mat");`) and 37 (`results = load("husky_ur5e_holo_results_full.mat");`).

`remApi.m` and `remoteApiProto.m` are used for communicating between MATLAB and CoppeliaSim.

Functions were written for sending command velocities (`setArmMotorVelocities`) and positions (`setArmJointPositions.m`) to the simulated manipulator and mobile platform motors. 

Helper methods were written to convert desired mobile platform velocities into motor velocities for the two mobile platforms tested (`getHuskyMotorFromVel` and `getHolonomicMotorFromVel`).