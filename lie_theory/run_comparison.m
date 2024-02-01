%% Comparitive study for inverse kinematic solvers for a mobile manipulator
clear
close all
% clc

addpath(genpath('data'))
addpath(genpath('helper'))
addpath(genpath('paths'))
addpath(genpath('robots'))
addpath(genpath('comparison'))
addpath(genpath(fileparts('../github_repo/')))

addpath(genpath(fileparts('../../IK_solver_UR5/')))
addpath(genpath(fileparts('../../Jaco2SwivelIK/')))
addpath(genpath(fileparts('../../sos/')))

test_reduced_dof