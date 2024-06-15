%% Inverse kinematic solver for a mobile manipulator
clear
close all
% clc

addpath(genpath('data'))
addpath(genpath('helper'))
addpath(genpath('paths'))
addpath(genpath('robots'))
addpath(genpath(fileparts('../export_fig/')))

%% Load Robot
% Load road and set user preferences and parameters here. No need to edit
% other files or code unless for plotting and visualization.

% Load robot file here with screws and dof fully defined
% robot = 'husky';
robot = 'holo';
switch robot
    case 'husky'
        husky_ur5e;
    case 'holo'
        husky_ur5e_holo;
    otherwise
        printf('Robot model does not exist')
        quit(0)
end
dt = 0.1; % Time step for the duration <second, R^1>
time = 20; % Simulated duration of the experiment (does not match realtime) <seconds, R^1>
num_paths = 3; % 1-3 to run up to the first 3 paths <unitless, R^1>
lambda_e = 25*[1,1,1,1,1,1]'; % Weights for error in screw vector [rx,ry,rz,x,y,z]' <unitless, R^6>
lambda_j = 0.001; % Weight for jerk <unitless, R^1>
switch robot
    case 'husky'
        lambda_v = 10*[0.1,0.1,0.025,0.025,0.01,0.01,0.01,0.01]'; % Must match the number of screws and dof (n) and be in the same order <unitless, R^n>
    case 'holo'
        lambda_v = 10*[0.1,0.1,0.1,0.025,0.025,0.01,0.01,0.01,0.01]'; % Must match the number of screws and dof (n) and be in the same order <unitless, R^n>
    otherwise
        printf('Robot model does not exist')
        quit(0)
end

%% Parameters definition
parameters.screws = screws;
parameters.config_state = config_state; % zero state pose for mobile manipulator (se3 matrix, pose at q = [0])
parameters.stateMin = stateMin;
parameters.stateMax = stateMax;
parameters.dt = dt;
parameters.lambda_e = lambda_e;
parameters.lambda_j = lambda_j;
parameters.lambda_v = lambda_v; 
parameters.time = time;
parameters.steps = parameters.time/parameters.dt;
parameters.base_dof = base_dof; 
parameters.arm_dof = arm_dof;
parameters.total_dof = parameters.base_dof+parameters.arm_dof;

dof.base = parameters.base_dof;
dof.arm = parameters.arm_dof;
dof.total = parameters.total_dof;
dof2.base = parameters.base_dof;
dof2.arm = 0;
dof2.total = parameters.base_dof;


%% Initialize
% set array of pose goals (4 by 4 by k)
pose_goals_helix = generate_helix(2.5,0.0125,parameters.dt,parameters.time); %% working
pose_goals_sine = generate_sine(0.25,0.25,parameters.dt,parameters.time); %% working
pose_goals_horizontal = generate_horizontal_helix(0.5,1,parameters.dt,parameters.time); %% working
pose_goals_spiral = generate_spiral_sine(2.5,0.5,parameters.dt,parameters.time);

timer = zeros([parameters.steps,num_paths]);
x = zeros([dof.total,1]);
xdot = zeros([dof.total,1,num_paths]);
dx = zeros([dof.total,1]);
parameters.base_pose = eye(4);
tool_pose = zeros([4,4,parameters.steps,num_paths]);
error = zeros([4,4,parameters.steps,num_paths]);


%% Main
pose = eye(4);

for trial = 1:1:num_paths
    if trial == 1
        pose_goals = pose_goals_helix;
    elseif trial == 2
        pose_goals = pose_goals_sine;
    elseif trial == 3
        pose_goals = pose_goals_horizontal;
    else
        pose_goals = pose_goals_spiral;
    end

    x = zeros([dof.total,1]);
    dx = zeros([dof.total,1]);
    parameters.base_pose = eye(4);

    for plot_step = 1:parameters.steps
    
        % get next pose goal
        ee_desired = tform(pose_goals(plot_step));
    
        parameters.base_pose = parameters.base_pose*step_forward(parameters.screws,dx,dof2,parameters.dt);
        parameters.base_pose;
    
        % perform ik optimization
        tic
        dx = ik_optimization(x, xdot, ee_desired, parameters);
        xdot(:,plot_step,trial) = dx';
        timed = toc;
        timer(plot_step,trial) = timed;
        
        tool_pose(:,:,plot_step,trial) = parameters.base_pose*step_forward(parameters.screws,dx,dof,parameters.dt)*parameters.config_state;
        x(1:dof.base) = zeros([dof.base,1]);
        x(dof.base+1:end) = dx(dof.base+1:end);
    
        goal = ee_desired;
        actual = tool_pose(:,:,plot_step,trial);
        error(:,:,plot_step,trial) = goal-actual;
    
    end

end
%% Plot parameters
linewidth = 2; % 2
fontsize = 16; % 20
labelsize = 20; % 28
titlesize = 32; % 40
save = false;
time = seconds(0:parameters.dt:parameters.time-parameters.dt);

%% Save results
switch robot
    case 'husky'
        save('husky_ur5e_results_full.mat')
    case 'holo'
        save('husky_ur5e_holo_results_full.mat')
otherwise
        printf('Robot model does not exist')
        quit(0)
end

%% Plot paths
plot_paths

%% Plot error
plot_error

%% Plot base states
switch robot
    case 'husky'
        plot_base_nh
    case 'holo'
        plot_base_h
    otherwise
        printf('Robot model does not exist')
        quit(0)
end

%% Plot joint states
plot_joints

%% Table parameters
table_parameters

%% Table metrics
switch robot
    case 'husky'
        table_metrics_nh
    case 'holo'
        table_metrics_h
otherwise
        printf('Robot model does not exist')
        quit(0)
end