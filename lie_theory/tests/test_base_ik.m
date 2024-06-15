%% Inverse kinematic solver for a mobile manipulator
clear
close all
% clc

%% Screw joints
Lbx = 0.34;
% Lbx = 0;
Lby = 0.0;
Lbz = 0.384;
% Lbz = 0;
husky_base_to_ur_base = [Lbx,Lby,Lbz];
husky_ur = eye(4);
husky_ur(1:3,4) = husky_base_to_ur_base;
 
S1_w = [0,0,1]; % base angular
S1_v = -cross(S1_w,[0,0,0]);
S1 = [S1_w,S1_v];

S2_w = [0,0,0]; % base linear
S2_v = [1,0,0];
S2 = [S2_w,S2_v];

S3_w = [0,0,1]; % shoulder pan
S3_v = -cross(S3_w,[0,0,0.163]+husky_base_to_ur_base);
S3 = [S3_w,S3_v];

S4_w = [0,1,0]; % shoulder lift
S4_v = -cross(S4_w,[0,0,0.163]+husky_base_to_ur_base);
S4 = [S4_w,S4_v];

S5_w = [0,1,0]; % elbow
S5_v = -cross(S5_w,[0.425,0,0.162]+husky_base_to_ur_base);
S5 = [S5_w,S5_v];

S6_w = [0,1,0]; % wrist 1
S6_v = -cross(S6_w,[0.817,0.133,0.162]+husky_base_to_ur_base);
S6 = [S6_w,S6_v];

S7_w = [0,0,-1]; % wrist 2
S7_v = -cross(S7_w,[0.817,0.133,0.063]+husky_base_to_ur_base); 
S7 = [S7_w,S7_v];

S8_w = [0,1,0]; % wrist 3
S8_v = -cross(S8_w,[0.817,0.233,0.063]+husky_base_to_ur_base);
S8 = [S8_w,S8_v];

parameters.screws = [S1; % base angular velocity
                     S2; % base linear velocity
                     S3; % shoulder pan
                     S4; % shoulder lift
                     S5; % elbow
                     S6]; % wrist 1

%% Parameters definition
parameters.config_state = [-1,0,0,0.817+Lbx;
                           0,0,1,0.233+Lby;
                           0,1,0,0.063+Lbz;
                           0,0,0,1]; % zero state pose for mobile manipulator (se3 matrix, pose at q = [0])
parameters.stateMin = [-2*pi,-5,-2*pi,-2*pi,-2*pi,-2*pi];%,-2*pi,-2*pi]';
parameters.stateMax = [2*pi,5,2*pi,2*pi,2*pi,2*pi];%,2*pi,2*pi]';
parameters.dt = 0.1;
parameters.lambda_e = [1,1,1,100,100,100]';
parameters.lambda_j = 0.000001;
% parameters.lambda_v = [10,10,5,5,5,1,1,1]'/; % length of n
parameters.lambda_v = [0,0,0,0,0,0];%,0,0]'; % length of n
parameters.time = 20;
parameters.steps = parameters.time/parameters.dt;
parameters.base_dof = 2;
parameters.arm_dof = 4;
parameters.total_dof = parameters.base_dof+parameters.arm_dof;
dof.base = parameters.base_dof;
dof.arm = parameters.arm_dof;
dof.total = parameters.total_dof;
dof2.base = 2;
dof2.arm = 0;
dof2.total = 2;


%% Initialize
% set array of pose goals (4 by 4 by k)
% pose_goals = generate_helix(2.5,0.0125,parameters.dt,parameters.t///ime); %% working
% pose_goals = generate_sine(0.25,0.25,parameters.dt,parameters.time); %% working
pose_goals = generate_horizontal_helix(0.5,1,parameters.dt,parameters.time); %% working

timer = zeros([1,parameters.steps]);
x = zeros([6,1]);
xdot = zeros([6,1]);
dx = zeros([6,1]);
parameters.base_pose = eye(4);

%% Main
pose = eye(4);

for plot_step = 1:parameters.steps

    % get next pose goal
    ee_desired = tform(pose_goals(plot_step));

    parameters.base_pose = parameters.base_pose*step_forward(parameters.screws,dx,dof2);
    parameters.base_pose;

    % perform ik optimization
    tic
    dx = ik_optimization(x, xdot, ee_desired, parameters)
    xdot(:,plot_step) = dx';
    timed = toc
    timer(plot_step) = timed;
    
    tool_pose(plot_step) = se3(parameters.base_pose*step_forward(parameters.screws,dx,dof)*parameters.config_state);
    tool_pose(plot_step);
    x(1:2) = [0;0];
    x(3:end) = dx(3:end);

    goal = ee_desired;
    actual = tform(tool_pose(plot_step));
    error(:,:,plot_step) = goal-actual;
    error(:,:,plot_step)
end

%% Plot
figure
hold on
plot(squeeze(error(1,4,:)))
plot(squeeze(error(2,4,:)))
plot(squeeze(error(3,4,:)))
legend('x','y','z')
grid minor

figure
plotTransforms(tool_pose)

figure
plot(xdot(1,:))
hold on
plot(xdot(2,:))
plot(xdot(3,:))
plot(xdot(4,:))
plot(xdot(5,:))
plot(xdot(6,:))
plot(xdot(7,:))
plot(xdot(8,:))