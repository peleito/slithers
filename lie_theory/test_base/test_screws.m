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

parameters.screws = [S1; % base angular velocity
                     S2]; % base linear velocity screw joints for mobile manipulator (n rows of [omegax,omegay,omegaz,x,y,z])

%% Parameters definition
parameters.config_state = [1,0,0,0;
                           0,1,0,0;
                           0,0,1,0;
                           0,0,0,1]; % zero state pose for mobile manipulator (se3 matrix, pose at q = [0])
parameters.adjoint = eye(6); % adjoint matrix for mobile manipulator (se3 matrix, current pose)
parameters.stateMin = [-100,-1000]';
parameters.stateMax = [100,1000]';
parameters.dt = 1;
parameters.lambda_e = 50;
parameters.lambda_j = 0.000001;
parameters.lambda_v = [1,1]'; % length of n
parameters.time = 10;
parameters.steps = parameters.time/parameters.dt;

pose = step_forward(parameters.screws,eye(6),[pi/2;0])*parameters.config_state

x = [0,0]';
xdot = [0,0]';
ee_desired = pose
parameters.base_pose = eye(4);
inverse = ik_optimization(x, xdot, ee_desired, parameters)
pose = step_forward(parameters.screws,eye(6),inverse)*parameters.config_state

x = [0,0];
% xdot = [0,0]';
ee_desired = [0,-1,0,0;1,0,0,5;0,0,1,0;0,0,0,1]
parameters.base_pose = pose;
inverse = ik_optimization(x, xdot, ee_desired, parameters)
pose = pose*step_forward(parameters.screws,eye(6),inverse)*parameters.config_state

ee_desired = [1,0,0,0;0,1,0,5;0,0,1,0;0,0,0,1]
parameters.base_pose = pose;
inverse = ik_optimization(x, xdot, ee_desired, parameters)
pose = pose*step_forward(parameters.screws,eye(6),inverse)*parameters.config_state

ee_desired = [1,0,0,2;0,1,0,7;0,0,1,0;0,0,0,1]
parameters.base_pose = pose;
inverse = ik_optimization(x, xdot, ee_desired, parameters)
pose = pose*step_forward(parameters.screws,eye(6),inverse)*parameters.config_state