%% Screw joints
Lbx = 0.34;
% Lbx = 0;
Lby = 0.0;
Lbz = 0.384;
% Lbz = 0;
W1 = 109/1000;
W2 = 82/1000;
L1 = 425/1000;
L2 = 392/1000;
H1 = 89/1000;
H2 = 95/1000;
husky_base_to_ur_base = [Lbx,Lby,Lbz];
husky_ur = eye(4);
husky_ur(1:3,4) = husky_base_to_ur_base;
 
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

parameters.screws = [S3; % shoulder pan
                     S4; % shoulder lift
                     S5; % elbow
                     S6; % wrist 1
                     S7; % wrist 2
                     S8]; % wrist 3 screw joints for mobile manipulator (n rows of [omegax,omegay,omegaz,x,y,z])

%% Parameters definition
parameters.config_state = [-1,0,0,0.817+Lbx;
                           0,0,1,0.233+Lby;
                           0,1,0,0.063+Lbz;
                           0,0,0,1];
% parameters.config_state = [-1,0,0,0.817+Lbx;
%                            0,0,1,0.233+Lby;
%                            0,1,0,-0.063+Lbz;
%                            0,0,0,1];% zero state pose for mobile manipulator (se3 matrix, pose at q = [0])
parameters.adjoint = eye(6); % adjoint matrix for mobile manipulator (se3 matrix, current pose)
parameters.stateMin = [-pi,-pi,-pi,-pi,-pi,-pi]';
parameters.stateMax = [pi,pi,pi,pi,pi,pi]';
parameters.dt = 1;
parameters.lambda_e = 50;
parameters.lambda_j = 0.000001;
parameters.lambda_v = [1,1,1,1,1,1]'; % length of n
parameters.time = 10;
parameters.steps = parameters.time/parameters.dt;

pose = step_forward(parameters.screws,eye(6),[0;-pi/2;0;0;pi/2;0])*parameters.config_state
